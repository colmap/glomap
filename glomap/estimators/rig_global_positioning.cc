#include "glomap/estimators/rig_global_positioning.h"

#include "glomap/estimators/cost_function.h"

namespace glomap {
namespace {

Eigen::Vector3d RandVector3d(std::mt19937& random_generator,
                             double low,
                             double high) {
  std::uniform_real_distribution<double> distribution(low, high);
  return Eigen::Vector3d(distribution(random_generator),
                         distribution(random_generator),
                         distribution(random_generator));
}

}  // namespace

RigGlobalPositioner::RigGlobalPositioner(
    const RigGlobalPositionerOptions& options)
    : options_(options) {
  random_generator_.seed(options_.seed);
}

bool RigGlobalPositioner::Solve(const ViewGraph& view_graph,
                                const std::vector<CameraRig>& camera_rigs,
                                std::unordered_map<camera_t, Camera>& cameras,
                                std::unordered_map<image_t, Image>& images,
                                std::unordered_map<track_t, Track>& tracks) {
  if (images.empty()) {
    LOG(ERROR) << "Number of images = " << images.size();
    return false;
  }
  if (view_graph.image_pairs.empty() &&
      options_.constraint_type != RigGlobalPositionerOptions::ONLY_POINTS) {
    LOG(ERROR) << "Number of image_pairs = " << view_graph.image_pairs.size();
    return false;
  }
  if (tracks.empty() &&
      options_.constraint_type != RigGlobalPositionerOptions::ONLY_CAMERAS) {
    LOG(ERROR) << "Number of tracks = " << tracks.size();
    return false;
  }

  LOG(INFO) << "Setting up the global positioner problem";

  // Setup the problem.
  SetupProblem(view_graph, camera_rigs, images, tracks);

  // Initialize camera translations to be random.
  // Also, convert the camera pose translation to be the camera center.
  InitializeRandomPositions(view_graph, images, tracks);

  // // Add the camera to camera constraints to the problem.
  // if (options_.constraint_type != RigGlobalPositionerOptions::ONLY_POINTS) {
  //   AddCameraToCameraConstraints(view_graph, images);
  // }

  // // Add the point to camera constraints to the problem.
  // if (options_.constraint_type != RigGlobalPositionerOptions::ONLY_CAMERAS) {
  // }
  AddPointToCameraConstraints(camera_rigs, cameras, images, tracks);

  AddCamerasAndPointsToParameterGroups(images, tracks);

  // Parameterize the variables, set image poses / tracks / scales to be
  // constant if desired
  ParameterizeVariables(images, tracks);

  LOG(INFO) << "Solving the global positioner problem";

  ceres::Solver::Summary summary;
  options_.solver_options.minimizer_progress_to_stdout = VLOG_IS_ON(2);
  ceres::Solve(options_.solver_options, problem_.get(), &summary);

  if (VLOG_IS_ON(2)) {
    LOG(INFO) << summary.FullReport();
  } else {
    LOG(INFO) << summary.BriefReport();
  }

  ConvertResults(images);
  return summary.IsSolutionUsable();
}

void RigGlobalPositioner::SetupProblem(
    const ViewGraph& view_graph,
    const std::vector<CameraRig>& camera_rigs,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks) {
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_ = std::make_unique<ceres::Problem>(problem_options);
  loss_function_ = options_.CreateLossFunction();

  // Allocate enough memory for the scales. One for each residual.
  // Due to possibly invalid image pairs or tracks, the actual number of
  // residuals may be smaller.
  scales_.clear();
  scales_.reserve(
      view_graph.image_pairs.size() +
      std::accumulate(tracks.begin(),
                      tracks.end(),
                      0,
                      [](int sum, const std::pair<track_t, Track>& track) {
                        return sum + track.second.observations.size();
                      }));

  // Check the validity of the provided camera rigs.
  std::unordered_set<camera_t> rig_camera_ids;
  // for (const CameraRig& camera_rig : camera_rigs) {
  for (size_t idx_rig = 0; idx_rig < camera_rigs.size(); idx_rig++) {
    // camera_rig.Check(reconstruction);
    const CameraRig& camera_rig = camera_rigs.at(idx_rig);
    for (const auto& camera_id : camera_rig.GetCameraIds()) {
      THROW_CHECK_EQ(rig_camera_ids.count(camera_id), 0)
          << "Camera must not be part of multiple camera rigs";
      rig_camera_ids.insert(camera_id);
    }

    for (const auto& snapshot : camera_rig.Snapshots()) {
      // for (const auto& image_id : snapshot) {
      for (size_t idx_snapshot = 0; idx_snapshot < snapshot.size(); idx_snapshot++) {
        image_t image_id = snapshot[idx_snapshot];
        THROW_CHECK_EQ(image_id_to_rig_from_world_.count(image_id), 0)
            << "Image must not be part of multiple camera rigs";
        // image_id_to_rig_from_world_.emplace(image_id, &camera_rig);
        image_id_to_rig_from_world_.emplace(image_id, &(rigs_from_world_[idx_rig][idx_snapshot]));
      }
    }
  }
  // Initialize the rig scales to be 1.0.
  rig_scales_.resize(camera_rigs.size(), 1.0);

  // // Establish the reconstruction without 3d points for colmap compatibility
  // ConvertGlomapToColmap(cameras, images, tracks, reconstruction, -1, false);

  ExtractRigsFromWorld(camera_rigs, images);
}

void RigGlobalPositioner::ExtractRigsFromWorld(
    const std::vector<CameraRig>& camera_rigs,
    const std::unordered_map<image_t, Image>& images) {
  rigs_from_world_.reserve(camera_rigs.size());
  for (const auto& camera_rig : camera_rigs) {
    rigs_from_world_.emplace_back();
    auto& rig_from_world = rigs_from_world_.back();
    const size_t num_snapshots = camera_rig.NumSnapshots();
    rig_from_world.resize(num_snapshots);
    for (size_t snapshot_idx = 0; snapshot_idx < num_snapshots;
         ++snapshot_idx) {
      rig_from_world[snapshot_idx] =
          // camera_rig.ComputeRigFromWorld(snapshot_idx, reconstruction_);
          camera_rig.ComputeRigFromWorld(snapshot_idx, images);
      for (const auto image_id : camera_rig.Snapshots()[snapshot_idx]) {
        image_id_to_rig_from_world_.emplace(image_id,
                                            &rig_from_world[snapshot_idx]);
      }
    }
  }
}

void RigGlobalPositioner::InitializeRandomPositions(
    const ViewGraph& view_graph,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  std::unordered_set<image_t> constrained_positions;
  constrained_positions.reserve(images.size());
  for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;

    constrained_positions.insert(image_pair.image_id1);
    constrained_positions.insert(image_pair.image_id2);
  }

  for (auto& rigs : rigs_from_world_) {
    for (auto& rig : rigs) {
      rig.translation = 100.0 * RandVector3d(random_generator_, -1, 1);
    }
  }

  for (const auto& [track_id, track] : tracks) {
    if (track.observations.size() < options_.min_num_view_per_track) continue;
    for (const auto& observation : tracks[track_id].observations) {
      if (images.find(observation.first) == images.end()) continue;
      Image& image = images[observation.first];
      if (!image.is_registered) continue;
      constrained_positions.insert(observation.first);
    }
  }

  // if (!options_.generate_random_positions || !options_.optimize_positions) {
  //   for (auto& [image_id, image] : images) {
  //     image.cam_from_world.translation = image.Center();
  //   }
  //   return;
  // }

  // // Generate random positions for the cameras centers.
  // for (auto& [image_id, image] : images) {
  //   // Only set the cameras to be random if they are needed to be optimized
  //   if (constrained_positions.find(image_id) != constrained_positions.end())
  //     image.cam_from_world.translation =
  //         100.0 * RandVector3d(random_generator_, -1, 1);
  //   else
  //     image.cam_from_world.translation = image.Center();
  // }

  VLOG(2) << "Constrained positions: " << constrained_positions.size();
}

// void RigGlobalPositioner::AddCameraToCameraConstraints(
//     const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images)
//     {
//   for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
//     if (image_pair.is_valid == false) continue;

//     const image_t image_id1 = image_pair.image_id1;
//     const image_t image_id2 = image_pair.image_id2;
//     if (images.find(image_id1) == images.end() ||
//         images.find(image_id2) == images.end()) {
//       continue;
//     }

//     CHECK_GT(scales_.capacity(), scales_.size())
//         << "Not enough capacity was reserved for the scales.";
//     double& scale = scales_.emplace_back(1);

//     const Eigen::Vector3d translation =
//         -(images[image_id2].cam_from_world.rotation.inverse() *
//           image_pair.cam2_from_cam1.translation);
//     ceres::CostFunction* cost_function =
//         BATAPairwiseDirectionError::Create(translation);
//     problem_->AddResidualBlock(
//         cost_function,
//         loss_function_.get(),
//         images[image_id1].cam_from_world.translation.data(),
//         images[image_id2].cam_from_world.translation.data(),
//         &scale);

//     problem_->SetParameterLowerBound(&scale, 0, 1e-5);
//   }

//   VLOG(2) << problem_->NumResidualBlocks()
//           << " camera to camera constraints were added to the position "
//              "estimation problem.";
// }

void RigGlobalPositioner::AddPointToCameraConstraints(
    const std::vector<CameraRig>& camera_rigs,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  // The number of camera-to-camera constraints coming from the relative poses

  const size_t num_cam_to_cam = problem_->NumResidualBlocks();
  // Find the tracks that are relevant to the current set of cameras
  const size_t num_pt_to_cam = tracks.size();

  VLOG(2) << num_pt_to_cam
          << " point to camera constriants were added to the position "
             "estimation problem.";

  if (num_pt_to_cam == 0) return;

  double weight_scale_pt = 1.0;
  // // Set the relative weight of the point to camera constraints based on
  // // the number of camera to camera constraints.
  // if (num_cam_to_cam > 0 &&
  //     options_.constraint_type ==
  //         RigGlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED) {
  //   weight_scale_pt = options_.constraint_reweight_scale *
  //                     static_cast<double>(num_cam_to_cam) /
  //                     static_cast<double>(num_pt_to_cam);
  // }
  VLOG(2) << "Point to camera weight scaled: " << weight_scale_pt;

  if (loss_function_ptcam_uncalibrated_ == nullptr) {
    loss_function_ptcam_uncalibrated_ =
        std::make_shared<ceres::ScaledLoss>(loss_function_.get(),
                                            0.5 * weight_scale_pt,
                                            ceres::DO_NOT_TAKE_OWNERSHIP);
  }

  // if (options_.constraint_type ==
  //     RigGlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED) {
  //   loss_function_ptcam_calibrated_ = std::make_shared<ceres::ScaledLoss>(
  //       loss_function_.get(), weight_scale_pt, ceres::DO_NOT_TAKE_OWNERSHIP);
  // } else {
  //   loss_function_ptcam_calibrated_ = loss_function_;
  // }
  loss_function_ptcam_calibrated_ = loss_function_;

  for (auto& [track_id, track] : tracks) {
    if (track.observations.size() < options_.min_num_view_per_track) continue;

    // Only set the points to be random if they are needed to be optimized
    if (options_.optimize_points && options_.generate_random_points) {
      track.xyz = 100.0 * RandVector3d(random_generator_, -1, 1);
      track.is_initialized = true;
    }

    AddTrackToProblem(track_id, camera_rigs, cameras, images, tracks);
  }
}

void RigGlobalPositioner::AddTrackToProblem(
    track_t track_id,
    const std::vector<CameraRig>& camera_rigs,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  // For each view in the track add the point to camera correspondences.
  for (const auto& observation : tracks[track_id].observations) {
    if (images.find(observation.first) == images.end()) continue;

    Image& image = images[observation.first];
    if (!image.is_registered) continue;

    const Eigen::Vector3d& feature_undist =
        image.features_undist[observation.second];
    if (feature_undist.array().isNaN().any()) {
      LOG(WARNING)
          << "Ignoring feature because it failed to undistort: track_id="
          << track_id << ", image_id=" << observation.first
          << ", feature_id=" << observation.second;
      continue;
    }

    const Eigen::Vector3d translation =
        image.cam_from_world.rotation.inverse() *
        image.features_undist[observation.second];

    double& scale = scales_.emplace_back(1);

    if (!options_.generate_scales && tracks[track_id].is_initialized) {
      const Eigen::Vector3d trans_calc =
          tracks[track_id].xyz - image.cam_from_world.translation;
      scale = std::max(1e-5,
                       translation.dot(trans_calc) / trans_calc.squaredNorm());
    }

    CHECK_GT(scales_.capacity(), scales_.size())
        << "Not enough capacity was reserved for the scales.";

    // If the image is not part of a camera rig, use the standard BATA error
    if (image_id_to_rig_from_world_.find(observation.first) ==
        image_id_to_rig_from_world_.end()) {
      ceres::CostFunction* cost_function =
          BATAPairwiseDirectionError::Create(translation);

      // For calibrated and uncalibrated cameras, use different loss functions
      // Down weight the uncalibrated cameras
      if (cameras[image.camera_id].has_prior_focal_length) {
        problem_->AddResidualBlock(cost_function,
                                   loss_function_ptcam_calibrated_.get(),
                                   image.cam_from_world.translation.data(),
                                   tracks[track_id].xyz.data(),
                                   &scale);
      } else {
        problem_->AddResidualBlock(cost_function,
                                   loss_function_ptcam_uncalibrated_.get(),
                                   image.cam_from_world.translation.data(),
                                   tracks[track_id].xyz.data(),
                                   &scale);
      }
      // If the image is part of a camera rig, use the RigBATA error
    } else {
      // const Eigen::Vector3d translation_rig =
      //     image.cam_from_world.rotation.inverse() *
      //     image_id_to_rig_from_world_[observation.first]->translation;
      const Eigen::Vector3d translation_rig =
          image.cam_from_world.rotation.inverse() *
          camera_rigs[image_id_to_camera_rig_index_[observation.first]]
              .CamFromRig(images[observation.first].camera_id)
              .translation;
      // image_id_to_camera_rig_index_[observation.first]
      //     : image.cam_from_world.rotation.inverse() *
      //           rigs_from_world_[image_id_to_camera_rig_index_[observation.first]]
      //               [image_id_to_camera_rig_index_[observation.first] - 1]
      //               .translation;

      ceres::CostFunction* cost_function =
          RigBATAPairwiseDirectionError::Create(translation, translation_rig);

      // For calibrated and uncalibrated cameras, use different loss functions
      // Down weight the uncalibrated cameras
      if (cameras[image.camera_id].has_prior_focal_length) {
        problem_->AddResidualBlock(
            cost_function,
            loss_function_ptcam_calibrated_.get(),
            image_id_to_rig_from_world_[observation.first]->translation.data(),
            tracks[track_id].xyz.data(),
            &rig_scales_[image_id_to_camera_rig_index_[observation.first]],
            &scale);
      } else {
        problem_->AddResidualBlock(
            cost_function,
            loss_function_ptcam_uncalibrated_.get(),
            image_id_to_rig_from_world_[observation.first]->translation.data(),
            tracks[track_id].xyz.data(),
            &rig_scales_[image_id_to_camera_rig_index_[observation.first]],
            &scale);
      }
    }

    problem_->SetParameterLowerBound(&scale, 0, 1e-5);
  }
}

void RigGlobalPositioner::AddCamerasAndPointsToParameterGroups(
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  // Create a custom ordering for Schur-based problems.
  options_.solver_options.linear_solver_ordering.reset(
      new ceres::ParameterBlockOrdering);
  ceres::ParameterBlockOrdering* parameter_ordering =
      options_.solver_options.linear_solver_ordering.get();

  // Add scale parameters to group 0 (large and independent)
  for (double& scale : scales_) {
    parameter_ordering->AddElementToGroup(&scale, 0);
  }

  // Add point parameters to group 1.
  int group_id = 1;
  if (tracks.size() > 0) {
    for (auto& [track_id, track] : tracks) {
      if (problem_->HasParameterBlock(track.xyz.data()))
        parameter_ordering->AddElementToGroup(track.xyz.data(), group_id);
    }
    group_id++;
  }

  // Add camera parameters to group 2 if there are tracks, otherwise group 1.
  for (auto& [image_id, image] : images) {
    if (problem_->HasParameterBlock(image.cam_from_world.translation.data())) {
      parameter_ordering->AddElementToGroup(
          image.cam_from_world.translation.data(), group_id);
    }
  }
  group_id++;

  for (auto& rigs : rigs_from_world_) {
    for (auto& rig : rigs) {
      if (problem_->HasParameterBlock(rig.translation.data())) {
        parameter_ordering->AddElementToGroup(rig.translation.data(), group_id);
      }
    }
  }
  // Also add the scales to the group
  for (double& scale : rig_scales_) {
    parameter_ordering->AddElementToGroup(&scale, group_id);
  }
}

void RigGlobalPositioner::ParameterizeVariables(
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  // For the global positioning, do not set any camera to be constant for easier
  // convergence

  // If do not optimize the positions, set the camera positions to be constant
  if (!options_.optimize_positions) {
    for (auto& [image_id, image] : images)
      if (problem_->HasParameterBlock(image.cam_from_world.translation.data()))
        problem_->SetParameterBlockConstant(
            image.cam_from_world.translation.data());
  }

  // If do not optimize the rotations, set the camera rotations to be constant
  if (!options_.optimize_points) {
    for (auto& [track_id, track] : tracks) {
      if (problem_->HasParameterBlock(track.xyz.data())) {
        problem_->SetParameterBlockConstant(track.xyz.data());
      }
    }
  }

  // If do not optimize the scales, set the scales to be constant
  if (!options_.optimize_scales) {
    for (double& scale : scales_) {
      problem_->SetParameterBlockConstant(&scale);
    }
  }

  // Also add the scales to the group
  for (double& scale : rig_scales_) {
    problem_->SetParameterLowerBound(&scale, 0, 1e-5);
  }

  // Set up the options for the solver
  // Do not use iterative solvers, for its suboptimal performance.
  if (tracks.size() > 0) {
    options_.solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
    options_.solver_options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;
  } else {
    options_.solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options_.solver_options.preconditioner_type = ceres::JACOBI;
  }
}

void RigGlobalPositioner::ConvertResults(
    std::unordered_map<image_t, Image>& images) {
  // translation now stores the camera position, needs to convert back to
  // translation
  for (auto& [image_id, image] : images) {
    image.cam_from_world.translation =
        -(image.cam_from_world.rotation * image.cam_from_world.translation);
  }
}

}  // namespace glomap
