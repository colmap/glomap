#include "glomap/estimators/global_positioning.h"

#include "glomap/estimators/cost_function.h"
#include "glomap/math/rigid3d.h"

#include <colmap/util/cuda.h>
#include <colmap/util/misc.h>

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

GlobalPositioner::GlobalPositioner(
    const GlobalPositionerOptions& options)
    : options_(options) {
  random_generator_.seed(options_.seed);
}

bool GlobalPositioner::Solve(const ViewGraph& view_graph,
                                std::unordered_map<rig_t, Rig>& rigs,
                                std::unordered_map<camera_t, Camera>& cameras,
                                std::unordered_map<frame_t, Frame>& frames,
                                std::unordered_map<image_t, Image>& images,
                                std::unordered_map<track_t, Track>& tracks) {
  if (rigs.size() > 1) {
    LOG(ERROR) << "Number of camera rigs = " << rigs.size();
  }
  if (images.empty()) {
    LOG(ERROR) << "Number of images = " << images.size();
    return false;
  }
  if (view_graph.image_pairs.empty() &&
      options_.constraint_type != GlobalPositionerOptions::ONLY_POINTS) {
    LOG(ERROR) << "Number of image_pairs = " << view_graph.image_pairs.size();
    return false;
  }
  if (tracks.empty() &&
      options_.constraint_type != GlobalPositionerOptions::ONLY_CAMERAS) {
    LOG(ERROR) << "Number of tracks = " << tracks.size();
    return false;
  }

  LOG(INFO) << "Setting up the global positioner problem";

  // Setup the problem.
  // SetupProblem(view_graph, rigs, images, frames, tracks);
  SetupProblem(view_graph, rigs, tracks);

  // Initialize camera translations to be random.
  // Also, convert the camera pose translation to be the camera center.
  InitializeRandomPositions(view_graph, frames, images, tracks);

  // // Add the camera to camera constraints to the problem.
  // if (options_.constraint_type != GlobalPositionerOptions::ONLY_POINTS) {
  //   AddCameraToCameraConstraints(view_graph, images);
  // }

  // // Add the point to camera constraints to the problem.
  // if (options_.constraint_type != GlobalPositionerOptions::ONLY_CAMERAS) {
  // }
  AddPointToCameraConstraints(rigs, cameras, frames, images, tracks);

  AddCamerasAndPointsToParameterGroups(rigs, frames, tracks);

  // Parameterize the variables, set image poses / tracks / scales to be
  // constant if desired
  ParameterizeVariables(rigs, frames, tracks);

  LOG(INFO) << "Solving the global positioner problem";

  ceres::Solver::Summary summary;
  // options_.solver_options.minimizer_progress_to_stdout = VLOG_IS_ON(2);
  options_.solver_options.minimizer_progress_to_stdout = true;
  ceres::Solve(options_.solver_options, problem_.get(), &summary);

  // if (VLOG_IS_ON(2)) {
  if (true) {
    LOG(INFO) << summary.FullReport();
  } else {
    LOG(INFO) << summary.BriefReport();
  }

  ConvertResults(rigs, frames);
  return summary.IsSolutionUsable();
}

void GlobalPositioner::SetupProblem(
    const ViewGraph& view_graph,
    const std::unordered_map<rig_t, Rig>& rigs,
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

  // ExtractRigsFromWorld(rigs, images);

  // Initialize the rig scales to be 1.0.
  for (const auto& [rig_id, rig] : rigs) {
    rig_scales_.emplace(rig_id, 1.0);
  }
}

// void GlobalPositioner::ExtractRigsFromWorld(
//     const std::unordered_map<rig_t, Rig>& rigs,
//     const std::unordered_map<image_t, Image>& images) {
//   rigs_from_world_.reserve(rigs.size());
//   for (size_t idx_rig = 0; idx_rig < rigs.size(); ++idx_rig) {
//     const auto& camera_rig = rigs.at(idx_rig);
//     rigs_from_world_.emplace_back();
//     auto& rig_from_world = rigs_from_world_.back();
//     const size_t num_snapshots = camera_rig.NumSnapshots();
//     rig_from_world.resize(num_snapshots);
//     for (size_t snapshot_idx = 0; snapshot_idx < num_snapshots;
//          ++snapshot_idx) {
//       rig_from_world[snapshot_idx] =
//           camera_rig.ComputeRigFromWorld(snapshot_idx, images);
//       for (const auto image_id : camera_rig.Snapshots()[snapshot_idx]) {
//         image_id_to_camera_rig_index_.emplace(image_id, idx_rig);
//         image_id_to_rig_from_world_.emplace(image_id,
//                                             &rig_from_world[snapshot_idx]);
//       }
//     }
//   }
// }

void GlobalPositioner::InitializeRandomPositions(
    const ViewGraph& view_graph,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  std::unordered_set<image_t> constrained_positions;
  constrained_positions.reserve(frames.size());
  for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;
    constrained_positions.insert(images[image_pair.image_id1].frame_id);
    constrained_positions.insert(images[image_pair.image_id2].frame_id);
    // // Only modify the camera positions if they are not part of a camera rig
    // if (image_id_to_camera_rig_index_.find(image_pair.image_id1) ==
    //     image_id_to_camera_rig_index_.end())
    //   constrained_positions.insert(image_pair.image_id1);
    // if (image_id_to_camera_rig_index_.find(image_pair.image_id2) ==
    //     image_id_to_camera_rig_index_.end())
    //   constrained_positions.insert(image_pair.image_id2);
  }

  // for (auto& rigs : rigs_from_world_) {
  //   for (auto& rig : rigs) {
  //     if (options_.optimize_positions) {
  //       rig.translation = 100.0 * RandVector3d(random_generator_, -1, 1);
  //     } else {
  //       rig.translation = colmap::Inverse(rig).translation;
  //       std::cout << rig.translation.transpose() << std::endl;
  //     }
  //   }
  // }

  for (const auto& [track_id, track] : tracks) {
    if (track.observations.size() < options_.min_num_view_per_track) continue;
    for (const auto& observation : tracks[track_id].observations) {
      if (images.find(observation.first) == images.end()) continue;
      Image& image = images[observation.first];
      if (!image.is_registered) continue;
      constrained_positions.insert(images[observation.first].frame_id);
    }
  }

  if (!options_.generate_random_positions || !options_.optimize_positions) {
    // for (auto& [image_id, image] : images) {
    //   if (constrained_positions.find(image_id) !=
    //   constrained_positions.end())
    //     image.cam_from_world.translation = image.Center();
    // }
    // return;
    for (auto& [frame_id, frame] : frames) {
      if (constrained_positions.find(frame_id) != constrained_positions.end())
        frame.RigFromWorld().translation = CenterFromPose(frame.RigFromWorld());
    }
    return;
  }

  // Generate random positions for the cameras centers.
  // for (auto& [image_id, image] : images) {
  for (auto& [frame_id, frame] : frames) {
    // Only set the cameras to be random if they are needed to be optimized
    if (constrained_positions.find(frame_id) != constrained_positions.end())
      frame.RigFromWorld().translation =
          100.0 * RandVector3d(random_generator_, -1, 1);
    else
      frame.RigFromWorld().translation = CenterFromPose(frame.RigFromWorld());
  }

  VLOG(2) << "Constrained positions: " << constrained_positions.size();
}

void GlobalPositioner::AddPointToCameraConstraints(
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<frame_t, Frame>& frames,
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
  VLOG(2) << "Point to camera weight scaled: " << weight_scale_pt;

  if (loss_function_ptcam_uncalibrated_ == nullptr) {
    loss_function_ptcam_uncalibrated_ =
        std::make_shared<ceres::ScaledLoss>(loss_function_.get(),
                                            0.5 * weight_scale_pt,
                                            ceres::DO_NOT_TAKE_OWNERSHIP);
  }

  loss_function_ptcam_calibrated_ = loss_function_;

  for (auto& [track_id, track] : tracks) {
    if (track.observations.size() < options_.min_num_view_per_track) continue;

    // Only set the points to be random if they are needed to be optimized
    if (options_.optimize_points && options_.generate_random_points) {
      track.xyz = 100.0 * RandVector3d(random_generator_, -1, 1);
      track.is_initialized = true;
    }

    AddTrackToProblem(track_id, rigs, cameras, frames, images, tracks);
  }
}

void GlobalPositioner::AddTrackToProblem(
    track_t track_id,
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<frame_t, Frame>& frames,
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
        image.CamFromWorld().rotation.inverse() *
        image.features_undist[observation.second];

    double& scale = scales_.emplace_back(1);

    if (!options_.generate_scales && tracks[track_id].is_initialized) {
      const Eigen::Vector3d trans_calc =
          tracks[track_id].xyz - image.CamFromWorld().translation;
      scale = std::max(1e-5,
                       translation.dot(trans_calc) / trans_calc.squaredNorm());
    }

    CHECK_GE(scales_.capacity(), scales_.size())
        << "Not enough capacity was reserved for the scales.";

    // For calibrated and uncalibrated cameras, use different loss
    // functions
    // Down weight the uncalibrated cameras
    ceres::LossFunction* loss_function =
        (cameras[image.camera_id].has_prior_focal_length)
            ? loss_function_ptcam_calibrated_.get()
            : loss_function_ptcam_uncalibrated_.get();

    // If the image is not part of a camera rig, use the standard BATA error
    // if (image_id_to_rig_from_world_.find(observation.first) ==
    //     image_id_to_rig_from_world_.end()) {
    if (image.HasTrivialFrame()) {
      ceres::CostFunction* cost_function =
          BATAPairwiseDirectionError::Create(translation);

      problem_->AddResidualBlock(
          cost_function,
          loss_function,
          image.frame_ptr->RigFromWorld().translation.data(),
          tracks[track_id].xyz.data(),
          &scale);
      // If the image is part of a camera rig, use the RigBATA error
    } else {
      rig_t rig_id = image.frame_ptr->RigId();
      // Otherwise, use the camera rig translation from the frame
      Rigid3d& cam_from_rig = rigs.at(rig_id).SensorFromRig(
          sensor_t(SensorType::CAMERA, image.camera_id));

      Eigen::Vector3d cam_from_rig_translation = cam_from_rig.translation;

      if (!cam_from_rig_translation.hasNaN()) {
        const Eigen::Vector3d translation_rig =
            // image.cam_from_world.rotation.inverse() *
            // cam_from_rig.translation;
            image.CamFromWorld().rotation.inverse() * cam_from_rig_translation;

        ceres::CostFunction* cost_function =
            RigBATAPairwiseDirectionError::Create(translation, translation_rig);

        problem_->AddResidualBlock(
            cost_function,
            loss_function,
            image.frame_ptr->RigFromWorld().translation.data(),
            tracks[track_id].xyz.data(),
            &scale,
            &rig_scales_[rig_id]);
      } else {
        // If the cam_from_rig contains nan values, it means that it needs to be
        // re-estimated In this case, use the rigged cost NOTE: the scale for
        // the rig is not needed, as it would natrually be consistent with the
        // global one

        // const Eigen::Vector3d translation_rig =
        //     // image.cam_from_world.rotation.inverse() *
        //     cam_from_rig.translation; image.CamFromWorld().rotation.inverse()
        //     * cam_from_rig_translation;

        ceres::CostFunction* cost_function =
            RigUnknownBATAPairwiseDirectionError::Create(translation,
                                                         image.frame_ptr->RigFromWorld().rotation);

        problem_->AddResidualBlock(
            cost_function,
            loss_function,
            tracks[track_id].xyz.data(),
            image.frame_ptr->RigFromWorld().translation.data(),
            cam_from_rig.translation.data(),
            &scale);
      }
    }

    problem_->SetParameterLowerBound(&scale, 0, 1e-5);
  }
}

void GlobalPositioner::AddCamerasAndPointsToParameterGroups(
    // std::unordered_map<image_t, Image>& images,
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<frame_t, Frame>& frames,
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

  // // Add camera parameters to group 2 if there are tracks, otherwise group 1.
  // for (auto& [image_id, image] : images) {
  //   if (problem_->HasParameterBlock(image.cam_from_world.translation.data()))
  //   {
  //     parameter_ordering->AddElementToGroup(
  //         image.cam_from_world.translation.data(), group_id);
  //   }
  // }

  // for (auto& rigs : rigs_from_world_) {
  //   for (auto& rig : rigs) {
  //     if (problem_->HasParameterBlock(rig.translation.data())) {
  //       parameter_ordering->AddElementToGroup(rig.translation.data(),
  //       group_id);
  //     }
  //   }
  // }
  for (auto& [frame_id, frame] : frames) {
    if (problem_->HasParameterBlock(frame.RigFromWorld().translation.data())) {
      parameter_ordering->AddElementToGroup(
          frame.RigFromWorld().translation.data(), group_id);
    }
  }

  // Add the cam_from_rigs to be estimated into the parameter group
  for (auto& [rig_id, rig] : rigs) {
    for (const auto& [sensor_id, sensor] : rig.Sensors()) {
      if (rig.IsRefSensor(sensor_id)) continue;
      if (sensor_id.type == SensorType::CAMERA) {
        Eigen::Vector3d& translation = rig.SensorFromRig(sensor_id).translation;
        if (problem_->HasParameterBlock(translation.data())) {
          parameter_ordering->AddElementToGroup(translation.data(), group_id);
        }
      }
    }
  }

  group_id++;

  // Also add the scales to the group
  for (auto& [rig_id, scale] : rig_scales_) {
    if (problem_->HasParameterBlock(&scale))
      parameter_ordering->AddElementToGroup(&scale, group_id);
  }
}

void GlobalPositioner::ParameterizeVariables(
    // std::unordered_map<image_t, Image>& images,
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<track_t, Track>& tracks) {
  // For the global positioning, do not set any camera to be constant for easier
  // convergence

  // First, for cam_from_rig that needs to be estimated, we need to initialize
  // the center
  if (options_.optimize_positions) {
    for (auto& [rig_id, rig] : rigs) {
      for (const auto& [sensor_id, sensor] : rig.Sensors()) {
        if (rig.IsRefSensor(sensor_id)) continue;
        if (sensor_id.type == SensorType::CAMERA) {
          Eigen::Vector3d& translation =
              rig.SensorFromRig(sensor_id).translation;
          if (problem_->HasParameterBlock(translation.data())) {
            translation = RandVector3d(random_generator_, -1, 1);
          }
        }
      }
    }
  }

  // If do not optimize the positions, set the camera positions to be constant
  if (!options_.optimize_positions) {
    // for (auto& [image_id, image] : images)
    //   if
    //   (problem_->HasParameterBlock(image.cam_from_world.translation.data()))
    //     problem_->SetParameterBlockConstant(
    //         image.cam_from_world.translation.data());

    // for (auto& rigs : rigs_from_world_) {
    for (auto& [frame_id, frame] : frames) {
      if (problem_->HasParameterBlock(frame.RigFromWorld().translation.data()))
        problem_->SetParameterBlockConstant(
            frame.RigFromWorld().translation.data());
    }
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
      if (problem_->HasParameterBlock(&scale)) {
        problem_->SetParameterBlockConstant(&scale);
      }
    }
  }
  // Set the first rig scale to be constant to remove the gauge ambiguity.
  for (double& scale : scales_) {
    if (problem_->HasParameterBlock(&scale)) {
      problem_->SetParameterBlockConstant(&scale);
      break;
    }
  }
  // Set the rig scales to be constant
  // TODO: add a flag to allow the scales to be optimized (if they are not in
  // metric scale)
  // for (double& scale : rig_scales_) {
  for (auto& [rig_id, scale] : rig_scales_) {
    if (problem_->HasParameterBlock(&scale)) {
      problem_->SetParameterBlockConstant(&scale);
    }
  }

  int num_images = frames.size();
#ifdef GLOMAP_CUDA_ENABLED
  bool cuda_solver_enabled = false;

#if (CERES_VERSION_MAJOR >= 3 ||                                \
     (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2)) && \
    !defined(CERES_NO_CUDA)
  if (options_.use_gpu && num_images >= options_.min_num_images_gpu_solver) {
    cuda_solver_enabled = true;
    options_.solver_options.dense_linear_algebra_library_type = ceres::CUDA;
  }
#else
  if (options_.use_gpu) {
    LOG_FIRST_N(WARNING, 1)
        << "Requested to use GPU for bundle adjustment, but Ceres was "
           "compiled without CUDA support. Falling back to CPU-based dense "
           "solvers.";
  }
#endif

#if (CERES_VERSION_MAJOR >= 3 ||                                \
     (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 3)) && \
    !defined(CERES_NO_CUDSS)
  if (options_.use_gpu && num_images >= options_.min_num_images_gpu_solver) {
    cuda_solver_enabled = true;
    options_.solver_options.sparse_linear_algebra_library_type =
        ceres::CUDA_SPARSE;
  }
#else
  if (options_.use_gpu) {
    LOG_FIRST_N(WARNING, 1)
        << "Requested to use GPU for bundle adjustment, but Ceres was "
           "compiled without cuDSS support. Falling back to CPU-based sparse "
           "solvers.";
  }
#endif

  if (cuda_solver_enabled) {
    const std::vector<int> gpu_indices =
        colmap::CSVToVector<int>(options_.gpu_index);
    THROW_CHECK_GT(gpu_indices.size(), 0);
    colmap::SetBestCudaDevice(gpu_indices[0]);
  }
#else
  if (options_.use_gpu) {
    LOG_FIRST_N(WARNING, 1)
        << "Requested to use GPU for bundle adjustment, but COLMAP was "
           "compiled without CUDA support. Falling back to CPU-based "
           "solvers.";
  }
#endif  // GLOMAP_CUDA_ENABLED

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

void GlobalPositioner::ConvertResults(
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<frame_t, Frame>& frames) {
  // // translation now stores the camera position, needs to convert back
  // // First, calculate the camera translations of the rigs
  // for (auto& rig_from_world_single : rigs_from_world_) {
  //   for (auto& rig_from_world : rig_from_world_single) {
  //     rig_from_world.translation =
  //         -(rig_from_world.rotation * rig_from_world.translation);
  //   }
  // }
  for (auto& [frame_id, frame] : frames) {
    frame.RigFromWorld().translation =
        -(frame.RigFromWorld().rotation * frame.RigFromWorld().translation);

    rig_t idx_rig = frame.RigId();
    frame.RigFromWorld().translation *= rig_scales_[idx_rig];
  }

  // Update the rig scales
  for (auto& [rig_id, rig] : rigs) {
    std::map<sensor_t, std::optional<Rigid3d>>& sensors = rig.Sensors();
    for (auto& [sensor_id, cam_from_rig] : sensors) {
      if (cam_from_rig.has_value()) {
        if (problem_->HasParameterBlock(rig.SensorFromRig(sensor_id).translation.data())) {
          cam_from_rig->translation =
              -(cam_from_rig->rotation * cam_from_rig->translation);
        } else {
          // If the camera is part of a rig, then scale the translation
          // by the rig scale
          cam_from_rig->translation *= rig_scales_[rig_id];
        }
      }
    }
  }

  // // For images that are not belong to any rig, directly use the center as
  // the for (auto& [image_id, image] : images) {
  //   if (image_id_to_rig_from_world_.count(image_id) == 0) {
  //     image.cam_from_world.translation =
  //         -(image.cam_from_world.rotation *
  //         image.cam_from_world.translation);
  //   }
  // }

  // for (size_t idx_rig = 0; idx_rig < rigs.size(); idx_rig++) {
  //   CameraRig& camera_rig = rigs.at(idx_rig);
  //   const size_t num_snapshots = camera_rig.NumSnapshots();
  //   // Go through all images in the rig and rescale the cam_from_rig
  //   std::vector<camera_t> cameras_ids = camera_rig.GetCameraIds();
  //   for (auto& camera_id : cameras_ids) {
  //     camera_rig.CamFromRig(camera_id).translation *= rig_scales_[idx_rig];
  //   }
  // }

  // // For images within rigs, use the chained translation
  // for (size_t idx_rig = 0; idx_rig < rigs.size(); idx_rig++) {
  //   const CameraRig& camera_rig = rigs.at(idx_rig);
  //   const size_t num_snapshots = camera_rig.NumSnapshots();
  //   for (size_t snapshot_idx = 0; snapshot_idx < num_snapshots;
  //        ++snapshot_idx) {
  //     for (const auto image_id : camera_rig.Snapshots()[snapshot_idx]) {
  //       camera_t camera_id = images[image_id].camera_id;
  //       const Rigid3d& cam_from_rig = camera_rig.CamFromRig(camera_id);
  //       images[image_id].cam_from_world =
  //           (cam_from_rig * rigs_from_world_[idx_rig][snapshot_idx]);
  //     }
  //   }
  // }

  // TODO: if the scale is optimized, then also update the rigs.
}

}  // namespace glomap
