#include "glomap/estimators/global_positioning.h"

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

GlobalPositioner::GlobalPositioner(const GlobalPositionerOptions& options)
    : options_(options) {
  random_generator_.seed(options_.seed);
}

bool GlobalPositioner::Solve(const ViewGraph& view_graph,
                             std::unordered_map<camera_t, Camera>& cameras,
                             std::unordered_map<image_t, Image>& images,
                             std::unordered_map<track_t, Track>& tracks) {
  if (images.empty()) {
    LOG(ERROR) << "Number of images = " << images.size() << std::endl;
    return false;
  }
  if (view_graph.image_pairs.empty() &&
      options_.constraint_type != GlobalPositionerOptions::ONLY_POINTS) {
    LOG(ERROR) << "Number of image_pairs = " << view_graph.image_pairs.size()
              << std::endl;
    return false;
  }
  if (tracks.empty() &&
      options_.constraint_type != GlobalPositionerOptions::ONLY_CAMERAS) {
    LOG(ERROR) << "Number of tracks = " << tracks.size() << std::endl;
    return false;
  }

  // Initialize the problem
  Reset();

  // Initialize camera translations to be random.
  // Also, convert the camera pose translation to be the camera center.
  InitializeRandomPositions(view_graph, images, tracks);

  // Add the camera to camera constraints to the problem.
  if (options_.constraint_type != GlobalPositionerOptions::ONLY_POINTS) {
    AddCameraToCameraConstraints(view_graph, images);
  }

  // Add the point to camera constraints to the problem.
  if (options_.constraint_type != GlobalPositionerOptions::ONLY_CAMERAS) {
    AddPointToCameraConstraints(cameras, images, tracks);
  }

  AddCamerasAndPointsToParameterGroups(images, tracks);

  // Parameterize the variables, set image poses / tracks / scales to be
  // constant if desired
  ParameterizeVariables(images, tracks);

  ceres::Solver::Summary summary;
  options_.solver_options.minimizer_progress_to_stdout = options_.verbose;
  ceres::Solve(options_.solver_options, problem_.get(), &summary);

  if (options_.verbose) {
    LOG(INFO) << summary.FullReport();
  }

  ConvertResults(images);
  return summary.IsSolutionUsable();
}

void GlobalPositioner::Reset() {
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_ = std::make_unique<ceres::Problem>(problem_options);
  scales_.clear();
}

void GlobalPositioner::InitializeRandomPositions(
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

  if (options_.constraint_type != GlobalPositionerOptions::ONLY_CAMERAS) {
    for (const auto& [track_id, track] : tracks) {
      if (track.observations.size() < options_.min_num_view_per_track) continue;
      for (const auto& observation : tracks[track_id].observations) {
        if (images.find(observation.first) == images.end()) continue;
        Image& image = images[observation.first];
        if (!image.is_registered) continue;
        constrained_positions.insert(observation.first);
      }
    }
  }

  if (!options_.generate_random_positions || !options_.optimize_positions) {
    for (auto& [image_id, image] : images) {
      image.cam_from_world.translation = image.Center();
    }
    return;
  }

  // Generate random positions for the cameras centers.
  for (auto& [image_id, image] : images) {
    // Only set the cameras to be random if they are needed to be optimized
    if (constrained_positions.find(image_id) != constrained_positions.end())
      image.cam_from_world.translation =
          100.0 * RandVector3d(random_generator_, -1, 1);
    else
      image.cam_from_world.translation = image.Center();
  }

  if (options_.verbose)
    LOG(INFO) << "Constrained positions: " << constrained_positions.size()
              << std::endl;
}

void GlobalPositioner::AddCameraToCameraConstraints(
    const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images) {
  for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;

    const image_t image_id1 = image_pair.image_id1;
    const image_t image_id2 = image_pair.image_id2;
    if (images.find(image_id1) == images.end() ||
        images.find(image_id2) == images.end())
      continue;

    track_t counter = scales_.size();
    scales_.insert(std::make_pair(counter, 1));

    Eigen::Vector3d translation = -images[image_id2].cam_from_world.Derotate(
        image_pair.cam2_from_cam1.translation);
    ceres::CostFunction* cost_function =
        BATAPairwiseDirectionError::Create(translation);
    problem_->AddResidualBlock(
        cost_function,
        options_.loss_function.get(),
        images[image_id1].cam_from_world.translation.data(),
        images[image_id2].cam_from_world.translation.data(),
        &(scales_[counter]));

    problem_->SetParameterLowerBound(&(scales_[counter]), 0, 1e-5);
  }

  if (options_.verbose)
    LOG(INFO) << problem_->NumResidualBlocks()
              << " camera to camera constraints were added to the position "
                 "estimation problem."
              << std::endl;
}

void GlobalPositioner::AddPointToCameraConstraints(
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  // The number of camera-to-camera constraints coming from the relative poses

  const size_t num_cam_to_cam = problem_->NumResidualBlocks();
  // Find the tracks that are relevant to the current set of cameras
  const size_t num_pt_to_cam = tracks.size();

  if (options_.verbose)
    LOG(INFO) << num_pt_to_cam
              << " point to camera constriants were added to the position "
                 "estimation problem."
              << std::endl;

  if (num_pt_to_cam == 0) return;

  double weight_scale_pt = 1.0;
  // Set the relative weight of the point to camera constraints based on
  // the number of camera to camera constraints.
  if (num_cam_to_cam > 0 &&
      options_.constraint_type ==
          GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED) {
    weight_scale_pt = options_.constraint_reweight_scale *
                      static_cast<double>(num_cam_to_cam) /
                      static_cast<double>(num_pt_to_cam);
  }
  if (options_.verbose)
    LOG(INFO) << "Point to camera weight scaled: " << weight_scale_pt
              << std::endl;

  if (loss_function_ptcam_uncalibrated_ == nullptr) {
    loss_function_ptcam_uncalibrated_ =
        std::make_shared<ceres::ScaledLoss>(options_.loss_function.get(),
                                            0.5 * weight_scale_pt,
                                            ceres::DO_NOT_TAKE_OWNERSHIP);
  }

  if (options_.constraint_type ==
      GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED) {
    loss_function_ptcam_calibrated_ =
        std::make_shared<ceres::ScaledLoss>(options_.loss_function.get(),
                                            weight_scale_pt,
                                            ceres::DO_NOT_TAKE_OWNERSHIP);
  } else {
    loss_function_ptcam_calibrated_ = options_.loss_function;
  }

  for (auto& [track_id, track] : tracks) {
    if (track.observations.size() < options_.min_num_view_per_track) continue;

    // Only set the points to be random if they are needed to be optimized
    if (options_.optimize_points && options_.generate_random_points) {
      track.xyz = 100.0 * RandVector3d(random_generator_, -1, 1);
      track.is_initialized = true;
    }

    AddTrackToProblem(track_id, cameras, images, tracks);
  }
}

void GlobalPositioner::AddTrackToProblem(
    const track_t& track_id,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  // For each view in the track add the point to camera correspondences.
  for (const auto& observation : tracks[track_id].observations) {
    if (images.find(observation.first) == images.end()) continue;

    Image& image = images[observation.first];
    if (!image.is_registered) continue;

    Eigen::Vector3d translation = image.cam_from_world.Derotate(
        image.features_undist[observation.second]);
    ceres::CostFunction* cost_function =
        BATAPairwiseDirectionError::Create(translation);

    track_t counter = scales_.size();
    if (options_.generate_scales || !tracks[track_id].is_initialized) {
      scales_.insert(std::make_pair(counter, 1));
    } else {
      Eigen::Vector3d trans_calc =
          tracks[track_id].xyz - image.cam_from_world.translation;
      double scale = translation.dot(trans_calc) / trans_calc.squaredNorm();
      scales_.insert(std::make_pair(counter, std::max(scale, 1e-5)));
    }

    // For calibrated and uncalibrated cameras, use different loss functions
    // Down weight the uncalibrated cameras
    (cameras[image.camera_id].has_prior_focal_length)
        ? problem_->AddResidualBlock(cost_function,
                                     loss_function_ptcam_calibrated_.get(),
                                     image.cam_from_world.translation.data(),
                                     tracks[track_id].xyz.data(),
                                     &(scales_[counter]))
        : problem_->AddResidualBlock(cost_function,
                                     loss_function_ptcam_uncalibrated_.get(),
                                     image.cam_from_world.translation.data(),
                                     tracks[track_id].xyz.data(),
                                     &(scales_[counter]));

    problem_->SetParameterLowerBound(&(scales_[counter]), 0, 1e-5);
  }
}

void GlobalPositioner::AddCamerasAndPointsToParameterGroups(
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  // Create a custom ordering for Schur-based problems.
  options_.solver_options.linear_solver_ordering.reset(
      new ceres::ParameterBlockOrdering);
  ceres::ParameterBlockOrdering* parameter_ordering =
      options_.solver_options.linear_solver_ordering.get();

  // Add scale parameters to group 0 (large and independent)
  for (auto& [i, scale] : scales_) {
    parameter_ordering->AddElementToGroup(&(scales_[i]), 0);
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
}

void GlobalPositioner::ParameterizeVariables(
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
    for (auto& [i, scale] : scales_) {
      problem_->SetParameterBlockConstant(&(scales_[i]));
    }
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

void GlobalPositioner::ConvertResults(
    std::unordered_map<image_t, Image>& images) {
  // translation now stores the camera position, needs to convert back to
  // translation
  for (auto& [image_id, image] : images) {
    image.cam_from_world.translation =
        -image.cam_from_world.Rotate(image.cam_from_world.translation);
  }
}

}  // namespace glomap