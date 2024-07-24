#include "bundle_adjustment.h"

#include <colmap/estimators/cost_functions.h>
#include <colmap/sensor/models.h>

namespace glomap {

bool BundleAdjuster::Solve(const ViewGraph& view_graph,
                           std::unordered_map<camera_t, Camera>& cameras,
                           std::unordered_map<image_t, Image>& images,
                           std::unordered_map<track_t, Track>& tracks) {
  // Check if the input data is valid
  if (images.empty()) {
    LOG(ERROR) << "Number of images = " << images.size();
    return false;
  }
  if (tracks.empty()) {
    LOG(ERROR) << "Number of tracks = " << tracks.size();
    return false;
  }

  // Reset the problem
  Reset();

  // Add the constraints that the point tracks impose on the problem
  AddPointToCameraConstraints(view_graph, cameras, images, tracks);

  // Add the cameras and points to the parameter groups for schur-based
  // optimization
  AddCamerasAndPointsToParameterGroups(cameras, images, tracks);

  // Parameterize the variables
  ParameterizeVariables(cameras, images, tracks);

  // Set the solver options.
  ceres::Solver::Summary summary;

  // Do not use the iterative solver, as it does not seem to be helpful
  options_.solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  options_.solver_options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;

  options_.solver_options.minimizer_progress_to_stdout = options_.verbose;
  ceres::Solve(options_.solver_options, problem_.get(), &summary);
  if (options_.verbose)
    LOG(INFO) << summary.FullReport();
  else
    LOG(INFO) << summary.BriefReport();

  return summary.IsSolutionUsable();
}

void BundleAdjuster::Reset() {
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_ = std::make_unique<ceres::Problem>(problem_options);
}

void BundleAdjuster::AddPointToCameraConstraints(
    const ViewGraph& view_graph,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  for (auto& [track_id, track] : tracks) {
    if (track.observations.size() < options_.min_num_view_per_track) continue;

    for (const auto& observation : tracks[track_id].observations) {
      if (images.find(observation.first) == images.end()) continue;

      Image& image = images[observation.first];

      ceres::CostFunction* cost_function = nullptr;
      switch (cameras[image.camera_id].model_id) {
        case colmap::CameraModelId::kSimplePinhole:
          cost_function = colmap::
              ReprojErrorCostFunction<colmap::SimplePinholeCameraModel>::Create(
                  image.features[observation.second]);
          break;
        case colmap::CameraModelId::kPinhole:
          cost_function =
              colmap::ReprojErrorCostFunction<colmap::PinholeCameraModel>::
                  Create(image.features[observation.second]);
          break;
        case colmap::CameraModelId::kSimpleRadial:
          cost_function =
              colmap::ReprojErrorCostFunction<colmap::SimpleRadialCameraModel>::
                  Create(image.features[observation.second]);
          break;
        case colmap::CameraModelId::kRadial:
          cost_function =
              colmap::ReprojErrorCostFunction<colmap::RadialCameraModel>::
                  Create(image.features[observation.second]);
          break;
        case colmap::CameraModelId::kOpenCV:
          cost_function =
              colmap::ReprojErrorCostFunction<colmap::OpenCVCameraModel>::
                  Create(image.features[observation.second]);
          break;
        case colmap::CameraModelId::kOpenCVFisheye:
          cost_function = colmap::
              ReprojErrorCostFunction<colmap::OpenCVFisheyeCameraModel>::Create(
                  image.features[observation.second]);
          break;
        default:
          LOG(ERROR) << "Camera model not supported";
          break;
      }

      if (cost_function != nullptr) {
        problem_->AddResidualBlock(
            cost_function,
            options_.loss_function.get(),
            image.cam_from_world.rotation.coeffs().data(),
            image.cam_from_world.translation.data(),
            tracks[track_id].xyz.data(),
            cameras[image.camera_id].params.data());
      }
    }
  }
}

void BundleAdjuster::AddCamerasAndPointsToParameterGroups(
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  if (tracks.size() == 0) return;

  // Create a custom ordering for Schur-based problems.
  options_.solver_options.linear_solver_ordering.reset(
      new ceres::ParameterBlockOrdering);
  ceres::ParameterBlockOrdering* parameter_ordering =
      options_.solver_options.linear_solver_ordering.get();
  // Add point parameters to group 0.
  for (auto& [track_id, track] : tracks) {
    if (problem_->HasParameterBlock(track.xyz.data()))
      parameter_ordering->AddElementToGroup(track.xyz.data(), 0);
  }

  // Add camera parameters to group 1.
  for (auto& [image_id, image] : images) {
    if (problem_->HasParameterBlock(image.cam_from_world.translation.data())) {
      parameter_ordering->AddElementToGroup(
          image.cam_from_world.translation.data(), 1);
      parameter_ordering->AddElementToGroup(
          image.cam_from_world.rotation.coeffs().data(), 1);
    }
  }

  // Add camera parameters to group 1.
  for (auto& [camera_id, camera] : cameras) {
    if (problem_->HasParameterBlock(camera.params.data()))
      parameter_ordering->AddElementToGroup(camera.params.data(), 1);
  }
}

void BundleAdjuster::ParameterizeVariables(
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  image_t center;

  // Parameterize rotations, and set rotations and translations to be constant
  // if desired FUTURE: Consider fix the scale of the reconstruction
  int counter = 0;
  for (auto& [image_id, image] : images) {
    if (problem_->HasParameterBlock(
            image.cam_from_world.rotation.coeffs().data())) {
      colmap::SetQuaternionManifold(
          problem_.get(), image.cam_from_world.rotation.coeffs().data());

      if (counter == 0) {
        center = image_id;
        counter++;
      }
      if (!options_.optimize_rotations)
        problem_->SetParameterBlockConstant(
            image.cam_from_world.rotation.coeffs().data());
      if (!options_.optimize_translation)
        problem_->SetParameterBlockConstant(
            image.cam_from_world.translation.data());
    }
  }

  // Set the first camera to be fixed to remove the gauge ambiguity.
  problem_->SetParameterBlockConstant(
      images[center].cam_from_world.rotation.coeffs().data());
  problem_->SetParameterBlockConstant(
      images[center].cam_from_world.translation.data());

  // Parameterize the camera parameters, or set them to be constant if desired
  if (options_.optimize_intrinsics) {
    for (auto& [camera_id, camera] : cameras) {
      if (problem_->HasParameterBlock(camera.params.data())) {
        std::vector<int> principal_point_idxs;
        for (auto idx : camera.PrincipalPointIdxs()) {
          principal_point_idxs.push_back(idx);
        }
        colmap::SetSubsetManifold(camera.params.size(),
                                  principal_point_idxs,
                                  problem_.get(),
                                  camera.params.data());
      }
    }

  } else {
    for (auto& [camera_id, camera] : cameras) {
      if (problem_->HasParameterBlock(camera.params.data())) {
        problem_->SetParameterBlockConstant(camera.params.data());
      }
    }
  }

  if (!options_.optimize_points) {
    for (auto& [track_id, track] : tracks) {
      if (problem_->HasParameterBlock(track.xyz.data())) {
        problem_->SetParameterBlockConstant(track.xyz.data());
      }
    }
  }
}

}  // namespace glomap