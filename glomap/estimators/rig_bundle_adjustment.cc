#include "rig_bundle_adjustment.h"

#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/manifold.h>
#include <colmap/sensor/models.h>
#include <colmap/util/cuda.h>
#include <colmap/util/misc.h>

namespace glomap {

bool RigBundleAdjuster::Solve(std::unordered_map<rig_t, Rig>& rigs,
                              std::unordered_map<camera_t, Camera>& cameras,
                              std::unordered_map<frame_t, Frame>& frames,
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
  AddPointToCameraConstraints(rigs, cameras, frames, images, tracks);

  // Add the cameras and points to the parameter groups for schur-based
  // optimization
  AddCamerasAndPointsToParameterGroups(cameras, frames, tracks);

  // Parameterize the variables
  ParameterizeVariables(cameras, frames, tracks);

  // Set the solver options.
  ceres::Solver::Summary summary;

  int num_images = images.size();
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

  // Do not use the iterative solver, as it does not seem to be helpful
  options_.solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  options_.solver_options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;

  options_.solver_options.minimizer_progress_to_stdout = VLOG_IS_ON(2);
  ceres::Solve(options_.solver_options, problem_.get(), &summary);
  if (VLOG_IS_ON(2))
    LOG(INFO) << summary.FullReport();
  else
    LOG(INFO) << summary.BriefReport();

  // ConvertResults(camera_rigs, images);

  return summary.IsSolutionUsable();
}

void RigBundleAdjuster::Reset() {
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_ = std::make_unique<ceres::Problem>(problem_options);
  loss_function_ = options_.CreateLossFunction();

  // ExtractRigsFromWorld(camera_rigs, images);
}

// void RigBundleAdjuster::ExtractRigsFromWorld(
//     const std::vector<CameraRig>& camera_rigs,
//     const std::unordered_map<image_t, Image>& images) {
//   rigs_from_world_.reserve(camera_rigs.size());
//   for (size_t idx_rig = 0; idx_rig < camera_rigs.size(); ++idx_rig) {
//     const auto& camera_rig = camera_rigs.at(idx_rig);
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

void RigBundleAdjuster::AddPointToCameraConstraints(
    std::unordered_map<rig_t, Rig>& rigs,
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images,
    std::unordered_map<track_t, Track>& tracks) {
  for (auto& [track_id, track] : tracks) {
    if (track.observations.size() < options_.min_num_view_per_track) continue;

    for (const auto& observation : tracks[track_id].observations) {
      if (images.find(observation.first) == images.end()) continue;

      Image& image = images[observation.first];
      Frame* frame_ptr = image.frame_ptr;
      camera_t camera_id = image.camera_id;
      image_t rig_id = image.frame_ptr->RigId();

      ceres::CostFunction* cost_function = nullptr;
      // if (image_id_to_camera_rig_index_.find(observation.first) ==
      //     image_id_to_camera_rig_index_.end()) {
      if (image.HasTrivialFrame()) {
        cost_function =
            colmap::CreateCameraCostFunction<colmap::ReprojErrorCostFunctor>(
                cameras[image.camera_id].model_id,
                image.features[observation.second]);
        problem_->AddResidualBlock(
            cost_function,
            loss_function_.get(),
            frame_ptr->RigFromWorld().rotation.coeffs().data(),
            frame_ptr->RigFromWorld().translation.data(),
            tracks[track_id].xyz.data(),
            cameras[image.camera_id].params.data());
      } else if (!options_.optimize_rig_poses) {
        const Rigid3d& cam_from_rig = rigs[rig_id].SensorFromRig(
            sensor_t(SensorType::CAMERA, image.camera_id));
        cost_function = colmap::CreateCameraCostFunction<
            colmap::RigReprojErrorConstantRigCostFunctor>(
            cameras[image.camera_id].model_id,
            image.features[observation.second],
            cam_from_rig);
        problem_->AddResidualBlock(
            cost_function,
            loss_function_.get(),
            frame_ptr->RigFromWorld().rotation.coeffs().data(),
            frame_ptr->RigFromWorld().translation.data(),
            tracks[track_id].xyz.data(),
            cameras[image.camera_id].params.data());
      } else {
        // If the image is part of a camera rig, use the RigBATA error
        // Down weight the uncalibrated cameras
        Rigid3d& cam_from_rig = rigs[rig_id].SensorFromRig(
            sensor_t(SensorType::CAMERA, image.camera_id));
        cost_function =
            colmap::CreateCameraCostFunction<colmap::RigReprojErrorCostFunctor>(
                cameras[image.camera_id].model_id,
                image.features[observation.second]);
        problem_->AddResidualBlock(
            cost_function,
            loss_function_.get(),
            cam_from_rig.rotation.coeffs().data(),
            cam_from_rig.translation.data(),
            frame_ptr->RigFromWorld().rotation.coeffs().data(),
            frame_ptr->RigFromWorld().translation.data(),
            tracks[track_id].xyz.data(),
            cameras[image.camera_id].params.data());
      }

      if (cost_function != nullptr) {
      } else {
        LOG(ERROR) << "Camera model not supported: "
                   << colmap::CameraModelIdToName(
                          cameras[image.camera_id].model_id);
      }
    }
  }
}

void RigBundleAdjuster::AddCamerasAndPointsToParameterGroups(
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<frame_t, Frame>& frames,
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
  // for (auto& [image_id, image] : images) {
  //   if (problem_->HasParameterBlock(image.cam_from_world.translation.data()))
  //   {
  //     parameter_ordering->AddElementToGroup(
  //         image.cam_from_world.translation.data(), 1);
  //     parameter_ordering->AddElementToGroup(
  //         image.cam_from_world.rotation.coeffs().data(), 1);
  //   }
  // }
  for (auto& [frame_id, frame] : frames) {
    if (problem_->HasParameterBlock(frame.RigFromWorld().translation.data())) {
      parameter_ordering->AddElementToGroup(
          frame.RigFromWorld().translation.data(), 1);
      parameter_ordering->AddElementToGroup(
          frame.RigFromWorld().rotation.coeffs().data(), 1);
    }
  }

  // for (auto& rigs : rigs_from_world_) {
  //   for (auto& rig : rigs) {
  //     if (problem_->HasParameterBlock(rig.translation.data())) {
  //       parameter_ordering->AddElementToGroup(rig.translation.data(), 1);
  //       parameter_ordering->AddElementToGroup(rig.rotation.coeffs().data(),
  //       1);
  //     }
  //   }
  // }

  // Add camera parameters to group 1.
  for (auto& [camera_id, camera] : cameras) {
    if (problem_->HasParameterBlock(camera.params.data()))
      parameter_ordering->AddElementToGroup(camera.params.data(), 1);
  }
}

void RigBundleAdjuster::ParameterizeVariables(
    std::unordered_map<camera_t, Camera>& cameras,
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<track_t, Track>& tracks) {
  frame_t center;

  // Parameterize rotations, and set rotations and translations to be constant
  // if desired FUTURE: Consider fix the scale of the reconstruction
  int counter = 0;
  for (auto& [frame_id, frame] : frames) {
    if (problem_->HasParameterBlock(
            frame.RigFromWorld().rotation.coeffs().data())) {
      colmap::SetQuaternionManifold(
          problem_.get(), frame.RigFromWorld().rotation.coeffs().data());

      if (!options_.optimize_rotations || counter == 0)
        problem_->SetParameterBlockConstant(
            frame.RigFromWorld().rotation.coeffs().data());
      if (!options_.optimize_translation || counter == 0)
        problem_->SetParameterBlockConstant(
            frame.RigFromWorld().translation.data());

      counter++;
    }
  }

  // for (auto& rigs : rigs_from_world_) {
  //   for (auto& rig : rigs) {
  //     if (problem_->HasParameterBlock(rig.rotation.coeffs().data())) {
  //       colmap::SetQuaternionManifold(problem_.get(),
  //                                     rig.rotation.coeffs().data());

  //       if (!options_.optimize_rotations || counter == 0)
  //         problem_->SetParameterBlockConstant(rig.rotation.coeffs().data());
  //       if (!options_.optimize_translation || counter == 0)
  //         problem_->SetParameterBlockConstant(rig.translation.data());

  //       counter++;
  //     }
  //   }
  // }

  // Parameterize the camera parameters, or set them to be constant if desired
  if (options_.optimize_intrinsics && !options_.optimize_principal_point) {
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
  } else if (!options_.optimize_intrinsics &&
             !options_.optimize_principal_point) {
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

// void RigBundleAdjuster::ConvertResults(
//     const std::vector<CameraRig>& camera_rigs,
//     std::unordered_map<image_t, Image>& images) {
//   // For images within rigs, use the chained translation
//   for (size_t idx_rig = 0; idx_rig < camera_rigs.size(); idx_rig++) {
//     const CameraRig& camera_rig = camera_rigs.at(idx_rig);
//     const size_t num_snapshots = camera_rig.NumSnapshots();
//     for (size_t snapshot_idx = 0; snapshot_idx < num_snapshots;
//          ++snapshot_idx) {
//       for (const auto image_id : camera_rig.Snapshots()[snapshot_idx]) {
//         camera_t camera_id = images[image_id].camera_id;
//         const Rigid3d& cam_from_rig = camera_rig.CamFromRig(camera_id);

//         images[image_id].cam_from_world =
//             (cam_from_rig * rigs_from_world_[idx_rig][snapshot_idx]);
//       }
//     }
//   }
// }

}  // namespace glomap
