#pragma once

// #include "glomap/estimators/bundle_adjustment.h"
#include "glomap/estimators/optimization_base.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

#include <ceres/ceres.h>

namespace glomap {

struct BundleAdjusterOptions : public OptimizationBaseOptions {
 public:
  // Flags for which parameters to optimize
  bool optimize_rig_poses = false;  // Whether to optimize the rig poses
  bool optimize_rotations = true;
  bool optimize_translation = true;
  bool optimize_intrinsics = true;
  bool optimize_principal_point = false;
  bool optimize_points = true;

  bool use_gpu = true;
  std::string gpu_index = "-1";
  int min_num_images_gpu_solver = 50;

  // Constrain the minimum number of views per track
  int min_num_view_per_track = 3;

  bool add_prior_loss = true;
  std::unordered_map<image_t, Eigen::Vector3d>
      image_center_priors;  // in world frame
  ceres::LossFunction* loss_function_prior = new ceres::ScaledLoss(
      new ceres::HuberLoss(1.), 100.0, ceres::TAKE_OWNERSHIP);

  BundleAdjusterOptions() : OptimizationBaseOptions() {
    thres_loss_function = 1.;
    solver_options.max_num_iterations = 200;
  }

  std::shared_ptr<ceres::LossFunction> CreateLossFunction() {
    return std::make_shared<ceres::HuberLoss>(thres_loss_function);
  }
};
class BundleAdjuster {
 public:
  BundleAdjuster(const BundleAdjusterOptions& options) : options_(options) {}

  // Returns true if the optimization was a success, false if there was a
  // failure.
  // Assume tracks here are already filtered
  bool Solve(std::unordered_map<rig_t, Rig>& rigs,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<frame_t, Frame>& frames,
             std::unordered_map<image_t, Image>& images,
             std::unordered_map<track_t, Track>& tracks);

  BundleAdjusterOptions& GetOptions() { return options_; }

 private:
  // Reset the problem
  void Reset();

  // Add tracks to the problem
  void AddPointToCameraConstraints(
      std::unordered_map<rig_t, Rig>& rigs,
      std::unordered_map<camera_t, Camera>& cameras,
      std::unordered_map<frame_t, Frame>& frames,
      std::unordered_map<image_t, Image>& images,
      std::unordered_map<track_t, Track>& tracks);

  void AddImagePositionPriorConstraints(
      std::unordered_map<image_t, Image>& images);

  // Set the parameter groups
  void AddCamerasAndPointsToParameterGroups(
      std::unordered_map<rig_t, Rig>& rigs,
      std::unordered_map<camera_t, Camera>& cameras,
      std::unordered_map<frame_t, Frame>& frames,
      std::unordered_map<track_t, Track>& tracks);

  // Parameterize the variables, set some variables to be constant if desired
  void ParameterizeVariables(std::unordered_map<rig_t, Rig>& rigs,
                             std::unordered_map<camera_t, Camera>& cameras,
                             std::unordered_map<frame_t, Frame>& frames,
                             std::unordered_map<track_t, Track>& tracks);

  BundleAdjusterOptions options_;

  std::unique_ptr<ceres::Problem> problem_;
  std::shared_ptr<ceres::LossFunction> loss_function_;
};

}  // namespace glomap
