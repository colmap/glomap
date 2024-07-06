#pragma once

#include "glomap/estimators/optimization_base.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

#include <ceres/ceres.h>

namespace glomap {

struct BundleAdjusterOptions : public OptimizationBaseOptions {
 public:
  // Flags for which parameters to optimize
  bool optimize_rotations = true;
  bool optimize_translation = true;
  bool optimize_intrinsics = true;
  bool optimize_points = true;

  // Constrain the minimum number of views per track
  int min_num_view_per_track = 3;

  BundleAdjusterOptions() : OptimizationBaseOptions() {
    thres_loss_function = 1.;
    loss_function = std::make_shared<ceres::HuberLoss>(thres_loss_function);
    solver_options.max_num_iterations = 200;
  }
};

class BundleAdjuster {
 public:
  BundleAdjuster(const BundleAdjusterOptions& options) : options_(options) {}

  // Returns true if the optimization was a success, false if there was a
  // failure.
  // Assume tracks here are already filtered
  bool Solve(const ViewGraph& view_graph,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<image_t, Image>& images,
             std::unordered_map<track_t, Track>& tracks);

  BundleAdjusterOptions& GetOptions() { return options_; }

 private:
  // Reset the problem
  void Reset();

  // Add tracks to the problem
  void AddPointToCameraConstraints(
      const ViewGraph& view_graph,
      std::unordered_map<camera_t, Camera>& cameras,
      std::unordered_map<image_t, Image>& images,
      std::unordered_map<track_t, Track>& tracks);

  // Set the parameter groups
  void AddCamerasAndPointsToParameterGroups(
      std::unordered_map<camera_t, Camera>& cameras,
      std::unordered_map<image_t, Image>& images,
      std::unordered_map<track_t, Track>& tracks);

  // Parameterize the variables, set some variables to be constant if desired
  void ParameterizeVariables(std::unordered_map<camera_t, Camera>& cameras,
                             std::unordered_map<image_t, Image>& images,
                             std::unordered_map<track_t, Track>& tracks);

  BundleAdjusterOptions options_;

  std::unique_ptr<ceres::Problem> problem_;
};

}  // namespace glomap
