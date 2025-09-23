#pragma once

#include "glomap/estimators/optimization_base.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

#include <colmap/geometry/sim3.h>

#include <ceres/ceres.h>

namespace glomap {

struct BundleAdjusterOptions : public OptimizationBaseOptions {
 public:
  // Flags for which parameters to optimize
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
  explicit BundleAdjuster(const BundleAdjusterOptions& options)
      : options_(options) {}
  virtual ~BundleAdjuster() = default;

  // Returns true if the optimization was a success, false if there was a
  // failure.
  // Assume tracks here are already filtered
  virtual bool Solve(const ViewGraph& view_graph,
                     std::unordered_map<camera_t, Camera>& cameras,
                     std::unordered_map<image_t, Image>& images,
                     std::unordered_map<track_t, Track>& tracks);

  BundleAdjusterOptions& GetOptions() { return options_; }

 protected:
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
  std::shared_ptr<ceres::LossFunction> loss_function_;
};

struct PosePriorBundleAdjusterOptions {
  // Whether to use a robust loss on prior locations.
  bool use_robust_loss_on_prior_position = false;

  // Threshold on the residual for the robust loss
  // (chi2 for 3DoF at 95% = 7.815).
  double prior_position_loss_threshold = 7.815;

  // The factor used by ceres::ScaledLoss to scale the loss function applied to
  // prior position residuals.
  double prior_position_scaled_loss_factor = 1.;

  // Maximum RANSAC error for Sim3 alignment.
  double ransac_max_error = 0.;

  // Loss function for pose prior.
  std::shared_ptr<ceres::LossFunction> prior_position_loss_function;

  PosePriorBundleAdjusterOptions(bool use_robust_loss,
                                 double loss_scale,
                                 double scaled_loss_factor,
                                 double max_error)
      : use_robust_loss_on_prior_position(use_robust_loss),
        prior_position_loss_threshold(loss_scale),
        prior_position_scaled_loss_factor(scaled_loss_factor),
        ransac_max_error(max_error) {};
};

// Solve bundle adjustment with pose prior constraints, now support position
// constraint only. Inherits from base class `BundleAdjuster` that provides
// basic BA functions, options, and `problem`.
// This code is "copied" from the corresponding class in COLMAP.
class PosePriorBundleAdjuster : public BundleAdjuster {
 public:
  using Sim3d = colmap::Sim3d;
  using PosePrior = colmap::PosePrior;

  explicit PosePriorBundleAdjuster(
      const BundleAdjusterOptions& options,
      const PosePriorBundleAdjusterOptions& prior_options);
  virtual ~PosePriorBundleAdjuster() = default;

  bool Solve(const ViewGraph& view_graph,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<image_t, Image>& images,
             std::unordered_map<track_t, Track>& tracks) override;

 protected:
  // Allign the reconstruction to pose position priors.
  bool AlignReconstruction(
      const std::unordered_map<image_t, PosePrior>& pose_priors,
      std::unordered_map<image_t, Image>& images,
      std::unordered_map<track_t, Track>& tracks);

  // Add pose position prior constraints to the problem.
  void AddPosePositionPriorConstraints(
      const std::unordered_map<image_t, PosePrior>& pose_priors,
      const Sim3d& normalized_from_metric,
      std::unordered_map<image_t, Image>& images);

  PosePriorBundleAdjusterOptions prior_options_;
  std::unordered_map<image_t, PosePrior> pose_priors_;
};
}  // namespace glomap
