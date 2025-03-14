#pragma once

#include <colmap/util/logging.h>

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace glomap {

constexpr double EPS = 1e-12;
constexpr double HALF_PI = 3.141592653589793238462643383279502884L / 2;
constexpr double TWO_PI = 2 * 3.141592653589793238462643383279502884L;

struct InlierThresholdOptions {
  // Thresholds for 3D-2D matches
  double max_angle_error = 1.;           // in degree, for global positioning
  double max_reprojection_error = 1e-2;  // for bundle adjustment
  double min_triangulation_angle = 1.;   // in degree, for triangulation

  // Thresholds for image_pair
  double max_epipolar_error_E = 1.;
  double max_epipolar_error_F = 4.;
  double max_epipolar_error_H = 4.;

  // Thresholds for edges
  double min_inlier_num = 30;
  double min_inlier_ratio = 0.25;
  double max_rotation_error = 10.;  // in degree, for rotation averaging
};

struct PosePriorOptions {
  // Whether to use pose prior constraints.
  bool use_pose_position_prior = false;  // now only support position prior

  // Settings for pose prior covariance, takes effect only when the option
  // `overwrite_position_priors_covariance` is true.
  double prior_position_std_x = 1.;
  double prior_position_std_y = 1.;
  double prior_position_std_z = 1.;

  // Now colmap may not have a logic for position priors covariance, make
  // overwrite as a choice.
  bool overwrite_position_priors_covariance = false;

  // Settings for loss function in BA.
  bool use_robust_loss_on_prior_position = false;
  // The threshold for robust loss function (e.g. CauchyLoss scaling parameter)
  // to handle large residuals. This determines the point at which the loss
  // becomes less sensitive to outliers.
  double prior_position_loss_threshold = 7.815;
  // The factor used by ceres::ScaledLoss to scale the loss function applied to
  // prior position residuals. This controls the overall scaling of the loss
  // applied to the residuals.
  // Note: This factor works in conjunction with the covariance matrix to
  // jointly influence the residual weights.
  double prior_position_scaled_loss_factor = 1.;

  // Settings for alignment with pose priors.
  double alignment_ransac_max_error = 0.;
};
}  // namespace glomap
