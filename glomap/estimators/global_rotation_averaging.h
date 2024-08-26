#pragma once

#include "glomap/math/l1_solver.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

#include <string>
#include <vector>

// Code is adapted from Theia's RobustRotationEstimator
// (http://www.theia-sfm.org/). For gravity aligned rotation averaging, refere
// to the paper "Gravity Aligned Rotation Averaging"
namespace glomap {

// The struct to store the temporary information for each image pair
struct ImagePairTempInfo {
  // The index of relative pose in the residual vector
  image_pair_t index = -1;

  // Whether the relative rotation is gravity aligned
  double has_gravity = false;

  // The relative rotation between the two images (x, z component)
  double xz_error = 0;

  // R_rel is gravity aligned if gravity prior is available, otherwise it is the
  // relative rotation between the two images
  Eigen::Matrix3d R_rel = Eigen::Matrix3d::Identity();

  // angle_rel is the converted angle if gravity prior is available for both
  // images
  double angle_rel = 0;
};

struct RotationEstimatorOptions {
  // Maximum number of times to run L1 minimization.
  int max_num_l1_iterations = 5;

  // Average step size threshold to terminate the L1 minimization
  double l1_step_convergence_threshold = 0.001;

  // The number of iterative reweighted least squares iterations to perform.
  int max_num_irls_iterations = 100;

  // Average step size threshold to termininate the IRLS minimization
  double irls_step_convergence_threshold = 0.001;

  Eigen::Vector3d axis = Eigen::Vector3d(0, 1, 0);

  // This is the point where the Huber-like cost function switches from L1 to
  // L2.
  double irls_loss_parameter_sigma = 5.0;  // in degree

  enum WeightType {
    // For Geman-McClure weight, refer to the paper "Efficient and robust
    // large-scale rotation averaging" (Chatterjee et. al, 2013)
    GEMAN_MCCLURE,
    // For Half Norm, refer to the paper "Robust Relative Rotation Averaging"
    // (Chatterjee et. al, 2017)
    HALF_NORM,
  } weight_type = GEMAN_MCCLURE;

  // Flg to use maximum spanning tree for initialization
  bool skip_initialization = false;

  // TODO: Implement the weighted version for rotation averaging
  bool use_weight = false;

  // Flag to use gravity for rotation averaging
  bool use_gravity = false;
};

// TODO: Implement the stratified camera rotation estimation
// TODO: Implement the HALF_NORM loss for IRLS
// TODO: Implement the weighted version for rotation averaging
// TODO: Implement the gravity as prior for rotation averaging
class RotationEstimator {
 public:
  explicit RotationEstimator(const RotationEstimatorOptions& options)
      : options_(options) {}

  // Estimates the global orientations of all views based on an initial
  // guess. Returns true on successful estimation and false otherwise.
  bool EstimateRotations(const ViewGraph& view_graph,
                         std::unordered_map<image_t, Image>& images);

 protected:
  // Initialize the rotation from the maximum spanning tree
  // Number of inliers serve as weights
  void InitializeFromMaximumSpanningTree(
      const ViewGraph& view_graph, std::unordered_map<image_t, Image>& images);

  // Sets up the sparse linear system such that dR_ij = dR_j - dR_i. This is the
  // first-order approximation of the angle-axis rotations. This should only be
  // called once.
  void SetupLinearSystem(const ViewGraph& view_graph,
                         std::unordered_map<image_t, Image>& images);

  // Performs the L1 robust loss minimization.
  bool SolveL1Regression(const ViewGraph& view_graph,
                         std::unordered_map<image_t, Image>& images);

  // Performs the iteratively reweighted least squares.
  bool SolveIRLS(const ViewGraph& view_graph,
                 std::unordered_map<image_t, Image>& images);

  // Updates the global rotations based on the current rotation change.
  void UpdateGlobalRotations(const ViewGraph& view_graph,
                             std::unordered_map<image_t, Image>& images);

  // Computes the relative rotation (tangent space) residuals based on the
  // current global orientation estimates.
  void ComputeResiduals(const ViewGraph& view_graph,
                        std::unordered_map<image_t, Image>& images);

  // Computes the average size of the most recent step of the algorithm.
  // The is the average over all non-fixed global_orientations_ of their
  // rotation magnitudes.
  double ComputeAverageStepSize(
      const std::unordered_map<image_t, Image>& images);

  // Data
  // Options for the solver.
  const RotationEstimatorOptions& options_;

  // The sparse matrix used to maintain the linear system. This is matrix A in
  // Ax = b.
  Eigen::SparseMatrix<double> sparse_matrix_;

  // x in the linear system Ax = b.
  Eigen::VectorXd tangent_space_step_;

  // b in the linear system Ax = b.
  Eigen::VectorXd tangent_space_residual_;

  Eigen::VectorXd rotation_estimated_;

  // Varaibles for intermidiate results
  std::unordered_map<image_t, image_t> image_id_to_idx_;
  std::unordered_map<image_pair_t, ImagePairTempInfo> rel_temp_info_;

  // The fixed camera id. This is used to remove the ambiguity of the linear
  image_t fixed_camera_id_ = -1;

  // The fixed camera rotation (if with initialization, it would not be identity
  // matrix)
  Eigen::Vector3d fixed_camera_rotation_;
};

}  // namespace glomap
