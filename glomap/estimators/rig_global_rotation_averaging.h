#pragma once
#include <string>
#include <vector>

#include "global_rotation_averaging.h"

// Code is adapted from Theia's RobustRotationEstimator
// (http://www.theia-sfm.org/). For gravity aligned rotation averaging, refere
// to the paper "Gravity Aligned Rotation Averaging"
namespace glomap {

struct RigRotationEstimatorOptions : public RotationEstimatorOptions {
  RigRotationEstimatorOptions() : RotationEstimatorOptions() {}
};

// TODO: Implement the stratified camera rotation estimation
// TODO: Implement the HALF_NORM loss for IRLS
// TODO: Implement the weighted version for rotation averaging
// TODO: Implement the gravity as prior for rotation averaging
class RigRotationEstimator : public RotationEstimator {
 public:
  explicit RigRotationEstimator(const RigRotationEstimatorOptions& options)
      : RotationEstimator(options), options_(options) {}

  // Estimates the global orientations of all views based on an initial
  // guess. Returns true on successful estimation and false otherwise.
  bool EstimateRotations(const ViewGraph& view_graph,
                         const std::vector<CameraRig>& camera_rigs,
                         std::unordered_map<image_t, Image>& images);

 protected:
  // Sets up the sparse linear system such that dR_ij = dR_j - dR_i. This is the
  // first-order approximation of the angle-axis rotations. This should only be
  // called once.
  void SetupLinearSystem(const ViewGraph& view_graph,
                         const std::vector<CameraRig>& camera_rigs,
                         std::unordered_map<image_t, Image>& images);

  void ConvertResults(const std::vector<CameraRig>& camera_rigs,
                      std::unordered_map<image_t, Image>& images);

  // Data
  // Options for the solver.
  const RigRotationEstimatorOptions& options_;

  std::unordered_map<image_t, int> image_id_to_camera_rig_index_;
  std::unordered_map<image_t, Rigid3d*> image_id_to_rig_from_world_;

  std::vector<std::vector<bool>> rig_is_registered_;
};

}  // namespace glomap
