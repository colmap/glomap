#pragma once

#include "glomap/estimators/bundle_adjustment.h"
#include "glomap/estimators/optimization_base.h"
#include "glomap/scene/types_sfm.h"
#include "glomap/types.h"

#include <ceres/ceres.h>

namespace glomap {

struct RigBundleAdjusterOptions : public BundleAdjusterOptions {
 public:
  RigBundleAdjusterOptions() : BundleAdjusterOptions() {};
};

class RigBundleAdjuster {
 public:
  RigBundleAdjuster(const RigBundleAdjusterOptions& options)
      : options_(options) {}

  // Returns true if the optimization was a success, false if there was a
  // failure.
  // Assume tracks here are already filtered
  bool Solve(const ViewGraph& view_graph,
             const std::vector<CameraRig>& camera_rigs,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<image_t, Image>& images,
             std::unordered_map<track_t, Track>& tracks);

  RigBundleAdjusterOptions& GetOptions() { return options_; }

 private:
  // Reset the problem
  void Reset(const std::vector<CameraRig>& camera_rigs,
             std::unordered_map<image_t, Image>& images);

  void ExtractRigsFromWorld(const std::vector<CameraRig>& camera_rigs,
                            const std::unordered_map<image_t, Image>& images);

  // Add tracks to the problem
  void AddPointToCameraConstraints(
      const ViewGraph& view_graph,
      const std::vector<CameraRig>& camera_rigs,
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

  // During the optimization, the camera translation is set to be the camera
  // center Convert the results back to camera poses
  void ConvertResults(const std::vector<CameraRig>& camera_rigs,
                      std::unordered_map<image_t, Image>& images);

  // Mapping from images to camera rigs.
  std::unordered_map<image_t, int> image_id_to_camera_rig_index_;
  std::unordered_map<image_t, Rigid3d*> image_id_to_rig_from_world_;

  // For each camera rig, the absolute camera rig poses for all snapshots.
  std::vector<std::vector<Rigid3d>> rigs_from_world_;

  RigBundleAdjusterOptions options_;

  std::unique_ptr<ceres::Problem> problem_;
  std::shared_ptr<ceres::LossFunction> loss_function_;
};

}  // namespace glomap
