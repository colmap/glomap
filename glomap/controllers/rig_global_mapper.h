#pragma once
#include "glomap/controllers/global_mapper.h"
#include "glomap/estimators/rig_bundle_adjustment.h"
#include "glomap/estimators/rig_global_positioning.h"
#include "glomap/estimators/rig_global_rotation_averaging.h"

namespace glomap {

struct RigGlobalMapperOptions : public GlobalMapperOptions {
  // Options for each component

  RigRotationEstimatorOptions opt_ra;
  RigGlobalPositionerOptions opt_gp;
  RigBundleAdjusterOptions opt_ba;
};

// TODO: Refactor the code to reuse the pipeline code more
class RigGlobalMapper {
 public:
  RigGlobalMapper(const RigGlobalMapperOptions& options) : options_(options) {}

  bool Solve(const colmap::Database& database,
             ViewGraph& view_graph,
             std::vector<CameraRig>& camera_rigs,
             std::unordered_map<camera_t, Camera>& cameras,
             std::unordered_map<image_t, Image>& images,
             std::unordered_map<track_t, Track>& tracks);

 private:
  const RigGlobalMapperOptions options_;
};

}  // namespace glomap
