#pragma once

#include "glomap/estimators/rig_global_rotation_averaging.h"

namespace glomap {

struct RotationAveragerOptions : public RigRotationEstimatorOptions {
  RotationAveragerOptions() = default;
  RotationAveragerOptions(const RigRotationEstimatorOptions& options)
      : RigRotationEstimatorOptions(options) {}
  bool use_stratified = true;
};

bool SolveRotationAveraging(ViewGraph& view_graph,
                            std::unordered_map<rig_t, Rig>& rigs,
                            std::unordered_map<frame_t, Frame>& frames,
                            std::unordered_map<image_t, Image>& images,
                            const RotationAveragerOptions& options);

}  // namespace glomap