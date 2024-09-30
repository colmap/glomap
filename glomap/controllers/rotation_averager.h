#pragma once

#include "glomap/estimators/global_rotation_averaging.h"

namespace glomap {

struct RotationAveragerOptions : public RotationEstimatorOptions {
  bool use_stratified = true;
};

bool SolveRotationAveraging(ViewGraph& view_graph,
                            std::unordered_map<image_t, Image>& images,
                            const RotationAveragerOptions& options);

}  // namespace glomap