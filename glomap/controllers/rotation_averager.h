#pragma once

#include "glomap/estimators/global_rotation_averaging.h"

namespace glomap {

struct RotationAveragerOptions : public RotationEstimatorOptions {
  bool use_stratified = true;
};

class RotationAverager {
 public:
  RotationAverager(const RotationAveragerOptions& options) : options_(options) {};

  bool Solve(ViewGraph& view_graph,
             std::unordered_map<image_t, Image>& images);

 private:
  const RotationAveragerOptions options_;
};

} // namespace glomap