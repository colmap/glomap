#pragma once

#include "glomap/scene/types_sfm.h"

#include <PoseLib/types.h>

namespace glomap {

struct RelativePoseEstimationOptions {
  // Options for poselib solver
  poselib::RansacOptions ransac_options;
  poselib::BundleOptions bundle_options;

  // When images size are large, the threshold of epipolar geometry might also
  // needs to be adjusted accordingly
  bool adaptive_threshold = false;

  RelativePoseEstimationOptions() { ransac_options.max_iterations = 50000; }
};

void EstimateRelativePoses(ViewGraph& view_graph,
                           std::unordered_map<camera_t, Camera>& cameras,
                           std::unordered_map<image_t, Image>& images,
                           const RelativePoseEstimationOptions& options);

}  // namespace glomap
