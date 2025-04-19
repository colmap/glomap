#pragma once

#include "glomap/scene/types_sfm.h"

#include <PoseLib/types.h>

namespace glomap {

struct RelativePoseEstimationOptions {
  // Options for poselib solver
  poselib::RelativePoseOptions relpose_options;

  RelativePoseEstimationOptions() { 
    relpose_options.ransac.max_iterations = 50000; 
  }
};

void EstimateRelativePoses(ViewGraph& view_graph,
                           std::unordered_map<camera_t, Camera>& cameras,
                           std::unordered_map<image_t, Image>& images,
                           const RelativePoseEstimationOptions& options);

}  // namespace glomap
