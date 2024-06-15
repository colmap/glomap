#ifndef GLOMAP_ESTIMATORS_RELPOSE_ESTIMATION_H_
#define GLOMAP_ESTIMATORS_RELPOSE_ESTIMATION_H_

#include "glomap/scene/types_sfm.h"

#include <PoseLib/robust.h>

namespace glomap {

struct RelativePoseEstimationOptions {
  // Options for poselib solver
  poselib::RansacOptions ransac_options;
  poselib::BundleOptions bundle_options;

  RelativePoseEstimationOptions() { ransac_options.max_iterations = 50000; }
};

void EstimateRelativePoses(ViewGraph& view_graph,
                           std::unordered_map<camera_t, Camera>& cameras,
                           std::unordered_map<image_t, Image>& images,
                           const RelativePoseEstimationOptions& options);

}  // namespace glomap

#endif  // GLOMAP_ESTIMATORS_RELPOSE_ESTIMATION_H_