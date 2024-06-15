
#ifndef GLOMAP_PROCESSORS_RELPOSE_FILTER_H_
#define GLOMAP_PROCESSORS_RELPOSE_FILTER_H_

#include "glomap/scene/types_sfm.h"

namespace glomap {

struct RelPoseFilter {
  // Filter relative pose based on rotation angle
  // max_angle: in degree
  static void FilterRotations(ViewGraph& view_graph,
                              const std::unordered_map<image_t, Image>& images,
                              double max_angle = 5.0);

  // Filter relative pose based on number of inliers
  // min_inlier_num: in degree
  static void FilterInlierNum(ViewGraph& view_graph, int min_inlier_num = 30);

  // Filter relative pose based on rate of inliers
  // min_weight: minimal ratio of inliers
  static void FilterInlierRatio(ViewGraph& view_graph,
                                double min_inlier_ratio = 0.25);
};

}  // namespace glomap

#endif  // GLOMAP_PROCESSORS_RELPOSE_FILTER_H_