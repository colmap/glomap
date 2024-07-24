#include "glomap/processors/relpose_filter.h"

#include "glomap/math/rigid3d.h"

namespace glomap {

void RelPoseFilter::FilterRotations(
    ViewGraph& view_graph,
    const std::unordered_map<image_t, Image>& images,
    double max_angle) {
  int num_invalid = 0;
  for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;

    const Image& image1 = images.at(image_pair.image_id1);
    const Image& image2 = images.at(image_pair.image_id2);

    if (image1.is_registered == false || image2.is_registered == false) {
      continue;
    }

    Rigid3d pose_calc = image2.cam_from_world * Inverse(image1.cam_from_world);

    double angle = CalcAngle(pose_calc, image_pair.cam2_from_cam1);
    if (angle > max_angle) {
      image_pair.is_valid = false;
      num_invalid++;
    }
  }

  LOG(INFO) << "Filtered " << num_invalid << " relative rotation with angle > "
            << max_angle << " degrees";
}

void RelPoseFilter::FilterInlierNum(ViewGraph& view_graph, int min_inlier_num) {
  int num_invalid = 0;
  for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;

    if (image_pair.inliers.size() < min_inlier_num) {
      image_pair.is_valid = false;
      num_invalid++;
    }
  }

  LOG(INFO) << "Filtered " << num_invalid
            << " relative poses with inlier number < " << min_inlier_num;
}

void RelPoseFilter::FilterInlierRatio(ViewGraph& view_graph,
                                      double min_inlier_ratio) {
  int num_invalid = 0;
  for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;

    if (image_pair.inliers.size() / double(image_pair.matches.rows()) <
        min_inlier_ratio) {
      image_pair.is_valid = false;
      num_invalid++;
    }
  }

  LOG(INFO) << "Filtered " << num_invalid
            << " relative poses with inlier ratio < " << min_inlier_ratio;
}

}  // namespace glomap