#pragma once

#include "glomap/scene/types.h"
#include "glomap/types.h"

#include <colmap/estimators/two_view_geometry.h>

#include <Eigen/Core>

namespace glomap {

// FUTURE: add covariance to the relative pose
struct ImagePair {
  ImagePair() : pair_id(-1), image_id1(-1), image_id2(-1) {}
  ImagePair(image_t img_id1, image_t img_id2, Rigid3d pose_rel = Rigid3d())
      : pair_id(ImagePairToPairId(img_id1, img_id2)),
        image_id1(img_id1),
        image_id2(img_id2),
        cam2_from_cam1(pose_rel) {}

  // Ids are kept constant
  const image_pair_t pair_id;
  const image_t image_id1;
  const image_t image_id2;

  // indicator whether the image pair is valid
  bool is_valid = true;

  // weight is the initial inlier rate
  double weight = -1;

  // one of `ConfigurationType`.
  int config = colmap::TwoViewGeometry::UNDEFINED;

  // Essential matrix.
  Eigen::Matrix3d E = Eigen::Matrix3d::Zero();
  // Fundamental matrix.
  Eigen::Matrix3d F = Eigen::Matrix3d::Zero();
  // Homography matrix.
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

  // Relative pose.
  Rigid3d cam2_from_cam1;

  // Matches between the two images.
  // First column is the index of the feature in the first image.
  // Second column is the index of the feature in the second image.
  Eigen::MatrixXi matches;

  // Row index of inliers in the matches matrix.
  std::vector<int> inliers;

  static inline image_pair_t ImagePairToPairId(const image_t image_id1,
                                               const image_t image_id2);

  static inline void PairIdToImagePair(const image_pair_t pair_id,
                                       image_t& image_id1,
                                       image_t& image_id2);
};

image_pair_t ImagePair::ImagePairToPairId(const image_t image_id1,
                                          const image_t image_id2) {
  if (image_id1 > image_id2) {
    return static_cast<image_pair_t>(kMaxNumImages) * image_id2 + image_id1;
  } else {
    return static_cast<image_pair_t>(kMaxNumImages) * image_id1 + image_id2;
  }
}

void ImagePair::PairIdToImagePair(const image_pair_t pair_id,
                                  image_t& image_id1,
                                  image_t& image_id2) {
  image_id1 = static_cast<image_t>(pair_id % kMaxNumImages);
  image_id2 = static_cast<image_t>((pair_id - image_id1) / kMaxNumImages);
}

}  // namespace glomap
