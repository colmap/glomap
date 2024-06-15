#include "glomap/estimators/relpose_estimation.h"

namespace glomap {
void EstimateRelativePoses(ViewGraph& view_graph,
                           std::unordered_map<camera_t, Camera>& cameras,
                           std::unordered_map<image_t, Image>& images,
                           const RelativePoseEstimationOptions& options) {
  std::vector<image_pair_t> image_pair_ids;
  for (auto& [image_pair_id, image_pair] : view_graph.image_pairs) {
    if (!image_pair.is_valid) continue;
    image_pair_ids.push_back(image_pair_id);
  }

  image_pair_t inverval = std::ceil(image_pair_ids.size() / 10.);
  std::cout << "Estimating relative pose for " << image_pair_ids.size()
            << " pairs" << std::endl;
  for (image_pair_t chunks = 0; chunks < 10; chunks++) {
    std::cout << "\r Estimating relative pose: " << chunks * 10 << "%"
              << std::flush;
    image_pair_t start = chunks * inverval;
    image_pair_t end = std::min((chunks + 1) * inverval, image_pair_ids.size());
#pragma omp parallel for
    for (image_pair_t pair = start; pair < end; pair++) {
      ImagePair& image_pair = view_graph.image_pairs[image_pair_ids[pair]];
      image_t idx1 = image_pair.image_id1;
      image_t idx2 = image_pair.image_id2;

      camera_t camera_id1 = images[idx1].camera_id;
      camera_t camera_id2 = images[idx2].camera_id;

      const Eigen::MatrixXi& matches = image_pair.matches;

      // Collect the original 2D points
      std::vector<Eigen::Vector2d> points2D_1, points2D_2;
      points2D_1.reserve(matches.rows());
      points2D_2.reserve(matches.rows());
      for (size_t idx = 0; idx < matches.rows(); idx++) {
        feature_t feature_id1 = matches(idx, 0);
        feature_t feature_id2 = matches(idx, 1);

        points2D_1.push_back(images[idx1].features[feature_id1]);
        points2D_2.push_back(images[idx2].features[feature_id2]);
      }

      // Estimate the relative pose with poselib
      std::vector<char> inliers;
      poselib::CameraPose pose_rel_calc;
      poselib::estimate_relative_pose(
          points2D_1,
          points2D_2,
          ColmapCameraToPoseLibCamera(cameras[images[idx1].camera_id]),
          ColmapCameraToPoseLibCamera(cameras[images[idx2].camera_id]),
          options.ransac_options,
          options.bundle_options,
          &pose_rel_calc,
          &inliers);

      // Convert the relative pose to the glomap format
      for (int i = 0; i < 4; i++)
        image_pair.cam2_from_cam1.rotation.coeffs()[i] =
            pose_rel_calc.q[(i + 1) % 4];
      image_pair.cam2_from_cam1.translation = pose_rel_calc.t;
    }
  }
  std::cout << "\r Estimating relative pose: 100%" << std::endl;
  std::cout << "Estimating relative pose done" << std::endl;
}

}  // namespace glomap