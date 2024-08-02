#include "glomap/estimators/relpose_estimation.h"

#include <iomanip>

#include <PoseLib/robust.h>

namespace glomap {

void EstimateRelativePoses(ViewGraph& view_graph,
                           std::unordered_map<camera_t, Camera>& cameras,
                           std::unordered_map<image_t, Image>& images,
                           const RelativePoseEstimationOptions& options) {
  std::vector<image_pair_t> valid_pair_ids;
  for (auto& [image_pair_id, image_pair] : view_graph.image_pairs) {
    if (!image_pair.is_valid) continue;
    valid_pair_ids.push_back(image_pair_id);
  }

  const size_t num_valid_pairs = valid_pair_ids.size();
  LOG(INFO) << "Estimating relative pose for " << num_valid_pairs << " pairs";

#pragma omp parallel
  {
    // Define variables outside parallel for loop to reuse allocations.
    std::vector<Eigen::Vector2d> points2D_1, points2D_2;
    std::vector<char> inliers;

    const int thread_id = omp_get_thread_num();
    size_t num_processed_pairs = 0;

#pragma omp parallel for schedule(dynamic)
    for (size_t pair_idx = 0; pair_idx < num_valid_pairs; ++pair_idx) {
      // Since we use dynamic scheduling, we can assume that each thread is
      // progressing approximately at the same pace, so we simply log out the
      // progress from the master thread.
      if (thread_id == 0 && num_processed_pairs++ % 100 == 0) {
        std::cout << "\r Estimating relative pose: " << std::setprecision(2)
                  << 100.0 * pair_idx / num_valid_pairs << "%" << std::flush;
      }

      ImagePair& image_pair = view_graph.image_pairs[valid_pair_ids[pair_idx]];
      const auto& image1 = images[image_pair.image_id1];
      const auto& image2 = images[image_pair.image_id2];

      const Eigen::MatrixXi& matches = image_pair.matches;

      // Collect the original 2D points
      points2D_1.clear();
      points2D_2.clear();
      for (size_t idx = 0; idx < matches.rows(); idx++) {
        points2D_1.push_back(image1.features[matches(idx, 0)]);
        points2D_2.push_back(image2.features[matches(idx, 1)]);
      }

      inliers.clear();
      poselib::CameraPose pose_rel_calc;
      poselib::estimate_relative_pose(
          points2D_1,
          points2D_2,
          ColmapCameraToPoseLibCamera(cameras[image1.camera_id]),
          ColmapCameraToPoseLibCamera(cameras[image2.camera_id]),
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
  LOG(INFO) << "Estimating relative pose done";
}

}  // namespace glomap
