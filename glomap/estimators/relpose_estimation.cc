#include "glomap/estimators/relpose_estimation.h"

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

  // Define outside loop to reuse memory and avoid reallocation.
  std::vector<Eigen::Vector2d> points2D_1, points2D_2;
  std::vector<char> inliers;

  const int64_t num_image_pairs = valid_pair_ids.size();
  const int64_t kNumChunks = 10;
  const int64_t interval =
      std::ceil(static_cast<double>(num_image_pairs) / kNumChunks);
  LOG(INFO) << "Estimating relative pose for " << num_image_pairs << " pairs";
  for (int64_t chunk_id = 0; chunk_id < kNumChunks; chunk_id++) {
    std::cout << "\r Estimating relative pose: " << chunk_id * kNumChunks << "%"
              << std::flush;
    const int64_t start = chunk_id * interval;
    const int64_t end =
        std::min<int64_t>((chunk_id + 1) * interval, num_image_pairs);

#pragma omp parallel for schedule(dynamic) private( \
    points2D_1, points2D_2, inliers)
    for (int64_t pair_idx = start; pair_idx < end; pair_idx++) {
      ImagePair& image_pair = view_graph.image_pairs[valid_pair_ids[pair_idx]];
      const Image& image1 = images[image_pair.image_id1];
      const Image& image2 = images[image_pair.image_id2];
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
      try {
        poselib::estimate_relative_pose(
            points2D_1,
            points2D_2,
            ColmapCameraToPoseLibCamera(cameras[image1.camera_id]),
            ColmapCameraToPoseLibCamera(cameras[image2.camera_id]),
            options.ransac_options,
            options.bundle_options,
            &pose_rel_calc,
            &inliers);
      } catch (const std::exception& e) {
        LOG(ERROR) << "Error in relative pose estimation: " << e.what();
        image_pair.is_valid = false;
        continue;
      }

      // Convert the relative pose to the glomap format
      for (int i = 0; i < 4; i++) {
        image_pair.cam2_from_cam1.rotation.coeffs()[i] =
            pose_rel_calc.q[(i + 1) % 4];
      }
      image_pair.cam2_from_cam1.translation = pose_rel_calc.t;
    }
  }

  std::cout << "\r Estimating relative pose: 100%" << std::endl;
  LOG(INFO) << "Estimating relative pose done";
}

}  // namespace glomap
