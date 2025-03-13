#include "glomap/controllers/rotation_averager.h"

namespace glomap {

bool SolveRotationAveraging(ViewGraph& view_graph,
                            std::unordered_map<image_t, Image>& images,
                            const RotationAveragerOptions& options) {
  view_graph.KeepLargestConnectedComponents(images);

  bool solve_1dof_system = options.use_gravity && options.use_stratified;

  ViewGraph view_graph_grav;
  image_pair_t total_pairs = 0;
  image_pair_t grav_pairs = 0;
  if (solve_1dof_system) {
    // Prepare two sets: ones all with gravity, and one does not have gravity.
    // Solve them separately first, then solve them in a single system
    for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
      if (!image_pair.is_valid) continue;

      image_t image_id1 = image_pair.image_id1;
      image_t image_id2 = image_pair.image_id2;

      Image& image1 = images[image_id1];
      Image& image2 = images[image_id2];

      if (!image1.is_registered || !image2.is_registered) continue;

      total_pairs++;

      if (image1.gravity_info.has_gravity && image2.gravity_info.has_gravity) {
        view_graph_grav.image_pairs.emplace(
            pair_id,
            ImagePair(image_id1, image_id2, image_pair.cam2_from_cam1));
        grav_pairs++;
      }
    }
  }

  // If there is no image pairs with gravity or most image pairs are with
  // gravity, then just run the 3dof version
  bool status = (grav_pairs == 0) || (grav_pairs > total_pairs * 0.95);
  solve_1dof_system = solve_1dof_system && (!status);

  if (solve_1dof_system) {
    // Run the 1dof optimization
    LOG(INFO) << "Solving subset 1DoF rotation averaging problem in the mixed "
                 "prior system";
    int num_img_grv = view_graph_grav.KeepLargestConnectedComponents(images);
    RotationEstimator rotation_estimator_grav(options);
    if (!rotation_estimator_grav.EstimateRotations(view_graph_grav, images)) {
      return false;
    }
    view_graph.KeepLargestConnectedComponents(images);
  }

  RotationEstimator rotation_estimator(options);
  return rotation_estimator.EstimateRotations(view_graph, images);
}

}  // namespace glomap