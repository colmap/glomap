#include "gravity_refinement.h"

#include "glomap/estimators/cost_function.h"
#include "glomap/math/gravity.h"

#include <colmap/estimators/manifold.h>

namespace glomap {
void GravityRefiner::RefineGravity(const ViewGraph& view_graph,
                                   std::unordered_map<image_t, Image>& images) {
  const std::unordered_map<image_pair_t, ImagePair>& image_pairs =
      view_graph.image_pairs;
  const std::unordered_map<image_t, std::unordered_set<image_t>>&
      adjacency_list = view_graph.GetAdjacencyList();
  if (adjacency_list.empty()) {
    LOG(INFO) << "Adjacency list not established" << std::endl;
    return;
  }

  // Identify the images that are error prone
  int counter_rect = 0;
  std::unordered_set<image_t> error_prone_images;
  IdentifyErrorProneGravity(view_graph, images, error_prone_images);

  if (error_prone_images.empty()) {
    LOG(INFO) << "No error prone images found" << std::endl;
    return;
  }

  loss_function_ = options_.CreateLossFunction();

  int counter_progress = 0;
  // Iterate through the error prone images
  for (auto image_id : error_prone_images) {
    if ((counter_progress + 1) % 10 == 0 ||
        counter_progress == error_prone_images.size() - 1) {
      std::cout << "\r Refining image " << counter_progress + 1 << " / "
                << error_prone_images.size() << std::flush;
    }
    counter_progress++;
    const std::unordered_set<image_t>& neighbors = adjacency_list.at(image_id);
    std::vector<Eigen::Vector3d> gravities;
    gravities.reserve(neighbors.size());

    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problem_options);
    int counter = 0;
    Eigen::Vector3d gravity = images[image_id].gravity_info.GetGravity();
    for (const auto& neighbor : neighbors) {
      image_pair_t pair_id = ImagePair::ImagePairToPairId(image_id, neighbor);

      image_t image_id1 = image_pairs.at(pair_id).image_id1;
      image_t image_id2 = image_pairs.at(pair_id).image_id2;
      if (images.at(image_id1).gravity_info.has_gravity == false ||
          images.at(image_id2).gravity_info.has_gravity == false)
        continue;

      if (image_id1 == image_id) {
        gravities.emplace_back((image_pairs.at(pair_id)
                                    .cam2_from_cam1.rotation.toRotationMatrix()
                                    .transpose() *
                                images[image_id2].gravity_info.GetRAlign())
                                   .col(1));
      } else {
        gravities.emplace_back(
            (image_pairs.at(pair_id)
                 .cam2_from_cam1.rotation.toRotationMatrix() *
             images[image_id1].gravity_info.GetRAlign())
                .col(1));
      }

      ceres::CostFunction* coor_cost =
          GravError::CreateCost(gravities[counter]);
      problem.AddResidualBlock(coor_cost, loss_function_.get(), gravity.data());
      counter++;
    }

    if (gravities.size() < options_.min_num_neighbors) continue;

    // Then, run refinment
    gravity = AverageGravity(gravities);
    colmap::SetSphereManifold<3>(&problem, gravity.data());
    ceres::Solver::Summary summary_solver;
    ceres::Solve(options_.solver_options, &problem, &summary_solver);

    // Check the error with respect to the neighbors
    int counter_outlier = 0;
    for (int i = 0; i < gravities.size(); i++) {
      double error = RadToDeg(
          std::acos(std::max(std::min(gravities[i].dot(gravity), 1.), -1.)));
      if (error > options_.max_gravity_error * 2) counter_outlier++;
    }
    // If the refined gravity now consistent with more images, then accept it
    if (double(counter_outlier) / double(gravities.size()) <
        options_.max_outlier_ratio) {
      counter_rect++;
      images[image_id].gravity_info.SetGravity(gravity);
    }
  }
  std::cout << std::endl;
  LOG(INFO) << "Number of rectified images: " << counter_rect << " / "
            << error_prone_images.size() << std::endl;
}

void GravityRefiner::IdentifyErrorProneGravity(
    const ViewGraph& view_graph,
    const std::unordered_map<image_t, Image>& images,
    std::unordered_set<image_t>& error_prone_images) {
  error_prone_images.clear();

  // image_id: (mistake, total)
  std::unordered_map<image_t, std::pair<int, int>> image_counter;
  // Set the counter of all images to 0
  for (const auto& [image_id, image] : images) {
    image_counter[image_id] = std::make_pair(0, 0);
  }

  for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (!image_pair.is_valid) continue;
    const auto& image1 = images.at(image_pair.image_id1);
    const auto& image2 = images.at(image_pair.image_id2);
    if (image1.gravity_info.has_gravity && image2.gravity_info.has_gravity) {
      // Calculate the gravity aligned relative rotation
      const Eigen::Matrix3d R_rel =
          image2.gravity_info.GetRAlign().transpose() *
          image_pair.cam2_from_cam1.rotation.toRotationMatrix() *
          image1.gravity_info.GetRAlign();
      // Convert it to the closest upright rotation
      const Eigen::Matrix3d R_rel_up = AngleToRotUp(RotUpToAngle(R_rel));

      const double angle = CalcAngle(R_rel, R_rel_up);

      // increment the total count
      image_counter[image_pair.image_id1].second++;
      image_counter[image_pair.image_id2].second++;

      // increment the mistake count
      if (angle > options_.max_gravity_error) {
        image_counter[image_pair.image_id1].first++;
        image_counter[image_pair.image_id2].first++;
      }
    }
  }

  const std::unordered_map<image_t, std::unordered_set<image_t>>&
      adjacency_list = view_graph.GetAdjacencyList();

  // Filter the images with too many mistakes
  for (auto& [image_id, counter] : image_counter) {
    if (images.at(image_id).gravity_info.has_gravity == false) continue;
    if (counter.second < options_.min_num_neighbors) continue;
    if (double(counter.first) / double(counter.second) >=
        options_.max_outlier_ratio) {
      error_prone_images.insert(image_id);
    }
  }
  LOG(INFO) << "Number of error prone images: " << error_prone_images.size()
            << std::endl;
}
}  // namespace glomap
