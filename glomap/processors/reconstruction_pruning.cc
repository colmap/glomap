#include "glomap/processors/reconstruction_pruning.h"

#include "glomap/processors/view_graph_manipulation.h"

namespace glomap {
image_t PruneWeaklyConnectedImages(std::unordered_map<frame_t, Frame>& frames,
                                   std::unordered_map<image_t, Image>& images,
                                   std::unordered_map<track_t, Track>& tracks,
                                   int min_num_images,
                                   int min_num_observations) {
  // Prepare the 2d-3d correspondences
  std::unordered_map<image_pair_t, int> pair_covisibility_count;
  std::unordered_map<frame_t, int> frame_observation_count;
  for (auto& [track_id, track] : tracks) {
    if (track.observations.size() <= 2) continue;

    for (size_t i = 0; i < track.observations.size(); i++) {
      image_t image_id1 = track.observations[i].first;
      frame_t frame_id1 = images[image_id1].frame_id;

      frame_observation_count[frame_id1]++;
      for (size_t j = i + 1; j < track.observations.size(); j++) {
        image_t image_id2 = track.observations[j].first;
        frame_t frame_id2 = images[image_id2].frame_id;
        if (frame_id1 == frame_id2) continue;
        image_pair_t pair_id =
            ImagePair::ImagePairToPairId(frame_id1, frame_id2);
        if (pair_covisibility_count.find(pair_id) ==
            pair_covisibility_count.end()) {
          pair_covisibility_count[pair_id] = 1;
        } else {
          pair_covisibility_count[pair_id]++;
        }
      }
    }
  }

  // Establish the visibility graph
  size_t counter = 0;
  ViewGraph visibility_graph_frame;
  std::vector<int> pair_count;
  for (auto& [pair_id, count] : pair_covisibility_count) {
    // since the relative pose is only fixed if there are more than 5 points,
    // then require each pair to have at least 5 points
    if (count >= 5) {
      counter++;
      image_t image_id1, image_id2;
      ImagePair::PairIdToImagePair(pair_id, image_id1, image_id2);

      if (frame_observation_count[image_id1] < min_num_observations ||
          frame_observation_count[image_id2] < min_num_observations)
        continue;

      visibility_graph_frame.image_pairs.insert(
          std::make_pair(pair_id, ImagePair(image_id1, image_id2)));

      pair_count.push_back(count);
      visibility_graph_frame.image_pairs[pair_id].is_valid = true;
      visibility_graph_frame.image_pairs[pair_id].weight = count;
    }
  }
  LOG(INFO) << "Established visibility graph with " << counter << " pairs";

  // Create the visibility graph
  // Connect the reference image of each frame with other reference image
  std::unordered_map<frame_t, image_t> frame_id_to_begin_img;
  for (auto& [frame_id, frame] : frames) {
    int counter = 0;
    for (const auto& data_id : frame.ImageIds()) {
      image_t image_id = data_id.id;
      if (images.find(image_id) == images.end()) continue;
      frame_id_to_begin_img[frame_id] = image_id;
      break;
    }
  }

  ViewGraph visibility_graph;
  for (auto& [pair_id, image_pair] : visibility_graph_frame.image_pairs) {
    frame_t frame_id1, frame_id2;
    ImagePair::PairIdToImagePair(pair_id, frame_id1, frame_id2);
    image_t image_id1 = frame_id_to_begin_img[frame_id1];
    image_t image_id2 = frame_id_to_begin_img[frame_id2];
    visibility_graph.image_pairs.insert(
        std::make_pair(pair_id, ImagePair(image_id1, image_id2)));
    visibility_graph.image_pairs[pair_id].weight = image_pair.weight;
  }

  int max_weight = std::max_element(pair_count.begin(), pair_count.end()) -
                   pair_count.begin();

  // within each frame, connect the reference image with all other images
  for (auto& [frame_id, frame] : frames) {
    image_t begin_image_id = frame_id_to_begin_img[frame_id];
    for (const auto& data_id : frame.ImageIds()) {
      image_t image_id = data_id.id;
      if (image_id == begin_image_id || images.find(image_id) == images.end())
        continue;
      image_pair_t pair_id =
          ImagePair::ImagePairToPairId(begin_image_id, image_id);
      visibility_graph.image_pairs.insert(
          std::make_pair(pair_id, ImagePair(begin_image_id, image_id)));

      // Never break th inner edge
      visibility_graph.image_pairs[pair_id].weight = max_weight;
    }
  }

  // sort the pair count
  std::sort(pair_count.begin(), pair_count.end());
  double median_count = pair_count[pair_count.size() / 2];

  // calculate the MAD (median absolute deviation)
  std::vector<int> pair_count_diff(pair_count.size());
  for (size_t i = 0; i < pair_count.size(); i++) {
    pair_count_diff[i] = std::abs(pair_count[i] - median_count);
  }
  std::sort(pair_count_diff.begin(), pair_count_diff.end());
  double median_count_diff = pair_count_diff[pair_count_diff.size() / 2];

  // The median
  LOG(INFO) << "Threshold for Strong Clustering: "
            << median_count - median_count_diff;

  return ViewGraphManipulater::EstablishStrongClusters(
      visibility_graph,
      frames,
      images,
      ViewGraphManipulater::WEIGHT,
      std::max(median_count - median_count_diff, 20.),
      min_num_images);

  // return visibility_graph.MarkConnectedComponents(images, min_num_images);
}

}  // namespace glomap