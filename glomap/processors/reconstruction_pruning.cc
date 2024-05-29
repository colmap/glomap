#include "glomap/processors/reconstruction_pruning.h"
#include "glomap/processors/view_graph_manipulation.h"


namespace glomap {
image_t PruneWeaklyConnectedImages(
                        std::unordered_map<image_t, Image>& images,
                        std::unordered_map<track_t, Track>& tracks,
                        int min_num_observations) {
    // Prepare the 2d-3d correspondences
    std::unordered_map<image_pair_t, int> pair_covisibility_count;
    std::unordered_map<image_t, int> image_observation_count;
    for (auto &[track_id, track] : tracks) {
        if (track.observations.size() <= 2)
            continue;
        
        for (size_t i = 0; i < track.observations.size(); i++) {
            image_observation_count[track.observations[i].first]++;
            for (size_t j = i + 1; j < track.observations.size(); j++) {
                image_t image_id1 = track.observations[i].first;
                image_t image_id2 = track.observations[j].first;
                if (image_id1 == image_id2)
                    continue;
                image_pair_t pair_id = ImagePair::ImagePairToPairId(image_id1, image_id2);
                if (pair_covisibility_count.find(pair_id) == pair_covisibility_count.end()) {
                    pair_covisibility_count[pair_id] = 1;
                } else {
                    pair_covisibility_count[pair_id]++;
                }
            }
        }
    }

    // Establish the visibility graph
    size_t counter = 0;
    ViewGraph visibility_graph;
    std::vector<int> pair_count;
    for (auto &[pair_id, count] : pair_covisibility_count) {
        // since the relative pose is only fixed if there are more than 5 points,
        // then require each pair to have at least 5 points
        if (count >= 5) { 
            counter++;
            image_t image_id1, image_id2;
            ImagePair::PairIdToImagePair(pair_id, image_id1, image_id2);

            if (image_observation_count[image_id1] < min_num_observations ||
                image_observation_count[image_id2] < min_num_observations)
                continue;

            visibility_graph.image_pairs.insert(std::make_pair(pair_id, ImagePair(image_id1, image_id2)));

            pair_count.push_back(count);
            visibility_graph.image_pairs[pair_id].is_valid = true;
            visibility_graph.image_pairs[pair_id].weight = count;
        }
    }
    std::cout << "Established visibility graph with " << counter << " pairs" << std::endl;

    // sort the pair count
    std::sort(pair_count.begin(), pair_count.end());
    // The median
    std::cout << "Median: " << pair_count[pair_count.size() / 2] << std::endl;

    ViewGraphManipulater::EstablishStrongClusters(visibility_graph, images, ViewGraphManipulater::WEIGHT, pair_count[pair_count.size() / 2] * 0.75);

    return visibility_graph.KeepLargestConnectedComponents(images);
}

}; // glomap