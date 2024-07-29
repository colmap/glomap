#pragma once

#include "glomap/scene/camera.h"
#include "glomap/scene/image.h"
#include "glomap/scene/image_pair.h"
#include "glomap/scene/types.h"
#include "glomap/types.h"

namespace glomap {

class ViewGraph {
 public:
  // Methods
  inline void RemoveInvalidPair(image_pair_t pair_id);

  // Mark the image which is not connected to any other images as not registered
  // Return: the number of images in the largest connected component
  int KeepLargestConnectedComponents(
      std::unordered_map<image_t, Image>& images);

  // Mark the cluster of the cameras (cluster_id sort by the the number of
  // images)
  int MarkConnectedComponents(std::unordered_map<image_t, Image>& images,
                              int min_num_img = -1);

  // Establish the adjacency list
  void EstablishAdjacencyList();

  inline const std::unordered_map<image_t, std::unordered_set<image_t>>&
  GetAdjacencyList() const;

  // Data
  std::unordered_map<image_pair_t, ImagePair> image_pairs;

  image_t num_images = 0;
  image_pair_t num_pairs = 0;

 private:
  int FindConnectedComponent();

  void BFS(image_t root,
           std::unordered_map<image_t, bool>& visited,
           std::unordered_set<image_t>& component);

  // Data for processing
  std::unordered_map<image_t, std::unordered_set<image_t>> adjacency_list;
  std::vector<std::unordered_set<image_t>> connected_components;
};

const std::unordered_map<image_t, std::unordered_set<image_t>>&
ViewGraph::GetAdjacencyList() const {
  return adjacency_list;
}

void ViewGraph::RemoveInvalidPair(image_pair_t pair_id) {
  ImagePair& pair = image_pairs.at(pair_id);
  pair.is_valid = false;
}

}  // namespace glomap
