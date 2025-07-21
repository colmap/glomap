#include "glomap/scene/view_graph.h"

#include "glomap/math/union_find.h"

#include <queue>

namespace glomap {

int ViewGraph::KeepLargestConnectedComponents(
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images) {
  EstablishAdjacencyList();
  EstablishAdjacencyListFrame(images);

  int num_comp = FindConnectedComponent();

  int max_idx = -1;
  int max_img = 0;
  for (int comp = 0; comp < num_comp; comp++) {
    if (connected_components[comp].size() > max_img) {
      max_img = connected_components[comp].size();
      max_idx = comp;
    }
  }

  if (max_img == 0) return 0;

  std::unordered_set<image_t> largest_component = connected_components[max_idx];

  // Set all frames to not registered
  for (auto& [frame_id, frame] : frames) {
    frame.is_registered = false;
  }
  // Set the frames in the largest component to registered
  for (auto frame_id : largest_component) {
    frames[frame_id].is_registered = true;
  }
  // set all pairs not in the largest component to invalid
  num_pairs = 0;
  for (auto& [pair_id, image_pair] : image_pairs) {
    if (!images[image_pair.image_id1].IsRegistered() ||
        !images[image_pair.image_id2].IsRegistered()) {
      image_pair.is_valid = false;
    }
    if (image_pair.is_valid) num_pairs++;
  }

  for (auto& [image_id, image] : images) {
    if (image.IsRegistered()) max_img++;
  }
  return max_img;
}

int ViewGraph::FindConnectedComponent() {
  connected_components.clear();
  std::unordered_map<image_t, bool> visited;
  for (auto& [frame_id, neighbors] : adjacency_list_frame) {
    visited[frame_id] = false;
  }

  for (auto& [frame_id, neighbors] : adjacency_list_frame) {
    if (!visited[frame_id]) {
      std::unordered_set<image_t> component;
      BFS(frame_id, visited, component);
      connected_components.push_back(component);
    }
  }

  return connected_components.size();
}

int ViewGraph::MarkConnectedComponents(
    std::unordered_map<frame_t, Frame>& frames,
    std::unordered_map<image_t, Image>& images,
    int min_num_img) {
  EstablishAdjacencyList();
  EstablishAdjacencyListFrame(images);

  int num_comp = FindConnectedComponent();

  std::vector<std::pair<int, int>> cluster_num_img(num_comp);
  for (int comp = 0; comp < num_comp; comp++) {
    cluster_num_img[comp] =
        std::make_pair(connected_components[comp].size(), comp);
  }
  std::sort(cluster_num_img.begin(), cluster_num_img.end(), std::greater<>());

  // Set the cluster number of every frame to be -1
  for (auto& [frame_id, frame] : frames) frame.cluster_id = -1;

  int comp = 0;
  for (; comp < num_comp; comp++) {
    if (cluster_num_img[comp].first < min_num_img) break;
    for (auto frame_id : connected_components[cluster_num_img[comp].second]) {
      frames[frame_id].cluster_id = comp;
    }
  }

  return comp;
}

void ViewGraph::BFS(image_t root,
                    std::unordered_map<image_t, bool>& visited,
                    std::unordered_set<image_t>& component) {
  std::queue<image_t> q;
  q.push(root);
  visited[root] = true;
  component.insert(root);

  while (!q.empty()) {
    image_t curr = q.front();
    q.pop();

    for (image_t neighbor : adjacency_list_frame[curr]) {
      if (!visited[neighbor]) {
        q.push(neighbor);
        visited[neighbor] = true;
        component.insert(neighbor);
      }
    }
  }
}

void ViewGraph::EstablishAdjacencyList() {
  adjacency_list.clear();
  for (auto& [pair_id, image_pair] : image_pairs) {
    if (image_pair.is_valid) {
      adjacency_list[image_pair.image_id1].insert(image_pair.image_id2);
      adjacency_list[image_pair.image_id2].insert(image_pair.image_id1);
    }
  }
}

void ViewGraph::EstablishAdjacencyListFrame(
    std::unordered_map<image_t, Image>& images) {
  adjacency_list_frame.clear();
  for (auto& [pair_id, image_pair] : image_pairs) {
    if (image_pair.is_valid) {
      frame_t frame_id1 = images[image_pair.image_id1].frame_id;
      frame_t frame_id2 = images[image_pair.image_id2].frame_id;
      adjacency_list_frame[frame_id1].insert(frame_id2);
      adjacency_list_frame[frame_id2].insert(frame_id1);
    }
  }
}
}  // namespace glomap
