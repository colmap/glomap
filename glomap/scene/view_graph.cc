#include "view_graph.h"

#include "glomap/math/union_find.h"

#include <queue>

namespace glomap {

int ViewGraph::KeepLargestConnectedComponents(std::unordered_map<image_t, Image>& images) {
    EstablishAdjacencyList();

    int num_comp = FindConnectedComponent();

    int max_idx = -1;
    int max_img = 0;
    for (int comp = 0; comp < num_comp; comp++) {
        if (connected_components[comp].size() > max_img) {
            max_img = connected_components[comp].size();
            max_idx = comp;
        }
    }
    
    std::unordered_set<image_t> largest_component = connected_components[max_idx];

    // Set all images to not registered
    for (auto &[image_id, image] : images)
        image.is_registered = false;

    // Set the images in the largest component to registered
    for (auto image_id : largest_component)
        images[image_id].is_registered = true;
    
    // set all pairs not in the largest component to invalid
    num_pairs = 0;
    for (auto &[pair_id, image_pair] : image_pairs) {
        if (!images[image_pair.image_id1].is_registered || !images[image_pair.image_id2].is_registered) {
            image_pair.is_valid = false;
        }
        if (image_pair.is_valid)
            num_pairs++;
    }

    num_images = largest_component.size();
    return max_img;
}

int ViewGraph::FindConnectedComponent() {
    connected_components.clear();
    std::unordered_map<image_t, bool> visited;
    for (auto &[image_id, neighbors] : adjacency_list) {
        visited[image_id] = false;
    }

    for (auto &[image_id, neighbors] : adjacency_list) {
        if (!visited[image_id]) {
            std::unordered_set<image_t> component;
            BFS(image_id, visited, component);
            connected_components.push_back(component);
        }
    }

    return connected_components.size();
}


void ViewGraph::BFS(image_t root, std::unordered_map<image_t, bool>& visited, std::unordered_set<image_t>& component) {
    std::queue<image_t> q;
    q.push(root);
    visited[root] = true;
    component.insert(root);

    while (!q.empty()) {
        image_t curr = q.front();
        q.pop();

        for (image_t neighbor : adjacency_list[curr]) {
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
    for (auto &[pair_id, image_pair] : image_pairs) {
        if (image_pair.is_valid) {
            adjacency_list[image_pair.image_id1].insert(image_pair.image_id2);
            adjacency_list[image_pair.image_id2].insert(image_pair.image_id1);
        }
    }
}
}; // glomap
