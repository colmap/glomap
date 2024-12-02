#include "tree.h"
#include <queue>

namespace glomap {

    struct Edge {
        int to;
        double weight;
        Edge(int t, double w) : to(t), weight(w) {}
    };

    struct CompareEdge {
        bool operator()(const Edge& e1, const Edge& e2) {
            return e1.weight < e2.weight;  // For maximum spanning tree, use < instead of >
        }
    };

    // Function to perform breadth-first search (BFS) on a graph represented by an
    // adjacency list
    int BFS(const std::vector<std::vector<int>>& graph,
            int root,
            std::vector<int>& parents,
            std::vector<std::pair<int, int>> banned_edges) {
        int num_vertices = graph.size();

        // Create a vector to store the visited status of each vertex
        std::vector<bool> visited(num_vertices, false);

        // Create a vector to store the parent vertex for each vertex
        parents.clear();
        parents.resize(num_vertices, -1);
        parents[root] = root;

        // Create a queue for BFS traversal
        std::queue<int> q;

        // Mark the start vertex as visited and enqueue it
        visited[root] = true;
        q.push(root);

        int counter = 0;
        while (!q.empty())
        {
            int current_vertex = q.front();
            q.pop();

            // Process the current vertex
            // Traverse the adjacent vertices
            for (int neighbor : graph[current_vertex])
            {
                if (std::find(banned_edges.begin(),
                              banned_edges.end(),
                              std::make_pair(current_vertex, neighbor)) !=
                    banned_edges.end())
                    continue;
                if (std::find(banned_edges.begin(),
                              banned_edges.end(),
                              std::make_pair(neighbor, current_vertex)) !=
                    banned_edges.end())
                    continue;

                if (!visited[neighbor])
                {
                    visited[neighbor] = true;
                    parents[neighbor] = current_vertex;
                    q.push(neighbor);
                    counter++;
                }
            }
        }

        return counter;
    }

    image_t MaximumSpanningTree(const ViewGraph& view_graph,
                                const std::unordered_map<image_t, Image>& images,
                                std::unordered_map<image_t, image_t>& parents,
                                WeightType type) {
        // Create mapping between image IDs and indices
        std::unordered_map<image_t, int> image_id_to_idx;
        std::unordered_map<int, image_t> idx_to_image_id;
        for (const auto& [image_id, image] : images) {
            if (!image.is_registered) continue;
            idx_to_image_id[image_id_to_idx.size()] = image_id;
            image_id_to_idx[image_id] = image_id_to_idx.size();
        }

        const int num_vertices = image_id_to_idx.size();
        if (num_vertices == 0) return 0;

        // Find maximum weight for weight normalization
        double max_weight = 0;
        for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
            if (!image_pair.is_valid) continue;
            max_weight = std::max(max_weight,
                                  type == INLIER_RATIO ? image_pair.weight :
                                                       static_cast<double>(image_pair.inliers.size()));
        }

        // Create adjacency list representation
        std::vector<std::vector<Edge>> adj_list(num_vertices);
        for (const auto& [pair_id, image_pair] : view_graph.image_pairs) {
            if (!image_pair.is_valid) continue;

            const Image& image1 = images.at(image_pair.image_id1);
            const Image& image2 = images.at(image_pair.image_id2);

            if (!image1.is_registered || !image2.is_registered) continue;

            int idx1 = image_id_to_idx[image_pair.image_id1];
            int idx2 = image_id_to_idx[image_pair.image_id2];

            double weight = type == INLIER_RATIO ?
                                                 image_pair.weight :
                                                 static_cast<double>(image_pair.inliers.size());

            // Add edges in both directions
            adj_list[idx1].emplace_back(idx2, weight);
            adj_list[idx2].emplace_back(idx1, weight);
        }

        // Prim's algorithm for maximum spanning tree
        std::vector<bool> visited(num_vertices, false);
        std::vector<int> parent_idx(num_vertices, -1);
        std::priority_queue<Edge, std::vector<Edge>, CompareEdge> pq;

        // Start from vertex 0
        visited[0] = true;
        for (const Edge& edge : adj_list[0]) {
            pq.push(edge);
        }

        // Build maximum spanning tree
        std::vector<std::vector<int>> tree_edges(num_vertices);
        while (!pq.empty()) {
            Edge current = pq.top();
            pq.pop();

            if (visited[current.to]) continue;

            visited[current.to] = true;

            // Add edges to tree representation
            int from = -1;
            for (int i = 0; i < num_vertices; ++i) {
                if (visited[i]) {
                    for (const Edge& edge : adj_list[i]) {
                        if (edge.to == current.to && edge.weight == current.weight) {
                            from = i;
                            break;
                        }
                    }
                    if (from != -1) break;
                }
            }

            tree_edges[from].push_back(current.to);
            tree_edges[current.to].push_back(from);
            parent_idx[current.to] = from;

            // Add edges from newly visited vertex
            for (const Edge& edge : adj_list[current.to]) {
                if (!visited[edge.to]) {
                    pq.push(edge);
                }
            }
        }

        // Convert indices back to image IDs
        parents.clear();
        for (int i = 0; i < num_vertices; ++i) {
            if (parent_idx[i] != -1) {
                parents[idx_to_image_id[i]] = idx_to_image_id[parent_idx[i]];
            } else {
                parents[idx_to_image_id[i]] = idx_to_image_id[i];  // Root points to itself
            }
        }

        return idx_to_image_id[0];
    }

}; // namespace glomap
