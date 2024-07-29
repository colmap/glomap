#pragma once
#include <cstdint>
#include <unordered_map>

namespace glomap {

// UnionFind class to maintain disjoint sets for creating tracks
template <typename DataType>
class UnionFind {
 public:
  // Find the root of the element x
  DataType Find(DataType x) {
    // If x is not in parent map, initialize it with x as its parent
    auto parentIt = parent_.find(x);
    if (parentIt == parent_.end()) {
      parent_.emplace_hint(parentIt, x, x);
      return x;
    }
    // Path compression: set the parent of x to the root of the set containing x
    if (parentIt->second != x) {
      parentIt->second = Find(parentIt->second);
    }
    return parentIt->second;
  }

  // Unite the sets containing x and y
  void Union(DataType x, DataType y) {
    DataType root_x = Find(x);
    DataType root_y = Find(y);
    if (root_x != root_y) parent_[root_x] = root_y;
  }

  void Clear() { parent_.clear(); }

 private:
  // Map to store the parent of each element
  std::unordered_map<DataType, DataType> parent_;
};

}  // namespace glomap
