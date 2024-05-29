#ifndef GLOMAP_MATH_UNION_FIND_H_
#define GLOMAP_MATH_UNION_FIND_H_
#include <unordered_map>
#include <cstdint>

namespace glomap {
// UnionFind class to maintain disjoint sets for creating tracks
template <typename DataType>
class UnionFind {
public:
    // Find the root of the element x
    DataType Find(DataType x) {
        // If x is not in parent map, initialize it with x as its parent
        if (parent.find(x) == parent.end()) {
            parent[x] = x;
            return x;
        }
        // Path compression: set the parent of x to the root of the set containing x
        if (parent[x] != x) {
            parent[x] = Find(parent[x]);
        }
        return parent[x];
    }

    // Unite the sets containing x and y
    void Unite(DataType x, DataType y) {
        DataType root_x = Find(x);
        DataType root_y = Find(y);
        if (root_x != root_y) 
            parent[root_x] = root_y;
    }

    void Clear() {
        parent.clear();
    }

private:
    // Map to store the parent of each element
    std::unordered_map<DataType, DataType> parent;
};

}; // glomap
#endif // GLOMAP_MATH_UNION_FIND_H_