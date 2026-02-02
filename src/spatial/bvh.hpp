// Dynamic bounding volume hierarchy. Leaves store "fat" AABBs (padded by a
// margin) so small motions do not require tree updates. Insertion descends by
// the surface-area cost of creating a new parent; after every structural change
// the ancestors are refit and locally rotated when a rotation reduces the
// surface area of the subtree (SAH tree rotations).
#pragma once
#include <cstdint>
#include <functional>
#include <vector>

#include "math/math.hpp"

namespace ifg {

class DynamicBVH {
public:
    static constexpr int kNull = -1;
    explicit DynamicBVH(float margin = 0.1f) : margin_(margin) {}

    int insert(const AABB& box, std::uint32_t user);
    void remove(int proxy);
    // Returns true if the tree changed (box escaped its fat AABB).
    bool move(int proxy, const AABB& box);
    std::uint32_t user(int proxy) const { return nodes_[proxy].user; }
    const AABB& fat_aabb(int proxy) const { return nodes_[proxy].box; }

    // visitor returns false to stop the query
    void query(const AABB& box, const std::function<bool(int proxy)>& visit) const;
    // visitor returns the new max distance (clip), or a negative value to stop.
    void raycast(const Ray& ray, float tmax, const std::function<float(int proxy, float tmax)>& visit) const;

    int height() const;
    int size() const { return leaf_count_; }
    float total_internal_area() const;  // sum of internal node surface areas (SAH quality metric)
    bool validate() const;              // structural + bounds invariants
    int root() const { return root_; }

private:
    struct Node {
        AABB box;
        int parent = kNull, left = kNull, right = kNull;
        int height = 0;  // 0 for leaves, -1 for free nodes
        std::uint32_t user = 0;
        bool leaf() const { return left == kNull; }
    };
    int alloc_node();
    void free_node(int i);
    void insert_leaf(int leaf);
    void remove_leaf(int leaf);
    void refit_upwards(int i);
    void rotate(int i);
    int height_of(int i) const;
    bool validate_node(int i) const;

    std::vector<Node> nodes_;
    int root_ = kNull, free_list_ = kNull, leaf_count_ = 0;
    float margin_;
};

}  // namespace ifg
