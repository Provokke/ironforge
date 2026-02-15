// Loose octree (looseness k = 2). An object is stored in the deepest node whose
// tight half-size is >= the object's largest half-extent, chosen by the
// object's center; the node's loose bounds (2x tight) then always contain it.
// Objects whose center lies outside the root are kept in an overflow list.
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include "math/math.hpp"

namespace ifg {

class LooseOctree {
public:
    LooseOctree(Vec3 center, float half_size, int max_depth = 8);
    ~LooseOctree();
    void insert(std::uint32_t id, const AABB& box);
    bool remove(std::uint32_t id);
    void update(std::uint32_t id, const AABB& box) { remove(id); insert(id, box); }
    void query(const AABB& box, const std::function<void(std::uint32_t)>& visit) const;
    void raycast(const Ray& ray, float tmax, const std::function<float(std::uint32_t, float)>& visit) const;
    std::size_t size() const { return where_.size(); }
    std::size_t node_count() const;

private:
    struct Entry { std::uint32_t id; AABB box; };
    struct Node {
        Vec3 center;
        float half;
        std::vector<Entry> items;
        std::array<std::unique_ptr<Node>, 8> kids;
        AABB loose() const { return AABB::from_center_extents(center, Vec3(half * 2.0f)); }
    };
    std::unique_ptr<Node> root_;
    std::vector<Entry> overflow_;
    int max_depth_;
    std::unordered_map<std::uint32_t, Node*> where_;  // nullptr => overflow
};

}  // namespace ifg
