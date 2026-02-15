#include "spatial/octree.hpp"

namespace ifg {

LooseOctree::LooseOctree(Vec3 center, float half_size, int max_depth)
    : root_(std::make_unique<Node>()), max_depth_(max_depth) {
    root_->center = center;
    root_->half = half_size;
}
LooseOctree::~LooseOctree() = default;

void LooseOctree::insert(std::uint32_t id, const AABB& box) {
    Vec3 c = box.center(), e = box.extents();
    float ext = std::max(e.x, std::max(e.y, e.z));
    Node* n = root_.get();
    Vec3 d = c - n->center;
    if (std::fabs(d.x) > n->half || std::fabs(d.y) > n->half || std::fabs(d.z) > n->half || ext > n->half) {
        overflow_.push_back({id, box});
        where_[id] = nullptr;
        return;
    }
    for (int depth = 0; depth < max_depth_; ++depth) {
        float child_half = n->half * 0.5f;
        if (ext > child_half) break;
        int idx = (c.x >= n->center.x ? 1 : 0) | (c.y >= n->center.y ? 2 : 0) | (c.z >= n->center.z ? 4 : 0);
        if (!n->kids[idx]) {
            auto k = std::make_unique<Node>();
            k->half = child_half;
            k->center = n->center + Vec3((idx & 1) ? child_half : -child_half, (idx & 2) ? child_half : -child_half,
                                         (idx & 4) ? child_half : -child_half);
            n->kids[idx] = std::move(k);
        }
        n = n->kids[idx].get();
    }
    n->items.push_back({id, box});
    where_[id] = n;
}

bool LooseOctree::remove(std::uint32_t id) {
    auto it = where_.find(id);
    if (it == where_.end()) return false;
    auto& vec = it->second ? it->second->items : overflow_;
    for (std::size_t i = 0; i < vec.size(); ++i)
        if (vec[i].id == id) {
            vec[i] = vec.back();
            vec.pop_back();
            break;
        }
    where_.erase(it);
    return true;
}

void LooseOctree::query(const AABB& box, const std::function<void(std::uint32_t)>& visit) const {
    for (const Entry& e : overflow_)
        if (e.box.overlaps(box)) visit(e.id);
    std::vector<const Node*> stack{root_.get()};
    while (!stack.empty()) {
        const Node* n = stack.back();
        stack.pop_back();
        if (!n->loose().overlaps(box)) continue;
        for (const Entry& e : n->items)
            if (e.box.overlaps(box)) visit(e.id);
        for (auto& k : n->kids)
            if (k) stack.push_back(k.get());
    }
}

void LooseOctree::raycast(const Ray& ray, float tmax, const std::function<float(std::uint32_t, float)>& visit) const {
    auto test = [&](const Entry& e) {
        if (intersect_ray_aabb(ray, e.box, tmax)) {
            float r = visit(e.id, tmax);
            if (r < 0) return false;
            tmax = std::min(tmax, r);
        }
        return true;
    };
    for (const Entry& e : overflow_)
        if (!test(e)) return;
    std::vector<const Node*> stack{root_.get()};
    while (!stack.empty()) {
        const Node* n = stack.back();
        stack.pop_back();
        if (!intersect_ray_aabb(ray, n->loose(), tmax)) continue;
        for (const Entry& e : n->items)
            if (!test(e)) return;
        for (auto& k : n->kids)
            if (k) stack.push_back(k.get());
    }
}

std::size_t LooseOctree::node_count() const {
    std::size_t c = 0;
    std::vector<const Node*> stack{root_.get()};
    while (!stack.empty()) {
        const Node* n = stack.back();
        stack.pop_back();
        ++c;
        for (auto& k : n->kids)
            if (k) stack.push_back(k.get());
    }
    return c;
}

}  // namespace ifg
