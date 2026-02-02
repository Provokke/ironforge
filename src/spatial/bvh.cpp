#include "spatial/bvh.hpp"

#include <cassert>

namespace ifg {

int DynamicBVH::alloc_node() {
    if (free_list_ != kNull) {
        int i = free_list_;
        free_list_ = nodes_[i].parent;
        nodes_[i] = Node{};
        return i;
    }
    nodes_.push_back(Node{});
    return static_cast<int>(nodes_.size()) - 1;
}

void DynamicBVH::free_node(int i) {
    nodes_[i].height = -1;
    nodes_[i].parent = free_list_;
    nodes_[i].left = nodes_[i].right = kNull;
    free_list_ = i;
}

int DynamicBVH::insert(const AABB& box, std::uint32_t user) {
    int leaf = alloc_node();
    nodes_[leaf].box = box.grown(margin_);
    nodes_[leaf].user = user;
    nodes_[leaf].height = 0;
    insert_leaf(leaf);
    ++leaf_count_;
    return leaf;
}

void DynamicBVH::remove(int proxy) {
    assert(proxy >= 0 && proxy < (int)nodes_.size() && nodes_[proxy].leaf() && nodes_[proxy].height == 0);
    remove_leaf(proxy);
    free_node(proxy);
    --leaf_count_;
}

bool DynamicBVH::move(int proxy, const AABB& box) {
    if (nodes_[proxy].box.contains(box)) return false;
    remove_leaf(proxy);
    nodes_[proxy].box = box.grown(margin_);
    insert_leaf(proxy);
    return true;
}

void DynamicBVH::insert_leaf(int leaf) {
    if (root_ == kNull) {
        root_ = leaf;
        nodes_[leaf].parent = kNull;
        return;
    }
    const AABB lbox = nodes_[leaf].box;
    // Greedy descent on the surface-area cost of attaching the leaf.
    int idx = root_;
    while (!nodes_[idx].leaf()) {
        const Node& n = nodes_[idx];
        float area = n.box.surface_area();
        float combined = merge(n.box, lbox).surface_area();
        float cost_here = 2.0f * combined;
        float inherit = 2.0f * (combined - area);
        auto child_cost = [&](int c) {
            const Node& cn = nodes_[c];
            float m = merge(cn.box, lbox).surface_area();
            return cn.leaf() ? m + inherit : (m - cn.box.surface_area()) + inherit;
        };
        float c1 = child_cost(n.left), c2 = child_cost(n.right);
        if (cost_here < c1 && cost_here < c2) break;
        idx = c1 < c2 ? n.left : n.right;
    }
    int sibling = idx;
    int old_parent = nodes_[sibling].parent;
    int np = alloc_node();
    nodes_[np].parent = old_parent;
    nodes_[np].box = merge(lbox, nodes_[sibling].box);
    nodes_[np].height = nodes_[sibling].height + 1;
    nodes_[np].left = sibling;
    nodes_[np].right = leaf;
    nodes_[sibling].parent = np;
    nodes_[leaf].parent = np;
    if (old_parent == kNull) {
        root_ = np;
    } else if (nodes_[old_parent].left == sibling) {
        nodes_[old_parent].left = np;
    } else {
        nodes_[old_parent].right = np;
    }
    refit_upwards(np);
}

void DynamicBVH::remove_leaf(int leaf) {
    if (leaf == root_) {
        root_ = kNull;
        return;
    }
    int parent = nodes_[leaf].parent;
    int grand = nodes_[parent].parent;
    int sibling = nodes_[parent].left == leaf ? nodes_[parent].right : nodes_[parent].left;
    if (grand == kNull) {
        root_ = sibling;
        nodes_[sibling].parent = kNull;
    } else {
        if (nodes_[grand].left == parent) nodes_[grand].left = sibling; else nodes_[grand].right = sibling;
        nodes_[sibling].parent = grand;
        refit_upwards(grand);
    }
    free_node(parent);
    nodes_[leaf].parent = kNull;
}

void DynamicBVH::refit_upwards(int i) {
    while (i != kNull) {
        Node& n = nodes_[i];
        n.box = merge(nodes_[n.left].box, nodes_[n.right].box);
        n.height = 1 + std::max(nodes_[n.left].height, nodes_[n.right].height);
        rotate(i);
        i = nodes_[i].parent;
    }
}

// Try the four grandchild/child swaps below node i and keep the one that
// reduces the surface area of the modified child the most.
void DynamicBVH::rotate(int a) {
    Node& A = nodes_[a];
    int b = A.left, c = A.right;
    enum { None, BF, BG, CD, CE } best = None;
    float best_gain = 0.0f;
    if (!nodes_[c].leaf()) {
        int f = nodes_[c].left, g = nodes_[c].right;
        float area_c = nodes_[c].box.surface_area();
        float g1 = area_c - merge(nodes_[b].box, nodes_[g].box).surface_area();  // swap B<->F
        float g2 = area_c - merge(nodes_[b].box, nodes_[f].box).surface_area();  // swap B<->G
        if (g1 > best_gain) { best_gain = g1; best = BF; }
        if (g2 > best_gain) { best_gain = g2; best = BG; }
    }
    if (!nodes_[b].leaf()) {
        int d = nodes_[b].left, e = nodes_[b].right;
        float area_b = nodes_[b].box.surface_area();
        float g3 = area_b - merge(nodes_[c].box, nodes_[e].box).surface_area();  // swap C<->D
        float g4 = area_b - merge(nodes_[c].box, nodes_[d].box).surface_area();  // swap C<->E
        if (g3 > best_gain) { best_gain = g3; best = CD; }
        if (g4 > best_gain) { best_gain = g4; best = CE; }
    }
    if (best == None) return;
    // generic swap: `x` (child of a) with `y` (child of `p`, the other child of a)
    auto swap_nodes = [&](int x, int p, int y) {
        Node& Ax = nodes_[a];
        if (Ax.left == x) Ax.left = y; else Ax.right = y;
        nodes_[y].parent = a;
        Node& P = nodes_[p];
        if (P.left == y) P.left = x; else P.right = x;
        nodes_[x].parent = p;
        P.box = merge(nodes_[P.left].box, nodes_[P.right].box);
        P.height = 1 + std::max(nodes_[P.left].height, nodes_[P.right].height);
        Node& A2 = nodes_[a];
        A2.height = 1 + std::max(nodes_[A2.left].height, nodes_[A2.right].height);
    };
    switch (best) {
        case BF: swap_nodes(b, c, nodes_[c].left); break;
        case BG: swap_nodes(b, c, nodes_[c].right); break;
        case CD: swap_nodes(c, b, nodes_[b].left); break;
        case CE: swap_nodes(c, b, nodes_[b].right); break;
        case None: break;
    }
}

void DynamicBVH::query(const AABB& box, const std::function<bool(int)>& visit) const {
    if (root_ == kNull) return;
    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(root_);
    while (!stack.empty()) {
        int i = stack.back();
        stack.pop_back();
        const Node& n = nodes_[i];
        if (!n.box.overlaps(box)) continue;
        if (n.leaf()) {
            if (!visit(i)) return;
        } else {
            stack.push_back(n.left);
            stack.push_back(n.right);
        }
    }
}

void DynamicBVH::raycast(const Ray& ray, float tmax, const std::function<float(int, float)>& visit) const {
    if (root_ == kNull) return;
    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(root_);
    while (!stack.empty()) {
        int i = stack.back();
        stack.pop_back();
        const Node& n = nodes_[i];
        auto t = intersect_ray_aabb(ray, n.box, tmax);
        if (!t) continue;
        if (n.leaf()) {
            float r = visit(i, tmax);
            if (r < 0) return;
            tmax = std::min(tmax, r);
        } else {
            // visit the nearer child first (pushed last)
            auto tl = intersect_ray_aabb(ray, nodes_[n.left].box, tmax);
            auto tr = intersect_ray_aabb(ray, nodes_[n.right].box, tmax);
            if (tl && tr) {
                if (*tl < *tr) { stack.push_back(n.right); stack.push_back(n.left); }
                else { stack.push_back(n.left); stack.push_back(n.right); }
            } else if (tl) {
                stack.push_back(n.left);
            } else if (tr) {
                stack.push_back(n.right);
            }
        }
    }
}

int DynamicBVH::height_of(int i) const {
    if (i == kNull) return 0;
    const Node& n = nodes_[i];
    if (n.leaf()) return 0;
    return 1 + std::max(height_of(n.left), height_of(n.right));
}

int DynamicBVH::height() const { return root_ == kNull ? 0 : height_of(root_); }

float DynamicBVH::total_internal_area() const {
    float a = 0;
    for (const Node& n : nodes_)
        if (n.height > 0) a += n.box.surface_area();
    return a;
}

bool DynamicBVH::validate_node(int i) const {
    const Node& n = nodes_[i];
    if (n.leaf()) return n.right == kNull && n.height == 0;
    if (n.right == kNull) return false;
    const Node& l = nodes_[n.left];
    const Node& r = nodes_[n.right];
    if (l.parent != i || r.parent != i) return false;
    if (!n.box.contains(l.box) || !n.box.contains(r.box)) return false;
    if (n.height != 1 + std::max(l.height, r.height)) return false;
    return validate_node(n.left) && validate_node(n.right);
}

bool DynamicBVH::validate() const {
    if (root_ == kNull) return leaf_count_ == 0;
    if (nodes_[root_].parent != kNull) return false;
    int leaves = 0;
    for (const Node& n : nodes_)
        if (n.height == 0) ++leaves;
    return leaves == leaf_count_ && validate_node(root_);
}

}  // namespace ifg
