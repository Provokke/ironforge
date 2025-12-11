#include "physics/physics.hpp"

namespace ifg {

std::uint32_t PhysicsWorld::add(const RigidBody& b) {
    auto id = static_cast<std::uint32_t>(bodies_.size());
    bodies_.push_back(b);
    proxies_.push_back(bvh_.insert(b.bounds(), id));
    return id;
}

static bool sphere_box(const RigidBody& s, const RigidBody& box, Contact& c, bool flip) {
    AABB b = box.bounds();
    Vec3 cp = closest_point_aabb(b, s.position);
    Vec3 d = cp - s.position;
    float dist2 = length_sq(d);
    if (dist2 > s.radius * s.radius) return false;
    if (dist2 > 1e-12f) {
        float dist = std::sqrt(dist2);
        c.normal = d / dist;  // sphere -> box
        c.depth = s.radius - dist;
    } else {
        // center inside the box: push out along the axis of least penetration
        Vec3 lo = s.position - b.min, hi = b.max - s.position;
        float best = lo.x;
        c.normal = {-1, 0, 0};
        const float cand[6] = {lo.x, hi.x, lo.y, hi.y, lo.z, hi.z};
        const Vec3 nrm[6] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
        for (int i = 0; i < 6; ++i)
            if (cand[i] < best) { best = cand[i]; c.normal = nrm[i]; }
        c.normal = -c.normal;  // sphere -> box direction is opposite of the exit direction
        c.depth = best + s.radius;
    }
    if (flip) c.normal = -c.normal;
    return true;
}

bool PhysicsWorld::collide(const RigidBody& a, const RigidBody& b, Contact& c) {
    using S = RigidBody::Shape;
    if (a.shape == S::Sphere && b.shape == S::Sphere) {
        Vec3 d = b.position - a.position;
        float r = a.radius + b.radius;
        float d2 = length_sq(d);
        if (d2 > r * r) return false;
        float dist = std::sqrt(d2);
        c.normal = dist > 1e-6f ? d / dist : Vec3(0, 1, 0);
        c.depth = r - dist;
        return true;
    }
    if (a.shape == S::Sphere) return sphere_box(a, b, c, false);
    if (b.shape == S::Sphere) return sphere_box(b, a, c, true);
    // box-box: minimum translation axis
    AABB ba = a.bounds(), bb = b.bounds();
    if (!ba.overlaps(bb)) return false;
    float best = std::numeric_limits<float>::max();
    for (int i = 0; i < 3; ++i) {
        float o1 = ba.max[i] - bb.min[i];  // push b in +axis
        float o2 = bb.max[i] - ba.min[i];  // push b in -axis
        if (o1 < best) { best = o1; c.normal = {}; c.normal[i] = 1; }
        if (o2 < best) { best = o2; c.normal = {}; c.normal[i] = -1; }
    }
    c.depth = best;
    return true;
}

void PhysicsWorld::step(float dt) {
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        RigidBody& b = bodies_[i];
        if (b.inv_mass == 0) continue;
        b.velocity += gravity * dt;
        b.position += b.velocity * dt;
        bvh_.move(proxies_[i], b.bounds());
    }
    contacts_.clear();
    pairs_ = 0;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> candidates;
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        bvh_.query(bodies_[i].bounds().grown(0.05f), [&](int proxy) {
            std::uint32_t j = bvh_.user(proxy);
            if (j <= i) return true;
            ++pairs_;
            if (bodies_[i].inv_mass == 0 && bodies_[j].inv_mass == 0) return true;
            candidates.emplace_back(static_cast<std::uint32_t>(i), j);
            return true;
        });
    }
    // Sequential impulses: re-run narrowphase with updated positions each iteration
    // so stacked bodies propagate their corrections instead of sinking.
    for (int iter = 0; iter < solver_iterations; ++iter) {
        for (auto [ia, ib] : candidates) {
            RigidBody& a = bodies_[ia];
            RigidBody& b = bodies_[ib];
            Contact c{ia, ib, {}, 0};
            if (!collide(a, b, c)) continue;
            if (iter == 0) contacts_.push_back(c);
            float inv_sum = a.inv_mass + b.inv_mass;
            float vn = dot(b.velocity - a.velocity, c.normal);
            if (vn < 0) {  // approaching
                float e = iter == 0 ? std::min(a.restitution, b.restitution) : 0.0f;
                float j = -(1 + e) * vn / inv_sum;
                a.velocity -= c.normal * (j * a.inv_mass);
                b.velocity += c.normal * (j * b.inv_mass);
            }
            // positional correction with a small slop
            float corr = std::max(c.depth - 0.005f, 0.0f) * 0.5f / inv_sum;
            a.position -= c.normal * (corr * a.inv_mass);
            b.position += c.normal * (corr * b.inv_mass);
        }
    }
    for (std::size_t i = 0; i < bodies_.size(); ++i)
        if (bodies_[i].inv_mass != 0) bvh_.move(proxies_[i], bodies_[i].bounds());
}

}  // namespace ifg
