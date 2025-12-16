// Minimal rigid-body physics (bonus module): linear motion only (no
// rotation), sphere and AABB shapes, BVH broadphase, impulse resolution with
// restitution and positional correction.
#pragma once
#include <cstdint>
#include <vector>

#include "math/math.hpp"
#include "spatial/bvh.hpp"

namespace ifg {

struct RigidBody {
    enum class Shape { Sphere, Box } shape = Shape::Sphere;
    Vec3 position;
    Vec3 velocity;
    Vec3 half_extents{0.5f, 0.5f, 0.5f};  // Box
    float radius = 0.5f;                   // Sphere
    float inv_mass = 1.0f;                 // 0 = static
    float restitution = 0.5f;
    AABB bounds() const {
        return shape == Shape::Sphere ? AABB::from_center_extents(position, Vec3(radius))
                                      : AABB::from_center_extents(position, half_extents);
    }
};

struct Contact {
    std::uint32_t a, b;
    Vec3 normal;  // from a to b
    float depth;
};

class PhysicsWorld {
public:
    Vec3 gravity{0, -9.81f, 0};
    int solver_iterations = 8;
    std::uint32_t add(const RigidBody& b);
    RigidBody& body(std::uint32_t id) { return bodies_[id]; }
    const std::vector<RigidBody>& bodies() const { return bodies_; }
    void step(float dt);
    const std::vector<Contact>& last_contacts() const { return contacts_; }
    std::size_t broadphase_pairs() const { return pairs_; }
    static bool collide(const RigidBody& a, const RigidBody& b, Contact& out);

private:
    std::vector<RigidBody> bodies_;
    std::vector<int> proxies_;
    DynamicBVH bvh_{0.05f};
    std::vector<Contact> contacts_;
    std::size_t pairs_ = 0;
};

}  // namespace ifg
