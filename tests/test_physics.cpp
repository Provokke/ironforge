#include "doctest/doctest.h"
#include "physics/physics.hpp"

using namespace ifg;

TEST_CASE("sphere comes to rest on a static floor box") {
    PhysicsWorld w;
    RigidBody floor;
    floor.shape = RigidBody::Shape::Box;
    floor.position = {0, -0.5f, 0};
    floor.half_extents = {10, 0.5f, 10};
    floor.inv_mass = 0;
    w.add(floor);
    RigidBody ball;
    ball.position = {0, 5, 0};
    ball.radius = 0.5f;
    ball.restitution = 0.3f;
    auto id = w.add(ball);
    for (int i = 0; i < 600; ++i) w.step(1.0f / 120.0f);
    const RigidBody& b = w.body(id);
    CHECK(b.position.y == doctest::Approx(0.5f).epsilon(0.02));
    CHECK(std::fabs(b.velocity.y) < 0.2f);
    CHECK(w.body(0).position.y == -0.5f);  // static body never moves
}

TEST_CASE("elastic sphere-sphere collision conserves momentum and swaps velocities") {
    PhysicsWorld w;
    w.gravity = {};
    RigidBody a, b;
    a.position = {-2, 0, 0};
    a.velocity = {3, 0, 0};
    b.position = {2, 0, 0};
    b.velocity = {-1, 0, 0};
    a.restitution = b.restitution = 1.0f;
    w.add(a);
    w.add(b);
    Vec3 p0 = w.body(0).velocity + w.body(1).velocity;
    for (int i = 0; i < 200; ++i) w.step(1.0f / 120.0f);
    Vec3 p1 = w.body(0).velocity + w.body(1).velocity;
    CHECK(approx(p0, p1, 1e-4f));
    CHECK(w.body(0).velocity.x == doctest::Approx(-1.0f).epsilon(1e-3));
    CHECK(w.body(1).velocity.x == doctest::Approx(3.0f).epsilon(1e-3));
}

TEST_CASE("narrowphase contact normals and depths") {
    RigidBody s;
    s.radius = 1;
    s.position = {0, 0, 0};
    RigidBody box;
    box.shape = RigidBody::Shape::Box;
    box.position = {1.5f, 0, 0};
    box.half_extents = {1, 1, 1};
    Contact c{};
    REQUIRE(PhysicsWorld::collide(s, box, c));
    CHECK(approx(c.normal, Vec3(1, 0, 0)));
    CHECK(c.depth == doctest::Approx(0.5f));
    REQUIRE(PhysicsWorld::collide(box, s, c));
    CHECK(approx(c.normal, Vec3(-1, 0, 0)));
    RigidBody box2 = box;
    box2.position = {1.8f, 0.5f, 0};
    RigidBody box1 = box;
    box1.position = {0, 0, 0};
    REQUIRE(PhysicsWorld::collide(box1, box2, c));
    CHECK(approx(c.normal, Vec3(1, 0, 0)));
    CHECK(c.depth == doctest::Approx(0.2f));
    box2.position = {5, 0, 0};
    CHECK_FALSE(PhysicsWorld::collide(box1, box2, c));
    // sphere center inside a box gets pushed out through the nearest face
    RigidBody big = box;
    big.position = {0, 0, 0};
    big.half_extents = {2, 2, 2};
    s.position = {0, 1.8f, 0};
    s.radius = 0.5f;
    REQUIRE(PhysicsWorld::collide(s, big, c));
    CHECK(approx(c.normal, Vec3(0, -1, 0)));  // box pushed down == sphere pushed up
    CHECK(c.depth == doctest::Approx(0.7f));
}

TEST_CASE("stack of spheres in a box settles without tunnelling") {
    PhysicsWorld w;
    RigidBody floor;
    floor.shape = RigidBody::Shape::Box;
    floor.half_extents = {5, 0.5f, 5};
    floor.position = {0, -0.5f, 0};
    floor.inv_mass = 0;
    w.add(floor);
    // frictionless spheres spread out, so walls keep them on the floor
    for (int k = 0; k < 4; ++k) {
        RigidBody wall;
        wall.shape = RigidBody::Shape::Box;
        wall.inv_mass = 0;
        bool xwall = k < 2;
        float side = (k % 2) ? 3.0f : -3.0f;
        wall.position = xwall ? Vec3(side, 2, 0) : Vec3(0, 2, side);
        wall.half_extents = xwall ? Vec3(0.5f, 3, 4) : Vec3(4, 3, 0.5f);
        w.add(wall);
    }
    std::size_t first_ball = w.bodies().size();
    for (int i = 0; i < 20; ++i) {
        RigidBody b;
        b.position = {float(i % 4) * 1.1f - 1.6f, 1.0f + float(i / 4) * 1.2f, float(i % 3) * 0.3f};
        b.radius = 0.5f;
        b.restitution = 0.1f;
        w.add(b);
    }
    for (int i = 0; i < 1200; ++i) w.step(1.0f / 120.0f);
    for (std::size_t i = first_ball; i < w.bodies().size(); ++i) {
        const RigidBody& b = w.bodies()[i];
        CHECK(b.position.y > 0.4f);  // nothing fell through the floor
        CHECK(std::fabs(b.position.x) < 2.6f);
        CHECK(std::fabs(b.position.z) < 2.6f);
    }
    // no deep interpenetration between resting spheres
    float worst = 0;
    for (std::size_t i = first_ball; i < w.bodies().size(); ++i)
        for (std::size_t j = i + 1; j < w.bodies().size(); ++j)
            worst = std::max(worst, 1.0f - length(w.bodies()[i].position - w.bodies()[j].position));
    CHECK(worst < 0.1f);
    CHECK(w.broadphase_pairs() > 0);
}
