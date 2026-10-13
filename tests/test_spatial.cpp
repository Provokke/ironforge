#include <algorithm>
#include <map>
#include <random>
#include <set>

#include "doctest/doctest.h"
#include "spatial/bvh.hpp"
#include "spatial/octree.hpp"

using namespace ifg;

namespace {
struct Scene {
    std::mt19937 rng;
    explicit Scene(unsigned seed) : rng(seed) {}
    float f(float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); }
    AABB box(float world = 100, float maxsz = 4) {
        Vec3 c{f(-world, world), f(-world, world), f(-world, world)};
        Vec3 e{f(0.05f, maxsz), f(0.05f, maxsz), f(0.05f, maxsz)};
        return AABB::from_center_extents(c, e);
    }
    Ray ray(float world = 100) {
        Vec3 o{f(-world, world), f(-world, world), f(-world, world)};
        Vec3 d{f(-1, 1), f(-1, 1), f(-1, 1)};
        return {o, normalize(d)};
    }
};

std::set<std::uint32_t> brute_query(const std::map<std::uint32_t, AABB>& objs, const AABB& q) {
    std::set<std::uint32_t> s;
    for (auto& [id, b] : objs)
        if (b.overlaps(q)) s.insert(id);
    return s;
}

std::optional<std::pair<float, std::uint32_t>> brute_ray(const std::map<std::uint32_t, AABB>& objs, const Ray& r) {
    std::optional<std::pair<float, std::uint32_t>> best;
    for (auto& [id, b] : objs)
        if (auto t = intersect_ray_aabb(r, b))
            if (!best || *t < best->first) best = {{*t, id}};
    return best;
}
}  // namespace

TEST_CASE("BVH matches brute force over random dynamic scenes") {
    for (unsigned seed = 1; seed <= 5; ++seed) {
        Scene s(seed);
        DynamicBVH bvh(0.0f);  // zero margin so leaf boxes equal object boxes
        std::map<std::uint32_t, AABB> objs;
        std::map<std::uint32_t, int> proxy;
        std::uint32_t next = 0;
        for (int i = 0; i < 500; ++i) {
            AABB b = s.box();
            objs[next] = b;
            proxy[next] = bvh.insert(b, next);
            ++next;
        }
        REQUIRE(bvh.validate());
        for (int round = 0; round < 20; ++round) {
            // mutate: remove some, move some, add some
            for (int k = 0; k < 25; ++k) {
                auto it = objs.begin();
                std::advance(it, s.rng() % objs.size());
                bvh.remove(proxy[it->first]);
                proxy.erase(it->first);
                objs.erase(it);
            }
            for (int k = 0; k < 50; ++k) {
                auto it = objs.begin();
                std::advance(it, s.rng() % objs.size());
                AABB nb = s.box();
                it->second = nb;
                bvh.move(proxy[it->first], nb);
            }
            for (int k = 0; k < 25; ++k) {
                AABB b = s.box();
                objs[next] = b;
                proxy[next] = bvh.insert(b, next);
                ++next;
            }
            REQUIRE(bvh.validate());
            CHECK(bvh.size() == static_cast<int>(objs.size()));
            for (int q = 0; q < 20; ++q) {
                AABB qb = s.box(100, 20);
                std::set<std::uint32_t> got;
                bvh.query(qb, [&](int p) {
                    std::uint32_t id = bvh.user(p);
                    if (objs[id].overlaps(qb)) got.insert(id);
                    return true;
                });
                CHECK(got == brute_query(objs, qb));
            }
            for (int q = 0; q < 20; ++q) {
                Ray r = s.ray();
                std::optional<std::pair<float, std::uint32_t>> best;
                bvh.raycast(r, 1e30f, [&](int p, float tmax) {
                    std::uint32_t id = bvh.user(p);
                    auto t = intersect_ray_aabb(r, objs[id], tmax);
                    if (t && (!best || *t < best->first)) best = {{*t, id}};
                    return best ? best->first : tmax;
                });
                auto expect = brute_ray(objs, r);
                REQUIRE(best.has_value() == expect.has_value());
                if (best) CHECK(best->first == doctest::Approx(expect->first));
            }
        }
    }
}

TEST_CASE("BVH fat margin avoids reinsertion for small motions") {
    DynamicBVH bvh(0.5f);
    int p = bvh.insert(AABB({0, 0, 0}, {1, 1, 1}), 7);
    CHECK_FALSE(bvh.move(p, AABB({0.2f, 0, 0}, {1.2f, 1, 1})));
    CHECK(bvh.move(p, AABB({5, 0, 0}, {6, 1, 1})));
    CHECK(bvh.user(p) == 7);
    CHECK(bvh.fat_aabb(p).contains(AABB({5, 0, 0}, {6, 1, 1})));
    bvh.remove(p);
    CHECK(bvh.size() == 0);
    CHECK(bvh.validate());
    // query on empty tree is a no-op
    int visits = 0;
    bvh.query(AABB({-10, -10, -10}, {10, 10, 10}), [&](int) { ++visits; return true; });
    CHECK(visits == 0);
}

TEST_CASE("BVH stays shallow for sorted insertion (rotations work)") {
    DynamicBVH bvh(0.0f);
    for (int i = 0; i < 1024; ++i) bvh.insert(AABB::from_center_extents({float(i), 0, 0}, Vec3(0.4f)), i);
    CHECK(bvh.validate());
    // a degenerate linked-list tree would have height ~1023
    MESSAGE("height after 1024 sorted inserts: " << bvh.height());
    CHECK(bvh.height() < 40);
}

TEST_CASE("BVH early-out stops the query") {
    DynamicBVH bvh;
    for (int i = 0; i < 100; ++i) bvh.insert(AABB::from_center_extents({0, 0, 0}, Vec3(1)), i);
    int visits = 0;
    bvh.query(AABB({-1, -1, -1}, {1, 1, 1}), [&](int) { return ++visits < 5; });
    CHECK(visits == 5);
}

TEST_CASE("loose octree matches brute force") {
    for (unsigned seed = 10; seed < 13; ++seed) {
        Scene s(seed);
        LooseOctree oct({0, 0, 0}, 128.0f, 7);
        std::map<std::uint32_t, AABB> objs;
        for (std::uint32_t i = 0; i < 800; ++i) {
            // some objects outside the root to exercise the overflow list
            AABB b = (i % 50 == 0) ? AABB::from_center_extents({300, 0, 0}, Vec3(1)) : s.box();
            objs[i] = b;
            oct.insert(i, b);
        }
        for (std::uint32_t i = 0; i < 800; i += 3) {
            AABB b = s.box();
            objs[i] = b;
            oct.update(i, b);
        }
        for (std::uint32_t i = 1; i < 800; i += 7) {
            CHECK(oct.remove(i));
            objs.erase(i);
        }
        CHECK_FALSE(oct.remove(1));
        CHECK(oct.size() == objs.size());
        for (int q = 0; q < 50; ++q) {
            AABB qb = s.box(150, 30);
            std::set<std::uint32_t> got;
            oct.query(qb, [&](std::uint32_t id) { got.insert(id); });
            CHECK(got == brute_query(objs, qb));
        }
        for (int q = 0; q < 50; ++q) {
            Ray r = s.ray();
            std::optional<std::pair<float, std::uint32_t>> best;
            oct.raycast(r, 1e30f, [&](std::uint32_t id, float tmax) {
                auto t = intersect_ray_aabb(r, objs[id], tmax);
                if (t && (!best || *t < best->first)) best = {{*t, id}};
                return best ? best->first : tmax;
            });
            auto expect = brute_ray(objs, r);
            REQUIRE(best.has_value() == expect.has_value());
            if (best) CHECK(best->first == doctest::Approx(expect->first));
        }
        CHECK(oct.node_count() > 1);
    }
}
