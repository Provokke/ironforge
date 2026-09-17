#include <random>

#include "doctest/doctest.h"
#include "math/math.hpp"

using namespace ifg;

namespace {
struct Rng {
    std::mt19937 g;
    explicit Rng(unsigned seed) : g(seed) {}
    float f(float lo, float hi) { return std::uniform_real_distribution<float>(lo, hi)(g); }
    Vec3 v(float lo = -10, float hi = 10) { return {f(lo, hi), f(lo, hi), f(lo, hi)}; }
    Vec3 unit() {
        Vec3 r;
        do r = v(-1, 1); while (length(r) < 0.1f);
        return normalize(r);
    }
    Quat q() { return Quat::from_axis_angle(unit(), f(-kPi, kPi)); }
    Mat4 affine() {
        Transform t{v(), q(), Vec3(f(0.3f, 3.0f), f(0.3f, 3.0f), f(0.3f, 3.0f))};
        return t.to_matrix();
    }
};
}  // namespace

TEST_CASE("vec3 basic identities") {
    Rng r(1);
    for (int i = 0; i < 1000; ++i) {
        Vec3 a = r.v(), b = r.v(), c = r.v();
        CHECK(approx(cross(a, b), -cross(b, a)));
        CHECK(std::fabs(dot(cross(a, b), a)) <= 1e-3f * length(a) * length(a) * length(b) + 1e-3f);
        CHECK(approx(dot(a, b + c), dot(a, b) + dot(a, c), 1e-3f));
        // Lagrange identity |a x b|^2 = |a|^2|b|^2 - (a.b)^2
        float lhs = length_sq(cross(a, b));
        float rhs = length_sq(a) * length_sq(b) - dot(a, b) * dot(a, b);
        CHECK(std::fabs(lhs - rhs) <= 1e-3f * std::max(1.0f, length_sq(a) * length_sq(b)));
        if (length(a) > 1e-3f) CHECK(approx(length(normalize(a)), 1.0f));
    }
    CHECK(normalize(Vec3{}) == Vec3{});
    CHECK(cross(Vec3(1, 0, 0), Vec3(0, 1, 0)) == Vec3(0, 0, 1));
    CHECK(cross(Vec2(1, 0), Vec2(0, 1)) == 1.0f);
    CHECK(lerp(Vec3(0), Vec3(2), 0.25f) == Vec3(0.5f));
    CHECK(dot(Vec4(1, 2, 3, 4), Vec4(1, 1, 1, 1)) == 10.0f);
}

TEST_CASE("mat3 inverse and determinant") {
    Rng r(2);
    for (int i = 0; i < 300; ++i) {
        Mat3 m = r.q().to_mat3();
        m.c[0] = m.c[0] * r.f(0.5f, 2.0f);
        auto inv = m.inverse();
        REQUIRE(inv);
        Mat3 id = m * *inv;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) CHECK(std::fabs(id.at(a, b) - (a == b ? 1.f : 0.f)) < 1e-4f);
        CHECK(approx(m.transposed().determinant(), m.determinant()));
    }
    Mat3 singular;
    singular.c[2] = singular.c[0];
    CHECK_FALSE(singular.inverse());
}

TEST_CASE("mat4 inverse, transpose and determinant properties") {
    Rng r(3);
    for (int i = 0; i < 500; ++i) {
        Mat4 a = r.affine(), b = r.affine();
        auto ia = a.inverse();
        REQUIRE(ia);
        CHECK(approx(a * *ia, Mat4::identity(), 1e-3f));
        CHECK(approx(*ia * a, Mat4::identity(), 1e-3f));
        // (AB)^-1 = B^-1 A^-1
        CHECK(approx(*(a * b).inverse(), *b.inverse() * *ia, 2e-3f));
        // det(AB) = det(A) det(B)
        float dab = (a * b).determinant(), dadb = a.determinant() * b.determinant();
        CHECK(std::fabs(dab - dadb) <= 1e-3f * std::max(1.0f, std::fabs(dadb)));
        // (AB)^T = B^T A^T
        CHECK(approx((a * b).transposed(), b.transposed() * a.transposed(), 1e-3f));
        CHECK(approx(a.transposed().transposed(), a, 0));
        Vec3 p = r.v();
        CHECK(approx(ia->transform_point(a.transform_point(p)), p, 1e-3f));
    }
    Mat4 z;
    z.c[1] = Vec4(0, 0, 0, 0);
    CHECK_FALSE(z.inverse());
    CHECK(z.determinant() == 0.0f);
}

TEST_CASE("mat4 constructors") {
    CHECK(approx(Mat4::translation({1, 2, 3}).transform_point({1, 1, 1}), Vec3(2, 3, 4)));
    CHECK(approx(Mat4::rotation_z(kPi / 2).transform_point({1, 0, 0}), Vec3(0, 1, 0)));
    CHECK(approx(Mat4::rotation_x(kPi / 2).transform_point({0, 1, 0}), Vec3(0, 0, 1)));
    CHECK(approx(Mat4::rotation_y(kPi / 2).transform_point({0, 0, 1}), Vec3(1, 0, 0)));
    CHECK(approx(Mat4::scale({2, 3, 4}).transform_point({1, 1, 1}), Vec3(2, 3, 4)));
    CHECK(approx(Mat4::translation({5, 0, 0}).transform_dir({1, 0, 0}), Vec3(1, 0, 0)));
    // look_at maps eye to the origin and target onto -Z
    Mat4 v = Mat4::look_at({3, 4, 5}, {0, 0, 0}, {0, 1, 0});
    CHECK(approx(v.transform_point({3, 4, 5}), Vec3(0, 0, 0)));
    Vec3 t = v.transform_point({0, 0, 0});
    CHECK(approx(t.x, 0.0f));
    CHECK(approx(t.y, 0.0f));
    CHECK(t.z < 0);
    // perspective: near plane maps to -1, far plane to +1
    Mat4 p = Mat4::perspective(radians(60), 1.5f, 0.5f, 50.0f);
    CHECK(approx(p.transform_point({0, 0, -0.5f}).z, -1.0f));
    CHECK(approx(p.transform_point({0, 0, -50.0f}).z, 1.0f));
    Mat4 o = Mat4::orthographic(-2, 2, -1, 1, 0.1f, 10);
    CHECK(approx(o.transform_point({2, 1, -0.1f}), Vec3(1, 1, -1)));
}

TEST_CASE("quaternion properties") {
    Rng r(4);
    for (int i = 0; i < 1000; ++i) {
        Quat a = r.q(), b = r.q();
        Vec3 v = r.v();
        CHECK(approx(a.norm(), 1.0f));
        // rotation preserves length and composes like matrices
        CHECK(approx(length(a.rotate(v)), length(v), 1e-3f));
        CHECK(approx((a * b).rotate(v), a.rotate(b.rotate(v)), 1e-3f));
        CHECK(approx(a.to_mat3() * v, a.rotate(v), 1e-3f));
        CHECK(approx(a.inverse().rotate(a.rotate(v)), v, 1e-3f));
        CHECK(approx_rotation(Quat::from_mat3(a.to_mat3()), a, 1e-3f));
        // slerp endpoints and unit length
        CHECK(approx_rotation(slerp(a, b, 0), a));
        CHECK(approx_rotation(slerp(a, b, 1), b, 1e-3f));
        CHECK(approx(slerp(a, b, r.f(0, 1)).norm(), 1.0f, 1e-3f));
    }
    // slerp halfway between identity and 90deg about Y is 45deg about Y
    Quat h = slerp(Quat{}, Quat::from_axis_angle({0, 1, 0}, kPi / 2), 0.5f);
    CHECK(approx_rotation(h, Quat::from_axis_angle({0, 1, 0}, kPi / 4)));
    // euler order: yaw * pitch * roll
    Quat e = Quat::from_euler(0.3f, 0.5f, 0.7f);
    Mat4 m = Mat4::rotation_y(0.5f) * Mat4::rotation_x(0.3f) * Mat4::rotation_z(0.7f);
    CHECK(approx(e.rotate({1, 2, 3}), m.transform_point({1, 2, 3}), 1e-4f));
}

TEST_CASE("transform composition and inverse") {
    Rng r(5);
    for (int i = 0; i < 500; ++i) {
        float s1 = r.f(0.5f, 2), s2 = r.f(0.5f, 2);
        Transform a{r.v(), r.q(), Vec3(s1)}, b{r.v(), r.q(), Vec3(s2)};
        Vec3 p = r.v();
        CHECK(approx((a * b).apply(p), a.apply(b.apply(p)), 1e-3f));
        CHECK(approx(a.inverse().apply(a.apply(p)), p, 1e-3f));
        CHECK(approx(a.to_matrix().transform_point(p), a.apply(p), 1e-3f));
        CHECK(approx((a * b).to_matrix(), a.to_matrix() * b.to_matrix(), 2e-3f));
    }
}

TEST_CASE("aabb operations") {
    AABB a({0, 0, 0}, {2, 2, 2});
    AABB b({1, 1, 1}, {3, 3, 3});
    AABB c({5, 5, 5}, {6, 6, 6});
    CHECK(a.overlaps(b));
    CHECK_FALSE(a.overlaps(c));
    CHECK(merge(a, c).contains(a));
    CHECK(merge(a, c).contains(c));
    CHECK(a.surface_area() == doctest::Approx(24));
    CHECK(a.volume() == doctest::Approx(8));
    CHECK(a.center() == Vec3(1, 1, 1));
    CHECK_FALSE(AABB().valid());
    AABB e;
    e.expand(Vec3(1, 2, 3));
    CHECK(e.valid());
    CHECK(e.min == e.max);
    // transformed AABB contains all transformed corners
    Rng r(6);
    for (int i = 0; i < 200; ++i) {
        Mat4 m = r.affine();
        AABB t = a.transformed(m).grown(1e-3f);
        for (int k = 0; k < 8; ++k) {
            Vec3 corner{(k & 1) ? 2.f : 0.f, (k & 2) ? 2.f : 0.f, (k & 4) ? 2.f : 0.f};
            CHECK(t.contains(m.transform_point(corner)));
        }
    }
}

TEST_CASE("ray intersections") {
    Ray r{{0, 0, -5}, {0, 0, 1}};
    auto ts = intersect_ray_sphere(r, {{0, 0, 0}, 1});
    REQUIRE(ts);
    CHECK(*ts == doctest::Approx(4));
    CHECK_FALSE(intersect_ray_sphere({{0, 2, -5}, {0, 0, 1}}, {{0, 0, 0}, 1}));
    CHECK(*intersect_ray_sphere({{0, 0, 0}, {0, 0, 1}}, {{0, 0, 0}, 1}) == 0.0f);  // inside
    CHECK_FALSE(intersect_ray_sphere({{0, 0, 5}, {0, 0, 1}}, {{0, 0, 0}, 1}));     // behind

    auto tb = intersect_ray_aabb(r, AABB({-1, -1, -1}, {1, 1, 1}));
    REQUIRE(tb);
    CHECK(*tb == doctest::Approx(4));
    CHECK_FALSE(intersect_ray_aabb({{0, 3, -5}, {0, 0, 1}}, AABB({-1, -1, -1}, {1, 1, 1})));
    CHECK_FALSE(intersect_ray_aabb(r, AABB({-1, -1, -1}, {1, 1, 1}), 3.0f));  // beyond tmax
    // axis-parallel ray with zero direction components
    CHECK(intersect_ray_aabb({{0.5f, 0.5f, -5}, {0, 0, 1}}, AABB({0, 0, 0}, {1, 1, 1})));

    auto tp = intersect_ray_plane(r, Plane::from_point_normal({0, 0, 2}, {0, 0, -1}));
    REQUIRE(tp);
    CHECK(*tp == doctest::Approx(7));
    CHECK_FALSE(intersect_ray_plane({{0, 0, 0}, {1, 0, 0}}, Plane::from_point_normal({0, 0, 2}, {0, 0, 1})));

    auto tt = intersect_ray_triangle(r, {-1, -1, 0}, {1, -1, 0}, {0, 1, 0});
    REQUIRE(tt);
    CHECK(tt->t == doctest::Approx(5));
    CHECK_FALSE(intersect_ray_triangle({{5, 5, -5}, {0, 0, 1}}, {-1, -1, 0}, {1, -1, 0}, {0, 1, 0}));
}

TEST_CASE("property: ray-triangle hit point lies on the triangle and matches barycentrics") {
    Rng r(7);
    int hits = 0;
    for (int i = 0; i < 2000; ++i) {
        Vec3 a = r.v(), b = r.v(), c = r.v();
        Vec3 o = r.v(-20, 20);
        float u = r.f(0, 1), v = r.f(0, 1);
        if (u + v > 1) { u = 1 - u; v = 1 - v; }
        Vec3 target = a + (b - a) * u + (c - a) * v;
        if (length(cross(b - a, c - a)) < 1.0f || length(target - o) < 0.5f) continue;
        auto h = intersect_ray_triangle({o, target - o}, a, b, c);
        REQUIRE(h);
        ++hits;
        CHECK(h->t == doctest::Approx(1.0f).epsilon(1e-2));
        CHECK(h->u == doctest::Approx(u).epsilon(1e-2).scale(1));
        CHECK(h->v == doctest::Approx(v).epsilon(1e-2).scale(1));
    }
    CHECK(hits > 1000);
}

TEST_CASE("overlap tests") {
    CHECK(overlap_sphere_aabb({{2, 0, 0}, 1.01f}, AABB({-1, -1, -1}, {1, 1, 1})));
    CHECK_FALSE(overlap_sphere_aabb({{2, 2, 0}, 1.0f}, AABB({-1, -1, -1}, {1, 1, 1})));  // corner gap
    CHECK(overlap_sphere_sphere({{0, 0, 0}, 1}, {{1.9f, 0, 0}, 1}));
    CHECK_FALSE(overlap_sphere_sphere({{0, 0, 0}, 1}, {{2.1f, 0, 0}, 1}));
    CHECK(closest_point_aabb(AABB({0, 0, 0}, {1, 1, 1}), {2, 0.5f, -1}) == Vec3(1, 0.5f, 0));
    Plane p = Plane::from_points({0, 0, 0}, {1, 0, 0}, {0, 0, -1});
    CHECK(approx(p.n, Vec3(0, 1, 0)));
    CHECK(p.distance({0, 3, 0}) == doctest::Approx(3));
}

TEST_CASE("frustum culling") {
    Mat4 proj = Mat4::perspective(radians(60), 1.0f, 0.1f, 100.0f);
    Mat4 view = Mat4::look_at({0, 0, 0}, {0, 0, -1}, {0, 1, 0});
    Frustum f = Frustum::from_matrix(proj * view);
    CHECK(f.classify(AABB::from_center_extents({0, 0, -10}, Vec3(1))) == Frustum::Result::Inside);
    CHECK(f.classify(AABB::from_center_extents({0, 0, 10}, Vec3(1))) == Frustum::Result::Outside);   // behind
    CHECK(f.classify(AABB::from_center_extents({0, 0, -200}, Vec3(1))) == Frustum::Result::Outside); // too far
    CHECK(f.classify(AABB::from_center_extents({50, 0, -10}, Vec3(1))) == Frustum::Result::Outside); // right
    CHECK(f.classify(AABB::from_center_extents({0, 0, -0.1f}, Vec3(0.5f))) == Frustum::Result::Intersect);
    // property: frustum.contains(p) agrees with the NDC cube test
    Rng r(8);
    Mat4 vp = proj * view;
    int agree = 0, total = 0;
    for (int i = 0; i < 5000; ++i) {
        Vec3 p = r.v(-30, 30);
        Vec4 c = vp * Vec4(p, 1);
        bool in_ndc = c.w > 0 && std::fabs(c.x) <= c.w && std::fabs(c.y) <= c.w && std::fabs(c.z) <= c.w;
        // skip points within float noise of a plane
        float margin = std::min({c.w - std::fabs(c.x), c.w - std::fabs(c.y), c.w - std::fabs(c.z)});
        if (std::fabs(margin) < 1e-3f) continue;
        ++total;
        agree += (f.contains(p) == in_ndc);
        // a point inside means its tiny box can't be classified Outside
        if (in_ndc) CHECK(f.visible(AABB::from_center_extents(p, Vec3(0.01f))));
    }
    CHECK(agree == total);
}
