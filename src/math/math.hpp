// IronForge math: vectors, matrices, quaternions, transforms, bounds and
// intersection primitives. Column-major matrices, right-handed coordinates,
// column vectors (p' = M * p).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace ifg {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kEpsilon = 1e-6f;

inline float radians(float deg) { return deg * (kPi / 180.0f); }
inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
inline float clampf(float v, float lo, float hi) { return std::min(std::max(v, lo), hi); }
inline bool approx(float a, float b, float eps = 1e-4f) {
    return std::fabs(a - b) <= eps * std::max(1.0f, std::max(std::fabs(a), std::fabs(b)));
}

// ---------------------------------------------------------------- vectors
struct Vec2 {
    float x = 0, y = 0;
    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}
    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(float s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(float s) const { return {x / s, y / s}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr bool operator==(const Vec2&) const = default;
};
inline float dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline float length(Vec2 v) { return std::sqrt(dot(v, v)); }

struct Vec3 {
    float x = 0, y = 0, z = 0;
    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    constexpr explicit Vec3(float s) : x(s), y(s), z(s) {}
    constexpr Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(Vec3 o) const { return {x * o.x, y * o.y, z * o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(Vec3 o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(Vec3 o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    constexpr float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    constexpr bool operator==(const Vec3&) const = default;
};
inline constexpr Vec3 operator*(float s, Vec3 v) { return v * s; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length_sq(Vec3 v) { return dot(v, v); }
inline float length(Vec3 v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(Vec3 v) {
    float l = length(v);
    return l > 0 ? v / l : Vec3{};
}
inline Vec3 vmin(Vec3 a, Vec3 b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 vmax(Vec3 a, Vec3 b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }
inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }
inline bool approx(Vec3 a, Vec3 b, float eps = 1e-4f) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

struct Vec4 {
    float x = 0, y = 0, z = 0, w = 0;
    constexpr Vec4() = default;
    constexpr Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    constexpr Vec4(Vec3 v, float w_) : x(v.x), y(v.y), z(v.z), w(w_) {}
    constexpr Vec4 operator+(Vec4 o) const { return {x + o.x, y + o.y, z + o.z, w + o.w}; }
    constexpr Vec4 operator-(Vec4 o) const { return {x - o.x, y - o.y, z - o.z, w - o.w}; }
    constexpr Vec4 operator*(float s) const { return {x * s, y * s, z * s, w * s}; }
    constexpr Vec3 xyz() const { return {x, y, z}; }
    float operator[](int i) const { return i == 0 ? x : i == 1 ? y : i == 2 ? z : w; }
    float& operator[](int i) { return i == 0 ? x : i == 1 ? y : i == 2 ? z : w; }
    constexpr bool operator==(const Vec4&) const = default;
};
inline float dot(Vec4 a, Vec4 b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
inline Vec4 lerp(Vec4 a, Vec4 b, float t) { return a + (b - a) * t; }

// ---------------------------------------------------------------- matrices
struct Mat3 {
    Vec3 c[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // columns
    static Mat3 identity() { return {}; }
    float at(int row, int col) const { return c[col][row]; }
    float& at(int row, int col) { return c[col][row]; }
    Vec3 operator*(Vec3 v) const { return c[0] * v.x + c[1] * v.y + c[2] * v.z; }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) r.c[i] = (*this) * o.c[i];
        return r;
    }
    Mat3 transposed() const;
    float determinant() const;
    std::optional<Mat3> inverse() const;
};

struct Mat4 {
    Vec4 c[4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    static Mat4 identity() { return {}; }
    float at(int row, int col) const { return c[col][row]; }
    float& at(int row, int col) { return c[col][row]; }
    Vec4 operator*(Vec4 v) const { return c[0] * v.x + c[1] * v.y + c[2] * v.z + c[3] * v.w; }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for (int i = 0; i < 4; ++i) r.c[i] = (*this) * o.c[i];
        return r;
    }
    Vec3 transform_point(Vec3 p) const {
        Vec4 r = (*this) * Vec4(p, 1.0f);
        return r.xyz() / r.w;
    }
    Vec3 transform_dir(Vec3 d) const { return ((*this) * Vec4(d, 0.0f)).xyz(); }
    Mat4 transposed() const;
    float determinant() const;
    std::optional<Mat4> inverse() const;

    static Mat4 translation(Vec3 t);
    static Mat4 scale(Vec3 s);
    static Mat4 rotation_x(float a);
    static Mat4 rotation_y(float a);
    static Mat4 rotation_z(float a);
    static Mat4 perspective(float fovy, float aspect, float znear, float zfar);
    static Mat4 orthographic(float l, float r, float b, float t, float n, float f);
    static Mat4 look_at(Vec3 eye, Vec3 target, Vec3 up);
};
bool approx(const Mat4& a, const Mat4& b, float eps = 1e-4f);

// ---------------------------------------------------------------- quaternion
struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
    constexpr Quat() = default;
    constexpr Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    static Quat from_axis_angle(Vec3 axis, float angle);
    static Quat from_euler(float pitch, float yaw, float roll);  // X, Y, Z (applied Z*Y*X? see cpp)
    static Quat from_mat3(const Mat3& m);
    Quat operator*(const Quat& o) const {
        return {w * o.x + x * o.w + y * o.z - z * o.y, w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w, w * o.w - x * o.x - y * o.y - z * o.z};
    }
    Quat conjugate() const { return {-x, -y, -z, w}; }
    float norm() const { return std::sqrt(x * x + y * y + z * z + w * w); }
    Quat normalized() const {
        float n = norm();
        return {x / n, y / n, z / n, w / n};
    }
    Quat inverse() const {
        float n2 = x * x + y * y + z * z + w * w;
        return {-x / n2, -y / n2, -z / n2, w / n2};
    }
    Vec3 rotate(Vec3 v) const;
    Mat3 to_mat3() const;
};
inline float dot(Quat a, Quat b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
Quat slerp(Quat a, Quat b, float t);
bool approx_rotation(Quat a, Quat b, float eps = 1e-4f);  // q and -q are equal

// ---------------------------------------------------------------- transform
struct Transform {
    Vec3 position{};
    Quat rotation{};
    Vec3 scale{1, 1, 1};
    Mat4 to_matrix() const;
    Vec3 apply(Vec3 p) const { return position + rotation.rotate(scale * p); }
    Transform operator*(const Transform& child) const;  // parent * child (uniform-scale exact)
    Transform inverse() const;                          // exact for uniform scale
};

// ---------------------------------------------------------------- bounds & primitives
struct AABB {
    Vec3 min{std::numeric_limits<float>::max()};
    Vec3 max{-std::numeric_limits<float>::max()};
    constexpr AABB() = default;
    constexpr AABB(Vec3 mn, Vec3 mx) : min(mn), max(mx) {}
    static AABB from_center_extents(Vec3 c, Vec3 e) { return {c - e, c + e}; }
    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    Vec3 center() const { return (min + max) * 0.5f; }
    Vec3 extents() const { return (max - min) * 0.5f; }
    Vec3 size() const { return max - min; }
    float surface_area() const {
        Vec3 d = max - min;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
    float volume() const { Vec3 d = max - min; return d.x * d.y * d.z; }
    void expand(Vec3 p) { min = vmin(min, p); max = vmax(max, p); }
    void expand(const AABB& b) { min = vmin(min, b.min); max = vmax(max, b.max); }
    AABB grown(float m) const { return {min - Vec3(m), max + Vec3(m)}; }
    bool contains(Vec3 p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
    }
    bool contains(const AABB& b) const {
        return b.min.x >= min.x && b.min.y >= min.y && b.min.z >= min.z && b.max.x <= max.x &&
               b.max.y <= max.y && b.max.z <= max.z;
    }
    bool overlaps(const AABB& b) const {
        return min.x <= b.max.x && max.x >= b.min.x && min.y <= b.max.y && max.y >= b.min.y &&
               min.z <= b.max.z && max.z >= b.min.z;
    }
    AABB transformed(const Mat4& m) const;
};
inline AABB merge(const AABB& a, const AABB& b) { return {vmin(a.min, b.min), vmax(a.max, b.max)}; }

struct Ray {
    Vec3 origin;
    Vec3 dir;  // need not be normalized; t is in units of dir
    Vec3 at(float t) const { return origin + dir * t; }
};

struct Plane {
    Vec3 n{0, 1, 0};  // unit normal
    float d = 0;      // dot(n, p) + d = 0
    static Plane from_point_normal(Vec3 p, Vec3 n) { Vec3 nn = normalize(n); return {nn, -dot(nn, p)}; }
    static Plane from_points(Vec3 a, Vec3 b, Vec3 c) { return from_point_normal(a, cross(b - a, c - a)); }
    float distance(Vec3 p) const { return dot(n, p) + d; }
};

struct Sphere {
    Vec3 center;
    float radius = 0;
};

// Intersections. Ray functions return the smallest t >= 0 (in [tmin, tmax]).
std::optional<float> intersect_ray_plane(const Ray& r, const Plane& p);
std::optional<float> intersect_ray_sphere(const Ray& r, const Sphere& s);
std::optional<float> intersect_ray_aabb(const Ray& r, const AABB& b, float tmax = std::numeric_limits<float>::infinity());
struct TriHit { float t, u, v; };
std::optional<TriHit> intersect_ray_triangle(const Ray& r, Vec3 a, Vec3 b, Vec3 c);
bool overlap_sphere_aabb(const Sphere& s, const AABB& b);
bool overlap_sphere_sphere(const Sphere& a, const Sphere& b);
Vec3 closest_point_aabb(const AABB& b, Vec3 p);

// ---------------------------------------------------------------- frustum
struct Frustum {
    Plane planes[6];  // left, right, bottom, top, near, far; normals point inward
    static Frustum from_matrix(const Mat4& view_proj);
    enum class Result { Outside, Intersect, Inside };
    Result classify(const AABB& b) const;
    bool visible(const AABB& b) const { return classify(b) != Result::Outside; }
    bool contains(Vec3 p) const;
};

}  // namespace ifg
