#include "math/math.hpp"

namespace ifg {

// ---------------------------------------------------------------- Mat3
Mat3 Mat3::transposed() const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.at(i, j) = at(j, i);
    return r;
}

float Mat3::determinant() const { return dot(c[0], cross(c[1], c[2])); }

std::optional<Mat3> Mat3::inverse() const {
    float det = determinant();
    if (std::fabs(det) < 1e-12f) return std::nullopt;
    // rows of the inverse are the cross products of columns / det
    Vec3 r0 = cross(c[1], c[2]) / det;
    Vec3 r1 = cross(c[2], c[0]) / det;
    Vec3 r2 = cross(c[0], c[1]) / det;
    Mat3 m;
    for (int j = 0; j < 3; ++j) {
        m.at(0, j) = r0[j];
        m.at(1, j) = r1[j];
        m.at(2, j) = r2[j];
    }
    return m;
}

// ---------------------------------------------------------------- Mat4
Mat4 Mat4::transposed() const {
    Mat4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) r.at(i, j) = at(j, i);
    return r;
}

float Mat4::determinant() const {
    // Laplace expansion along the first row using 3x3 minors.
    double det = 0;
    for (int col = 0; col < 4; ++col) {
        double minor[3][3];
        for (int i = 1; i < 4; ++i) {
            int cj = 0;
            for (int j = 0; j < 4; ++j) {
                if (j == col) continue;
                minor[i - 1][cj++] = at(i, j);
            }
        }
        double m = minor[0][0] * (minor[1][1] * minor[2][2] - minor[1][2] * minor[2][1]) -
                   minor[0][1] * (minor[1][0] * minor[2][2] - minor[1][2] * minor[2][0]) +
                   minor[0][2] * (minor[1][0] * minor[2][1] - minor[1][1] * minor[2][0]);
        det += ((col & 1) ? -1.0 : 1.0) * at(0, col) * m;
    }
    return static_cast<float>(det);
}

std::optional<Mat4> Mat4::inverse() const {
    // Gauss-Jordan elimination with partial pivoting, in double precision.
    double a[4][8];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            a[i][j] = at(i, j);
            a[i][j + 4] = (i == j) ? 1.0 : 0.0;
        }
    for (int col = 0; col < 4; ++col) {
        int piv = col;
        for (int r = col + 1; r < 4; ++r)
            if (std::fabs(a[r][col]) > std::fabs(a[piv][col])) piv = r;
        if (std::fabs(a[piv][col]) < 1e-12) return std::nullopt;
        if (piv != col)
            for (int j = 0; j < 8; ++j) std::swap(a[piv][j], a[col][j]);
        double inv = 1.0 / a[col][col];
        for (int j = 0; j < 8; ++j) a[col][j] *= inv;
        for (int r = 0; r < 4; ++r) {
            if (r == col) continue;
            double f = a[r][col];
            if (f == 0.0) continue;
            for (int j = 0; j < 8; ++j) a[r][j] -= f * a[col][j];
        }
    }
    Mat4 m;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) m.at(i, j) = static_cast<float>(a[i][j + 4]);
    return m;
}

Mat4 Mat4::translation(Vec3 t) {
    Mat4 m;
    m.c[3] = {t.x, t.y, t.z, 1};
    return m;
}
Mat4 Mat4::scale(Vec3 s) {
    Mat4 m;
    m.c[0].x = s.x;
    m.c[1].y = s.y;
    m.c[2].z = s.z;
    return m;
}
Mat4 Mat4::rotation_x(float a) {
    Mat4 m;
    float c = std::cos(a), s = std::sin(a);
    m.c[1] = {0, c, s, 0};
    m.c[2] = {0, -s, c, 0};
    return m;
}
Mat4 Mat4::rotation_y(float a) {
    Mat4 m;
    float c = std::cos(a), s = std::sin(a);
    m.c[0] = {c, 0, -s, 0};
    m.c[2] = {s, 0, c, 0};
    return m;
}
Mat4 Mat4::rotation_z(float a) {
    Mat4 m;
    float c = std::cos(a), s = std::sin(a);
    m.c[0] = {c, s, 0, 0};
    m.c[1] = {-s, c, 0, 0};
    return m;
}
Mat4 Mat4::perspective(float fovy, float aspect, float n, float f) {
    float t = 1.0f / std::tan(fovy * 0.5f);
    Mat4 m;
    m.c[0] = {t / aspect, 0, 0, 0};
    m.c[1] = {0, t, 0, 0};
    m.c[2] = {0, 0, (f + n) / (n - f), -1};
    m.c[3] = {0, 0, 2 * f * n / (n - f), 0};
    return m;
}
Mat4 Mat4::orthographic(float l, float r, float b, float t, float n, float f) {
    Mat4 m;
    m.c[0] = {2 / (r - l), 0, 0, 0};
    m.c[1] = {0, 2 / (t - b), 0, 0};
    m.c[2] = {0, 0, -2 / (f - n), 0};
    m.c[3] = {-(r + l) / (r - l), -(t + b) / (t - b), -(f + n) / (f - n), 1};
    return m;
}
Mat4 Mat4::look_at(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 f = normalize(target - eye);
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);
    Mat4 m;
    m.c[0] = {s.x, u.x, -f.x, 0};
    m.c[1] = {s.y, u.y, -f.y, 0};
    m.c[2] = {s.z, u.z, -f.z, 0};
    m.c[3] = {-dot(s, eye), -dot(u, eye), dot(f, eye), 1};
    return m;
}

bool approx(const Mat4& a, const Mat4& b, float eps) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::fabs(a.at(i, j) - b.at(i, j)) > eps) return false;
    return true;
}

// ---------------------------------------------------------------- Quat
Quat Quat::from_axis_angle(Vec3 axis, float angle) {
    Vec3 n = normalize(axis);
    float s = std::sin(angle * 0.5f);
    return {n.x * s, n.y * s, n.z * s, std::cos(angle * 0.5f)};
}

Quat Quat::from_euler(float pitch, float yaw, float roll) {
    // R = Ry(yaw) * Rx(pitch) * Rz(roll)
    return from_axis_angle({0, 1, 0}, yaw) * from_axis_angle({1, 0, 0}, pitch) *
           from_axis_angle({0, 0, 1}, roll);
}

Vec3 Quat::rotate(Vec3 v) const {
    Vec3 q{x, y, z};
    Vec3 t = cross(q, v) * 2.0f;
    return v + t * w + cross(q, t);
}

Mat3 Quat::to_mat3() const {
    Mat3 m;
    float xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z, wx = w * x, wy = w * y,
          wz = w * z;
    m.c[0] = {1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy)};
    m.c[1] = {2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx)};
    m.c[2] = {2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy)};
    return m;
}

Quat Quat::from_mat3(const Mat3& m) {
    float tr = m.at(0, 0) + m.at(1, 1) + m.at(2, 2);
    Quat q;
    if (tr > 0) {
        float s = std::sqrt(tr + 1.0f) * 2;
        q.w = 0.25f * s;
        q.x = (m.at(2, 1) - m.at(1, 2)) / s;
        q.y = (m.at(0, 2) - m.at(2, 0)) / s;
        q.z = (m.at(1, 0) - m.at(0, 1)) / s;
    } else if (m.at(0, 0) > m.at(1, 1) && m.at(0, 0) > m.at(2, 2)) {
        float s = std::sqrt(1.0f + m.at(0, 0) - m.at(1, 1) - m.at(2, 2)) * 2;
        q.w = (m.at(2, 1) - m.at(1, 2)) / s;
        q.x = 0.25f * s;
        q.y = (m.at(0, 1) + m.at(1, 0)) / s;
        q.z = (m.at(0, 2) + m.at(2, 0)) / s;
    } else if (m.at(1, 1) > m.at(2, 2)) {
        float s = std::sqrt(1.0f + m.at(1, 1) - m.at(0, 0) - m.at(2, 2)) * 2;
        q.w = (m.at(0, 2) - m.at(2, 0)) / s;
        q.x = (m.at(0, 1) + m.at(1, 0)) / s;
        q.y = 0.25f * s;
        q.z = (m.at(1, 2) + m.at(2, 1)) / s;
    } else {
        float s = std::sqrt(1.0f + m.at(2, 2) - m.at(0, 0) - m.at(1, 1)) * 2;
        q.w = (m.at(1, 0) - m.at(0, 1)) / s;
        q.x = (m.at(0, 2) + m.at(2, 0)) / s;
        q.y = (m.at(1, 2) + m.at(2, 1)) / s;
        q.z = 0.25f * s;
    }
    return q.normalized();
}

Quat slerp(Quat a, Quat b, float t) {
    float d = dot(a, b);
    if (d < 0) {
        b = {-b.x, -b.y, -b.z, -b.w};
        d = -d;
    }
    if (d > 0.9995f) {
        Quat r{lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t), lerp(a.w, b.w, t)};
        return r.normalized();
    }
    float th = std::acos(clampf(d, -1.0f, 1.0f));
    float s = std::sin(th);
    float wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
    return {a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb};
}

bool approx_rotation(Quat a, Quat b, float eps) { return std::fabs(std::fabs(dot(a, b)) - 1.0f) <= eps; }

// ---------------------------------------------------------------- Transform
Mat4 Transform::to_matrix() const {
    Mat3 r = rotation.to_mat3();
    Mat4 m;
    m.c[0] = Vec4(r.c[0] * scale.x, 0);
    m.c[1] = Vec4(r.c[1] * scale.y, 0);
    m.c[2] = Vec4(r.c[2] * scale.z, 0);
    m.c[3] = Vec4(position, 1);
    return m;
}

Transform Transform::operator*(const Transform& child) const {
    Transform t;
    t.position = apply(child.position);
    t.rotation = (rotation * child.rotation).normalized();
    t.scale = scale * child.scale;
    return t;
}

Transform Transform::inverse() const {
    Transform t;
    t.rotation = rotation.conjugate();
    t.scale = {1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z};
    t.position = t.scale * t.rotation.rotate(-position);
    return t;
}

// ---------------------------------------------------------------- AABB
AABB AABB::transformed(const Mat4& m) const {
    // Arvo's method: transform the center/extents with |M|.
    Vec3 c = center(), e = extents();
    Vec3 nc = m.transform_point(c);
    Vec3 ne;
    for (int i = 0; i < 3; ++i)
        ne[i] = std::fabs(m.at(i, 0)) * e.x + std::fabs(m.at(i, 1)) * e.y + std::fabs(m.at(i, 2)) * e.z;
    return {nc - ne, nc + ne};
}

// ---------------------------------------------------------------- intersections
std::optional<float> intersect_ray_plane(const Ray& r, const Plane& p) {
    float denom = dot(p.n, r.dir);
    if (std::fabs(denom) < kEpsilon) return std::nullopt;
    float t = -(dot(p.n, r.origin) + p.d) / denom;
    if (t < 0) return std::nullopt;
    return t;
}

std::optional<float> intersect_ray_sphere(const Ray& r, const Sphere& s) {
    Vec3 oc = r.origin - s.center;
    float a = dot(r.dir, r.dir);
    float b = dot(oc, r.dir);
    float c = dot(oc, oc) - s.radius * s.radius;
    float disc = b * b - a * c;
    if (disc < 0 || a == 0) return std::nullopt;
    float sq = std::sqrt(disc);
    float t0 = (-b - sq) / a, t1 = (-b + sq) / a;
    if (t0 >= 0) return t0;
    if (t1 >= 0) return 0.0f;  // origin inside the sphere
    return std::nullopt;
}

std::optional<float> intersect_ray_aabb(const Ray& r, const AABB& b, float tmax) {
    float tmin = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float o = r.origin[i], d = r.dir[i];
        if (std::fabs(d) < 1e-12f) {
            if (o < b.min[i] || o > b.max[i]) return std::nullopt;
            continue;
        }
        float inv = 1.0f / d;
        float t0 = (b.min[i] - o) * inv, t1 = (b.max[i] - o) * inv;
        if (t0 > t1) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmin > tmax) return std::nullopt;
    }
    return tmin;
}

std::optional<TriHit> intersect_ray_triangle(const Ray& r, Vec3 a, Vec3 b, Vec3 c) {
    // Moller-Trumbore
    Vec3 e1 = b - a, e2 = c - a;
    Vec3 p = cross(r.dir, e2);
    float det = dot(e1, p);
    if (std::fabs(det) < 1e-10f) return std::nullopt;
    float inv = 1.0f / det;
    Vec3 s = r.origin - a;
    float u = dot(s, p) * inv;
    if (u < 0 || u > 1) return std::nullopt;
    Vec3 q = cross(s, e1);
    float v = dot(r.dir, q) * inv;
    if (v < 0 || u + v > 1) return std::nullopt;
    float t = dot(e2, q) * inv;
    if (t < 0) return std::nullopt;
    return TriHit{t, u, v};
}

Vec3 closest_point_aabb(const AABB& b, Vec3 p) { return vmin(vmax(p, b.min), b.max); }

bool overlap_sphere_aabb(const Sphere& s, const AABB& b) {
    return length_sq(closest_point_aabb(b, s.center) - s.center) <= s.radius * s.radius;
}

bool overlap_sphere_sphere(const Sphere& a, const Sphere& b) {
    float r = a.radius + b.radius;
    return length_sq(a.center - b.center) <= r * r;
}

// ---------------------------------------------------------------- Frustum
Frustum Frustum::from_matrix(const Mat4& m) {
    auto row = [&](int i) { return Vec4(m.at(i, 0), m.at(i, 1), m.at(i, 2), m.at(i, 3)); };
    Vec4 r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
    Vec4 eq[6] = {r3 + r0, r3 - r0, r3 + r1, r3 - r1, r3 + r2, r3 - r2};
    Frustum f;
    for (int i = 0; i < 6; ++i) {
        float len = length(eq[i].xyz());
        f.planes[i].n = eq[i].xyz() / len;
        f.planes[i].d = eq[i].w / len;
    }
    return f;
}

Frustum::Result Frustum::classify(const AABB& b) const {
    Result res = Result::Inside;
    for (const Plane& p : planes) {
        Vec3 pos{p.n.x >= 0 ? b.max.x : b.min.x, p.n.y >= 0 ? b.max.y : b.min.y, p.n.z >= 0 ? b.max.z : b.min.z};
        Vec3 neg{p.n.x >= 0 ? b.min.x : b.max.x, p.n.y >= 0 ? b.min.y : b.max.y, p.n.z >= 0 ? b.min.z : b.max.z};
        if (p.distance(pos) < 0) return Result::Outside;
        if (p.distance(neg) < 0) res = Result::Intersect;
    }
    return res;
}

bool Frustum::contains(Vec3 pt) const {
    for (const Plane& p : planes)
        if (p.distance(pt) < 0) return false;
    return true;
}

}  // namespace ifg
