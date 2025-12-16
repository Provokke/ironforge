#include "render/mesh.hpp"

namespace ifg {

AABB Mesh::bounds() const {
    AABB b;
    for (const Vertex& v : vertices) b.expand(v.position);
    return b;
}

Mesh Mesh::cube(Vec3 color) {
    Mesh m;
    const Vec3 normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const Vec3& n : normals) {
        // build a face basis (u, v) with u x v = n so the quad winds CCW seen from outside
        Vec3 u = std::fabs(n.y) > 0.5f ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
        Vec3 v = cross(n, u);
        u = cross(v, n);
        auto base = static_cast<std::uint32_t>(m.vertices.size());
        const float s[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
        for (auto& c : s) m.vertices.push_back({(n + u * c[0] + v * c[1]) * 0.5f, n, color});
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return m;
}

Mesh Mesh::checker_plane(float size, int cells, Vec3 a, Vec3 b) {
    Mesh m;
    float cell = size / static_cast<float>(cells);
    float o = -size * 0.5f;
    for (int z = 0; z < cells; ++z)
        for (int x = 0; x < cells; ++x) {
            Vec3 col = ((x + z) & 1) ? a : b;
            auto base = static_cast<std::uint32_t>(m.vertices.size());
            float x0 = o + x * cell, z0 = o + z * cell;
            m.vertices.push_back({{x0, 0, z0}, {0, 1, 0}, col});
            m.vertices.push_back({{x0, 0, z0 + cell}, {0, 1, 0}, col});
            m.vertices.push_back({{x0 + cell, 0, z0 + cell}, {0, 1, 0}, col});
            m.vertices.push_back({{x0 + cell, 0, z0}, {0, 1, 0}, col});
            m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        }
    return m;
}

Mesh Mesh::uv_sphere(int stacks, int slices, Vec3 color) {
    Mesh m;
    for (int i = 0; i <= stacks; ++i) {
        float phi = kPi * static_cast<float>(i) / static_cast<float>(stacks);
        for (int j = 0; j <= slices; ++j) {
            float th = 2 * kPi * static_cast<float>(j) / static_cast<float>(slices);
            Vec3 n{std::sin(phi) * std::cos(th), std::cos(phi), std::sin(phi) * std::sin(th)};
            m.vertices.push_back({n * 0.5f, n, color});
        }
    }
    auto idx = [&](int i, int j) { return static_cast<std::uint32_t>(i * (slices + 1) + j); };
    for (int i = 0; i < stacks; ++i)
        for (int j = 0; j < slices; ++j) {
            std::uint32_t a = idx(i, j), b = idx(i + 1, j), c = idx(i + 1, j + 1), d = idx(i, j + 1);
            if (i != 0) m.indices.insert(m.indices.end(), {a, d, b});
            if (i != stacks - 1) m.indices.insert(m.indices.end(), {b, d, c});
        }
    return m;
}

}  // namespace ifg
