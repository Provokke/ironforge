#pragma once
#include <cstdint>
#include <vector>

#include "math/math.hpp"

namespace ifg {

struct Vertex {
    Vec3 position;
    Vec3 normal;
    Vec3 color;
};

// Indexed triangle list, counter-clockwise front faces.
struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    AABB bounds() const;
    std::size_t triangle_count() const { return indices.size() / 3; }

    static Mesh cube(Vec3 color);
    static Mesh checker_plane(float size, int cells, Vec3 a, Vec3 b);  // XZ plane, normal +Y
    static Mesh uv_sphere(int stacks, int slices, Vec3 color);
};

}  // namespace ifg
