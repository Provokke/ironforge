// Multi-threaded tile-binned software rasterizer.
//
//   draw list --> [vertex stage]   parallel over vertices: model->clip, normals->world
//             --> [setup + bin]    parallel over fixed-size triangle chunks: backface cull,
//                                  near-plane clip, viewport transform, bin to tiles
//             --> [raster]         parallel over screen tiles: walk bins in submission
//                                  order, edge functions (top-left rule), perspective-
//                                  correct attributes, depth test, Lambert shading
//
// The chunk size is independent of the thread count and each tile consumes
// triangles in submission order, so the output is bit-identical for any
// number of worker threads.
#pragma once
#include <cstdint>
#include <vector>

#include "math/math.hpp"
#include "render/image.hpp"
#include "render/mesh.hpp"

namespace ifg {

class JobSystem;

struct DrawItem {
    const Mesh* mesh;
    Mat4 model;
    Vec3 tint{1, 1, 1};
};

struct Camera {
    Mat4 view;
    Mat4 proj;
};

struct RenderSettings {
    int tile_size = 32;
    Vec3 light_dir = normalize(Vec3(-0.5f, -1.0f, -0.3f));  // direction light travels
    Vec3 light_color{1.0f, 0.96f, 0.9f};
    Vec3 ambient{0.18f, 0.2f, 0.26f};
    Vec3 sky_top{0.25f, 0.45f, 0.8f};
    Vec3 sky_bottom{0.8f, 0.86f, 0.95f};
    bool cull_backfaces = true;
    bool frustum_cull = true;
};

struct RenderStats {
    std::size_t triangles_in = 0, triangles_binned = 0, objects_culled = 0, tile_entries = 0;
    double ms_vertex = 0, ms_setup = 0, ms_raster = 0, ms_total = 0;
};

class Rasterizer {
public:
    Rasterizer(int width, int height) : width_(width), height_(height), color_(width, height),
                                        depth_(static_cast<std::size_t>(width) * height) {}
    // jobs may be null: everything then runs on the calling thread.
    RenderStats render(const std::vector<DrawItem>& items, const Camera& cam, const RenderSettings& rs, JobSystem* jobs);
    const Image& image() const { return color_; }
    float depth_at(int x, int y) const { return depth_[static_cast<std::size_t>(y) * width_ + x]; }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    int width_, height_;
    Image color_;
    std::vector<float> depth_;
};

}  // namespace ifg
