#include "render/rasterizer.hpp"

#include <chrono>

#include "jobs/job_system.hpp"

namespace ifg {

namespace {

struct ClipVert {
    Vec4 clip;
    Vec3 normal;  // world space
    Vec3 color;
};

struct ScreenTri {
    Vec2 p[3];
    float z[3];
    float iw[3];
    Vec3 n_w[3];  // normal / w
    Vec3 c_w[3];  // color / w
    int x0, y0, x1, y1;  // inclusive pixel bbox
    bool top_left[3];    // per edge (opposite vertex i)
};

// Triangles per setup chunk: depends only on the triangle count (never on the
// thread count) and keeps the number of per-chunk tile bins bounded.
std::size_t chunk_size(std::size_t ntris) { return std::max<std::size_t>(256, (ntris + 63) / 64); }

using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

void run_parallel(JobSystem* jobs, std::size_t n, std::size_t grain, const std::function<void(std::size_t, std::size_t)>& fn) {
    if (jobs) jobs->parallel_for(0, n, grain, fn);
    else if (n) fn(0, n);
}

float edge(Vec2 a, Vec2 b, Vec2 p) { return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x); }

int clip_near(const ClipVert in[3], ClipVert out[4]) {
    int n = 0;
    for (int i = 0; i < 3; ++i) {
        const ClipVert& a = in[i];
        const ClipVert& b = in[(i + 1) % 3];
        float da = a.clip.z + a.clip.w, db = b.clip.z + b.clip.w;
        if (da >= 0) out[n++] = a;
        if ((da >= 0) != (db >= 0)) {
            float t = da / (da - db);
            out[n++] = {lerp(a.clip, b.clip, t), lerp(a.normal, b.normal, t), lerp(a.color, b.color, t)};
        }
    }
    return n;
}

}  // namespace

RenderStats Rasterizer::render(const std::vector<DrawItem>& items, const Camera& cam, const RenderSettings& rs, JobSystem* jobs) {
    RenderStats st;
    auto t_start = Clock::now();
    const Mat4 vp = cam.proj * cam.view;
    const Frustum frustum = Frustum::from_matrix(vp);

    // ---- vertex stage
    auto t0 = Clock::now();
    struct ItemRange { const DrawItem* item; std::size_t vbase; Mat4 mvp; Mat3 nmat; };
    std::vector<ItemRange> ranges;
    std::size_t total_verts = 0;
    for (const DrawItem& it : items) {
        if (rs.frustum_cull && !frustum.visible(it.mesh->bounds().transformed(it.model))) {
            ++st.objects_culled;
            continue;
        }
        Mat3 m3;
        for (int c = 0; c < 3; ++c) m3.c[c] = it.model.c[c].xyz();
        Mat3 nmat = m3.inverse().value_or(Mat3{}).transposed();
        ranges.push_back({&it, total_verts, vp * it.model, nmat});
        total_verts += it.mesh->vertices.size();
        st.triangles_in += it.mesh->triangle_count();
    }
    std::vector<ClipVert> verts(total_verts);
    for (const ItemRange& r : ranges) {
        const auto& src = r.item->mesh->vertices;
        run_parallel(jobs, src.size(), 1024, [&](std::size_t b, std::size_t e) {
            for (std::size_t i = b; i < e; ++i) {
                const Vertex& v = src[i];
                verts[r.vbase + i] = {r.mvp * Vec4(v.position, 1.0f), normalize(r.nmat * v.normal), v.color * r.item->tint};
            }
        });
    }
    // flattened triangle index list in submission order
    std::vector<std::uint32_t> tri_idx;
    tri_idx.reserve(st.triangles_in * 3);
    for (const ItemRange& r : ranges)
        for (std::uint32_t i : r.item->mesh->indices) tri_idx.push_back(static_cast<std::uint32_t>(r.vbase + i));
    st.ms_vertex = ms_since(t0);

    // ---- setup + binning
    t0 = Clock::now();
    const int ts = rs.tile_size;
    const int tiles_x = (width_ + ts - 1) / ts, tiles_y = (height_ + ts - 1) / ts;
    const std::size_t ntiles = static_cast<std::size_t>(tiles_x) * tiles_y;
    const std::size_t ntris = tri_idx.size() / 3;
    const std::size_t kChunk = chunk_size(ntris);
    const std::size_t nchunks = (ntris + kChunk - 1) / kChunk;
    std::vector<std::vector<ScreenTri>> chunk_tris(nchunks);
    std::vector<std::vector<std::vector<std::uint32_t>>> bins(nchunks, std::vector<std::vector<std::uint32_t>>(ntiles));
    const float W = static_cast<float>(width_), H = static_cast<float>(height_);

    run_parallel(jobs, nchunks, 1, [&](std::size_t cb, std::size_t ce) {
        for (std::size_t c = cb; c < ce; ++c) {
            auto& out = chunk_tris[c];
            auto& cbins = bins[c];
            std::size_t tb = c * kChunk, te = std::min(ntris, tb + kChunk);
            for (std::size_t t = tb; t < te; ++t) {
                ClipVert in[3] = {verts[tri_idx[t * 3]], verts[tri_idx[t * 3 + 1]], verts[tri_idx[t * 3 + 2]]};
                // trivial reject when all three are outside the same clip plane
                bool reject = false;
                for (int axis = 0; axis < 3 && !reject; ++axis) {
                    bool all_lo = true, all_hi = true;
                    for (auto& v : in) {
                        all_lo &= v.clip[axis] < -v.clip.w;
                        all_hi &= v.clip[axis] > v.clip.w;
                    }
                    reject = all_lo || all_hi;
                }
                if (reject) continue;
                ClipVert poly[4];
                int n = clip_near(in, poly);
                for (int k = 1; k + 1 < n; ++k) {
                    const ClipVert* tv[3] = {&poly[0], &poly[k], &poly[k + 1]};
                    ScreenTri s;
                    for (int i = 0; i < 3; ++i) {
                        float iw = 1.0f / tv[i]->clip.w;
                        s.p[i] = {(tv[i]->clip.x * iw * 0.5f + 0.5f) * W, (0.5f - tv[i]->clip.y * iw * 0.5f) * H};
                        s.z[i] = tv[i]->clip.z * iw;
                        s.iw[i] = iw;
                        s.n_w[i] = tv[i]->normal * iw;
                        s.c_w[i] = tv[i]->color * iw;
                    }
                    float area = edge(s.p[0], s.p[1], s.p[2]);
                    // CCW in NDC (y up) is negative area in y-down screen space
                    if (area == 0.0f || (rs.cull_backfaces && area > 0.0f)) continue;
                    if (area < 0.0f) {
                        std::swap(s.p[1], s.p[2]);
                        std::swap(s.z[1], s.z[2]);
                        std::swap(s.iw[1], s.iw[2]);
                        std::swap(s.n_w[1], s.n_w[2]);
                        std::swap(s.c_w[1], s.c_w[2]);
                    }
                    float minx = std::min({s.p[0].x, s.p[1].x, s.p[2].x}), maxx = std::max({s.p[0].x, s.p[1].x, s.p[2].x});
                    float miny = std::min({s.p[0].y, s.p[1].y, s.p[2].y}), maxy = std::max({s.p[0].y, s.p[1].y, s.p[2].y});
                    s.x0 = std::max(0, static_cast<int>(std::floor(minx)));
                    s.y0 = std::max(0, static_cast<int>(std::floor(miny)));
                    s.x1 = std::min(width_ - 1, static_cast<int>(std::ceil(maxx)));
                    s.y1 = std::min(height_ - 1, static_cast<int>(std::ceil(maxy)));
                    if (s.x0 > s.x1 || s.y0 > s.y1) continue;
                    for (int i = 0; i < 3; ++i) {  // edge i runs from vertex i+1 to i+2
                        Vec2 a = s.p[(i + 1) % 3], b = s.p[(i + 2) % 3];
                        float dy = b.y - a.y, dx = b.x - a.x;
                        s.top_left[i] = (dy == 0.0f && dx > 0.0f) || dy < 0.0f;
                    }
                    auto local = static_cast<std::uint32_t>(out.size());
                    out.push_back(s);
                    for (int ty = s.y0 / ts; ty <= s.y1 / ts; ++ty)
                        for (int tx = s.x0 / ts; tx <= s.x1 / ts; ++tx)
                            cbins[static_cast<std::size_t>(ty) * tiles_x + tx].push_back(local);
                }
            }
        }
    });
    for (std::size_t c = 0; c < nchunks; ++c) {
        st.triangles_binned += chunk_tris[c].size();
        for (auto& b : bins[c]) st.tile_entries += b.size();
    }
    st.ms_setup = ms_since(t0);

    // ---- raster: one job per tile (clear + draw)
    t0 = Clock::now();
    const Vec3 to_light = -normalize(rs.light_dir);
    run_parallel(jobs, ntiles, 1, [&](std::size_t tb, std::size_t te) {
        for (std::size_t tile = tb; tile < te; ++tile) {
            int tx0 = static_cast<int>(tile % tiles_x) * ts, ty0 = static_cast<int>(tile / tiles_x) * ts;
            int tx1 = std::min(width_, tx0 + ts) - 1, ty1 = std::min(height_, ty0 + ts) - 1;
            for (int y = ty0; y <= ty1; ++y) {
                Vec3 sky = lerp(rs.sky_top, rs.sky_bottom, (static_cast<float>(y) + 0.5f) / H);
                for (int x = tx0; x <= tx1; ++x) {
                    depth_[static_cast<std::size_t>(y) * width_ + x] = 1.0f;
                    std::uint8_t* px = color_.px(x, y);
                    px[0] = static_cast<std::uint8_t>(clampf(sky.x, 0, 1) * 255.0f + 0.5f);
                    px[1] = static_cast<std::uint8_t>(clampf(sky.y, 0, 1) * 255.0f + 0.5f);
                    px[2] = static_cast<std::uint8_t>(clampf(sky.z, 0, 1) * 255.0f + 0.5f);
                }
            }
            for (std::size_t c = 0; c < nchunks; ++c) {
                for (std::uint32_t local : bins[c][tile]) {
                    const ScreenTri& s = chunk_tris[c][local];
                    float area = edge(s.p[0], s.p[1], s.p[2]);
                    float inv_area = 1.0f / area;
                    int x0 = std::max(s.x0, tx0), x1 = std::min(s.x1, tx1);
                    int y0 = std::max(s.y0, ty0), y1 = std::min(s.y1, ty1);
                    for (int y = y0; y <= y1; ++y) {
                        for (int x = x0; x <= x1; ++x) {
                            Vec2 p{static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f};
                            float w[3] = {edge(s.p[1], s.p[2], p), edge(s.p[2], s.p[0], p), edge(s.p[0], s.p[1], p)};
                            bool inside = true;
                            for (int i = 0; i < 3; ++i)
                                inside &= w[i] > 0.0f || (w[i] == 0.0f && s.top_left[i]);
                            if (!inside) continue;
                            float b0 = w[0] * inv_area, b1 = w[1] * inv_area, b2 = w[2] * inv_area;
                            float z = b0 * s.z[0] + b1 * s.z[1] + b2 * s.z[2];
                            float& dz = depth_[static_cast<std::size_t>(y) * width_ + x];
                            if (!(z < dz) || z < -1.0f) continue;
                            dz = z;
                            float iw = b0 * s.iw[0] + b1 * s.iw[1] + b2 * s.iw[2];
                            float r = 1.0f / iw;
                            Vec3 n = normalize((s.n_w[0] * b0 + s.n_w[1] * b1 + s.n_w[2] * b2) * r);
                            Vec3 col = (s.c_w[0] * b0 + s.c_w[1] * b1 + s.c_w[2] * b2) * r;
                            float diff = std::max(0.0f, dot(n, to_light));
                            Vec3 lit = col * (rs.ambient + rs.light_color * diff);
                            std::uint8_t* px = color_.px(x, y);
                            for (int ch = 0; ch < 3; ++ch)
                                px[ch] = static_cast<std::uint8_t>(std::sqrt(clampf(lit[ch], 0, 1)) * 255.0f + 0.5f);
                        }
                    }
                }
            }
        }
    });
    st.ms_raster = ms_since(t0);
    st.ms_total = ms_since(t_start);
    return st;
}

}  // namespace ifg
