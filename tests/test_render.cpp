#include <cstring>

#include "doctest/doctest.h"
#include "engine/demo_scene.hpp"
#include "jobs/job_system.hpp"
#include "render/image.hpp"
#include "render/rasterizer.hpp"

using namespace ifg;

namespace {
RenderSettings flat_settings() {
    RenderSettings rs;
    rs.ambient = {1, 1, 1};
    rs.light_color = {0, 0, 0};
    rs.sky_top = rs.sky_bottom = {0, 0, 0};
    rs.cull_backfaces = false;
    return rs;
}
Mesh quad(Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 color) {
    Mesh m;
    m.vertices = {{a, {0, 0, 1}, color}, {b, {0, 0, 1}, color}, {c, {0, 0, 1}, color}, {d, {0, 0, 1}, color}};
    m.indices = {0, 1, 2, 0, 2, 3};
    return m;
}
Camera ortho_cam(float w, float h) {
    return {Mat4::identity(), Mat4::orthographic(0, w, 0, h, -10, 10)};  // pixel units, y up
}
std::string golden_path() { return std::string(IFG_SOURCE_DIR) + "/tests/golden/demo_320x180.png"; }
}  // namespace

TEST_CASE("checksums match reference values") {
    const char* s = "123456789";
    CHECK(crc32(reinterpret_cast<const std::uint8_t*>(s), 9) == 0xCBF43926u);
    const char* w = "Wikipedia";
    CHECK(adler32(reinterpret_cast<const std::uint8_t*>(w), 9) == 0x11E60398u);
    CHECK(fnv1a64(nullptr, 0) == 0xcbf29ce484222325ull);
}

TEST_CASE("PNG encode/decode round trip, including multi-block IDAT") {
    for (auto [w, h] : {std::pair{1, 1}, std::pair{7, 3}, std::pair{300, 200}}) {
        Image img(w, h);
        for (std::size_t i = 0; i < img.rgb.size(); ++i) img.rgb[i] = static_cast<std::uint8_t>(i * 31 + 7);
        auto bytes = encode_png(img);
        CHECK(bytes[1] == 'P');
        auto back = decode_png(bytes);
        REQUIRE(back);
        CHECK(back->width == w);
        CHECK(back->height == h);
        CHECK(back->rgb == img.rgb);
        // corrupting a byte must be detected by the CRC
        bytes[bytes.size() / 2] ^= 0xFF;
        CHECK_FALSE(decode_png(bytes));
    }
    CHECK_FALSE(decode_png({1, 2, 3}));
}

TEST_CASE("two triangles sharing an edge cover a rectangle exactly (no gaps)") {
    Rasterizer r(64, 64);
    Mesh m = quad({8, 8, 0}, {40, 8, 0}, {40, 40, 0}, {8, 40, 0}, {1, 1, 1});
    r.render({{&m, Mat4::identity()}}, ortho_cam(64, 64), flat_settings(), nullptr);
    int covered = 0;
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x) covered += r.image().px(x, y)[0] == 255;
    CHECK(covered == 32 * 32);
    // y flip: world y=8..40 maps to rows 24..55
    CHECK(r.image().px(8, 24)[0] == 255);
    CHECK(r.image().px(8, 23)[0] == 0);
    CHECK(r.image().px(39, 55)[0] == 255);
    CHECK(r.image().px(40, 55)[0] == 0);
}

TEST_CASE("depth test keeps the nearest surface regardless of order") {
    Mesh nearq = quad({0, 0, 1}, {32, 0, 1}, {32, 32, 1}, {0, 32, 1}, {1, 0, 0});
    Mesh farq = quad({16, 16, -1}, {48, 16, -1}, {48, 48, -1}, {16, 48, -1}, {0, 0, 1});
    for (int order = 0; order < 2; ++order) {
        Rasterizer r(64, 64);
        std::vector<DrawItem> items = {{&nearq, Mat4::identity()}, {&farq, Mat4::identity()}};
        if (order) std::swap(items[0], items[1]);
        r.render(items, ortho_cam(64, 64), flat_settings(), nullptr);
        const std::uint8_t* overlap = r.image().px(20, 40);  // world (20, 23)
        CHECK(overlap[0] == 255);
        CHECK(overlap[2] == 0);
        CHECK(r.image().px(40, 20)[2] == 255);  // far-only region
    }
}

TEST_CASE("attributes are interpolated perspective-correctly") {
    // A floor quad receding from z=-1 to z=-21 with color.r going 0 -> 1 along depth.
    Mesh m;
    m.vertices = {{{-5, -1, -1}, {0, 1, 0}, {0, 0, 0}},
                  {{5, -1, -1}, {0, 1, 0}, {0, 0, 0}},
                  {{5, -1, -21}, {0, 1, 0}, {1, 0, 0}},
                  {{-5, -1, -21}, {0, 1, 0}, {1, 0, 0}}};
    m.indices = {0, 1, 2, 0, 2, 3};
    const int W = 64, H = 64;
    Camera cam{Mat4::identity(), Mat4::perspective(radians(90), 1.0f, 0.1f, 100.0f)};
    Rasterizer r(W, H);
    r.render({{&m, Mat4::identity()}}, cam, flat_settings(), nullptr);
    int checked = 0;
    for (int y = 40; y < H; y += 3) {
        int x = W / 2;
        // ray through the pixel center
        float ndc_y = 1.0f - 2.0f * (y + 0.5f) / H;
        float ndc_x = 2.0f * (x + 0.5f) / W - 1.0f;
        Vec3 dir{ndc_x, ndc_y, -1.0f};  // tan(45deg) = 1
        float t = -1.0f / dir.y;
        float z = dir.z * t;
        if (z > -1 || z < -21) continue;
        float expect_r = (-z - 1.0f) / 20.0f;
        float expect = std::sqrt(expect_r) * 255.0f;
        CHECK(std::fabs(float(r.image().px(x, y)[0]) - expect) <= 2.0f);
        ++checked;
    }
    CHECK(checked >= 5);
    // affine (non-perspective) interpolation would put r=0.5 at the screen midpoint of the
    // projected quad; perspective-correct puts it much closer to the horizon.
}

TEST_CASE("near-plane clipping: geometry through the camera still renders") {
    Mesh m;
    m.vertices = {{{-1, -0.5f, 5}, {0, 0, 1}, {1, 1, 1}}, {{1, -0.5f, 5}, {0, 0, 1}, {1, 1, 1}}, {{0, -0.5f, -5}, {0, 0, 1}, {1, 1, 1}}};
    m.indices = {0, 1, 2};
    Camera cam{Mat4::identity(), Mat4::perspective(radians(90), 1.0f, 0.1f, 100.0f)};
    Rasterizer r(32, 32);
    auto st = r.render({{&m, Mat4::identity()}}, cam, flat_settings(), nullptr);
    CHECK(st.triangles_binned >= 1);
    int lit = 0;
    for (int y = 0; y < 32; ++y)
        for (int x = 0; x < 32; ++x) lit += r.image().px(x, y)[0] > 0;
    CHECK(lit > 20);
    for (int x = 0; x < 32; ++x) CHECK(r.image().px(x, 0)[0] == 0);  // nothing above the horizon
}

TEST_CASE("backface culling and frustum culling") {
    Mesh cube = Mesh::cube({1, 1, 1});
    Camera cam{Mat4::look_at({3, 2, 4}, {0, 0, 0}, {0, 1, 0}), Mat4::perspective(radians(60), 1.0f, 0.1f, 50.0f)};
    Rasterizer r(64, 64);
    RenderSettings rs;
    auto st = r.render({{&cube, Mat4::identity()}}, cam, rs, nullptr);
    CHECK(st.triangles_in == 12);
    CHECK(st.triangles_binned == 6);  // three faces visible from a corner view
    auto behind = r.render({{&cube, Mat4::translation({40, 0, -10})}}, cam, rs, nullptr);
    CHECK(behind.objects_culled == 1);
    CHECK(behind.triangles_in == 0);
}

TEST_CASE("rendering is bit-identical for any thread count") {
    DemoScene demo;
    REQUIRE(demo.load(read_text_file(std::string(IFG_SOURCE_DIR) + "/demo/scripts/demo.fs")));
    REQUIRE(demo.simulate(90));
    MeshLibrary lib;
    auto items = gather_draw_items(*demo.root, lib);
    Rasterizer ref(320, 180);
    ref.render(items, demo.camera(320, 180), RenderSettings{}, nullptr);
    std::uint64_t h0 = fnv1a64(ref.image().rgb.data(), ref.image().rgb.size());
    for (unsigned t : {1u, 2u, 3u, 4u, 8u}) {
        JobSystem js(t);
        Rasterizer r(320, 180);
        r.render(items, demo.camera(320, 180), RenderSettings{}, &js);
        CHECK(fnv1a64(r.image().rgb.data(), r.image().rgb.size()) == h0);
    }
}

TEST_CASE("golden image: script-driven demo scene matches the committed reference") {
    DemoScene demo;
    REQUIRE(demo.load(read_text_file(std::string(IFG_SOURCE_DIR) + "/demo/scripts/demo.fs")));
    REQUIRE(demo.simulate(90));
    MeshLibrary lib;
    JobSystem js(4);
    Rasterizer r(320, 180);
    r.render(gather_draw_items(*demo.root, lib), demo.camera(320, 180), RenderSettings{}, &js);
    auto bytes = read_file(golden_path());
    REQUIRE_MESSAGE(bytes, "missing golden image " << golden_path());
    auto golden = decode_png(*bytes);
    REQUIRE(golden);
    ImageDiff d = compare_images(r.image(), *golden, 8);
    MESSAGE("golden diff: differing=" << d.differing_pixels << " max_delta=" << d.max_delta << " mae=" << d.mean_abs_error);
    // tolerance: allow 0.5% of pixels to differ (different compilers / FP contraction)
    CHECK(d.differing_pixels <= 320 * 180 / 200);
    CHECK(d.mean_abs_error < 1.0);
}
