// ironforge_bench: BVH/octree vs brute force, rasterizer ms/frame by thread
// count, forge script VM throughput, allocator throughput.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <sstream>

#include "engine/demo_scene.hpp"
#include "jobs/job_system.hpp"
#include "memory/allocators.hpp"
#include "spatial/bvh.hpp"
#include "spatial/octree.hpp"

using namespace ifg;
using Clock = std::chrono::steady_clock;

static double ms(Clock::time_point a) { return std::chrono::duration<double, std::milli>(Clock::now() - a).count(); }

static void bench_spatial() {
    std::printf("== spatial queries (100k random boxes in a 1000^3 world) ==\n");
    std::mt19937 rng(7);
    auto f = [&](float a, float b) { return std::uniform_real_distribution<float>(a, b)(rng); };
    const int N = 100000;
    std::vector<AABB> boxes(N);
    for (auto& b : boxes) b = AABB::from_center_extents({f(-500, 500), f(-500, 500), f(-500, 500)}, Vec3(f(0.5f, 3)));
    auto t = Clock::now();
    DynamicBVH bvh(0.0f);
    for (int i = 0; i < N; ++i) bvh.insert(boxes[i], i);
    double build = ms(t);
    t = Clock::now();
    LooseOctree oct({0, 0, 0}, 512, 8);
    for (int i = 0; i < N; ++i) oct.insert(i, boxes[i]);
    double obuild = ms(t);
    std::printf("build: BVH %.1f ms (height %d), octree %.1f ms (%zu nodes)\n", build, bvh.height(), obuild, oct.node_count());

    const int Q = 2000;
    std::vector<AABB> qs(Q);
    for (auto& q : qs) q = AABB::from_center_extents({f(-500, 500), f(-500, 500), f(-500, 500)}, Vec3(20));
    long hits_b = 0, hits_v = 0, hits_o = 0;
    t = Clock::now();
    for (auto& q : qs)
        for (auto& b : boxes) hits_b += b.overlaps(q);
    double brute = ms(t);
    t = Clock::now();
    for (auto& q : qs) bvh.query(q, [&](int) { ++hits_v; return true; });
    double tb = ms(t);
    t = Clock::now();
    for (auto& q : qs) oct.query(q, [&](std::uint32_t id) { hits_o += boxes[id].overlaps(q); });
    double to = ms(t);
    std::printf("AABB query x%d: brute %.1f ms, BVH %.2f ms (%.0fx), octree %.2f ms (%.0fx)  [hits %ld/%ld/%ld]\n", Q, brute,
                tb, brute / tb, to, brute / to, hits_b, hits_v, hits_o);

    std::vector<Ray> rays(Q);
    for (auto& r : rays) r = {{f(-500, 500), f(-500, 500), f(-500, 500)}, normalize(Vec3(f(-1, 1), f(-1, 1), f(-1, 1)))};
    double sum_b = 0, sum_v = 0, sum_o = 0;
    t = Clock::now();
    for (auto& r : rays) {
        float best = 1e30f;
        for (auto& b : boxes)
            if (auto h = intersect_ray_aabb(r, b, best)) best = std::min(best, *h);
        sum_b += best < 1e30f ? best : 0;
    }
    double rbrute = ms(t);
    t = Clock::now();
    for (auto& r : rays) {
        float best = 1e30f;
        bvh.raycast(r, best, [&](int p, float tmax) {
            if (auto h = intersect_ray_aabb(r, boxes[bvh.user(p)], tmax)) best = std::min(best, *h);
            return best;
        });
        sum_v += best < 1e30f ? best : 0;
    }
    double rb = ms(t);
    t = Clock::now();
    for (auto& r : rays) {
        float best = 1e30f;
        oct.raycast(r, best, [&](std::uint32_t id, float tmax) {
            if (auto h = intersect_ray_aabb(r, boxes[id], tmax)) best = std::min(best, *h);
            return best;
        });
        sum_o += best < 1e30f ? best : 0;
    }
    double ro = ms(t);
    std::printf("closest-hit raycast x%d: brute %.1f ms, BVH %.2f ms (%.0fx), octree %.2f ms (%.0fx)  [checksum %.1f/%.1f/%.1f]\n",
                Q, rbrute, rb, rbrute / rb, ro, rbrute / ro, sum_b, sum_v, sum_o);
}

static void bench_raster(int w, int h) {
    std::printf("== rasterizer: demo scene %dx%d, median of 15 frames ==\n", w, h);
    DemoScene demo;
    demo.load(read_text_file(std::string(IFG_SOURCE_DIR) + "/demo/scripts/demo.fs"));
    demo.simulate(90);
    MeshLibrary lib;
    auto items = gather_draw_items(*demo.root, lib);
    // add a field of extra spheres so there is real triangle load
    std::vector<DrawItem> heavy = items;
    for (int i = 0; i < 64; ++i)
        heavy.push_back({&lib.sphere, Mat4::translation({float(i % 8) * 2.5f - 9, 0.5f, -float(i / 8) * 2.5f - 4}),
                         Vec3(0.4f + 0.07f * (i % 8), 0.5f, 0.9f - 0.08f * (i / 8))});
    for (auto* set : {&items, &heavy}) {
        std::size_t tris = 0;
        for (auto& it : *set) tris += it.mesh->triangle_count();
        double base = 0;
        for (unsigned threads : {0u, 1u, 2u, 4u}) {
            std::unique_ptr<JobSystem> js;
            if (threads) js = std::make_unique<JobSystem>(threads);
            Rasterizer r(w, h);
            std::vector<double> times;
            for (int i = 0; i < 15; ++i) times.push_back(r.render(*set, demo.camera(w, h), RenderSettings{}, js.get()).ms_total);
            std::sort(times.begin(), times.end());
            double med = times[times.size() / 2];
            if (threads == 0) base = med;
            std::printf("  %6zu tris, %s: %7.2f ms/frame (%.2fx vs inline)\n", tris,
                        threads ? (std::to_string(threads) + " worker(s)").c_str() : "inline (no pool)", med, base / med);
        }
    }
}

static void bench_vm() {
    std::printf("== forge script VM ==\n");
    struct Case { const char* name; const char* src; };
    Case cases[] = {
        {"fib(27) recursive", "fn fib(n) { if (n < 2) return n; return fib(n - 1) + fib(n - 2); } print fib(27);"},
        {"loop sum 3M", "let s = 0; for (let i = 0; i < 3000000; i = i + 1) { s = s + i; } print s;"},
        {"closure counter 1M",
         "fn mk() { let c = 0; return fn() { c = c + 1; return c; }; } let f = mk(); let x = 0;"
         " for (let i = 0; i < 1000000; i = i + 1) { x = f(); } print x;"},
        {"list push/index 1M", "let l = []; for (let i = 0; i < 1000000; i = i + 1) { l.push(i); } let s = 0;"
                               " for (let i = 0; i < len(l); i = i + 1) { s = s + l[i]; } print s;"},
    };
    for (auto& c : cases) {
        std::ostringstream out;
        script::VM vm(out);
        auto t = Clock::now();
        auto r = vm.run(c.src);
        double el = ms(t);
        std::string o = out.str();
        if (!o.empty() && o.back() == '\n') o.pop_back();
        std::printf("  %-22s %8.1f ms  %6.1f M instr  %6.1f M instr/s  -> %s%s\n", c.name, el,
                    vm.instructions_executed() / 1e6, vm.instructions_executed() / 1e3 / el, o.c_str(),
                    r.ok ? "" : r.error.c_str());
    }
}

static void bench_alloc() {
    std::printf("== allocators: 1M alloc/free pairs of 64 bytes ==\n");
    const int N = 1000000;
    std::vector<void*> ptrs(1000);
    auto run = [&](const char* name, auto&& alloc, auto&& dealloc) {
        auto t = Clock::now();
        for (int i = 0; i < N / 1000; ++i) {
            for (auto& p : ptrs) p = alloc();
            for (auto& p : ptrs) dealloc(p);
        }
        std::printf("  %-10s %7.2f ms\n", name, ms(t));
    };
    PoolAllocator pool(64, 1000);
    FreeListAllocator fl(1 << 20);
    run("malloc", [] { return std::malloc(64); }, [](void* p) { std::free(p); });
    run("pool", [&] { return pool.allocate(64); }, [&](void* p) { pool.deallocate(p); });
    run("freelist", [&] { return fl.allocate(64); }, [&](void* p) { fl.deallocate(p); });
    FrameAllocator fa(1000 * 64 + 64);
    auto t = Clock::now();
    for (int i = 0; i < N / 1000; ++i) {
        for (auto& p : ptrs) p = fa.allocate(64, 16);
        fa.reset();
    }
    std::printf("  %-10s %7.2f ms\n", "frame", ms(t));
}

int main(int argc, char** argv) {
    std::string which = argc > 1 ? argv[1] : "all";
    if (which == "all" || which == "spatial") bench_spatial();
    if (which == "all" || which == "raster") bench_raster(960, 540);
    if (which == "all" || which == "vm") bench_vm();
    if (which == "all" || which == "alloc") bench_alloc();
    return 0;
}
