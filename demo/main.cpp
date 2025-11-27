// ironforge_demo [out.png] [threads] [width] [height] [ticks]
// Runs demo/scripts/demo.fs for N fixed ticks and renders the scene to PNG.
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "engine/demo_scene.hpp"
#include "jobs/job_system.hpp"

using namespace ifg;

int main(int argc, char** argv) {
    std::string out = argc > 1 ? argv[1] : "demo.png";
    unsigned threads = argc > 2 ? static_cast<unsigned>(std::atoi(argv[2])) : 4;
    int w = argc > 3 ? std::atoi(argv[3]) : 960;
    int h = argc > 4 ? std::atoi(argv[4]) : 540;
    int ticks = argc > 5 ? std::atoi(argv[5]) : 90;

    DemoScene demo;
    std::string src = read_text_file(std::string(IFG_SOURCE_DIR) + "/demo/scripts/demo.fs");
    if (!demo.load(src) || !demo.simulate(ticks)) {
        std::cerr << "script error:\n" << demo.error << "\n";
        return 1;
    }
    std::cout << demo.script_output.str();
    MeshLibrary lib;
    auto items = gather_draw_items(*demo.root, lib);
    JobSystem jobs(threads);
    Rasterizer rast(w, h);
    RenderSettings rs;
    RenderStats st = rast.render(items, demo.camera(w, h), rs, &jobs);
    if (!write_file(out, encode_png(rast.image()))) {
        std::cerr << "failed to write " << out << "\n";
        return 1;
    }
    std::printf("nodes=%zu draw_items=%zu tris_in=%zu tris_binned=%zu tile_entries=%zu\n", demo.root->subtree_size(),
                items.size(), st.triangles_in, st.triangles_binned, st.tile_entries);
    std::printf("sim ticks=%llu  render %dx%d threads=%u: vertex %.2f ms, setup %.2f ms, raster %.2f ms, total %.2f ms\n",
                static_cast<unsigned long long>(demo.loop.tick()), w, h, jobs.thread_count(), st.ms_vertex, st.ms_setup,
                st.ms_raster, st.ms_total);
    std::printf("image hash fnv1a64=%016llx -> %s\n",
                static_cast<unsigned long long>(fnv1a64(rast.image().rgb.data(), rast.image().rgb.size())), out.c_str());
    return 0;
}
