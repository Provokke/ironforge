// The script-driven demo scene shared by the demo binary, the golden-image
// test and the benchmarks.
#pragma once
#include <memory>
#include <sstream>
#include <string>

#include "render/rasterizer.hpp"
#include "scene/game_loop.hpp"
#include "scene/node.hpp"
#include "script/vm.hpp"

namespace ifg {

class JobSystem;

struct MeshLibrary {
    Mesh cube = Mesh::cube({1, 1, 1});
    Mesh sphere = Mesh::uv_sphere(24, 48, {1, 1, 1});
    Mesh plane = Mesh::checker_plane(40.0f, 20, {0.85f, 0.85f, 0.8f}, {0.35f, 0.38f, 0.42f});
    const Mesh* get(const std::string& kind) const;
};

// Collects every visible MeshNode in the tree (pre-order) as a draw item.
std::vector<DrawItem> gather_draw_items(Node& root, const MeshLibrary& lib);

struct DemoScene {
    std::unique_ptr<Node> root;
    std::ostringstream script_output;
    std::unique_ptr<script::VM> vm;
    FixedStepLoop loop{1.0 / 60.0};
    std::string error;  // non-empty if the script failed

    // Loads the script source, binds the scene and runs the top level.
    bool load(const std::string& source);
    // Advances the simulation by `ticks` fixed steps (calls script `update(dt)`).
    bool simulate(int ticks);
    Camera camera(int width, int height) const;
};

std::string read_text_file(const std::string& path);

}  // namespace ifg
