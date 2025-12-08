#include "engine/demo_scene.hpp"

#include <fstream>

#include "script/bindings.hpp"

namespace ifg {

const Mesh* MeshLibrary::get(const std::string& kind) const {
    if (kind == "cube") return &cube;
    if (kind == "sphere") return &sphere;
    if (kind == "plane") return &plane;
    return nullptr;
}

static void gather(Node& n, const Mat4& parent, const MeshLibrary& lib, std::vector<DrawItem>& out) {
    Mat4 world = parent * n.transform.to_matrix();
    if (auto* mn = dynamic_cast<MeshNode*>(&n); mn && mn->visible) {
        if (const Mesh* m = lib.get(mn->kind)) out.push_back({m, world, mn->color});
    }
    for (auto& c : n.children()) gather(*c, world, lib, out);
}

std::vector<DrawItem> gather_draw_items(Node& root, const MeshLibrary& lib) {
    std::vector<DrawItem> out;
    gather(root, Mat4::identity(), lib, out);
    return out;
}

bool DemoScene::load(const std::string& source) {
    root = std::make_unique<Node>("World");
    root->add_child(std::make_unique<MeshNode>("Ground", "plane", Vec3(1, 1, 1)));
    vm = std::make_unique<script::VM>(script_output);
    script::bind_scene(*vm, root.get());
    auto r = vm->run(source);
    if (!r.ok) error = r.error;
    return r.ok;
}

bool DemoScene::simulate(int ticks) {
    bool ok = true;
    loop.on_fixed_update = [&](double step, std::uint64_t) {
        if (!ok) return;
        script::Value dt = script::Value::number(step);
        if (vm->get_global("update")) {
            auto r = vm->call_global("update", std::span<const script::Value>(&dt, 1));
            if (!r.ok) {
                error = r.error;
                ok = false;
                return;
            }
        }
        root->propagate_process(static_cast<float>(step));
    };
    for (int i = 0; i < ticks && ok; ++i) loop.advance(loop.step());
    return ok;
}

Camera DemoScene::camera(int width, int height) const {
    Camera c;
    c.view = Mat4::look_at({0, 5.5f, 13.0f}, {0, 1.0f, 0}, {0, 1, 0});
    c.proj = Mat4::perspective(radians(55.0f), float(width) / float(height), 0.1f, 100.0f);
    return c;
}

std::string read_text_file(const std::string& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

}  // namespace ifg
