// Native bindings that expose scene nodes to forge script.
//
//   scene.find("Path/To/Node")      -> node or nil
//   n.name  n.x  n.y  n.z  n.scale  n.path  n.child_count  n.parent  n.alive
//   n.set_position(x, y, z)   n.translate(dx, dy, dz)
//   n.rotate(ax, ay, az, radians)   n.yaw(radians)   n.get_child(i)
//   n.spawn(kind, name)  -> new MeshNode child ("cube" | "sphere")
//   n.set_color(r, g, b)  (MeshNode only)   n.queue_free()
//   n.connect(signal, fn)   n.emit(signal, value)   ("process" = per-tick signal)
#pragma once
#include <functional>
#include <memory>

#include "scene/node.hpp"
#include "script/vm.hpp"

namespace ifg::script {

Value wrap_node(Node* node);
Node* unwrap_node(const Value& v);  // nullptr if not a live node handle
// Registers the global `scene` (the given root) and node helpers.
void bind_scene(VM& vm, Node* root);

}  // namespace ifg::script
