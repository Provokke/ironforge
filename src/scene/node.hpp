// Scene tree: named nodes with local transforms, parent/child ownership,
// path lookup, signals and deferred deletion.
#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "math/math.hpp"
#include "scene/signal.hpp"

namespace ifg {

class Node {
public:
    explicit Node(std::string name = "Node") : name_(std::move(name)) {}
    virtual ~Node() = default;
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    const std::string& name() const { return name_; }
    void set_name(std::string n) { name_ = std::move(n); }
    Node* parent() const { return parent_; }
    const std::vector<std::unique_ptr<Node>>& children() const { return children_; }

    Node* add_child(std::unique_ptr<Node> child);
    template <class T, class... A>
    T* emplace_child(A&&... a) {
        return static_cast<T*>(add_child(std::make_unique<T>(std::forward<A>(a)...)));
    }
    std::unique_ptr<Node> remove_child(Node* child);
    Node* find(const std::string& path);  // "A/B/C", relative; ".." goes up
    Node* root();
    std::string path() const;             // absolute "/Root/A/B"
    std::size_t subtree_size() const;

    Transform transform;  // local
    Transform global_transform() const;

    // Runs process() depth-first, pre-order, then deletes queued nodes.
    void propagate_process(float dt);
    void queue_free() { free_queued_ = true; }
    bool is_queued_for_free() const { return free_queued_; }

    // User signals carrying a float payload, by name.
    Signal<float>::Id connect(const std::string& sig, Signal<float>::Slot s) { return user_signals_[sig].connect(std::move(s)); }
    bool disconnect(const std::string& sig, Signal<float>::Id id) { return user_signals_[sig].disconnect(id); }
    void emit(const std::string& sig, float v) {
        auto it = user_signals_.find(sig);
        if (it != user_signals_.end()) it->second.emit(v);
    }

    // Expires when the node is destroyed; lets script handles detect dangling nodes.
    std::weak_ptr<int> lifetime_token() const { return life_; }

    Signal<Node*> child_added;
    Signal<Node*> child_removed;
    Signal<float> processed;

protected:
    virtual void process(float /*dt*/) {}

private:
    std::string name_;
    Node* parent_ = nullptr;
    std::vector<std::unique_ptr<Node>> children_;
    std::unordered_map<std::string, Signal<float>> user_signals_;
    bool free_queued_ = false;
    std::shared_ptr<int> life_ = std::make_shared<int>(0);
};

// A node that the renderer draws: a mesh kind ("cube", "sphere", "plane") and a tint.
class MeshNode : public Node {
public:
    MeshNode(std::string node_name, std::string mesh_kind, Vec3 tint = {0.8f, 0.8f, 0.8f})
        : Node(std::move(node_name)), kind(std::move(mesh_kind)), color(tint) {}
    std::string kind;
    Vec3 color;
    bool visible = true;
};

}  // namespace ifg
