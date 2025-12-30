#include "scene/node.hpp"

#include <algorithm>
#include <sstream>

namespace ifg {

Node* Node::add_child(std::unique_ptr<Node> child) {
    // keep sibling names unique, Godot-style: "Name", "Name2", "Name3"...
    std::string base = child->name_;
    int n = 1;
    auto taken = [&](const std::string& s) {
        return std::any_of(children_.begin(), children_.end(), [&](auto& c) { return c->name_ == s; });
    };
    while (taken(child->name_)) child->name_ = base + std::to_string(++n);
    child->parent_ = this;
    Node* raw = child.get();
    children_.push_back(std::move(child));
    child_added.emit(raw);
    return raw;
}

std::unique_ptr<Node> Node::remove_child(Node* child) {
    for (auto it = children_.begin(); it != children_.end(); ++it)
        if (it->get() == child) {
            std::unique_ptr<Node> out = std::move(*it);
            children_.erase(it);
            out->parent_ = nullptr;
            child_removed.emit(child);
            return out;
        }
    return nullptr;
}

Node* Node::find(const std::string& path) {
    if (path.empty()) return this;
    if (path[0] == '/') {
        Node* r = root();
        std::string rest = path.substr(1);
        auto slash = rest.find('/');
        std::string first = rest.substr(0, slash);
        if (first != r->name_) return nullptr;
        return slash == std::string::npos ? r : r->find(rest.substr(slash + 1));
    }
    Node* cur = this;
    std::stringstream ss(path);
    std::string part;
    while (cur && std::getline(ss, part, '/')) {
        if (part.empty() || part == ".") continue;
        if (part == "..") { cur = cur->parent_; continue; }
        Node* next = nullptr;
        for (auto& c : cur->children_)
            if (c->name_ == part) { next = c.get(); break; }
        cur = next;
    }
    return cur;
}

Node* Node::root() {
    Node* n = this;
    while (n->parent_) n = n->parent_;
    return n;
}

std::string Node::path() const {
    return parent_ ? parent_->path() + "/" + name_ : "/" + name_;
}

std::size_t Node::subtree_size() const {
    std::size_t s = 1;
    for (auto& c : children_) s += c->subtree_size();
    return s;
}

Transform Node::global_transform() const {
    return parent_ ? parent_->global_transform() * transform : transform;
}

void Node::propagate_process(float dt) {
    process(dt);
    processed.emit(dt);
    // index loop: children may be added during processing
    for (std::size_t i = 0; i < children_.size(); ++i) children_[i]->propagate_process(dt);
    for (std::size_t i = 0; i < children_.size();) {
        if (children_[i]->free_queued_) {
            Node* c = children_[i].get();
            std::unique_ptr<Node> dead = std::move(children_[i]);
            children_.erase(children_.begin() + static_cast<std::ptrdiff_t>(i));
            child_removed.emit(c);
        } else {
            ++i;
        }
    }
}

}  // namespace ifg
