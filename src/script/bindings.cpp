#include "script/bindings.hpp"

namespace ifg::script {

namespace {

class NodeHandle final : public HostObject {
public:
    explicit NodeHandle(Node* n) : node_(n), life_(n->lifetime_token()) {}
    std::string host_type() const override { return "Node"; }
    Node* get_node() const { return life_.expired() ? nullptr : node_; }

    std::optional<Value> get(const std::string& k) override {
        Node* n = get_node();
        if (k == "alive") return Value::boolean(n != nullptr);
        if (!n) return std::nullopt;
        const Vec3& p = n->transform.position;
        if (k == "name") return Value::string(n->name());
        if (k == "x") return Value::number(p.x);
        if (k == "y") return Value::number(p.y);
        if (k == "z") return Value::number(p.z);
        if (k == "scale") return Value::number(n->transform.scale.x);
        if (k == "path") return Value::string(n->path());
        if (k == "child_count") return Value::number(static_cast<double>(n->children().size()));
        if (k == "parent") return n->parent() ? wrap_node(n->parent()) : Value::nil();
        return std::nullopt;
    }

    bool set(const std::string& k, const Value& v) override {
        Node* n = get_node();
        if (!n) return false;
        if (k == "name" && v.is(ObjType::String)) { n->set_name(static_cast<StringObj&>(*v.obj).str); return true; }
        if (!v.is_number()) return false;
        float f = static_cast<float>(v.n);
        if (k == "x") n->transform.position.x = f;
        else if (k == "y") n->transform.position.y = f;
        else if (k == "z") n->transform.position.z = f;
        else if (k == "scale") n->transform.scale = Vec3(f);
        else return false;
        return true;
    }

    std::optional<Value> invoke(VM& vm, const std::string& m, std::span<Value> a) override {
        Node* n = get_node();
        if (!n) vm.runtime_error("node used after it was freed (method '" + m + "')");
        auto num = [&](std::size_t i) {
            if (!a[i].is_number()) vm.runtime_error(m + "() argument " + std::to_string(i + 1) + " must be a number");
            return static_cast<float>(a[i].n);
        };
        auto str = [&](std::size_t i) -> const std::string& {
            if (!a[i].is(ObjType::String)) vm.runtime_error(m + "() argument " + std::to_string(i + 1) + " must be a string");
            return static_cast<StringObj&>(*a[i].obj).str;
        };
        const std::size_t argc = a.size();
        if (m == "find" && argc == 1) {
            Node* f = n->find(str(0));
            return f ? wrap_node(f) : Value::nil();
        }
        if (m == "get_child" && argc == 1) {
            float i = num(0);
            if (i < 0 || i >= static_cast<float>(n->children().size())) return Value::nil();
            return wrap_node(n->children()[static_cast<std::size_t>(i)].get());
        }
        if (m == "set_position" && argc == 3) {
            n->transform.position = {num(0), num(1), num(2)};
            return Value::nil();
        }
        if (m == "translate" && argc == 3) {
            n->transform.position += Vec3(num(0), num(1), num(2));
            return Value::nil();
        }
        if (m == "rotate" && argc == 4) {
            Vec3 axis{num(0), num(1), num(2)};
            if (length(axis) == 0) vm.runtime_error("rotate() axis must be non-zero");
            n->transform.rotation = (Quat::from_axis_angle(axis, num(3)) * n->transform.rotation).normalized();
            return Value::nil();
        }
        if (m == "yaw" && argc == 1) {
            n->transform.rotation = (Quat::from_axis_angle({0, 1, 0}, num(0)) * n->transform.rotation).normalized();
            return Value::nil();
        }
        if (m == "spawn" && argc == 2) {
            const std::string& kind = str(0);
            if (kind != "cube" && kind != "sphere") vm.runtime_error("spawn(): unknown mesh kind '" + kind + "'");
            return wrap_node(n->add_child(std::make_unique<MeshNode>(str(1), kind)));
        }
        if (m == "set_color" && argc == 3) {
            auto* mn = dynamic_cast<MeshNode*>(n);
            if (!mn) vm.runtime_error("set_color() requires a mesh node");
            mn->color = {num(0), num(1), num(2)};
            return Value::nil();
        }
        if (m == "queue_free" && argc == 0) {
            n->queue_free();
            return Value::nil();
        }
        if (m == "emit" && argc == 2) {
            n->emit(str(0), num(1));
            return Value::nil();
        }
        if (m == "connect" && argc == 2) {
            const std::string sig = str(0);
            Value fn = a[1];
            if (!fn.is(ObjType::Closure) && !fn.is(ObjType::Native)) vm.runtime_error("connect() expects a function");
            VM* vmp = &vm;
            auto slot = [vmp, fn, sig](float v) {
                Value arg = Value::number(v);
                RunResult r = vmp->call(fn, std::span<const Value>(&arg, 1));
                if (!r.ok) vmp->out() << "error in '" << sig << "' handler: " << r.error << "\n";
            };
            auto id = sig == "process" ? n->processed.connect(slot) : n->connect(sig, slot);
            return Value::number(static_cast<double>(id));
        }
        return std::nullopt;
    }

private:
    Node* node_;
    std::weak_ptr<int> life_;
};

}  // namespace

Value wrap_node(Node* node) { return Value::object(std::make_shared<NodeHandle>(node)); }

Node* unwrap_node(const Value& v) {
    if (!v.is(ObjType::Host)) return nullptr;
    auto* h = dynamic_cast<NodeHandle*>(v.obj.get());
    return h ? h->get_node() : nullptr;
}

void bind_scene(VM& vm, Node* root) {
    vm.set_global("scene", wrap_node(root));
    vm.define_native("is_node", 1, [](VM&, std::span<Value> a) { return Value::boolean(unwrap_node(a[0]) != nullptr); });
}

}  // namespace ifg::script
