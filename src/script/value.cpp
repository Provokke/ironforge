#include "script/value.hpp"

#include <cmath>
#include <cstdio>

namespace ifg::script {

Value Value::string(std::string s) { return object(std::make_shared<StringObj>(std::move(s))); }

bool Value::is(ObjType t) const { return type == Type::Object && obj->type == t; }

int Chunk::add_constant(Value v) {
    constants.push_back(std::move(v));
    return static_cast<int>(constants.size()) - 1;
}

bool values_equal(const Value& a, const Value& b) {
    if (a.type != b.type) return false;
    switch (a.type) {
        case Value::Type::Nil: return true;
        case Value::Type::Bool: return a.b == b.b;
        case Value::Type::Number: return a.n == b.n;
        case Value::Type::Object:
            if (a.obj->type == ObjType::String && b.obj->type == ObjType::String)
                return static_cast<StringObj&>(*a.obj).str == static_cast<StringObj&>(*b.obj).str;
            return a.obj == b.obj;
    }
    return false;
}

static std::string number_to_string(double d) {
    if (std::isnan(d)) return "nan";
    if (std::isinf(d)) return d > 0 ? "inf" : "-inf";
    if (d == std::floor(d) && std::fabs(d) < 1e15) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.0f", d);
        return std::string(buf) == "-0" ? "0" : buf;
    }
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.14g", d);
    return buf;
}

std::string to_display(const Value& v) {
    switch (v.type) {
        case Value::Type::Nil: return "nil";
        case Value::Type::Bool: return v.b ? "true" : "false";
        case Value::Type::Number: return number_to_string(v.n);
        case Value::Type::Object: break;
    }
    switch (v.obj->type) {
        case ObjType::String: return static_cast<StringObj&>(*v.obj).str;
        case ObjType::Function: return "<fn " + static_cast<FunctionObj&>(*v.obj).name + ">";
        case ObjType::Closure: {
            auto& n = static_cast<ClosureObj&>(*v.obj).fn->name;
            return n.empty() ? "<script>" : "<fn " + n + ">";
        }
        case ObjType::Native: return "<native " + static_cast<NativeObj&>(*v.obj).name + ">";
        case ObjType::List: {
            std::string s = "[";
            auto& items = static_cast<ListObj&>(*v.obj).items;
            for (std::size_t i = 0; i < items.size(); ++i) {
                if (i) s += ", ";
                if (items[i].is(ObjType::String)) s += "\"" + to_display(items[i]) + "\"";
                else if (items[i].is(ObjType::List) && items[i].obj == v.obj) s += "[...]";
                else s += to_display(items[i]);
            }
            return s + "]";
        }
        case ObjType::Host: return "<" + static_cast<HostObject&>(*v.obj).host_type() + ">";
    }
    return "?";
}

std::string type_name(const Value& v) {
    switch (v.type) {
        case Value::Type::Nil: return "nil";
        case Value::Type::Bool: return "bool";
        case Value::Type::Number: return "number";
        case Value::Type::Object: break;
    }
    switch (v.obj->type) {
        case ObjType::String: return "string";
        case ObjType::Function:
        case ObjType::Closure:
        case ObjType::Native: return "function";
        case ObjType::List: return "list";
        case ObjType::Host: return static_cast<HostObject&>(*v.obj).host_type();
    }
    return "?";
}

}  // namespace ifg::script
