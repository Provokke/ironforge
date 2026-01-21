// Forge script runtime values. Heap objects are reference counted
// (std::shared_ptr); the VM breaks remaining reference cycles at teardown.
#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ifg::script {

struct Obj;
class VM;

enum class ObjType : std::uint8_t { String, Function, Closure, Native, List, Host };

struct Value {
    enum class Type : std::uint8_t { Nil, Bool, Number, Object };
    Type type = Type::Nil;
    bool b = false;
    double n = 0;
    std::shared_ptr<Obj> obj;

    Value() = default;
    static Value nil() { return {}; }
    static Value boolean(bool v) { Value r; r.type = Type::Bool; r.b = v; return r; }
    static Value number(double v) { Value r; r.type = Type::Number; r.n = v; return r; }
    static Value object(std::shared_ptr<Obj> o) { Value r; r.type = Type::Object; r.obj = std::move(o); return r; }
    static Value string(std::string s);

    bool is_nil() const { return type == Type::Nil; }
    bool is_bool() const { return type == Type::Bool; }
    bool is_number() const { return type == Type::Number; }
    bool is_obj() const { return type == Type::Object; }
    bool is(ObjType t) const;
    bool truthy() const { return !(type == Type::Nil || (type == Type::Bool && !b)); }
};

bool values_equal(const Value& a, const Value& b);
std::string to_display(const Value& v);  // as printed by `print`
std::string type_name(const Value& v);

struct Obj {
    explicit Obj(ObjType t) : type(t) {}
    virtual ~Obj() = default;
    ObjType type;
};

struct StringObj : Obj {
    explicit StringObj(std::string s) : Obj(ObjType::String), str(std::move(s)) {}
    std::string str;
};

struct Chunk {
    std::vector<std::uint8_t> code;
    std::vector<int> lines;
    std::vector<Value> constants;
    void write(std::uint8_t b, int line) { code.push_back(b); lines.push_back(line); }
    int add_constant(Value v);
};

struct FunctionObj : Obj {
    FunctionObj() : Obj(ObjType::Function) {}
    std::string name;
    int arity = 0;
    int upvalue_count = 0;
    Chunk chunk;
};

struct Upvalue {
    int slot = -1;  // stack slot while open
    bool open = true;
    Value closed;
};

struct ClosureObj : Obj {
    explicit ClosureObj(std::shared_ptr<FunctionObj> f) : Obj(ObjType::Closure), fn(std::move(f)) {}
    std::shared_ptr<FunctionObj> fn;
    std::vector<std::shared_ptr<Upvalue>> upvalues;
};

using NativeFn = std::function<Value(VM&, std::span<Value> args)>;
struct NativeObj : Obj {
    NativeObj(std::string n, int a, NativeFn f) : Obj(ObjType::Native), name(std::move(n)), arity(a), fn(std::move(f)) {}
    std::string name;
    int arity;  // -1 = variadic
    NativeFn fn;
};

struct ListObj : Obj {
    ListObj() : Obj(ObjType::List) {}
    std::vector<Value> items;
};

// Engine objects exposed to scripts implement this interface.
struct HostObject : Obj {
    HostObject() : Obj(ObjType::Host) {}
    virtual std::string host_type() const = 0;
    virtual std::optional<Value> get(const std::string& name) = 0;
    virtual bool set(const std::string& name, const Value& v) = 0;
    virtual std::optional<Value> invoke(VM& vm, const std::string& method, std::span<Value> args) = 0;
};

}  // namespace ifg::script
