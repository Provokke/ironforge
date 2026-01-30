#include "script/vm.hpp"

#include <cmath>
#include <sstream>

namespace ifg::script {

VM::VM(std::ostream& out) : out_(out) {
    stack_.reserve(1024);
    frames_.reserve(max_frames + 1);  // frame pointers held by execute() must stay valid
    install_builtins();
}

VM::~VM() {
    // Reference counting cannot reclaim cycles (a closure capturing itself, a
    // list containing itself). Break them by clearing every object we created.
    globals_.clear();
    stack_.clear();
    open_upvalues_.clear();
    for (auto& w : tracked_)
        if (auto o = w.lock()) {
            if (o->type == ObjType::Closure) static_cast<ClosureObj&>(*o).upvalues.clear();
            else if (o->type == ObjType::List) static_cast<ListObj&>(*o).items.clear();
        }
    for (auto& s : scripts_) s->chunk.constants.clear();
}

Value* VM::get_global(const std::string& name) {
    auto it = globals_.find(name);
    return it == globals_.end() ? nullptr : &it->second;
}

void VM::define_native(const std::string& name, int arity, NativeFn fn) {
    globals_[name] = Value::object(std::make_shared<NativeObj>(name, arity, std::move(fn)));
}

int VM::current_line() const {
    if (frames_.empty()) return 0;
    const Frame& f = frames_.back();
    const auto& lines = f.closure->fn->chunk.lines;
    std::size_t at = f.ip > 0 ? f.ip - 1 : 0;
    return at < lines.size() ? lines[at] : 0;
}

void VM::runtime_error(const std::string& msg) {
    std::ostringstream os;
    os << "[line " << current_line() << "] RuntimeError: " << msg;
    for (std::size_t i = frames_.size(); i-- > 0;) {
        const Frame& f = frames_[i];
        const auto& fn = *f.closure->fn;
        std::size_t at = f.ip > 0 ? f.ip - 1 : 0;
        os << "\n  at " << (fn.name.empty() ? "<script>" : fn.name) << " (line "
           << (at < fn.chunk.lines.size() ? fn.chunk.lines[at] : 0) << ")";
    }
    throw ScriptError{os.str()};
}

RunResult VM::run(std::string_view source) {
    CompileResult cr = compile(source);
    if (!cr.function) {
        std::string all;
        for (auto& e : cr.errors) all += (all.empty() ? "" : "\n") + e;
        return {false, all};
    }
    scripts_.push_back(cr.function);
    auto closure = std::make_shared<ClosureObj>(cr.function);
    return call(Value::object(closure), {}, nullptr);
}

RunResult VM::call_global(const std::string& name, std::span<const Value> args, Value* result) {
    Value* g = get_global(name);
    if (!g) return {false, "RuntimeError: undefined function '" + name + "'"};
    Value fn = *g;
    return call(fn, args, result);
}

RunResult VM::call(const Value& fn, std::span<const Value> args, Value* result) {
    std::size_t stack_mark = stack_.size(), frame_mark = frames_.size();
    try {
        stack_.push_back(fn);
        for (const Value& a : args) stack_.push_back(a);
        call_value(stack_[stack_.size() - args.size() - 1], static_cast<int>(args.size()));
        if (frames_.size() > frame_mark) execute(frame_mark);
        if (result) *result = stack_.back();
        stack_.resize(stack_mark);
        return {};
    } catch (const ScriptError& e) {
        close_upvalues(stack_mark);
        stack_.resize(stack_mark);
        frames_.resize(frame_mark);
        return {false, e.message};
    }
}

std::shared_ptr<Upvalue> VM::capture(std::size_t slot) {
    for (auto& u : open_upvalues_)
        if (static_cast<std::size_t>(u->slot) == slot) return u;
    auto u = std::make_shared<Upvalue>();
    u->slot = static_cast<int>(slot);
    open_upvalues_.push_back(u);
    return u;
}

void VM::close_upvalues(std::size_t from) {
    for (std::size_t i = 0; i < open_upvalues_.size();) {
        auto& u = open_upvalues_[i];
        if (static_cast<std::size_t>(u->slot) >= from) {
            u->closed = u->slot < static_cast<int>(stack_.size()) ? stack_[u->slot] : Value{};
            u->open = false;
            u->slot = -1;
            open_upvalues_[i] = std::move(open_upvalues_.back());
            open_upvalues_.pop_back();
        } else {
            ++i;
        }
    }
}

void VM::call_value(const Value& callee_ref, int argc) {
    if (!callee_ref.is_obj()) runtime_error("cannot call a value of type " + type_name(callee_ref));
    Obj* o = callee_ref.obj.get();
    if (o->type == ObjType::Closure) {
        auto* c = static_cast<ClosureObj*>(o);
        if (argc != c->fn->arity)
            runtime_error("function '" + (c->fn->name.empty() ? std::string("<script>") : c->fn->name) + "' expects " +
                          std::to_string(c->fn->arity) + " argument(s) but got " + std::to_string(argc));
        if (frames_.size() >= max_frames) runtime_error("stack overflow");
        frames_.push_back({c, 0, stack_.size() - argc - 1});
        return;
    }
    if (o->type == ObjType::Native) {
        auto native = std::static_pointer_cast<NativeObj>(callee_ref.obj);  // keep alive during call
        if (native->arity >= 0 && argc != native->arity)
            runtime_error("native '" + native->name + "' expects " + std::to_string(native->arity) +
                          " argument(s) but got " + std::to_string(argc));
        std::size_t base = stack_.size() - argc;
        std::vector<Value> args(stack_.begin() + static_cast<std::ptrdiff_t>(base), stack_.end());
        Value r = native->fn(*this, std::span<Value>(args));
        stack_.resize(base - 1);
        stack_.push_back(std::move(r));
        return;
    }
    runtime_error("cannot call a value of type " + type_name(callee_ref));
}

static double as_index(VM& vm, const Value& idx, std::size_t size) {
    if (!idx.is_number() || idx.n != std::floor(idx.n)) vm.runtime_error("index must be an integer");
    double i = idx.n < 0 ? idx.n + static_cast<double>(size) : idx.n;
    if (i < 0 || i >= static_cast<double>(size))
        vm.runtime_error("index " + to_display(idx) + " out of range (size " + std::to_string(size) + ")");
    return i;
}

void VM::execute(std::size_t stop_depth) {
    Frame* f = &frames_.back();
    const std::uint8_t* code = f->closure->fn->chunk.code.data();
    const Value* consts = f->closure->fn->chunk.constants.data();
    std::size_t ip = f->ip;
    auto reload = [&] {
        f = &frames_.back();
        code = f->closure->fn->chunk.code.data();
        consts = f->closure->fn->chunk.constants.data();
        ip = f->ip;
    };
    auto rd8 = [&] { return code[ip++]; };
    auto rd16 = [&] {
        ip += 2;
        return static_cast<std::uint16_t>((code[ip - 2] << 8) | code[ip - 1]);
    };
    auto pop = [&] {
        Value v = std::move(stack_.back());
        stack_.pop_back();
        return v;
    };
    auto peek = [&](std::size_t d) -> Value& { return stack_[stack_.size() - 1 - d]; };
    auto err = [&](const std::string& m) {
        f->ip = ip;
        runtime_error(m);
    };
    auto num_operands = [&](const char* op) {
        if (!peek(0).is_number() || !peek(1).is_number())
            err(std::string("operands of '") + op + "' must be numbers, got " + type_name(peek(1)) + " and " +
                type_name(peek(0)));
    };

    for (;;) {
        ++executed_;
        Op op = static_cast<Op>(rd8());
        switch (op) {
            case Op::Const: stack_.push_back(consts[rd16()]); break;
            case Op::Nil: stack_.emplace_back(); break;
            case Op::True: stack_.push_back(Value::boolean(true)); break;
            case Op::False: stack_.push_back(Value::boolean(false)); break;
            case Op::Pop: stack_.pop_back(); break;
            case Op::GetLocal: stack_.push_back(stack_[f->base + rd8()]); break;
            case Op::SetLocal: stack_[f->base + rd8()] = peek(0); break;
            case Op::GetGlobal: {
                const auto& name = static_cast<StringObj&>(*consts[rd16()].obj).str;
                auto it = globals_.find(name);
                if (it == globals_.end()) err("undefined variable '" + name + "'");
                stack_.push_back(it->second);
                break;
            }
            case Op::DefGlobal: {
                const auto& name = static_cast<StringObj&>(*consts[rd16()].obj).str;
                globals_[name] = pop();
                break;
            }
            case Op::SetGlobal: {
                const auto& name = static_cast<StringObj&>(*consts[rd16()].obj).str;
                auto it = globals_.find(name);
                if (it == globals_.end()) err("assignment to undefined variable '" + name + "'");
                it->second = peek(0);
                break;
            }
            case Op::GetUpval: {
                auto& u = f->closure->upvalues[rd8()];
                stack_.push_back(u->open ? stack_[u->slot] : u->closed);
                break;
            }
            case Op::SetUpval: {
                auto& u = f->closure->upvalues[rd8()];
                (u->open ? stack_[u->slot] : u->closed) = peek(0);
                break;
            }
            case Op::GetProp: {
                const auto& name = static_cast<StringObj&>(*consts[rd16()].obj).str;
                Value obj = pop();
                if (!obj.is(ObjType::Host)) err("cannot read property '" + name + "' of " + type_name(obj));
                f->ip = ip;
                auto v = static_cast<HostObject&>(*obj.obj).get(name);
                if (!v) err("undefined property '" + name + "' on " + type_name(obj));
                stack_.push_back(std::move(*v));
                break;
            }
            case Op::SetProp: {
                const auto& name = static_cast<StringObj&>(*consts[rd16()].obj).str;
                Value v = pop();
                Value obj = pop();
                if (!obj.is(ObjType::Host)) err("cannot set property '" + name + "' on " + type_name(obj));
                f->ip = ip;
                if (!static_cast<HostObject&>(*obj.obj).set(name, v))
                    err("cannot set property '" + name + "' on " + type_name(obj) + " to " + type_name(v));
                stack_.push_back(std::move(v));
                break;
            }
            case Op::GetIndex: {
                Value idx = pop();
                Value obj = pop();
                f->ip = ip;
                if (obj.is(ObjType::List)) {
                    auto& items = static_cast<ListObj&>(*obj.obj).items;
                    stack_.push_back(items[static_cast<std::size_t>(as_index(*this, idx, items.size()))]);
                } else if (obj.is(ObjType::String)) {
                    auto& s = static_cast<StringObj&>(*obj.obj).str;
                    stack_.push_back(Value::string(std::string(1, s[static_cast<std::size_t>(as_index(*this, idx, s.size()))])));
                } else {
                    err("cannot index a value of type " + type_name(obj));
                }
                break;
            }
            case Op::SetIndex: {
                Value v = pop();
                Value idx = pop();
                Value obj = pop();
                f->ip = ip;
                if (!obj.is(ObjType::List)) err("cannot assign into a value of type " + type_name(obj));
                auto& items = static_cast<ListObj&>(*obj.obj).items;
                items[static_cast<std::size_t>(as_index(*this, idx, items.size()))] = v;
                stack_.push_back(std::move(v));
                break;
            }
            case Op::Eq: { Value b = pop(); Value a = pop(); stack_.push_back(Value::boolean(values_equal(a, b))); break; }
            case Op::Ne: { Value b = pop(); Value a = pop(); stack_.push_back(Value::boolean(!values_equal(a, b))); break; }
            case Op::Lt: case Op::Le: case Op::Gt: case Op::Ge: {
                bool r;
                if (peek(0).is(ObjType::String) && peek(1).is(ObjType::String)) {
                    int c = static_cast<StringObj&>(*peek(1).obj).str.compare(static_cast<StringObj&>(*peek(0).obj).str);
                    r = op == Op::Lt ? c < 0 : op == Op::Le ? c <= 0 : op == Op::Gt ? c > 0 : c >= 0;
                } else {
                    num_operands(op == Op::Lt ? "<" : op == Op::Le ? "<=" : op == Op::Gt ? ">" : ">=");
                    double a = peek(1).n, b = peek(0).n;
                    r = op == Op::Lt ? a < b : op == Op::Le ? a <= b : op == Op::Gt ? a > b : a >= b;
                }
                stack_.pop_back();
                stack_.back() = Value::boolean(r);
                break;
            }
            case Op::Add: {
                Value& a = peek(1);
                Value& b = peek(0);
                if (a.is_number() && b.is_number()) {
                    a.n += b.n;
                    stack_.pop_back();
                } else if (a.is(ObjType::String) || b.is(ObjType::String)) {
                    Value r = Value::string(to_display(a) + to_display(b));
                    stack_.pop_back();
                    stack_.back() = std::move(r);
                } else if (a.is(ObjType::List) && b.is(ObjType::List)) {
                    auto l = std::make_shared<ListObj>();
                    l->items = static_cast<ListObj&>(*a.obj).items;
                    auto& bi = static_cast<ListObj&>(*b.obj).items;
                    l->items.insert(l->items.end(), bi.begin(), bi.end());
                    track(l);
                    stack_.pop_back();
                    stack_.back() = Value::object(l);
                } else {
                    err("cannot add " + type_name(a) + " and " + type_name(b));
                }
                break;
            }
            case Op::Sub: num_operands("-"); peek(1).n -= peek(0).n; stack_.pop_back(); break;
            case Op::Mul: num_operands("*"); peek(1).n *= peek(0).n; stack_.pop_back(); break;
            case Op::Div: num_operands("/"); peek(1).n /= peek(0).n; stack_.pop_back(); break;
            case Op::Mod:
                num_operands("%");
                if (peek(0).n == 0) err("modulo by zero");
                peek(1).n = std::fmod(peek(1).n, peek(0).n);
                stack_.pop_back();
                break;
            case Op::Neg:
                if (!peek(0).is_number()) err("operand of unary '-' must be a number, got " + type_name(peek(0)));
                peek(0).n = -peek(0).n;
                break;
            case Op::Not: stack_.back() = Value::boolean(!stack_.back().truthy()); break;
            case Op::Print: out_ << to_display(pop()) << "\n"; break;
            case Op::Jump: { std::uint16_t o = rd16(); ip += o; break; }
            case Op::JumpIfFalse: { std::uint16_t o = rd16(); if (!peek(0).truthy()) ip += o; break; }
            case Op::Loop: { std::uint16_t o = rd16(); ip -= o; break; }
            case Op::Call: {
                int argc = rd8();
                f->ip = ip;
                Value callee = peek(static_cast<std::size_t>(argc));
                call_value(callee, argc);
                reload();
                break;
            }
            case Op::Invoke: {
                const std::string name = static_cast<StringObj&>(*consts[rd16()].obj).str;
                int argc = rd8();
                f->ip = ip;
                Value recv = peek(static_cast<std::size_t>(argc));
                std::size_t base = stack_.size() - argc;
                std::vector<Value> args(stack_.begin() + static_cast<std::ptrdiff_t>(base), stack_.end());
                std::optional<Value> r;
                if (recv.is(ObjType::Host)) {
                    r = static_cast<HostObject&>(*recv.obj).invoke(*this, name, std::span<Value>(args));
                } else if (recv.is(ObjType::List)) {
                    auto& items = static_cast<ListObj&>(*recv.obj).items;
                    if (name == "push" && argc == 1) { items.push_back(args[0]); r = recv; }
                    else if (name == "pop" && argc == 0) {
                        if (items.empty()) err("pop from empty list");
                        r = items.back();
                        items.pop_back();
                    } else if (name == "len" && argc == 0) r = Value::number(static_cast<double>(items.size()));
                } else {
                    err("cannot call method '" + name + "' on " + type_name(recv));
                }
                if (!r) err("undefined method '" + name + "' (with " + std::to_string(argc) + " args) on " + type_name(recv));
                stack_.resize(base - 1);
                stack_.push_back(std::move(*r));
                reload();  // host code may have called back into the VM
                break;
            }
            case Op::Closure: {
                auto fn = std::static_pointer_cast<FunctionObj>(consts[rd16()].obj);
                auto c = std::make_shared<ClosureObj>(fn);
                c->upvalues.reserve(fn->upvalue_count);
                for (int i = 0; i < fn->upvalue_count; ++i) {
                    bool is_local = rd8();
                    int index = rd8();
                    c->upvalues.push_back(is_local ? capture(f->base + index) : f->closure->upvalues[index]);
                }
                if (fn->upvalue_count) track(c);
                stack_.push_back(Value::object(std::move(c)));
                break;
            }
            case Op::CloseUpval:
                close_upvalues(stack_.size() - 1);
                stack_.pop_back();
                break;
            case Op::Return: {
                Value result = pop();
                close_upvalues(f->base);
                stack_.resize(f->base);
                stack_.push_back(std::move(result));
                frames_.pop_back();
                if (frames_.size() == stop_depth) return;
                reload();
                break;
            }
            case Op::List: {
                int n = rd16();
                auto l = std::make_shared<ListObj>();
                l->items.assign(stack_.end() - n, stack_.end());
                stack_.resize(stack_.size() - n);
                track(l);
                stack_.push_back(Value::object(std::move(l)));
                break;
            }
        }
    }
}

void VM::install_builtins() {
    auto num_arg = [](VM& vm, const Value& v, const char* fn) {
        if (!v.is_number()) vm.runtime_error(std::string(fn) + "() expects a number, got " + type_name(v));
        return v.n;
    };
    define_native("len", 1, [](VM& vm, std::span<Value> a) {
        if (a[0].is(ObjType::List)) return Value::number(double(static_cast<ListObj&>(*a[0].obj).items.size()));
        if (a[0].is(ObjType::String)) return Value::number(double(static_cast<StringObj&>(*a[0].obj).str.size()));
        vm.runtime_error("len() expects a list or string, got " + type_name(a[0]));
    });
    define_native("push", 2, [](VM& vm, std::span<Value> a) {
        if (!a[0].is(ObjType::List)) vm.runtime_error("push() expects a list, got " + type_name(a[0]));
        static_cast<ListObj&>(*a[0].obj).items.push_back(a[1]);
        return a[0];
    });
    define_native("str", 1, [](VM&, std::span<Value> a) { return Value::string(to_display(a[0])); });
    define_native("type", 1, [](VM&, std::span<Value> a) { return Value::string(type_name(a[0])); });
    define_native("num", 1, [](VM& vm, std::span<Value> a) {
        if (a[0].is_number()) return a[0];
        if (!a[0].is(ObjType::String)) vm.runtime_error("num() expects a string or number");
        const std::string& s = static_cast<StringObj&>(*a[0].obj).str;
        char* end = nullptr;
        double d = std::strtod(s.c_str(), &end);
        if (s.empty() || *end) return Value::nil();
        return Value::number(d);
    });
    define_native("sqrt", 1, [num_arg](VM& vm, std::span<Value> a) { return Value::number(std::sqrt(num_arg(vm, a[0], "sqrt"))); });
    define_native("sin", 1, [num_arg](VM& vm, std::span<Value> a) { return Value::number(std::sin(num_arg(vm, a[0], "sin"))); });
    define_native("cos", 1, [num_arg](VM& vm, std::span<Value> a) { return Value::number(std::cos(num_arg(vm, a[0], "cos"))); });
    define_native("floor", 1, [num_arg](VM& vm, std::span<Value> a) { return Value::number(std::floor(num_arg(vm, a[0], "floor"))); });
    define_native("abs", 1, [num_arg](VM& vm, std::span<Value> a) { return Value::number(std::fabs(num_arg(vm, a[0], "abs"))); });
    define_native("min", 2, [num_arg](VM& vm, std::span<Value> a) {
        return Value::number(std::min(num_arg(vm, a[0], "min"), num_arg(vm, a[1], "min")));
    });
    define_native("max", 2, [num_arg](VM& vm, std::span<Value> a) {
        return Value::number(std::max(num_arg(vm, a[0], "max"), num_arg(vm, a[1], "max")));
    });
    define_native("assert", -1, [](VM& vm, std::span<Value> a) {
        if (a.empty() || a.size() > 2) vm.runtime_error("assert() expects 1 or 2 arguments");
        if (!a[0].truthy()) vm.runtime_error("assertion failed" + (a.size() == 2 ? ": " + to_display(a[1]) : std::string()));
        return Value::nil();
    });
    define_native("range", -1, [num_arg](VM& vm, std::span<Value> a) {
        if (a.empty() || a.size() > 2) vm.runtime_error("range() expects 1 or 2 arguments");
        double lo = a.size() == 2 ? num_arg(vm, a[0], "range") : 0, hi = num_arg(vm, a.back(), "range");
        if (hi - lo > 1e7) vm.runtime_error("range() too large");
        auto l = std::make_shared<ListObj>();
        for (double i = lo; i < hi; ++i) l->items.push_back(Value::number(i));
        return Value::object(l);
    });
}

}  // namespace ifg::script
