// Forge script virtual machine: stack-based bytecode interpreter with call
// frames, closures (open/closed upvalues), native functions and host objects.
#pragma once
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "script/compiler.hpp"
#include "script/value.hpp"

namespace ifg::script {

struct ScriptError {
    std::string message;  // already formatted: "[line N] RuntimeError: ..."
};

struct RunResult {
    bool ok = true;
    std::string error;  // compile errors (newline-separated) or the runtime error
};

class VM {
public:
    explicit VM(std::ostream& out);
    ~VM();
    VM(const VM&) = delete;
    VM& operator=(const VM&) = delete;

    RunResult run(std::string_view source);  // compile + execute top level
    // Call a script value (closure or native) from C++. Errors are returned, not thrown.
    RunResult call(const Value& fn, std::span<const Value> args, Value* result = nullptr);
    RunResult call_global(const std::string& name, std::span<const Value> args, Value* result = nullptr);

    void define_native(const std::string& name, int arity, NativeFn fn);
    void set_global(const std::string& name, Value v) { globals_[name] = std::move(v); }
    Value* get_global(const std::string& name);
    [[noreturn]] void runtime_error(const std::string& msg);

    std::uint64_t instructions_executed() const { return executed_; }
    std::size_t max_frames = 256;
    std::ostream& out() { return out_; }

private:
    struct Frame {
        ClosureObj* closure;
        std::size_t ip;
        std::size_t base;  // stack index of slot 0 (the callee)
    };
    void execute(std::size_t stop_depth);
    void call_value(const Value& callee, int argc);
    std::shared_ptr<Upvalue> capture(std::size_t slot);
    void close_upvalues(std::size_t from);
    void install_builtins();
    void track(const std::shared_ptr<Obj>& o) { tracked_.push_back(o); }
    int current_line() const;

    std::ostream& out_;
    std::vector<Value> stack_;
    std::vector<Frame> frames_;
    std::vector<std::shared_ptr<Upvalue>> open_upvalues_;
    std::unordered_map<std::string, Value> globals_;
    std::vector<std::weak_ptr<Obj>> tracked_;  // closures & lists, for cycle breaking at teardown
    std::vector<std::shared_ptr<FunctionObj>> scripts_;
    std::uint64_t executed_ = 0;
};

}  // namespace ifg::script
