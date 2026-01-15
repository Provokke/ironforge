// Forge script compiler: a single-pass Pratt parser that emits stack bytecode
// directly (no intermediate AST), resolving locals and closure upvalues at
// compile time.
#pragma once
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "script/value.hpp"

namespace ifg::script {

enum class Op : std::uint8_t {
    Const, Nil, True, False, Pop,
    GetLocal, SetLocal, GetGlobal, DefGlobal, SetGlobal, GetUpval, SetUpval,
    GetProp, SetProp, GetIndex, SetIndex,
    Eq, Ne, Lt, Le, Gt, Ge, Add, Sub, Mul, Div, Mod, Neg, Not,
    Print, Jump, JumpIfFalse, Loop, Call, Invoke, Closure, CloseUpval, Return, List,
};

struct CompileResult {
    std::shared_ptr<FunctionObj> function;  // null on error
    std::vector<std::string> errors;
};

CompileResult compile(std::string_view source, const std::string& name = "script");
std::string disassemble(const FunctionObj& fn);

}  // namespace ifg::script
