// forge <script.fs> [--disasm]
// Runs a forge script; prints output to stdout and errors to stdout as well
// (so golden files capture both). Exit code: 0 ok, 65 compile error, 70 runtime error.
#include <cstring>
#include <iostream>

#include "engine/demo_scene.hpp"
#include "script/compiler.hpp"
#include "script/vm.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: forge <script.fs> [--disasm]\n";
        return 64;
    }
    std::string src = ifg::read_text_file(argv[1]);
    if (argc > 2 && std::strcmp(argv[2], "--disasm") == 0) {
        auto cr = ifg::script::compile(src);
        if (!cr.function) {
            for (auto& e : cr.errors) std::cout << e << "\n";
            return 65;
        }
        std::cout << ifg::script::disassemble(*cr.function);
        for (auto& c : cr.function->chunk.constants)
            if (c.is(ifg::script::ObjType::Function))
                std::cout << ifg::script::disassemble(static_cast<ifg::script::FunctionObj&>(*c.obj));
        return 0;
    }
    ifg::script::VM vm(std::cout);
    auto r = vm.run(src);
    if (!r.ok) {
        std::cout << r.error << "\n";
        return r.error.find("CompileError") != std::string::npos ? 65 : 70;
    }
    return 0;
}
