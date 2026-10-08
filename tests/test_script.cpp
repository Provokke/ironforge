#include <algorithm>
#include <filesystem>
#include <random>
#include <sstream>

#include "doctest/doctest.h"
#include "engine/demo_scene.hpp"
#include "script/bindings.hpp"
#include "script/compiler.hpp"
#include "script/lexer.hpp"
#include "script/vm.hpp"

using namespace ifg;
using namespace ifg::script;

namespace {
std::string run_capture(const std::string& src) {
    std::ostringstream out;
    VM vm(out);
    auto r = vm.run(src);
    if (!r.ok) out << r.error << "\n";
    return out.str();
}
}  // namespace

TEST_CASE("golden script outputs (tests/scripts/*.fs)") {
    namespace fs = std::filesystem;
    fs::path dir = fs::path(IFG_SOURCE_DIR) / "tests" / "scripts";
    std::vector<fs::path> scripts;
    for (auto& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".fs") scripts.push_back(e.path());
    std::sort(scripts.begin(), scripts.end());
    REQUIRE(scripts.size() >= 25);
    int compared = 0;
    for (auto& p : scripts) {
        CAPTURE(p.filename().string());
        std::string got = run_capture(read_text_file(p.string()));
        fs::path expect_path = p;
        expect_path.replace_extension(".out");
        if (fs::exists(expect_path)) {
            CHECK(got == read_text_file(expect_path.string()));
            ++compared;
        } else {
            // scripts without a .out file must fail with a stack overflow
            CHECK(got.rfind("[line 1] RuntimeError: stack overflow", 0) == 0);
        }
    }
    MESSAGE("compared " << compared << " golden outputs of " << scripts.size() << " scripts");
}

TEST_CASE("lexer produces expected tokens") {
    auto toks = Lexer::tokenize("let x = 3.25; // comment\n fn f(a) { return a >= 2 and !nil; } # hash");
    std::vector<Tok> types;
    for (auto& t : toks) types.push_back(t.type);
    std::vector<Tok> expect = {Tok::Let, Tok::Ident, Tok::Eq, Tok::Number, Tok::Semicolon, Tok::Fn, Tok::Ident,
                               Tok::LParen, Tok::Ident, Tok::RParen, Tok::LBrace, Tok::Return, Tok::Ident,
                               Tok::GreaterEq, Tok::Number, Tok::And, Tok::Bang, Tok::Nil, Tok::Semicolon,
                               Tok::RBrace, Tok::Eof};
    CHECK(types == expect);
    CHECK(toks[3].text == "3.25");
    CHECK(toks[5].line == 2);
}

TEST_CASE("compiler emits bytecode visible through the disassembler") {
    auto cr = compile("let a = 1 + 2; print a;");
    REQUIRE(cr.function);
    std::string d = disassemble(*cr.function);
    CHECK(d.find("CONST 1") != std::string::npos);
    CHECK(d.find("ADD") != std::string::npos);
    CHECK(d.find("DEF_GLOBAL a") != std::string::npos);
    CHECK(d.find("PRINT") != std::string::npos);
    auto bad = compile("let;");
    CHECK_FALSE(bad.function);
    CHECK(bad.errors.size() == 1);
}

TEST_CASE("calling script functions from C++ and natives from script") {
    std::ostringstream out;
    VM vm(out);
    int native_calls = 0;
    vm.define_native("host_add", 2, [&](VM&, std::span<Value> a) {
        ++native_calls;
        return Value::number(a[0].n + a[1].n);
    });
    REQUIRE(vm.run("fn twice(x) { return host_add(x, x); } let g = 7;").ok);
    Value arg = Value::number(21), result;
    auto r = vm.call_global("twice", std::span<const Value>(&arg, 1), &result);
    REQUIRE(r.ok);
    CHECK(result.n == 42);
    CHECK(native_calls == 1);
    CHECK(vm.get_global("g")->n == 7);
    // errors from a C++-initiated call are reported, and the VM stays usable
    auto bad = vm.call_global("twice", {}, nullptr);
    CHECK_FALSE(bad.ok);
    CHECK(bad.error.find("expects 1 argument") != std::string::npos);
    CHECK(vm.run("print g + 1;").ok);
    CHECK(out.str() == "8\n");
    CHECK_FALSE(vm.call_global("nope", {}).ok);
}

TEST_CASE("runtime error inside a native-invoked callback unwinds cleanly") {
    std::ostringstream out;
    VM vm(out);
    vm.define_native("call_it", 1, [](VM& v, std::span<Value> a) {
        auto r = v.call(a[0], {});
        return Value::boolean(r.ok);
    });
    REQUIRE(vm.run("print call_it(fn() { return 1 + nil; }); print call_it(fn() { return 1; });").ok);
    CHECK(out.str() == "false\ntrue\n");
}

TEST_CASE("scene bindings: script drives nodes") {
    std::ostringstream out;
    Node root("World");
    VM vm(out);
    bind_scene(vm, &root);
    auto r = vm.run(R"(
        let c = scene.spawn("cube", "Box");
        c.set_position(1, 2, 3);
        c.x = c.x + 10;
        c.scale = 2;
        c.yaw(3.14159265 / 2);
        c.set_color(1, 0, 0);
        let s = c.spawn("sphere", "Ball");
        print s.path;
        print scene.find("Box/Ball").name;
        print scene.find("Nope");
        print scene.child_count;
        print c.parent.name;
        let hits = 0;
        c.connect("hit", fn(v) { hits = hits + v; });
        c.emit("hit", 2);
        c.emit("hit", 3);
        print hits;
        print is_node(c);
        print is_node(5);
        print c;
    )");
    REQUIRE_MESSAGE(r.ok, r.error);
    CHECK(out.str() == "/World/Box/Ball\nBall\nnil\n1\nWorld\n5\ntrue\nfalse\n<Node>\n");
    auto* box = dynamic_cast<MeshNode*>(root.find("Box"));
    REQUIRE(box);
    CHECK(approx(box->transform.position, Vec3(11, 2, 3)));
    CHECK(box->transform.scale.x == 2.0f);
    CHECK(box->color == Vec3(1, 0, 0));
    CHECK(approx(box->transform.rotation.rotate({1, 0, 0}), Vec3(0, 0, -1), 1e-3f));
}

TEST_CASE("scene bindings: errors and freed nodes") {
    std::ostringstream out;
    Node root("World");
    VM vm(out);
    bind_scene(vm, &root);
    REQUIRE(vm.run("let c = scene.spawn(\"cube\", \"Box\"); c.queue_free();").ok);
    root.propagate_process(0.016f);
    auto r = vm.run("print c.alive; c.translate(1, 0, 0);");
    CHECK_FALSE(r.ok);
    CHECK(out.str() == "false\n");
    CHECK(r.error.find("node used after it was freed") != std::string::npos);
    CHECK(vm.run("scene.spawn(\"teapot\", \"T\");").error.find("unknown mesh kind") != std::string::npos);
    CHECK(vm.run("scene.set_position(1, 2);").error.find("undefined method 'set_position' (with 2 args)") != std::string::npos);
    CHECK(vm.run("scene.x = \"a\";").error.find("cannot set property 'x'") != std::string::npos);
    CHECK(vm.run("print scene.bogus;").error.find("undefined property 'bogus' on Node") != std::string::npos);
    CHECK(vm.run("scene.set_color(1, 1, 1);").error.find("requires a mesh node") != std::string::npos);
}

TEST_CASE("process signal connected from script runs every tick") {
    DemoScene demo;
    REQUIRE(demo.load(read_text_file(std::string(IFG_SOURCE_DIR) + "/demo/scripts/demo.fs")));
    REQUIRE(demo.simulate(30));
    CHECK(demo.vm->get_global("ticks")->n == 30);
    CHECK(demo.vm->get_global("t")->n == doctest::Approx(0.5));
    CHECK(demo.root->subtree_size() == 1 + 1 + 1 + 10 + 1);  // World, Ground, Core, 10 cubes, Moon
}

TEST_CASE("VM counts instructions and survives deep (non-overflowing) recursion") {
    std::ostringstream out;
    VM vm(out);
    REQUIRE(vm.run("fn depth(n) { if (n == 0) return 0; return 1 + depth(n - 1); } print depth(200);").ok);
    CHECK(out.str() == "200\n");
    CHECK(vm.instructions_executed() > 2000);
}

TEST_CASE("fuzz: random token soup never crashes the compiler or VM") {
    const char* pieces[] = {"let ", "x", "y", "fn ", "(", ")", "{", "}", "[", "]", ";", ",", ".", "=", "==", "+", "-",
                            "*", "/", "%", "!", "<", ">", "1", "2.5", "\"s\"", "if ", "else ", "while ", "for ",
                            "return ", "break", "continue", "true", "false", "nil", "print ", "and ", "or ", "len",
                            "push", "\n", " ", "@", "\"unterminated"};
    std::mt19937 rng(1234);
    int compiled = 0;
    for (int i = 0; i < 3000; ++i) {
        std::string src;
        int n = 1 + static_cast<int>(rng() % 40);
        for (int k = 0; k < n; ++k) src += pieces[rng() % (sizeof(pieces) / sizeof(pieces[0]))];
        std::ostringstream out;
        VM vm(out);
        vm.max_frames = 64;
        auto r = vm.run(src);  // may fail; must not crash, hang or leak
        compiled += r.ok || r.error.find("RuntimeError") != std::string::npos;
    }
    MESSAGE("fuzz: " << compiled << " of 3000 random programs compiled");
    CHECK(compiled > 0);
}
