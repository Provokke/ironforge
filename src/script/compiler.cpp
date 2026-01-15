#include "script/compiler.hpp"

#include <charconv>
#include <cstdlib>
#include <sstream>

#include "script/lexer.hpp"

namespace ifg::script {

namespace {

enum Prec { P_None, P_Assign, P_Or, P_And, P_Equality, P_Compare, P_Term, P_Factor, P_Unary, P_Call, P_Primary };

struct Local {
    std::string_view name;
    int depth;  // -1 while its initializer is being compiled
    bool captured = false;
};
struct UpvalRef {
    std::uint8_t index;
    bool is_local;
};
struct Loop {
    int continue_target;
    int scope_depth;
    std::vector<int> breaks;
    std::vector<int> continues;  // forward jumps (for-loops patch these to the increment)
    bool continue_known;
};

struct FnState {
    FnState* enclosing = nullptr;
    std::shared_ptr<FunctionObj> fn = std::make_shared<FunctionObj>();
    std::vector<Local> locals;
    std::vector<UpvalRef> upvals;
    std::vector<Loop> loops;
    int scope_depth = 0;
    bool is_script = false;
};

class Compiler {
public:
    explicit Compiler(std::string_view src) : lex_(src) {}

    CompileResult run(const std::string& name) {
        FnState top;
        top.is_script = true;
        top.fn->name = "";
        (void)name;
        top.locals.push_back({"", 0});
        fs_ = &top;
        advance();
        while (!check(Tok::Eof)) declaration();
        emit(Op::Nil);
        emit(Op::Return);
        CompileResult r;
        if (errors_.empty()) r.function = top.fn;
        r.errors = std::move(errors_);
        return r;
    }

private:
    // ------------------------------------------------------------ token stream
    void advance() {
        prev_ = cur_;
        for (;;) {
            cur_ = lex_.next();
            if (cur_.type != Tok::Error) break;
            error_at(cur_, cur_.error);
        }
    }
    bool check(Tok t) const { return cur_.type == t; }
    bool match(Tok t) {
        if (!check(t)) return false;
        advance();
        return true;
    }
    void consume(Tok t, const char* msg) {
        if (check(t)) { advance(); return; }
        error_at(cur_, msg);
    }
    void error_at(const Token& t, const std::string& msg) {
        if (panic_) return;
        panic_ = true;
        std::ostringstream os;
        os << "[line " << t.line << "] CompileError";
        if (t.type == Tok::Eof) os << " at end";
        else if (t.type != Tok::Error) os << " at '" << t.text << "'";
        os << ": " << msg;
        errors_.push_back(os.str());
    }
    void error(const std::string& msg) { error_at(prev_, msg); }
    void synchronize() {
        panic_ = false;
        while (!check(Tok::Eof)) {
            if (prev_.type == Tok::Semicolon) return;
            switch (cur_.type) {
                case Tok::Let: case Tok::Fn: case Tok::If: case Tok::While: case Tok::For:
                case Tok::Print: case Tok::Return: return;
                default: advance();
            }
        }
    }

    // ------------------------------------------------------------ emission
    Chunk& chunk() { return fs_->fn->chunk; }
    void emit_byte(std::uint8_t b) { chunk().write(b, prev_.line); }
    void emit(Op op) { emit_byte(static_cast<std::uint8_t>(op)); }
    void emit_u16(std::uint16_t v) { emit_byte(std::uint8_t(v >> 8)); emit_byte(std::uint8_t(v & 0xFF)); }
    void emit_op16(Op op, int v) { emit(op); emit_u16(static_cast<std::uint16_t>(v)); }
    int make_const(Value v) {
        int i = chunk().add_constant(std::move(v));
        if (i > 0xFFFF) error("too many constants in one function");
        return i;
    }
    int name_const(std::string_view s) {
        // reuse identical string constants (names are looked up often)
        auto& cs = chunk().constants;
        for (std::size_t i = 0; i < cs.size(); ++i)
            if (cs[i].is(ObjType::String) && static_cast<StringObj&>(*cs[i].obj).str == s) return static_cast<int>(i);
        return make_const(Value::string(std::string(s)));
    }
    int emit_jump(Op op) {
        emit(op);
        emit_u16(0xFFFF);
        return static_cast<int>(chunk().code.size()) - 2;
    }
    void patch_jump(int at) {
        int dist = static_cast<int>(chunk().code.size()) - at - 2;
        if (dist > 0xFFFF) error("jump too large");
        chunk().code[at] = std::uint8_t(dist >> 8);
        chunk().code[at + 1] = std::uint8_t(dist & 0xFF);
    }
    void emit_loop(int start) {
        emit(Op::Loop);
        int dist = static_cast<int>(chunk().code.size()) - start + 2;
        if (dist > 0xFFFF) error("loop body too large");
        emit_u16(static_cast<std::uint16_t>(dist));
    }

    // ------------------------------------------------------------ scopes
    void begin_scope() { ++fs_->scope_depth; }
    void end_scope() {
        --fs_->scope_depth;
        while (!fs_->locals.empty() && fs_->locals.back().depth > fs_->scope_depth) {
            emit(fs_->locals.back().captured ? Op::CloseUpval : Op::Pop);
            fs_->locals.pop_back();
        }
    }
    // pops locals deeper than `depth` without forgetting them (for break/continue)
    void discard_locals_above(int depth) {
        for (int i = static_cast<int>(fs_->locals.size()) - 1; i >= 0 && fs_->locals[i].depth > depth; --i)
            emit(fs_->locals[i].captured ? Op::CloseUpval : Op::Pop);
    }
    void add_local(std::string_view name) {
        if (fs_->locals.size() >= 255) { error("too many local variables in function"); return; }
        fs_->locals.push_back({name, -1});
    }
    void declare_variable(std::string_view name) {
        if (fs_->scope_depth == 0) return;
        for (int i = static_cast<int>(fs_->locals.size()) - 1; i >= 0; --i) {
            const Local& l = fs_->locals[i];
            if (l.depth != -1 && l.depth < fs_->scope_depth) break;
            if (l.name == name) error("variable '" + std::string(name) + "' already declared in this scope");
        }
        add_local(name);
    }
    void mark_initialized() {
        if (fs_->scope_depth == 0) return;
        fs_->locals.back().depth = fs_->scope_depth;
    }
    void define_variable(int global) {
        if (fs_->scope_depth > 0) { mark_initialized(); return; }
        emit_op16(Op::DefGlobal, global);
    }
    static int resolve_local(FnState* f, std::string_view name, Compiler* c) {
        for (int i = static_cast<int>(f->locals.size()) - 1; i >= 0; --i)
            if (f->locals[i].name == name) {
                if (f->locals[i].depth == -1) c->error("cannot read local variable in its own initializer");
                return i;
            }
        return -1;
    }
    int add_upval(FnState* f, std::uint8_t index, bool is_local) {
        for (std::size_t i = 0; i < f->upvals.size(); ++i)
            if (f->upvals[i].index == index && f->upvals[i].is_local == is_local) return static_cast<int>(i);
        if (f->upvals.size() >= 255) { error("too many closure variables"); return 0; }
        f->upvals.push_back({index, is_local});
        f->fn->upvalue_count = static_cast<int>(f->upvals.size());
        return static_cast<int>(f->upvals.size()) - 1;
    }
    int resolve_upval(FnState* f, std::string_view name) {
        if (!f->enclosing) return -1;
        int local = resolve_local(f->enclosing, name, this);
        if (local != -1) {
            f->enclosing->locals[local].captured = true;
            return add_upval(f, static_cast<std::uint8_t>(local), true);
        }
        int up = resolve_upval(f->enclosing, name);
        if (up != -1) return add_upval(f, static_cast<std::uint8_t>(up), false);
        return -1;
    }

    // ------------------------------------------------------------ declarations & statements
    void declaration() {
        if (match(Tok::Let)) let_declaration();
        else if (check(Tok::Fn) && peek_is_named_fn()) { advance(); fn_declaration(); }
        else statement();
        if (panic_) synchronize();
    }
    bool peek_is_named_fn() {
        // `fn name(` is a declaration; `fn (` is a lambda expression statement
        Lexer copy = lex_;
        return copy.next().type == Tok::Ident;
    }
    void let_declaration() {
        consume(Tok::Ident, "expected variable name");
        std::string_view name = prev_.text;
        int global = fs_->scope_depth == 0 ? name_const(name) : 0;
        declare_variable(name);
        if (match(Tok::Eq)) expression();
        else emit(Op::Nil);
        consume(Tok::Semicolon, "expected ';' after variable declaration");
        define_variable(global);
    }
    void fn_declaration() {
        consume(Tok::Ident, "expected function name");
        std::string_view name = prev_.text;
        int global = fs_->scope_depth == 0 ? name_const(name) : 0;
        declare_variable(name);
        mark_initialized();  // allows recursion
        function(name);
        define_variable(global);
    }
    void function(std::string_view name) {
        FnState st;
        st.enclosing = fs_;
        st.fn->name = std::string(name);
        st.locals.push_back({"", 0});
        fs_ = &st;
        begin_scope();
        consume(Tok::LParen, "expected '(' after function name");
        if (!check(Tok::RParen)) {
            do {
                if (++st.fn->arity > 255) error("too many parameters");
                consume(Tok::Ident, "expected parameter name");
                declare_variable(prev_.text);
                mark_initialized();
            } while (match(Tok::Comma));
        }
        consume(Tok::RParen, "expected ')' after parameters");
        consume(Tok::LBrace, "expected '{' before function body");
        block();
        emit(Op::Nil);
        emit(Op::Return);
        fs_ = st.enclosing;
        emit_op16(Op::Closure, make_const(Value::object(st.fn)));
        for (auto& u : st.upvals) {
            emit_byte(u.is_local ? 1 : 0);
            emit_byte(u.index);
        }
    }
    void statement() {
        if (match(Tok::Print)) {
            expression();
            consume(Tok::Semicolon, "expected ';' after value");
            emit(Op::Print);
        } else if (match(Tok::If)) {
            if_statement();
        } else if (match(Tok::While)) {
            while_statement();
        } else if (match(Tok::For)) {
            for_statement();
        } else if (match(Tok::Return)) {
            if (fs_->is_script) error("cannot return from top-level code");
            if (match(Tok::Semicolon)) emit(Op::Nil);
            else {
                expression();
                consume(Tok::Semicolon, "expected ';' after return value");
            }
            emit(Op::Return);
        } else if (match(Tok::Break)) {
            if (fs_->loops.empty()) { error("'break' outside of a loop"); return; }
            consume(Tok::Semicolon, "expected ';' after 'break'");
            discard_locals_above(fs_->loops.back().scope_depth);
            fs_->loops.back().breaks.push_back(emit_jump(Op::Jump));
        } else if (match(Tok::Continue)) {
            if (fs_->loops.empty()) { error("'continue' outside of a loop"); return; }
            consume(Tok::Semicolon, "expected ';' after 'continue'");
            Loop& lp = fs_->loops.back();
            discard_locals_above(lp.scope_depth);
            if (lp.continue_known) emit_loop(lp.continue_target);
            else lp.continues.push_back(emit_jump(Op::Jump));
        } else if (match(Tok::LBrace)) {
            begin_scope();
            block();
            end_scope();
        } else {
            expression();
            consume(Tok::Semicolon, "expected ';' after expression");
            emit(Op::Pop);
        }
    }
    void block() {
        while (!check(Tok::RBrace) && !check(Tok::Eof)) declaration();
        consume(Tok::RBrace, "expected '}' after block");
    }
    void if_statement() {
        consume(Tok::LParen, "expected '(' after 'if'");
        expression();
        consume(Tok::RParen, "expected ')' after condition");
        int then_jump = emit_jump(Op::JumpIfFalse);
        emit(Op::Pop);
        statement();
        int else_jump = emit_jump(Op::Jump);
        patch_jump(then_jump);
        emit(Op::Pop);
        if (match(Tok::Else)) statement();
        patch_jump(else_jump);
    }
    void end_loop() {
        Loop lp = std::move(fs_->loops.back());
        fs_->loops.pop_back();
        for (int b : lp.breaks) patch_jump(b);
    }
    void while_statement() {
        int start = static_cast<int>(chunk().code.size());
        fs_->loops.push_back({start, fs_->scope_depth, {}, {}, true});
        consume(Tok::LParen, "expected '(' after 'while'");
        expression();
        consume(Tok::RParen, "expected ')' after condition");
        int exit = emit_jump(Op::JumpIfFalse);
        emit(Op::Pop);
        statement();
        emit_loop(start);
        patch_jump(exit);
        emit(Op::Pop);
        end_loop();
    }
    void for_statement() {
        begin_scope();
        consume(Tok::LParen, "expected '(' after 'for'");
        if (match(Tok::Semicolon)) {
        } else if (match(Tok::Let)) {
            let_declaration();
        } else {
            expression();
            consume(Tok::Semicolon, "expected ';' after loop initializer");
            emit(Op::Pop);
        }
        int start = static_cast<int>(chunk().code.size());
        int exit = -1;
        if (!match(Tok::Semicolon)) {
            expression();
            consume(Tok::Semicolon, "expected ';' after loop condition");
            exit = emit_jump(Op::JumpIfFalse);
            emit(Op::Pop);
        }
        if (!match(Tok::RParen)) {
            int body_jump = emit_jump(Op::Jump);
            int incr = static_cast<int>(chunk().code.size());
            expression();
            emit(Op::Pop);
            consume(Tok::RParen, "expected ')' after for clauses");
            emit_loop(start);
            start = incr;
            patch_jump(body_jump);
        }
        fs_->loops.push_back({start, fs_->scope_depth, {}, {}, true});
        statement();
        emit_loop(start);
        if (exit != -1) {
            patch_jump(exit);
            emit(Op::Pop);
        }
        end_loop();
        end_scope();
    }

    // ------------------------------------------------------------ expressions (Pratt)
    using Fn = void (Compiler::*)(bool);
    struct Rule { Fn prefix; Fn infix; Prec prec; };
    static const Rule& rule(Tok t) {
        static const Rule table[] = {
            /* (  */ {&Compiler::grouping, &Compiler::call, P_Call},
            /* )  */ {nullptr, nullptr, P_None},
            /* {  */ {nullptr, nullptr, P_None},
            /* }  */ {nullptr, nullptr, P_None},
            /* [  */ {&Compiler::list, &Compiler::index, P_Call},
            /* ]  */ {nullptr, nullptr, P_None},
            /* ,  */ {nullptr, nullptr, P_None},
            /* .  */ {nullptr, &Compiler::dot, P_Call},
            /* ;  */ {nullptr, nullptr, P_None},
            /* :  */ {nullptr, nullptr, P_None},
            /* +  */ {nullptr, &Compiler::binary, P_Term},
            /* -  */ {&Compiler::unary, &Compiler::binary, P_Term},
            /* *  */ {nullptr, &Compiler::binary, P_Factor},
            /* /  */ {nullptr, &Compiler::binary, P_Factor},
            /* %  */ {nullptr, &Compiler::binary, P_Factor},
            /* !  */ {&Compiler::unary, nullptr, P_None},
            /* != */ {nullptr, &Compiler::binary, P_Equality},
            /* =  */ {nullptr, nullptr, P_None},
            /* == */ {nullptr, &Compiler::binary, P_Equality},
            /* <  */ {nullptr, &Compiler::binary, P_Compare},
            /* <= */ {nullptr, &Compiler::binary, P_Compare},
            /* >  */ {nullptr, &Compiler::binary, P_Compare},
            /* >= */ {nullptr, &Compiler::binary, P_Compare},
            /* id */ {&Compiler::variable, nullptr, P_None},
            /* str*/ {&Compiler::string, nullptr, P_None},
            /* num*/ {&Compiler::number, nullptr, P_None},
            /* and*/ {nullptr, &Compiler::and_, P_And},
            /* or */ {nullptr, &Compiler::or_, P_Or},
            /* let*/ {nullptr, nullptr, P_None},
            /* fn */ {&Compiler::lambda, nullptr, P_None},
            /* ret*/ {nullptr, nullptr, P_None},
            /* if */ {nullptr, nullptr, P_None},
            /* else*/ {nullptr, nullptr, P_None},
            /* while*/ {nullptr, nullptr, P_None},
            /* for*/ {nullptr, nullptr, P_None},
            /* break*/ {nullptr, nullptr, P_None},
            /* continue*/ {nullptr, nullptr, P_None},
            /* true*/ {&Compiler::literal, nullptr, P_None},
            /* false*/ {&Compiler::literal, nullptr, P_None},
            /* nil*/ {&Compiler::literal, nullptr, P_None},
            /* print*/ {nullptr, nullptr, P_None},
            /* err*/ {nullptr, nullptr, P_None},
            /* eof*/ {nullptr, nullptr, P_None},
        };
        static_assert(sizeof(table) / sizeof(table[0]) == static_cast<int>(Tok::Eof) + 1);
        return table[static_cast<int>(t)];
    }

    void expression() { parse(P_Assign); }
    void parse(Prec prec) {
        advance();
        Fn prefix = rule(prev_.type).prefix;
        if (!prefix) { error("expected expression"); return; }
        bool can_assign = prec <= P_Assign;
        (this->*prefix)(can_assign);
        while (prec <= rule(cur_.type).prec) {
            advance();
            (this->*rule(prev_.type).infix)(can_assign);
        }
        if (can_assign && match(Tok::Eq)) error("invalid assignment target");
    }

    void number(bool) {
        double v = std::strtod(std::string(prev_.text).c_str(), nullptr);
        emit_op16(Op::Const, make_const(Value::number(v)));
    }
    void string(bool) {
        std::string s;
        for (std::size_t i = 0; i < prev_.text.size(); ++i) {
            char c = prev_.text[i];
            if (c == '\\' && i + 1 < prev_.text.size()) {
                char e = prev_.text[++i];
                switch (e) {
                    case 'n': s += '\n'; break;
                    case 't': s += '\t'; break;
                    case '"': s += '"'; break;
                    case '\\': s += '\\'; break;
                    default: error(std::string("unknown escape '\\") + e + "'"); return;
                }
            } else {
                s += c;
            }
        }
        emit_op16(Op::Const, make_const(Value::string(std::move(s))));
    }
    void literal(bool) {
        switch (prev_.type) {
            case Tok::True: emit(Op::True); break;
            case Tok::False: emit(Op::False); break;
            default: emit(Op::Nil); break;
        }
    }
    void grouping(bool) {
        expression();
        consume(Tok::RParen, "expected ')' after expression");
    }
    void list(bool) {
        int n = 0;
        if (!check(Tok::RBracket)) {
            do {
                if (check(Tok::RBracket)) break;  // trailing comma
                expression();
                ++n;
            } while (match(Tok::Comma));
        }
        consume(Tok::RBracket, "expected ']' after list elements");
        emit_op16(Op::List, n);
    }
    void lambda(bool) { function("<lambda>"); }
    void unary(bool) {
        Tok op = prev_.type;
        parse(P_Unary);
        emit(op == Tok::Minus ? Op::Neg : Op::Not);
    }
    void binary(bool) {
        Tok op = prev_.type;
        parse(static_cast<Prec>(rule(op).prec + 1));
        switch (op) {
            case Tok::Plus: emit(Op::Add); break;
            case Tok::Minus: emit(Op::Sub); break;
            case Tok::Star: emit(Op::Mul); break;
            case Tok::Slash: emit(Op::Div); break;
            case Tok::Percent: emit(Op::Mod); break;
            case Tok::EqEq: emit(Op::Eq); break;
            case Tok::BangEq: emit(Op::Ne); break;
            case Tok::Less: emit(Op::Lt); break;
            case Tok::LessEq: emit(Op::Le); break;
            case Tok::Greater: emit(Op::Gt); break;
            case Tok::GreaterEq: emit(Op::Ge); break;
            default: break;
        }
    }
    void and_(bool) {
        int end = emit_jump(Op::JumpIfFalse);
        emit(Op::Pop);
        parse(P_And);
        patch_jump(end);
    }
    void or_(bool) {
        int else_j = emit_jump(Op::JumpIfFalse);
        int end = emit_jump(Op::Jump);
        patch_jump(else_j);
        emit(Op::Pop);
        parse(P_Or);
        patch_jump(end);
    }
    int arguments() {
        int argc = 0;
        if (!check(Tok::RParen)) {
            do {
                expression();
                if (++argc > 255) error("too many arguments");
            } while (match(Tok::Comma));
        }
        consume(Tok::RParen, "expected ')' after arguments");
        return argc;
    }
    void call(bool) {
        int argc = arguments();
        emit(Op::Call);
        emit_byte(static_cast<std::uint8_t>(argc));
    }
    void index(bool can_assign) {
        expression();
        consume(Tok::RBracket, "expected ']' after index");
        if (can_assign && match(Tok::Eq)) {
            expression();
            emit(Op::SetIndex);
        } else {
            emit(Op::GetIndex);
        }
    }
    void dot(bool can_assign) {
        consume(Tok::Ident, "expected property name after '.'");
        int name = name_const(prev_.text);
        if (can_assign && match(Tok::Eq)) {
            expression();
            emit_op16(Op::SetProp, name);
        } else if (match(Tok::LParen)) {
            int argc = arguments();
            emit_op16(Op::Invoke, name);
            emit_byte(static_cast<std::uint8_t>(argc));
        } else {
            emit_op16(Op::GetProp, name);
        }
    }
    void variable(bool can_assign) {
        std::string_view name = prev_.text;
        Op get, set;
        int arg = resolve_local(fs_, name, this);
        bool wide = false;
        if (arg != -1) {
            get = Op::GetLocal;
            set = Op::SetLocal;
        } else if ((arg = resolve_upval(fs_, name)) != -1) {
            get = Op::GetUpval;
            set = Op::SetUpval;
        } else {
            arg = name_const(name);
            get = Op::GetGlobal;
            set = Op::SetGlobal;
            wide = true;
        }
        bool assign = can_assign && match(Tok::Eq);
        if (assign) expression();
        Op op = assign ? set : get;
        if (wide) emit_op16(op, arg);
        else {
            emit(op);
            emit_byte(static_cast<std::uint8_t>(arg));
        }
    }

    Lexer lex_;
    Token cur_{Tok::Eof, {}, 1, {}}, prev_{Tok::Eof, {}, 1, {}};
    FnState* fs_ = nullptr;
    std::vector<std::string> errors_;
    bool panic_ = false;
};

}  // namespace

CompileResult compile(std::string_view source, const std::string& name) { return Compiler(source).run(name); }

std::string disassemble(const FunctionObj& fn) {
    static const char* names[] = {"CONST", "NIL", "TRUE", "FALSE", "POP", "GET_LOCAL", "SET_LOCAL", "GET_GLOBAL",
                                  "DEF_GLOBAL", "SET_GLOBAL", "GET_UPVAL", "SET_UPVAL", "GET_PROP", "SET_PROP",
                                  "GET_INDEX", "SET_INDEX", "EQ", "NE", "LT", "LE", "GT", "GE", "ADD", "SUB", "MUL",
                                  "DIV", "MOD", "NEG", "NOT", "PRINT", "JUMP", "JUMP_IF_FALSE", "LOOP", "CALL",
                                  "INVOKE", "CLOSURE", "CLOSE_UPVAL", "RETURN", "LIST"};
    std::ostringstream os;
    const Chunk& c = fn.chunk;
    os << "== " << (fn.name.empty() ? "<script>" : fn.name) << " ==\n";
    for (std::size_t i = 0; i < c.code.size();) {
        Op op = static_cast<Op>(c.code[i]);
        os << i << "\t" << names[c.code[i]];
        auto u16 = [&](std::size_t at) { return (c.code[at] << 8) | c.code[at + 1]; };
        switch (op) {
            case Op::Const: case Op::GetGlobal: case Op::DefGlobal: case Op::SetGlobal: case Op::GetProp:
            case Op::SetProp:
                os << " " << to_display(c.constants[u16(i + 1)]);
                i += 3;
                break;
            case Op::List: os << " " << u16(i + 1); i += 3; break;
            case Op::Jump: case Op::JumpIfFalse: os << " -> " << i + 3 + u16(i + 1); i += 3; break;
            case Op::Loop: os << " -> " << i + 3 - u16(i + 1); i += 3; break;
            case Op::GetLocal: case Op::SetLocal: case Op::GetUpval: case Op::SetUpval: case Op::Call:
                os << " " << int(c.code[i + 1]);
                i += 2;
                break;
            case Op::Invoke: os << " " << to_display(c.constants[u16(i + 1)]) << " argc=" << int(c.code[i + 3]); i += 4; break;
            case Op::Closure: {
                auto& f = static_cast<FunctionObj&>(*c.constants[u16(i + 1)].obj);
                os << " " << f.name;
                i += 3 + 2 * f.upvalue_count;
                break;
            }
            default: i += 1; break;
        }
        os << "\n";
    }
    return os.str();
}

}  // namespace ifg::script
