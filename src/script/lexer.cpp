#include "script/lexer.hpp"

#include <cctype>
#include <unordered_map>

namespace ifg::script {

static bool is_ident_start(char c) { return std::isalpha(static_cast<unsigned char>(c)) || c == '_'; }
static bool is_digit(char c) { return c >= '0' && c <= '9'; }

Token Lexer::next() {
    // skip whitespace and comments
    for (;;) {
        char c = peek();
        if (c == '\n') { ++line_; ++pos_; }
        else if (c == ' ' || c == '\t' || c == '\r') ++pos_;
        else if (c == '#' || (c == '/' && peek(1) == '/')) { while (peek() && peek() != '\n') ++pos_; }
        else break;
    }
    std::size_t start = pos_;
    if (pos_ >= src_.size()) return {Tok::Eof, {}, line_, {}};
    char c = src_[pos_++];
    if (is_ident_start(c)) {
        while (is_ident_start(peek()) || is_digit(peek())) ++pos_;
        static const std::unordered_map<std::string_view, Tok> kw = {
            {"and", Tok::And}, {"or", Tok::Or}, {"let", Tok::Let}, {"fn", Tok::Fn}, {"return", Tok::Return},
            {"if", Tok::If}, {"else", Tok::Else}, {"while", Tok::While}, {"for", Tok::For}, {"break", Tok::Break},
            {"continue", Tok::Continue}, {"true", Tok::True}, {"false", Tok::False}, {"nil", Tok::Nil},
            {"print", Tok::Print}};
        auto t = make(Tok::Ident, start);
        auto it = kw.find(t.text);
        if (it != kw.end()) t.type = it->second;
        return t;
    }
    if (is_digit(c)) {
        while (is_digit(peek())) ++pos_;
        if (peek() == '.' && is_digit(peek(1))) {
            ++pos_;
            while (is_digit(peek())) ++pos_;
        }
        return make(Tok::Number, start);
    }
    switch (c) {
        case '(': return make(Tok::LParen, start);
        case ')': return make(Tok::RParen, start);
        case '{': return make(Tok::LBrace, start);
        case '}': return make(Tok::RBrace, start);
        case '[': return make(Tok::LBracket, start);
        case ']': return make(Tok::RBracket, start);
        case ',': return make(Tok::Comma, start);
        case '.': return make(Tok::Dot, start);
        case ';': return make(Tok::Semicolon, start);
        case ':': return make(Tok::Colon, start);
        case '+': return make(Tok::Plus, start);
        case '-': return make(Tok::Minus, start);
        case '*': return make(Tok::Star, start);
        case '/': return make(Tok::Slash, start);
        case '%': return make(Tok::Percent, start);
        case '!': if (peek() == '=') { ++pos_; return make(Tok::BangEq, start); } return make(Tok::Bang, start);
        case '=': if (peek() == '=') { ++pos_; return make(Tok::EqEq, start); } return make(Tok::Eq, start);
        case '<': if (peek() == '=') { ++pos_; return make(Tok::LessEq, start); } return make(Tok::Less, start);
        case '>': if (peek() == '=') { ++pos_; return make(Tok::GreaterEq, start); } return make(Tok::Greater, start);
        case '"': {
            int start_line = line_;
            while (peek() && peek() != '"') {
                if (peek() == '\n') ++line_;
                if (peek() == '\\' && peek(1)) ++pos_;
                ++pos_;
            }
            if (!peek()) return {Tok::Error, src_.substr(start, pos_ - start), start_line, "unterminated string"};
            ++pos_;
            return {Tok::String, src_.substr(start + 1, pos_ - start - 2), start_line, {}};
        }
        default: break;
    }
    return {Tok::Error, src_.substr(start, 1), line_, std::string("unexpected character '") + c + "'"};
}

std::vector<Token> Lexer::tokenize(std::string_view src) {
    Lexer lx(src);
    std::vector<Token> out;
    for (;;) {
        out.push_back(lx.next());
        if (out.back().type == Tok::Eof) break;
    }
    return out;
}

const char* tok_name(Tok t) {
    static const char* names[] = {"(", ")", "{", "}", "[", "]", ",", ".", ";", ":", "+", "-", "*", "/", "%",
                                  "!", "!=", "=", "==", "<", "<=", ">", ">=", "identifier", "string", "number",
                                  "and", "or", "let", "fn", "return", "if", "else", "while", "for", "break",
                                  "continue", "true", "false", "nil", "print", "error", "end of file"};
    return names[static_cast<int>(t)];
}

}  // namespace ifg::script
