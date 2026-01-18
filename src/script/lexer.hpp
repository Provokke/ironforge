#pragma once
#include <string>
#include <string_view>
#include <vector>

namespace ifg::script {

enum class Tok {
    LParen, RParen, LBrace, RBrace, LBracket, RBracket, Comma, Dot, Semicolon, Colon,
    Plus, Minus, Star, Slash, Percent,
    Bang, BangEq, Eq, EqEq, Less, LessEq, Greater, GreaterEq,
    Ident, String, Number,
    And, Or, Let, Fn, Return, If, Else, While, For, Break, Continue, True, False, Nil, Print,
    Error, Eof
};

struct Token {
    Tok type;
    std::string_view text;  // lexeme (for String: without quotes, escapes unprocessed)
    int line;
    std::string error;      // for Tok::Error
};

class Lexer {
public:
    explicit Lexer(std::string_view src) : src_(src) {}
    Token next();
    static std::vector<Token> tokenize(std::string_view src);  // includes trailing Eof

private:
    char peek(int o = 0) const { return pos_ + o < src_.size() ? src_[pos_ + o] : '\0'; }
    Token make(Tok t, std::size_t start) { return {t, src_.substr(start, pos_ - start), line_, {}}; }
    std::string_view src_;
    std::size_t pos_ = 0;
    int line_ = 1;
};

const char* tok_name(Tok t);

}  // namespace ifg::script
