#include "carvetto/document/variables.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <system_error>

namespace carvetto::document {

namespace {

bool isAsciiLetter(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

bool isAsciiDigit(char c) {
    return c >= '0' && c <= '9';
}

std::string_view trim(std::string_view s) {
    const auto notSpace = [](char c) { return c != ' ' && c != '\t' && c != '\r'; };
    while (!s.empty() && !notSpace(s.front())) {
        s.remove_prefix(1);
    }
    while (!s.empty() && !notSpace(s.back())) {
        s.remove_suffix(1);
    }
    return s;
}

// Shortest round-trip representation of a double.
std::string formatDouble(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

bool parseDouble(std::string_view s, double& out) {
    const auto res = std::from_chars(s.data(), s.data() + s.size(), out);
    return res.ec == std::errc{} && res.ptr == s.data() + s.size();
}

}  // namespace

bool DocumentVariables::isValidName(std::string_view name) {
    if (name.empty() || !isAsciiLetter(name.front())) {
        return false;
    }
    for (char c : name.substr(1)) {
        if (!isAsciiLetter(c) && !isAsciiDigit(c) && c != '_') {
            return false;
        }
    }
    return true;
}

bool DocumentVariables::set(const std::string& name, double value) {
    if (!isValidName(name)) {
        return false;
    }
    values_[name] = value;
    return true;
}

std::optional<double> DocumentVariables::get(std::string_view name) const {
    const auto it = values_.find(name);
    if (it == values_.end()) {
        return std::nullopt;
    }
    return it->second;
}

bool DocumentVariables::contains(std::string_view name) const {
    return values_.find(name) != values_.end();
}

bool DocumentVariables::remove(std::string_view name) {
    const auto it = values_.find(name);
    if (it == values_.end() || referenceCount(name) > 0) {
        return false;
    }
    values_.erase(it);
    return true;
}

void DocumentVariables::forceRemove(std::string_view name) {
    const auto it = values_.find(name);
    if (it != values_.end()) {
        values_.erase(it);
    }
}

void DocumentVariables::addReference(std::string_view name) {
    if (!contains(name)) {
        return;
    }
    const auto it = references_.find(name);
    if (it == references_.end()) {
        references_.emplace(std::string(name), 1);
    } else {
        ++it->second;
    }
}

void DocumentVariables::removeReference(std::string_view name) {
    const auto it = references_.find(name);
    if (it == references_.end()) {
        return;
    }
    if (--it->second == 0) {
        references_.erase(it);
    }
}

std::size_t DocumentVariables::referenceCount(std::string_view name) const {
    const auto it = references_.find(name);
    return it == references_.end() ? 0 : it->second;
}

std::vector<std::string> DocumentVariables::names() const {
    std::vector<std::string> out;
    out.reserve(values_.size());
    for (const auto& [name, value] : values_) {
        out.push_back(name);
    }
    return out;
}

std::string DocumentVariables::toText() const {
    std::string out;
    for (const auto& [name, value] : values_) {
        out += name;
        out += " = ";
        out += formatDouble(value);
        out += '\n';
    }
    return out;
}

DocumentVariables::ImportResult DocumentVariables::importText(std::string_view text) {
    ImportResult result;
    std::size_t lineNo = 0;
    while (!text.empty()) {
        ++lineNo;
        const std::size_t nl = text.find('\n');
        std::string_view line = text.substr(0, nl);
        text = (nl == std::string_view::npos) ? std::string_view{} : text.substr(nl + 1);
        line = trim(line);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string_view::npos) {
            result.rejectedLines.push_back(lineNo);
            continue;
        }
        const std::string_view name = trim(line.substr(0, eq));
        const std::string_view valueText = trim(line.substr(eq + 1));
        double value = 0.0;
        if (!isValidName(name) || !parseDouble(valueText, value)) {
            result.rejectedLines.push_back(lineNo);
            continue;
        }
        values_[std::string(name)] = value;
        ++result.imported;
    }
    return result;
}

}  // namespace carvetto::document

namespace carvetto::document {

namespace {

constexpr double kPi = 3.14159265358979323846;

class Parser {
   public:
    Parser(std::string_view t, const DocumentVariables& v, const ExpressionContext& c,
           ExpressionResult& r)
        : t_(t), vars_(v), ctx_(c), res_(r) {}

    void run() {
        skipSpace();
        if (pos_ < t_.size() && t_[pos_] == '=') {
            ++pos_;
        }
        const double v = expr(1);
        if (failed_) {
            return;
        }
        skipSpace();
        if (pos_ < t_.size()) {
            fail("unexpected character");
            return;
        }
        if (!std::isfinite(v)) {
            fail("result is not finite");
            return;
        }
        res_.value = v;
    }

   private:
    void fail(const char* m) {
        if (!failed_) {
            failed_ = true;
            res_.error = m;
        }
    }
    void skipSpace() {
        while (pos_ < t_.size() && (t_[pos_] == ' ' || t_[pos_] == '\t')) {
            ++pos_;
        }
    }
    bool peek(char c) {
        skipSpace();
        return pos_ < t_.size() && t_[pos_] == c;
    }

    // Additive level (1) and multiplicative level (2); unary and power below.
    double expr(int level) {
        if (++depth_ > kMaxExpressionDepth) {
            fail("expression nested too deeply");
            --depth_;
            return 0.0;
        }
        double v = level == 1 ? term() : unary();
        while (!failed_) {
            skipSpace();
            if (pos_ >= t_.size()) {
                break;
            }
            const char op = t_[pos_];
            if (level == 1 && (op == '+' || op == '-')) {
                ++pos_;
                const double r = term();
                v = op == '+' ? v + r : v - r;
            } else if (level == 2 && (op == '*' || op == '/')) {
                ++pos_;
                const double r = unary();
                if (op == '/' && r == 0.0) {
                    fail("division by zero");
                    break;
                }
                v = op == '*' ? v * r : v / r;
            } else {
                break;
            }
        }
        --depth_;
        return failed_ ? 0.0 : v;
    }
    double term() { return expr(2); }

    double unary() {
        // Sign chain handled iteratively so "-----...1" cannot recurse.
        bool neg = false;
        std::size_t signs = 0;
        while (!failed_ && (peek('-') || peek('+'))) {
            neg = (t_[pos_] == '-') ? !neg : neg;
            ++pos_;
            if (++signs > kMaxExpressionLength) {
                fail("too many signs");
                return 0.0;
            }
        }
        const double v = power();
        return neg ? -v : v;
    }

    double power() {
        const double base = postfix();
        if (failed_ || !peek('^')) {
            return base;
        }
        ++pos_;
        if (++depth_ > kMaxExpressionDepth) {
            fail("expression nested too deeply");
            --depth_;
            return 0.0;
        }
        // Right-associative; the exponent may carry a sign.
        bool neg = false;
        while (peek('-') || peek('+')) {
            neg = (t_[pos_] == '-') ? !neg : neg;
            ++pos_;
        }
        double e = power();
        --depth_;
        if (failed_) {
            return 0.0;
        }
        if (neg) {
            e = -e;
        }
        const double r = std::pow(base, e);
        if (!std::isfinite(r)) {
            fail("power is not finite");
            return 0.0;
        }
        return r;
    }

    double postfix() {
        double v = primary();
        while (!failed_ && peek('\'')) {
            ++pos_;
            v *= ctx_.feet;
        }
        return v;
    }

    double primary() {
        skipSpace();
        if (pos_ >= t_.size()) {
            fail("unexpected end of expression");
            return 0.0;
        }
        const char c = t_[pos_];
        if (c == '(') {
            ++pos_;
            const double v = expr(1);
            if (failed_) {
                return 0.0;
            }
            if (!peek(')')) {
                fail("missing closing parenthesis");
                return 0.0;
            }
            ++pos_;
            return v;
        }
        if (c == '{') {
            const std::size_t close = t_.find('}', pos_);
            if (close == std::string_view::npos) {
                fail("missing closing brace");
                return 0.0;
            }
            const std::string_view name = t_.substr(pos_ + 1, close - pos_ - 1);
            pos_ = close + 1;
            if (!DocumentVariables::isValidName(name)) {
                fail("invalid variable name");
                return 0.0;
            }
            if (std::find(res_.usedVariables.begin(), res_.usedVariables.end(), name) ==
                res_.usedVariables.end()) {
                res_.usedVariables.emplace_back(name);
            }
            const auto val = vars_.get(name);
            if (!val) {
                fail("undefined document variable");
                return 0.0;
            }
            return *val;
        }
        if (isAsciiDigit(c) || c == '.') {
            return number();
        }
        if (isAsciiLetter(c)) {
            std::size_t end = pos_;
            while (end < t_.size() && (isAsciiLetter(t_[end]) || isAsciiDigit(t_[end]) || t_[end] == '_')) {
                ++end;
            }
            if (end - pos_ != 1) {
                fail("unknown name (use {Name} for document variables)");
                return 0.0;
            }
            ++pos_;
            switch (c) {
                case 'W': case 'w': case 'X': case 'x': return ctx_.width;
                case 'H': case 'h': case 'Y': case 'y': return ctx_.height;
                case 'T': case 't': case 'Z': case 'z': return ctx_.thickness;
                case 'P': case 'p': return kPi;
                case 'I': case 'i': return 1.0 / 25.4;
                case 'M': case 'm': return 25.4;
                default: fail("unknown letter variable"); return 0.0;
            }
        }
        fail("unexpected character");
        return 0.0;
    }

    double number() {
        std::size_t end = pos_;
        bool digits = false;
        while (end < t_.size() && isAsciiDigit(t_[end])) { ++end; digits = true; }
        if (end < t_.size() && t_[end] == '.') {
            ++end;
            while (end < t_.size() && isAsciiDigit(t_[end])) { ++end; digits = true; }
        }
        if (!digits) {
            fail("malformed number");
            return 0.0;
        }
        if (end < t_.size() && (t_[end] == 'e' || t_[end] == 'E')) {
            std::size_t e = end + 1;
            if (e < t_.size() && (t_[e] == '+' || t_[e] == '-')) { ++e; }
            if (e < t_.size() && isAsciiDigit(t_[e])) {
                while (e < t_.size() && isAsciiDigit(t_[e])) { ++e; }
                end = e;
            }
        }
        double v = 0.0;
        const auto sv = t_.substr(pos_, end - pos_);
        const auto r = std::from_chars(sv.data(), sv.data() + sv.size(), v);
        if (r.ec != std::errc{} || r.ptr != sv.data() + sv.size() || !std::isfinite(v)) {
            fail("malformed number");
            return 0.0;
        }
        pos_ = end;
        return v;
    }

    std::string_view t_;
    const DocumentVariables& vars_;
    const ExpressionContext& ctx_;
    ExpressionResult& res_;
    std::size_t pos_{0};
    int depth_{0};
    bool failed_{false};
};

}  // namespace

ExpressionResult evaluateExpression(std::string_view text, const DocumentVariables& vars,
                                    const ExpressionContext& ctx) {
    ExpressionResult res;
    if (text.size() > kMaxExpressionLength) {
        res.error = "expression too long";
        return res;
    }
    Parser(text, vars, ctx, res).run();
    std::sort(res.usedVariables.begin(), res.usedVariables.end());
    if (!res.value && res.error.empty()) {
        res.error = "invalid expression";
    }
    return res;
}

}  // namespace carvetto::document
