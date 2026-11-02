#pragma once

// Private string helpers shared by the post module sources.

#include <cctype>
#include <charconv>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

namespace carvetto::post::detail {

inline bool isSpace(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v';
}

inline std::string_view trim(std::string_view s) {
    while (!s.empty() && isSpace(s.front())) {
        s.remove_prefix(1);
    }
    while (!s.empty() && isSpace(s.back())) {
        s.remove_suffix(1);
    }
    return s;
}

inline std::string upper(std::string_view s) {
    std::string out(s);
    for (char& c : out) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return out;
}

inline bool iequals(std::string_view a, std::string_view b) {
    return upper(a) == upper(b);
}

/// Where a text value lands in an output line, judged from the line text written before it.
enum class CommentContext {
    None,   ///< Live G-code: no comment is open.
    Paren,  ///< Inside a `( … )` comment that is still open.
    Line,   ///< After a `;`: the rest of the line is a comment.
};

/// The comment context at the end of `before` (the output line so far). `(` opens a comment
/// and `)` closes it; `;` outside parentheses comments out the rest of the line.
inline CommentContext commentContextAt(std::string_view before) {
    bool paren = false;
    for (char c : before) {
        if (paren) {
            paren = c != ')';
        } else if (c == '(') {
            paren = true;
        } else if (c == ';') {
            return CommentContext::Line;
        }
    }
    return paren ? CommentContext::Paren : CommentContext::None;
}

/// Makes a text value safe to write into one G-code line (T-0133, ADR-0020): every control
/// character (bytes 0x00-0x1F and 0x7F, which includes LF, CR, tab and NUL) becomes a space, so
/// the value can never start a new block; inside an open `( … )` comment, `(` becomes `[` and `)`
/// becomes `]`, so the value can neither close the comment early nor nest one. Runs after the
/// post's SUBSTITUTE pairs, so no SUBSTITUTE setting can undo it.
inline void sanitiseText(std::string& text, CommentContext context) {
    for (char& c : text) {
        const auto u = static_cast<unsigned char>(c);
        if (u < 0x20 || u == 0x7F) {
            c = ' ';
        } else if (context == CommentContext::Paren && c == '(') {
            c = '[';
        } else if (context == CommentContext::Paren && c == ')') {
            c = ']';
        }
    }
}

/// Parses the whole of `s` (trimmed) as a double; locale independent.
inline std::optional<double> parseDouble(std::string_view s) {
    s = trim(s);
    if (!s.empty() && s.front() == '+') {
        s.remove_prefix(1);
    }
    double v = 0.0;
    const auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    if (s.empty() || r.ec != std::errc{} || r.ptr != s.data() + s.size()) {
        return std::nullopt;
    }
    return v;
}

/// Parses the whole of `s` (trimmed) as a signed integer.
inline std::optional<long long> parseInteger(std::string_view s) {
    s = trim(s);
    if (!s.empty() && s.front() == '+') {
        s.remove_prefix(1);
    }
    long long v = 0;
    const auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    if (s.empty() || r.ec != std::errc{} || r.ptr != s.data() + s.size()) {
        return std::nullopt;
    }
    return v;
}

}  // namespace carvetto::post::detail
