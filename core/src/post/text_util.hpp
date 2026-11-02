#pragma once

// Private string helpers shared by the post module sources.

#include <cctype>
#include <charconv>
#include <cstddef>
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

/// The comment position at the end of an output line. While everything written since the
/// comment opened (template text, VAR prefixes and earlier values alike) is only letters, digits,
/// `_` and spaces, `plain` is set and `lead` holds that text's word characters, upper-cased with
/// the spaces dropped: a value written there continues a word that could still become an active
/// comment keyword.
struct CommentPosition {
    CommentContext context{CommentContext::None};
    bool plain{false};
    std::string lead;
};

/// True for ASCII control characters (0x00-0x1F, 0x7F).
inline bool isControl(char c) {
    const auto u = static_cast<unsigned char>(c);
    return u < 0x20 || u == 0x7F;
}

/// True for the characters an active-comment keyword is made of (ASCII letters, digits, `_`).
inline bool isWordChar(char c) {
    return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
}

/// The comment position at the end of `before` (the output line so far). `(` opens a comment
/// and `)` closes it; `;` outside parentheses comments out the rest of the line.
inline CommentPosition commentPositionAt(std::string_view before) {
    CommentPosition pos;
    for (char c : before) {
        if (pos.context == CommentContext::Paren && c == ')') {
            pos = {};
        } else if (pos.context != CommentContext::None) {
            if (isWordChar(c)) {
                pos.lead += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            } else if (c != ' ' && !isControl(c)) {
                // Every control character counts as a space here, as LinuxCNC skips all
                // whitespace in a comment's keyword; the parser refuses them in templates anyway.
                pos.plain = false;
            }
        } else if (c == '(') {
            pos = {CommentContext::Paren, true, {}};
        } else if (c == ';') {
            pos = {CommentContext::Line, true, {}};
        }
    }
    if (!pos.plain) {
        pos.lead.clear();
    }
    return pos;
}

/// The plain ASCII letter for a Latin-1 letter U+00C0..U+00FF, or '_' (×, ÷, Æ, ß, Ð, Þ …).
inline char asciiFold(unsigned codePoint) {
    // Indexed by codePoint - 0xC0.
    static constexpr char kFold[] =
        "AAAAAA_CEEEEIIII_NOOOOO_OUUUUY__"
        "aaaaaa_ceeeeiiii_nooooo_ouuuuy_y";
    return codePoint >= 0xC0 && codePoint <= 0xFF ? kFold[codePoint - 0xC0] : '_';
}

/// Length of the well-formed UTF-8 sequence starting at `s[i]` (a byte >= 0x80) and its code
/// point; 0 when the bytes there are not a well-formed sequence.
inline std::size_t utf8Sequence(std::string_view s, std::size_t i, unsigned& codePoint) {
    const auto b = [&](std::size_t k) { return static_cast<unsigned>(static_cast<unsigned char>(s[k])); };
    const unsigned lead = b(i);
    std::size_t len = 0;
    unsigned lo = 0x80;
    unsigned hi = 0xBF;  // allowed range of the first continuation byte
    if (lead >= 0xC2 && lead <= 0xDF) {
        len = 2;
        codePoint = lead & 0x1Fu;
    } else if (lead >= 0xE0 && lead <= 0xEF) {
        len = 3;
        codePoint = lead & 0x0Fu;
        lo = lead == 0xE0 ? 0xA0 : 0x80;
        hi = lead == 0xED ? 0x9F : 0xBF;
    } else if (lead >= 0xF0 && lead <= 0xF4) {
        len = 4;
        codePoint = lead & 0x07u;
        lo = lead == 0xF0 ? 0x90 : 0x80;
        hi = lead == 0xF4 ? 0x8F : 0xBF;
    } else {
        return 0;
    }
    if (i + len > s.size()) {
        return 0;
    }
    for (std::size_t k = 1; k < len; ++k) {
        const unsigned c = b(i + k);
        if (c < (k == 1 ? lo : 0x80u) || c > (k == 1 ? hi : 0xBFu)) {
            return 0;
        }
        codePoint = (codePoint << 6) | (c & 0x3Fu);
    }
    return len;
}

/// Makes a text value safe to write into one G-code line, for every controller (T-0133,
/// ADR-0020). The result is printable ASCII:
/// - every control character (0x00-0x1F, 0x7F: LF, CR, tab, NUL …) becomes a space, so the value
///   can never start a new block;
/// - `~`, `!` and `?` (Grbl realtime commands, acted on even inside comments) become `_`;
/// - non-ASCII: a well-formed UTF-8 Latin-1 letter becomes its plain letter (Ñ -> N, é -> e),
///   any other well-formed character one `_`, and every byte of a malformed sequence `_`, so no
///   byte >= 0x80 (Grbl realtime commands again) is written;
/// - inside an open `( … )` comment, `(` becomes `[` and `)` becomes `]`, so the value can neither
///   close the comment early nor nest one;
/// - while the comment so far holds only letters, digits, `_` and spaces (`pos.plain`; template
///   text, prefixes and earlier values included), the comment so far plus this value is judged
///   as one: every comma in this value that continues the word run becomes `_`, and when
///   this value completes a leading PROBEOPEN / PROBECLOSE / LOGCLOSE (case and spaces ignored)
///   it gets a `_` in front, so values cannot form an active comment such as LinuxCNC's
///   `(LOGOPEN,file)` or `(MSG,text)`, alone or together.
/// Runs after the post's SUBSTITUTE pairs, so no SUBSTITUTE setting can undo it.
inline std::string sanitiseText(std::string_view text, CommentPosition pos) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size();) {
        const auto u = static_cast<unsigned char>(text[i]);
        if (u >= 0x80) {
            unsigned codePoint = 0;
            const std::size_t len = utf8Sequence(text, i, codePoint);
            out += len > 0 ? asciiFold(codePoint) : '_';
            i += len > 0 ? len : 1;
            continue;
        }
        char c = text[i];
        if (u < 0x20 || u == 0x7F) {
            c = ' ';
        } else if (c == '~' || c == '!' || c == '?') {
            c = '_';
        } else if (pos.context == CommentContext::Paren && c == '(') {
            c = '[';
        } else if (pos.context == CommentContext::Paren && c == ')') {
            c = ']';
        }
        out += c;
        ++i;
    }
    if (pos.context != CommentContext::None && pos.plain) {
        // The comment so far is one run of words; judge it together with this value, and
        // neutralise inside this value.
        std::string word = pos.lead;
        std::size_t k = 0;
        for (; k < out.size(); ++k) {
            if (out[k] == ',') {
                out[k] = '_';  // a word char: the run goes on, so every comma in it is replaced
            }
            if (isWordChar(out[k])) {
                word += static_cast<char>(std::toupper(static_cast<unsigned char>(out[k])));
            } else if (out[k] != ' ') {
                break;
            }
        }
        for (std::string_view keyword : {"PROBEOPEN", "PROBECLOSE", "LOGCLOSE"}) {
            // Only when this value completes the keyword: one the template wrote by itself
            // before the value is the post author's own text.
            if (pos.lead.size() < keyword.size() && word.compare(0, keyword.size(), keyword) == 0) {
                out.insert(0, 1, '_');
                break;
            }
        }
    }
    return out;
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
