#include "dxf_text.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "carvetto/io/dxf.hpp"

namespace carvetto::io::detail {

void appendUtf8(std::string& out, char32_t cp) {
    if (cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) {
        cp = 0xFFFD;
    }
    auto byte = [&out](std::uint32_t v) { out.push_back(static_cast<char>(static_cast<unsigned char>(v))); };
    const auto c = static_cast<std::uint32_t>(cp);
    if (c < 0x80) {
        byte(c);
    } else if (c < 0x800) {
        byte(0xC0 | (c >> 6));
        byte(0x80 | (c & 0x3F));
    } else if (c < 0x10000) {
        byte(0xE0 | (c >> 12));
        byte(0x80 | ((c >> 6) & 0x3F));
        byte(0x80 | (c & 0x3F));
    } else {
        byte(0xF0 | (c >> 18));
        byte(0x80 | ((c >> 12) & 0x3F));
        byte(0x80 | ((c >> 6) & 0x3F));
        byte(0x80 | (c & 0x3F));
    }
}

namespace {

// Windows-1252 code points for bytes 0x80-0x9F (0 = undefined, mapped to U+FFFD).
constexpr std::array<char32_t, 32> kCp1252High{
    0x20AC, 0,      0x201A, 0x0192, 0x201E, 0x2026, 0x2020, 0x2021, 0x02C6, 0x2030, 0x0160,
    0x2039, 0x0152, 0,      0x017D, 0,      0,      0x2018, 0x2019, 0x201C, 0x201D, 0x2022,
    0x2013, 0x2014, 0x02DC, 0x2122, 0x0161, 0x203A, 0x0153, 0,      0x017E, 0x0178};

char32_t cp1252(unsigned char b) {
    if (b >= 0x80 && b < 0xA0) {
        const char32_t c = kCp1252High[b - 0x80];
        return c == 0 ? char32_t{0xFFFD} : c;
    }
    return b;
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/// Parses "\U+XXXX" at `s[i]`; returns the code point and sets `len`, or 0.
char32_t unicodeEscape(std::string_view s, std::size_t i, std::size_t& len) {
    if (i + 7 > s.size() || s[i] != '\\' || (s[i + 1] != 'U' && s[i + 1] != 'u') || s[i + 2] != '+') {
        return 0;
    }
    std::uint32_t v = 0;
    for (std::size_t k = 0; k < 4; ++k) {
        const int h = hexValue(s[i + 3 + k]);
        if (h < 0) {
            return 0;
        }
        v = v * 16u + static_cast<std::uint32_t>(h);
    }
    len = 7;
    // Surrogate pair written as two escapes.
    if (v >= 0xD800 && v <= 0xDBFF) {
        std::size_t len2 = 0;
        const char32_t lo = unicodeEscape(s, i + 7, len2);
        if (lo >= 0xDC00 && lo <= 0xDFFF) {
            len = 14;
            return static_cast<char32_t>(0x10000 + ((v - 0xD800) << 10) +
                                         (static_cast<std::uint32_t>(lo) - 0xDC00));
        }
        return 0xFFFD;
    }
    return v == 0 ? char32_t{0xFFFD} : static_cast<char32_t>(v);
}

/// Decodes one UTF-8 sequence at `s[i]`; invalid input yields U+FFFD and advances one byte.
char32_t utf8At(std::string_view s, std::size_t i, std::size_t& len) {
    const auto b0 = static_cast<unsigned char>(s[i]);
    len = 1;
    if (b0 < 0x80) {
        return b0;
    }
    std::size_t need = 0;
    std::uint32_t cp = 0;
    if ((b0 & 0xE0) == 0xC0) {
        need = 1;
        cp = b0 & 0x1Fu;
    } else if ((b0 & 0xF0) == 0xE0) {
        need = 2;
        cp = b0 & 0x0Fu;
    } else if ((b0 & 0xF8) == 0xF0) {
        need = 3;
        cp = b0 & 0x07u;
    } else {
        return 0xFFFD;
    }
    if (i + need >= s.size()) {
        return 0xFFFD;
    }
    for (std::size_t k = 1; k <= need; ++k) {
        const auto b = static_cast<unsigned char>(s[i + k]);
        if ((b & 0xC0) != 0x80) {
            return 0xFFFD;
        }
        cp = (cp << 6) | (b & 0x3Fu);
    }
    static constexpr std::array<std::uint32_t, 4> kMin{0, 0x80, 0x800, 0x10000};
    if (cp < kMin[need] || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) {
        return 0xFFFD;
    }
    len = need + 1;
    return static_cast<char32_t>(cp);
}

}  // namespace

std::string decodeDxfString(std::string_view raw, bool utf8) {
    std::string out;
    out.reserve(raw.size());
    std::size_t i = 0;
    while (i < raw.size()) {
        std::size_t len = 0;
        if (raw[i] == '\\') {
            const char32_t cp = unicodeEscape(raw, i, len);
            if (cp != 0) {
                appendUtf8(out, cp);
                i += len;
                continue;
            }
        }
        if (utf8) {
            appendUtf8(out, utf8At(raw, i, len));
            i += len;
        } else {
            appendUtf8(out, cp1252(static_cast<unsigned char>(raw[i])));
            ++i;
        }
    }
    return out;
}

std::string decodeControlCodes(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    std::size_t i = 0;
    while (i < text.size()) {
        if (text[i] == '%' && i + 2 < text.size() && text[i + 1] == '%') {
            const char c = text[i + 2];
            switch (c) {
                case 'd':
                case 'D':
                    appendUtf8(out, 0x00B0);
                    i += 3;
                    continue;
                case 'p':
                case 'P':
                    appendUtf8(out, 0x00B1);
                    i += 3;
                    continue;
                case 'c':
                case 'C':
                    appendUtf8(out, 0x2300);
                    i += 3;
                    continue;
                case '%':
                    out.push_back('%');
                    i += 3;
                    continue;
                case 'u':
                case 'U':
                case 'o':
                case 'O':
                case 'k':
                case 'K':
                    i += 3;
                    continue;
                default:
                    break;
            }
            if (c >= '0' && c <= '9') {
                std::uint32_t v = 0;
                std::size_t k = i + 2;
                while (k < text.size() && k < i + 5 && text[k] >= '0' && text[k] <= '9') {
                    v = v * 10u + static_cast<std::uint32_t>(text[k] - '0');
                    ++k;
                }
                appendUtf8(out, v > 0 && v < 256 ? cp1252(static_cast<unsigned char>(v)) : char32_t{0xFFFD});
                i = k;
                continue;
            }
        }
        out.push_back(text[i]);
        ++i;
    }
    return out;
}

std::string encodeDxfString(std::string_view utf8) {
    std::string out;
    out.reserve(utf8.size());
    std::size_t i = 0;
    auto escape = [&out](std::uint32_t v) {
        static constexpr char kHex[] = "0123456789ABCDEF";
        out += "\\U+";
        for (int s = 12; s >= 0; s -= 4) {
            out.push_back(kHex[(v >> s) & 0xFu]);
        }
    };
    while (i < utf8.size()) {
        std::size_t len = 0;
        const char32_t cp = utf8At(utf8, i, len);
        i += len;
        const auto v = static_cast<std::uint32_t>(cp);
        if (v < 0x20 || v == 0x7F) {
            out.push_back('_');
        } else if (v == '\\' && i < utf8.size() && (utf8[i] == 'U' || utf8[i] == 'u')) {
            escape(v);  // keep a literal backslash from being read back as an escape
        } else if (v < 0x80) {
            out.push_back(static_cast<char>(v));
        } else if (v > 0xFFFF) {
            const std::uint32_t x = v - 0x10000;
            escape(0xD800 + (x >> 10));
            escape(0xDC00 + (x & 0x3FF));
        } else {
            escape(v);
        }
    }
    return out;
}

// ------------------------------------------------------------------ MTEXT formatting

std::string stripMTextFormatting(std::string_view raw) {
    std::string out;
    out.reserve(raw.size());
    std::size_t i = 0;
    auto skipTo = [&](char terminator) {
        const std::size_t end = raw.find(terminator, i);
        i = end == std::string_view::npos ? raw.size() : end + 1;
    };
    while (i < raw.size()) {
        const char c = raw[i];
        if (c == '{' || c == '}') {
            ++i;
            continue;
        }
        if (c != '\\' || i + 1 >= raw.size()) {
            out.push_back(c);
            ++i;
            continue;
        }
        const char code = raw[i + 1];
        switch (code) {
            case 'P':
            case 'N':
            case 'X':
                out.push_back('\n');
                i += 2;
                break;
            case '~':
                appendUtf8(out, 0x00A0);
                i += 2;
                break;
            case '\\':
            case '{':
            case '}':
                out.push_back(code);
                i += 2;
                break;
            case 'L':
            case 'l':
            case 'O':
            case 'o':
            case 'K':
            case 'k':
                i += 2;
                break;
            case 'U':
            case 'u': {
                std::size_t len = 0;
                const char32_t cp = unicodeEscape(raw, i, len);
                if (cp != 0) {
                    appendUtf8(out, cp);
                    i += len;
                } else {
                    i += 2;
                }
                break;
            }
            case 'M':
            case 'm':
                // \M+nXXXX: multibyte character in an Asian code page; not decodable here.
                i = std::min(raw.size(), i + 8);
                appendUtf8(out, 0xFFFD);
                break;
            case 'S': {
                // Stacked fraction \Sa^b; / \Sa/b; / \Sa#b; -> "a/b".
                i += 2;
                const std::size_t end = raw.find(';', i);
                const std::size_t stop = end == std::string_view::npos ? raw.size() : end;
                for (std::size_t k = i; k < stop; ++k) {
                    const char s = raw[k];
                    if (s == '^' || s == '#') {
                        out.push_back('/');
                    } else if (s == '\\' && k + 1 < stop) {
                        out.push_back(raw[++k]);
                    } else {
                        out.push_back(s);
                    }
                }
                i = end == std::string_view::npos ? raw.size() : end + 1;
                break;
            }
            case 'A':
            case 'a':
            case 'C':
            case 'c':
            case 'F':
            case 'f':
            case 'H':
            case 'h':
            case 'Q':
            case 'q':
            case 'T':
            case 't':
            case 'W':
            case 'w':
            case 'p':
                i += 2;
                skipTo(';');
                break;
            default:
                // Unknown escape: keep the character, drop the backslash.
                out.push_back(code);
                i += 2;
                break;
        }
    }
    return out;
}

}  // namespace carvetto::io::detail

namespace carvetto::io {

std::string stripMTextFormatting(std::string_view raw) {
    return detail::stripMTextFormatting(raw);
}

}  // namespace carvetto::io
