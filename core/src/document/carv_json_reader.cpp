#include "carv_json_reader.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace carvetto::document::detail {

// ------------------------------------------------------------------------------------------------
// Text helpers
// ------------------------------------------------------------------------------------------------

std::size_t StringSource::read(char* buf, std::size_t n) {
    const std::size_t take = std::min(n, text_.size() - pos_);
    if (take > 0) {
        std::memcpy(buf, text_.data() + pos_, take);
        pos_ += take;
    }
    return take;
}

namespace {

/// Length of the well-formed UTF-8 sequence starting at `s[i]`, or 0 when it is malformed.
std::size_t utf8SequenceLength(std::string_view s, std::size_t i) {
    const auto at = [&s](std::size_t k) { return static_cast<unsigned char>(s[k]); };
    const unsigned char c = at(i);
    if (c < 0x80) {
        return 1;
    }
    const std::size_t left = s.size() - i;
    const auto cont = [&](std::size_t k, unsigned char lo, unsigned char hi) {
        return k < s.size() && at(k) >= lo && at(k) <= hi;
    };
    if (c >= 0xC2 && c <= 0xDF) {
        return left >= 2 && cont(i + 1, 0x80, 0xBF) ? 2 : 0;
    }
    if (c >= 0xE0 && c <= 0xEF) {
        const auto lo = static_cast<unsigned char>(c == 0xE0 ? 0xA0 : 0x80);
        const auto hi = static_cast<unsigned char>(c == 0xED ? 0x9F : 0xBF);
        return left >= 3 && cont(i + 1, lo, hi) && cont(i + 2, 0x80, 0xBF) ? 3 : 0;
    }
    if (c >= 0xF0 && c <= 0xF4) {
        const auto lo = static_cast<unsigned char>(c == 0xF0 ? 0x90 : 0x80);
        const auto hi = static_cast<unsigned char>(c == 0xF4 ? 0x8F : 0xBF);
        return left >= 4 && cont(i + 1, lo, hi) && cont(i + 2, 0x80, 0xBF) && cont(i + 3, 0x80, 0xBF) ? 4 : 0;
    }
    return 0;
}

void appendUtf8(std::string& out, std::uint32_t cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

/// True for a token matching -?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][+-]?[0-9]+)?
bool isJsonNumber(std::string_view t) {
    std::size_t i = 0;
    const auto digit = [&t](std::size_t k) { return k < t.size() && t[k] >= '0' && t[k] <= '9'; };
    if (i < t.size() && t[i] == '-') {
        ++i;
    }
    if (!digit(i)) {
        return false;
    }
    if (t[i] == '0') {
        ++i;
    } else {
        while (digit(i)) ++i;
    }
    if (i < t.size() && t[i] == '.') {
        ++i;
        if (!digit(i)) return false;
        while (digit(i)) ++i;
    }
    if (i < t.size() && (t[i] == 'e' || t[i] == 'E')) {
        ++i;
        if (i < t.size() && (t[i] == '+' || t[i] == '-')) ++i;
        if (!digit(i)) return false;
        while (digit(i)) ++i;
    }
    return i == t.size();
}

}  // namespace

bool isValidUtf8(std::string_view s) {
    for (std::size_t i = 0; i < s.size();) {
        const std::size_t n = utf8SequenceLength(s, i);
        if (n == 0) {
            return false;
        }
        i += n;
    }
    return true;
}

void appendJsonString(std::string& out, std::string_view s) {
    static constexpr char kHex[] = "0123456789abcdef";
    out += '"';
    std::size_t i = 0;
    while (i < s.size()) {
        const auto c = static_cast<unsigned char>(s[i]);
        if (c >= 0x80) {
            const std::size_t n = utf8SequenceLength(s, i);
            if (n == 0) {
                out += "\xEF\xBF\xBD";  // U+FFFD for a malformed byte
                ++i;
            } else {
                out.append(s.data() + i, n);
                i += n;
            }
            continue;
        }
        switch (c) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\b':
                out += "\\b";
                break;
            case '\f':
                out += "\\f";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                if (c < 0x20) {
                    out += "\\u00";
                    out += kHex[c >> 4];
                    out += kHex[c & 0x0F];
                } else {
                    out += static_cast<char>(c);
                }
        }
        ++i;
    }
    out += '"';
}

void appendDouble(std::string& out, double v) {
    if (std::isnan(v)) {
        out += "\"NaN\"";
        return;
    }
    if (std::isinf(v)) {
        out += v > 0 ? "\"Infinity\"" : "\"-Infinity\"";
        return;
    }
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    const std::string_view text(buf.data(), static_cast<std::size_t>(res.ptr - buf.data()));
    out += text;
    if (text.find_first_of(".eE") == std::string_view::npos) {
        out += ".0";
    }
}

void appendCanonicalNumber(std::string& out, std::string_view token, const std::string& member) {
    const char* first = token.data();
    const char* last = token.data() + token.size();
    if (token.find_first_of(".eE") == std::string_view::npos) {
        if (token.front() == '-') {
            std::int64_t v = 0;
            if (std::from_chars(first, last, v).ec == std::errc{}) {
                out += std::to_string(v);
                return;
            }
        } else {
            std::uint64_t v = 0;
            if (std::from_chars(first, last, v).ec == std::errc{}) {
                out += std::to_string(v);
                return;
            }
        }
    }
    double d = 0.0;
    const auto res = std::from_chars(first, last, d);
    if (res.ec != std::errc{} || res.ptr != last) {
        throw LoadFailure{CarvError::Code::InvalidContent,
                          member + ": number " + std::string(token) + " is out of range"};
    }
    appendDouble(out, d);
}

// ------------------------------------------------------------------------------------------------
// JsonReader
// ------------------------------------------------------------------------------------------------

JsonReader::JsonReader(CharSource& source, LoadBudget& budget, std::string member, std::size_t bufferBytes)
    : source_(source),
      budget_(budget),
      member_(std::move(member)),
      buf_(new char[std::max<std::size_t>(bufferBytes, 16)]),
      data_(buf_.get()),
      capacity_(std::max<std::size_t>(bufferBytes, 16)) {}

bool JsonReader::refill() {
    consumed_ += len_;
    pos_ = 0;
    len_ = source_.read(data_, capacity_);
    return len_ > 0;
}

void JsonReader::syntaxError(const std::string& what) const {
    throw LoadFailure{CarvError::Code::InvalidJson,
                      member_ + ": malformed JSON at byte " + std::to_string(consumed_ + pos_) + ": " + what};
}

void JsonReader::skipWs() {
    for (;;) {
        while (pos_ < len_) {
            const char c = data_[pos_];
            if (c != ' ' && c != '\n' && c != '\r' && c != '\t') {
                return;
            }
            ++pos_;
        }
        if (!refill()) {
            return;
        }
    }
}

void JsonReader::expect(char c) {
    skipWs();
    if (peekChar() != static_cast<unsigned char>(c)) {
        syntaxError(std::string("expected '") + c + "'");
    }
    ++pos_;
}

void JsonReader::expectLiteral(std::string_view word) {
    for (const char c : word) {
        if (peekChar() != static_cast<unsigned char>(c)) {
            syntaxError("invalid literal");
        }
        ++pos_;
    }
}

JsonReader::Kind JsonReader::peek() {
    skipWs();
    const int c = peekChar();
    switch (c) {
        case '{':
            return Kind::Object;
        case '[':
            return Kind::Array;
        case '"':
            return Kind::String;
        case 't':
        case 'f':
            return Kind::Bool;
        case 'n':
            return Kind::Null;
        default:
            if (c == '-' || (c >= '0' && c <= '9')) {
                return Kind::Number;
            }
            syntaxError(c < 0 ? "unexpected end of input" : "unexpected character");
    }
}

void JsonReader::push(bool object, bool unknown) {
    budget_.chargeValue();
    if (static_cast<int>(levels_.size()) >= budget_.limits().maxDepth) {
        throw LoadFailure{
            CarvError::Code::ResourceLimit,
            member_ + ": JSON nests deeper than " + std::to_string(budget_.limits().maxDepth) + " levels"};
    }
    levels_.push_back(Level{object, true, unknown, 0});
}

void JsonReader::countElement() {
    Level& level = levels_.back();
    if (level.unknown && ++level.count > budget_.limits().maxUnknownContainerElements) {
        throw LoadFailure{CarvError::Code::ResourceLimit,
                          member_ + ": an unknown field holds more than " +
                              std::to_string(budget_.limits().maxUnknownContainerElements) + " elements"};
    }
}

void JsonReader::beginObject(bool unknown) {
    expect('{');
    push(true, unknown);
}

void JsonReader::beginArray(bool unknown) {
    expect('[');
    push(false, unknown);
}

bool JsonReader::nextKey(std::string& key) {
    skipWs();
    Level& level = levels_.back();
    const int c = peekChar();
    if (c == '}') {
        ++pos_;
        levels_.pop_back();
        return false;
    }
    if (!level.first) {
        if (c != ',') {
            syntaxError("expected ',' or '}'");
        }
        ++pos_;
        skipWs();
    }
    level.first = false;
    countElement();
    if (peekChar() != '"') {
        syntaxError("expected a string key");
    }
    budget_.chargeValue();
    if (level.unknown) {
        budget_.chargeUnknownValue();
    }
    key = readStringRaw(budget_.limits().maxStringBytes, false);
    expect(':');
    return true;
}

bool JsonReader::nextElement() {
    skipWs();
    Level& level = levels_.back();
    const int c = peekChar();
    if (c == ']') {
        ++pos_;
        levels_.pop_back();
        return false;
    }
    if (!level.first) {
        if (c != ',') {
            syntaxError("expected ',' or ']'");
        }
        ++pos_;
    }
    level.first = false;
    countElement();
    return true;
}

std::string JsonReader::readStringRaw(std::size_t limit, bool unknown) {
    // Precondition: positioned at the opening quote.
    ++pos_;
    std::string s;
    for (;;) {
        if (pos_ == len_ && !refill()) {
            syntaxError("unterminated string");
        }
        // Copy the longest run of ordinary characters in one go.
        std::size_t run = pos_;
        while (run < len_) {
            const auto ch = static_cast<unsigned char>(data_[run]);
            if (ch == '"' || ch == '\\' || ch < 0x20) {
                break;
            }
            ++run;
        }
        s.append(data_ + pos_, run - pos_);
        pos_ = run;
        if (s.size() > limit) {
            throw LoadFailure{CarvError::Code::ResourceLimit, member_ + ": a string is longer than " +
                                                                  std::to_string(limit) + " bytes" +
                                                                  (unknown ? " (unknown field)" : "")};
        }
        if (pos_ == len_) {
            continue;
        }
        const char c = data_[pos_++];
        if (c == '"') {
            break;
        }
        if (c != '\\') {
            syntaxError("control character in string");
        }
        const int e = peekChar();
        if (e < 0) {
            syntaxError("unterminated string");
        }
        ++pos_;
        switch (e) {
            case '"':
                s += '"';
                break;
            case '\\':
                s += '\\';
                break;
            case '/':
                s += '/';
                break;
            case 'b':
                s += '\b';
                break;
            case 'f':
                s += '\f';
                break;
            case 'n':
                s += '\n';
                break;
            case 'r':
                s += '\r';
                break;
            case 't':
                s += '\t';
                break;
            case 'u': {
                const auto hex4 = [this]() {
                    std::uint32_t v = 0;
                    for (int k = 0; k < 4; ++k) {
                        const int h = peekChar();
                        std::uint32_t d = 0;
                        if (h >= '0' && h <= '9') {
                            d = static_cast<std::uint32_t>(h - '0');
                        } else if (h >= 'a' && h <= 'f') {
                            d = static_cast<std::uint32_t>(h - 'a' + 10);
                        } else if (h >= 'A' && h <= 'F') {
                            d = static_cast<std::uint32_t>(h - 'A' + 10);
                        } else {
                            syntaxError("invalid \\u escape");
                        }
                        ++pos_;
                        v = v * 16 + d;
                    }
                    return v;
                };
                std::uint32_t cp = hex4();
                if (cp >= 0xD800 && cp <= 0xDBFF) {
                    if (peekChar() != '\\') syntaxError("unpaired surrogate");
                    ++pos_;
                    if (peekChar() != 'u') syntaxError("unpaired surrogate");
                    ++pos_;
                    const std::uint32_t lo = hex4();
                    if (lo < 0xDC00 || lo > 0xDFFF) syntaxError("unpaired surrogate");
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
                    syntaxError("unpaired surrogate");
                }
                appendUtf8(s, cp);
                break;
            }
            default:
                syntaxError("invalid escape");
        }
    }
    if (!isValidUtf8(s)) {
        syntaxError("invalid UTF-8 in string");
    }
    return s;
}

std::string JsonReader::readString(bool unknown) {
    skipWs();
    if (peekChar() != '"') {
        syntaxError("expected a string");
    }
    budget_.chargeValue();
    return readStringRaw(unknown ? budget_.limits().maxUnknownStringBytes : budget_.limits().maxStringBytes,
                         unknown);
}

std::string_view JsonReader::readNumber() {
    skipWs();
    budget_.chargeValue();
    number_.clear();
    for (;;) {
        const int c = peekChar();
        if (!((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E')) {
            break;
        }
        if (number_.size() >= budget_.limits().maxNumberChars) {
            throw LoadFailure{CarvError::Code::ResourceLimit,
                              member_ + ": a number is longer than " +
                                  std::to_string(budget_.limits().maxNumberChars) + " characters"};
        }
        number_ += static_cast<char>(c);
        ++pos_;
    }
    if (!isJsonNumber(number_)) {
        syntaxError("invalid number");
    }
    return number_;
}

bool JsonReader::readBool() {
    skipWs();
    budget_.chargeValue();
    if (peekChar() == 't') {
        expectLiteral("true");
        return true;
    }
    expectLiteral("false");
    return false;
}

void JsonReader::readNull() {
    skipWs();
    budget_.chargeValue();
    expectLiteral("null");
}

void JsonReader::readCanonical(std::string* out) {
    // Recursion depth is bounded by maxDepth (enforced in push).
    budget_.chargeUnknownValue();
    switch (peek()) {
        case Kind::Object: {
            beginObject(true);
            std::vector<std::pair<std::string, std::string>> members;
            std::string key;
            while (nextKey(key)) {
                if (out) {
                    std::string value;
                    readCanonical(&value);
                    budget_.chargeExtras(key.size() + value.size() + 64);
                    members.emplace_back(std::move(key), std::move(value));
                } else {
                    readCanonical(nullptr);
                }
            }
            if (!out) {
                return;
            }
            std::sort(members.begin(), members.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });
            for (std::size_t i = 1; i < members.size(); ++i) {
                if (members[i].first == members[i - 1].first) {
                    syntaxError("duplicate key '" + members[i].first + "'");
                }
            }
            const std::size_t before = out->size();
            *out += '{';
            for (std::size_t i = 0; i < members.size(); ++i) {
                if (i > 0) {
                    *out += ", ";
                }
                appendJsonString(*out, members[i].first);
                *out += ": ";
                *out += members[i].second;
            }
            *out += '}';
            budget_.chargeExtras(out->size() - before);
            return;
        }
        case Kind::Array: {
            beginArray(true);
            if (out) {
                *out += '[';
            }
            bool first = true;
            while (nextElement()) {
                if (out && !first) {
                    *out += ", ";
                    budget_.chargeExtras(2);
                }
                first = false;
                readCanonical(out);
            }
            if (out) {
                *out += ']';
                budget_.chargeExtras(2);
            }
            return;
        }
        case Kind::String: {
            const std::string s = readString(true);
            if (out) {
                const std::size_t before = out->size();
                appendJsonString(*out, s);
                budget_.chargeExtras(out->size() - before);
            }
            return;
        }
        case Kind::Number: {
            const std::string_view token = readNumber();
            if (out) {
                const std::size_t before = out->size();
                appendCanonicalNumber(*out, token, member_);
                budget_.chargeExtras(out->size() - before);
            }
            return;
        }
        case Kind::Bool: {
            const bool b = readBool();
            if (out) {
                *out += b ? "true" : "false";
                budget_.chargeExtras(5);
            }
            return;
        }
        case Kind::Null:
            readNull();
            if (out) {
                *out += "null";
                budget_.chargeExtras(4);
            }
            return;
    }
}

void JsonReader::finish() {
    skipWs();
    if (peekChar() >= 0) {
        syntaxError("unexpected content after the JSON value");
    }
}

std::string canonicalJson(std::string_view text, LoadBudget& budget) {
    StringSource source(text);
    JsonReader reader(source, budget, "json", text.size() + 1);
    std::string out;
    reader.readCanonical(&out);
    reader.finish();
    return out;
}

}  // namespace carvetto::document::detail
