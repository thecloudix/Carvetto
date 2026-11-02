#include "dxf_pairs.hpp"

#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace carvetto::io::detail {

std::optional<ValueType> valueTypeFor(int code) {
    // Ranges from the public DXF reference ("Group Code Value Types"). 280-289 are documented as
    // 8-bit integers; binary files store them in 16 bits like the other small integers.
    if (code < 0) {
        return ValueType::String;  // application-defined codes (-1..-5), ASCII only in practice
    }
    if (code <= 9) return ValueType::String;
    if (code <= 59) return ValueType::Double;  // 10-59
    if (code <= 79) return ValueType::Int16;
    if (code >= 90 && code <= 99) return ValueType::Int32;
    if (code == 100 || code == 102 || code == 105) return ValueType::String;
    if (code >= 110 && code <= 149) return ValueType::Double;
    if (code >= 160 && code <= 169) return ValueType::Int64;
    if (code >= 170 && code <= 179) return ValueType::Int16;
    if (code >= 210 && code <= 239) return ValueType::Double;
    if (code >= 270 && code <= 289) return ValueType::Int16;
    if (code >= 290 && code <= 299) return ValueType::Bool;
    if (code >= 300 && code <= 309) return ValueType::String;
    if (code >= 310 && code <= 319) return ValueType::Binary;
    if (code >= 320 && code <= 369) return ValueType::String;
    if (code >= 370 && code <= 389) return ValueType::Int16;
    if (code >= 390 && code <= 399) return ValueType::String;
    if (code >= 400 && code <= 409) return ValueType::Int16;
    if (code >= 410 && code <= 419) return ValueType::String;
    if (code >= 420 && code <= 429) return ValueType::Int32;
    if (code >= 430 && code <= 439) return ValueType::String;
    if (code >= 440 && code <= 459) return ValueType::Int32;
    if (code >= 460 && code <= 469) return ValueType::Double;
    if (code >= 470 && code <= 481) return ValueType::String;
    if (code == 999) return ValueType::String;
    if (code >= 1000 && code <= 1003) return ValueType::String;
    if (code == 1004) return ValueType::Binary;
    if (code >= 1005 && code <= 1009) return ValueType::String;
    if (code >= 1010 && code <= 1059) return ValueType::Double;
    if (code >= 1060 && code <= 1070) return ValueType::Int16;
    if (code == 1071) return ValueType::Int32;
    return std::nullopt;
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
        s.remove_prefix(1);
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
        s.remove_suffix(1);
    }
    return s;
}

namespace {
char upperChar(char c) {
    return (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
}
}  // namespace

bool iequals(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (upperChar(a[i]) != upperChar(b[i])) {
            return false;
        }
    }
    return true;
}

std::string upper(std::string_view s) {
    std::string out(s);
    for (char& c : out) {
        c = upperChar(c);
    }
    return out;
}

std::string printable(std::string_view s, std::size_t maxLength) {
    std::string out;
    for (const char c : s) {
        if (out.size() >= maxLength) {
            out += "...";
            break;
        }
        const auto u = static_cast<unsigned char>(c);
        out += (u >= 0x20 && u < 0x7F) ? c : '?';
    }
    return out;
}

namespace {

bool parseDouble(std::string_view t, double& out) {
    t = trim(t);
    if (!t.empty() && t.front() == '+') {
        t.remove_prefix(1);
    }
    if (t.empty()) {
        return false;
    }
    double v = 0.0;
    const auto [ptr, ec] = std::from_chars(t.data(), t.data() + t.size(), v);
    if (ec != std::errc{} || ptr != t.data() + t.size() || !std::isfinite(v)) {
        return false;
    }
    out = v;
    return true;
}

bool parseInteger(std::string_view t, std::int64_t& out) {
    t = trim(t);
    if (!t.empty() && t.front() == '+') {
        t.remove_prefix(1);
    }
    if (t.empty()) {
        return false;
    }
    std::int64_t v = 0;
    const auto [ptr, ec] = std::from_chars(t.data(), t.data() + t.size(), v);
    if (ec == std::errc{} && ptr == t.data() + t.size()) {
        out = v;
        return true;
    }
    // Some writers put "1.0" in integer fields; accept integral doubles.
    double d = 0.0;
    if (parseDouble(t, d) && d == std::floor(d) && std::fabs(d) < 9.0e18) {
        out = static_cast<std::int64_t>(d);
        return true;
    }
    return false;
}

bool isBlank(std::string_view s) {
    for (const char c : s) {
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n' && c != '\0' && c != '\x1a') {
            return false;
        }
    }
    return true;
}

std::uint64_t readLe(const char* p, std::size_t n) {
    std::uint64_t v = 0;
    for (std::size_t i = 0; i < n; ++i) {
        v |= static_cast<std::uint64_t>(static_cast<unsigned char>(p[i])) << (8 * i);
    }
    return v;
}

}  // namespace

PairReader::PairReader(std::span<const std::uint8_t> bytes)
    : data_(reinterpret_cast<const char*>(bytes.data()), bytes.size()) {
    if (data_.size() >= kBinarySentinel.size() &&
        data_.substr(0, kBinarySentinel.size()) == kBinarySentinel) {
        binary_ = true;
        pos_ = kBinarySentinel.size();
        // R13+ files store group codes in two bytes, R12 files in one (255 escapes a 2-byte
        // code). The first pair is "0 SECTION" (or a 999 comment), which tells the layouts apart.
        if (pos_ + 1 < data_.size()) {
            const auto b0 = static_cast<unsigned char>(data_[pos_]);
            const auto b1 = static_cast<unsigned char>(data_[pos_ + 1]);
            wideCodes_ = (b0 == 0 && b1 == 0) || (b0 == 0xE7 && b1 == 0x03);
        }
    } else if (data_.size() >= 3 && data_.substr(0, 3) == "\xEF\xBB\xBF") {
        pos_ = 3;  // UTF-8 byte order mark
    }
}

bool PairReader::fail(DxfError::Code code, std::string message, std::size_t line) {
    if (!error_) {
        error_ = DxfError{code, std::move(message), line};
    }
    return false;
}

bool PairReader::next(Pair& out) {
    if (peeked_) {
        out = *peeked_;
        peeked_.reset();
        return true;
    }
    while (readRaw(out)) {
        if (out.code != 999) {
            return true;
        }
    }
    return false;
}

bool PairReader::peek(Pair& out) {
    if (!peeked_) {
        Pair p;
        if (!next(p)) {
            return false;
        }
        peeked_ = p;
    }
    out = *peeked_;
    return true;
}

bool PairReader::readRaw(Pair& out) {
    if (error_) {
        return false;
    }
    return binary_ ? readBinary(out) : readAscii(out);
}

bool PairReader::readLine(std::string_view& line) {
    if (pos_ >= data_.size()) {
        return false;
    }
    const std::size_t nl = data_.find('\n', pos_);
    const std::size_t end = nl == std::string_view::npos ? data_.size() : nl;
    line = data_.substr(pos_, end - pos_);
    if (!line.empty() && line.back() == '\r') {
        line.remove_suffix(1);
    }
    pos_ = nl == std::string_view::npos ? data_.size() : nl + 1;
    ++line_;
    return true;
}

bool PairReader::readAscii(Pair& out) {
    std::string_view codeLine;
    if (!readLine(codeLine)) {
        return false;
    }
    const std::size_t codeLineNo = line_;
    const std::string_view codeText = trim(codeLine);
    if (codeText.empty() && isBlank(data_.substr(pos_))) {
        pos_ = data_.size();
        return false;  // trailing blank lines
    }
    std::int64_t code = 0;
    if (!parseInteger(codeText, code) || code < -32768 || code > 32767) {
        return fail(DxfError::Code::InvalidFormat, "group code expected, found '" + printable(codeText) + "'",
                    codeLineNo);
    }
    std::string_view value;
    if (!readLine(value)) {
        return fail(DxfError::Code::Truncated, "data ends after group code " + std::to_string(code),
                    codeLineNo);
    }
    out = Pair{};
    out.code = static_cast<int>(code);
    out.text = value;
    out.line = codeLineNo;
    const ValueType type = valueTypeFor(out.code).value_or(ValueType::String);
    switch (type) {
        case ValueType::Double:
            if (!parseDouble(value, out.real)) {
                return fail(
                    DxfError::Code::InvalidFormat,
                    "invalid number '" + printable(trim(value)) + "' for group code " + std::to_string(code),
                    line_);
            }
            out.integer = std::fabs(out.real) < 9.0e18 ? static_cast<std::int64_t>(out.real) : 0;
            break;
        case ValueType::Int16:
        case ValueType::Int32:
        case ValueType::Int64:
        case ValueType::Bool:
            if (!parseInteger(value, out.integer)) {
                return fail(
                    DxfError::Code::InvalidFormat,
                    "invalid integer '" + printable(trim(value)) + "' for group code " + std::to_string(code),
                    line_);
            }
            out.real = static_cast<double>(out.integer);
            break;
        case ValueType::String:
        case ValueType::Binary:
            break;
    }
    return true;
}

bool PairReader::readBinary(Pair& out) {
    if (pos_ >= data_.size()) {
        return false;
    }
    const std::size_t start = pos_;
    auto need = [&](std::size_t n) { return data_.size() - pos_ >= n; };
    int code = 0;
    if (wideCodes_) {
        if (!need(2)) {
            return fail(DxfError::Code::Truncated, "binary data ends inside a group code", start);
        }
        code = static_cast<int>(
            static_cast<std::int16_t>(static_cast<std::uint16_t>(readLe(data_.data() + pos_, 2))));
        pos_ += 2;
    } else {
        code = static_cast<unsigned char>(data_[pos_]);
        pos_ += 1;
        if (code == 255) {
            if (!need(2)) {
                return fail(DxfError::Code::Truncated, "binary data ends inside a group code", start);
            }
            code = static_cast<int>(
                static_cast<std::int16_t>(static_cast<std::uint16_t>(readLe(data_.data() + pos_, 2))));
            pos_ += 2;
        }
    }
    const std::optional<ValueType> type = valueTypeFor(code);
    if (!type || code < 0) {
        return fail(DxfError::Code::InvalidFormat,
                    "unknown group code " + std::to_string(code) + " in binary DXF", start);
    }
    out = Pair{};
    out.code = code;
    out.line = start;
    auto readInt = [&](std::size_t n) -> bool {
        if (!need(n)) {
            return fail(DxfError::Code::Truncated, "binary data ends inside a value", start);
        }
        const std::uint64_t raw = readLe(data_.data() + pos_, n);
        pos_ += n;
        switch (n) {
            case 1:
                out.integer = static_cast<std::int64_t>(raw);
                break;
            case 2:
                out.integer = static_cast<std::int16_t>(static_cast<std::uint16_t>(raw));
                break;
            case 4:
                out.integer = static_cast<std::int32_t>(static_cast<std::uint32_t>(raw));
                break;
            default:
                out.integer = static_cast<std::int64_t>(raw);
                break;
        }
        out.real = static_cast<double>(out.integer);
        return true;
    };
    switch (*type) {
        case ValueType::String: {
            const std::size_t nul = data_.find('\0', pos_);
            if (nul == std::string_view::npos) {
                return fail(DxfError::Code::Truncated, "binary data ends inside a string", start);
            }
            out.text = data_.substr(pos_, nul - pos_);
            pos_ = nul + 1;
            return true;
        }
        case ValueType::Binary: {
            if (!need(1)) {
                return fail(DxfError::Code::Truncated, "binary data ends inside a value", start);
            }
            const auto n = static_cast<std::size_t>(static_cast<unsigned char>(data_[pos_]));
            pos_ += 1;
            if (!need(n)) {
                return fail(DxfError::Code::Truncated, "binary data ends inside a value", start);
            }
            out.text = data_.substr(pos_, n);
            pos_ += n;
            return true;
        }
        case ValueType::Double: {
            if (!need(8)) {
                return fail(DxfError::Code::Truncated, "binary data ends inside a value", start);
            }
            const double v = std::bit_cast<double>(readLe(data_.data() + pos_, 8));
            pos_ += 8;
            if (!std::isfinite(v)) {
                return fail(DxfError::Code::InvalidFormat,
                            "non-finite number for group code " + std::to_string(code), start);
            }
            out.real = v;
            out.integer = std::fabs(v) < 9.0e18 ? static_cast<std::int64_t>(v) : 0;
            return true;
        }
        case ValueType::Int16:
            return readInt(2);
        case ValueType::Int32:
            return readInt(4);
        case ValueType::Int64:
            return readInt(8);
        case ValueType::Bool:
            break;
    }
    return readInt(1);  // Bool: one byte
}

}  // namespace carvetto::io::detail
