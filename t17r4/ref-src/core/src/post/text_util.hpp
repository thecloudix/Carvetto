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
