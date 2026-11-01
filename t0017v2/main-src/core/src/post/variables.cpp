#include "carvetto/post/variables.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <string>
#include <system_error>
#include <vector>

#include "text_util.hpp"

namespace carvetto::post {

namespace {

constexpr std::array kTags{
    // Position and arcs.
    TagInfo{"X", TagKind::Length, true},
    TagInfo{"Y", TagKind::Length, true},
    TagInfo{"Z", TagKind::Length, true},
    TagInfo{"A", TagKind::Angle, true},
    TagInfo{"I", TagKind::Length, false},
    TagInfo{"J", TagKind::Length, false},
    TagInfo{"IA", TagKind::Length, false},
    TagInfo{"JA", TagKind::Length, false},
    TagInfo{"ArcStartX", TagKind::Length, false},
    TagInfo{"ArcStartY", TagKind::Length, false},
    TagInfo{"ArcMidX", TagKind::Length, false},
    TagInfo{"ArcMidY", TagKind::Length, false},
    TagInfo{"Radius", TagKind::Length, false},
    TagInfo{"Angle", TagKind::Angle, false},
    // Feeds and speeds.
    TagInfo{"F", TagKind::FeedRate, false},
    TagInfo{"FC", TagKind::FeedRate, false},
    TagInfo{"FP", TagKind::FeedRate, false},
    TagInfo{"FI", TagKind::Number, false},
    TagInfo{"S", TagKind::Number, false},
    // Tools and toolpaths.
    TagInfo{"T", TagKind::Number, false},
    TagInfo{"TP", TagKind::Number, false},
    TagInfo{"TOOLNAME", TagKind::Text, false},
    TagInfo{"TOOL_NOTES", TagKind::Text, false},
    TagInfo{"TDIA", TagKind::Length, false},
    TagInfo{"TOOLPATH_NAME", TagKind::Text, false},
    TagInfo{"TOOLS_USED", TagKind::Text, false},
    // Line, time and file.
    TagInfo{"N", TagKind::Number, false},
    TagInfo{"TIME", TagKind::Text, false},
    TagInfo{"DATE", TagKind::Text, false},
    TagInfo{"TP_FILENAME", TagKind::Text, false},
    TagInfo{"TP_DIR", TagKind::Text, false},
    TagInfo{"TP_EXT", TagKind::Text, false},
    TagInfo{"PATHNAME", TagKind::Text, false},
    TagInfo{"PRODUCT", TagKind::Text, false},
    TagInfo{"FILE_NOTES", TagKind::Text, false},
    // Home and material.
    TagInfo{"XH", TagKind::Length, false},
    TagInfo{"YH", TagKind::Length, false},
    TagInfo{"ZH", TagKind::Length, false},
    TagInfo{"SAFEZ", TagKind::Length, false},
    TagInfo{"XMIN", TagKind::Length, false},
    TagInfo{"XMAX", TagKind::Length, false},
    TagInfo{"XLENGTH", TagKind::Length, false},
    TagInfo{"YMIN", TagKind::Length, false},
    TagInfo{"YMAX", TagKind::Length, false},
    TagInfo{"YLENGTH", TagKind::Length, false},
    TagInfo{"ZMIN", TagKind::Length, false},
    TagInfo{"ZMAX", TagKind::Length, false},
    TagInfo{"ZLENGTH", TagKind::Length, false},
    // Dwell.
    TagInfo{"DWELL", TagKind::Number, false},
    // Recognised so posts using them parse; their blocks are not emitted yet (always 0 / empty).
    TagInfo{"DB", TagKind::Number, false},
    TagInfo{"DBX", TagKind::Length, false},
    TagInfo{"DBY", TagKind::Length, false},
    TagInfo{"DL", TagKind::Length, false},
    TagInfo{"CD", TagKind::Length, false},
    TagInfo{"CUTTER_COMP", TagKind::Text, false},
};

}  // namespace

const TagInfo* findTag(std::string_view tag) {
    for (const TagInfo& t : kTags) {
        if (t.tag == tag) {
            return &t;
        }
    }
    return nullptr;
}

Result<NumberFormat> parseNumberFormat(std::string_view text, int line) {
    const std::string_view original = text;
    text = detail::trim(text);
    NumberFormat f;
    if (!text.empty() && text.front() == '+') {
        f.forceSign = true;
        text.remove_prefix(1);
    }
    const std::size_t dot = text.find('.');
    const std::string_view whole = dot == std::string_view::npos ? text : text.substr(0, dot);
    const std::string_view frac = dot == std::string_view::npos ? std::string_view{} : text.substr(dot + 1);
    const auto parseDigits = [](std::string_view s, int& out) {
        if (s.empty() || s.size() > 2) {
            return false;
        }
        const auto r = std::from_chars(s.data(), s.data() + s.size(), out);
        return r.ec == std::errc{} && r.ptr == s.data() + s.size() && out >= 0;
    };
    if (!parseDigits(whole, f.integerDigits) ||
        (dot != std::string_view::npos && !parseDigits(frac, f.decimals)) || f.decimals > 12) {
        return Error{line, "invalid number format '" + std::string(original) + "' (expected [+]W.D)"};
    }
    if (dot == std::string_view::npos) {
        f.decimals = 0;
    }
    return f;
}

Result<VarDef> parseVarSpec(std::string_view spec, int line) {
    std::string_view s = detail::trim(spec);
    if (s.size() < 2 || s.front() != '[' || s.back() != ']') {
        return Error{line, "variable specification must be enclosed in [ ]"};
    }
    s = s.substr(1, s.size() - 2);
    std::vector<std::string_view> fields;
    for (;;) {
        const std::size_t bar = s.find('|');
        fields.push_back(s.substr(0, bar));
        if (bar == std::string_view::npos) {
            break;
        }
        s.remove_prefix(bar + 1);
    }
    if (fields.size() != 4 && fields.size() != 5) {
        return Error{line, "variable specification needs 4 or 5 fields [tag|mode|prefix|format|multiplier]"};
    }
    VarDef v;
    v.line = line;
    v.tag = std::string(detail::trim(fields[0]));
    const TagInfo* info = findTag(v.tag);
    if (info == nullptr) {
        return Error{line, "unknown variable tag '" + v.tag + "'"};
    }
    const std::string mode = detail::upper(detail::trim(fields[1]));
    if (mode == "A" || mode == "AI") {
        v.mode = OutputMode::Always;
    } else if (mode == "C" || mode == "CI") {
        v.mode = OutputMode::Changed;
    } else {
        return Error{line,
                     "invalid output mode '" + mode + "' for tag '" + v.tag + "' (expected A, C, AI or CI)"};
    }
    v.incremental = mode.size() == 2;
    if (v.incremental && !info->axis) {
        return Error{line, "incremental mode is only allowed on X, Y, Z and A (tag '" + v.tag + "')"};
    }
    v.prefix = std::string(fields[2]);
    auto fmt = parseNumberFormat(fields[3], line);
    if (!fmt) {
        return fmt.error();
    }
    v.format = fmt.value();
    if (fields.size() == 5) {
        const auto mult = detail::parseDouble(fields[4]);
        if (!mult) {
            return Error{line, "invalid multiplier '" + std::string(fields[4]) + "'"};
        }
        v.multiplier = *mult;
    }
    return v;
}

std::string formatNumber(const NumberFormat& format, double value) {
    const double scale = std::pow(10.0, format.decimals);
    double rounded = std::round(value * scale) / scale;  // half away from zero
    if (rounded == 0.0) {
        rounded = 0.0;  // drop -0
    }
    std::array<char, 400> buf{};
    const auto r = std::to_chars(buf.data(), buf.data() + buf.size(), std::fabs(rounded),
                                 std::chars_format::fixed, format.decimals);
    std::string digits = r.ec == std::errc{} ? std::string(buf.data(), r.ptr) : std::string("0");
    const std::size_t dot = digits.find('.');
    const std::size_t intLen = dot == std::string::npos ? digits.size() : dot;
    if (intLen < static_cast<std::size_t>(format.integerDigits)) {
        digits.insert(0, static_cast<std::size_t>(format.integerDigits) - intLen, '0');
    }
    if (rounded < 0.0) {
        return "-" + digits;
    }
    return format.forceSign ? "+" + digits : digits;
}

VarDef defaultVarDef(std::string_view tag) {
    VarDef v;
    v.tag = std::string(tag);
    if (tag == "N" || tag == "T" || tag == "TP" || tag == "S" || tag == "DB") {
        v.format.decimals = 0;
    }
    return v;
}

}  // namespace carvetto::post
