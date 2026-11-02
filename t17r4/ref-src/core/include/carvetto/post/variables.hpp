#pragma once

#include <string>
#include <string_view>

#include "carvetto/post/result.hpp"

namespace carvetto::post {

/// When a variable is written: every time (`A`) or only when its rendered text changed (`C`).
enum class OutputMode {
    Always,   ///< `A`: written on every reference.
    Changed,  ///< `C`: omitted when the rendered text equals the last one written (modal).
};

/// Numeric format of a variable, written `[+]W.D` in a `.pp` file: `+` forces a sign on
/// positive values, W is the minimum number of integer digits (zero padded) and D the number
/// of decimals (0 = no decimal point). Example: `1.3` renders 12.3456 as `12.346`.
struct NumberFormat {
    bool forceSign{false};
    int integerDigits{1};
    int decimals{3};

    bool operator==(const NumberFormat&) const = default;
};

/// One `VAR NAME = [tag|mode|prefix|format|multiplier]` definition (ADR-0004, ADR-0011).
///
/// `mode` is `A` or `C`, optionally followed by `I` for incremental output (`AI`, `CI`), which
/// is only allowed on the axis tags X, Y, Z and A and renders the move's delta instead of the
/// absolute end position. `prefix` is copied verbatim (including spaces). `multiplier` is
/// optional (default 1) and applies after unit conversion.
struct VarDef {
    /// The symbolic VAR name (empty for an implicit default definition).
    std::string name;
    /// The tag referenced as `[tag]` in block lines, e.g. `X`, `F`, `TOOLNAME`.
    std::string tag;
    OutputMode mode{OutputMode::Always};
    bool incremental{false};
    std::string prefix;
    NumberFormat format;
    double multiplier{1.0};
    /// Source line of the definition (0 for implicit defaults).
    int line{0};
};

/// What a tag's value means; decides unit conversion and whether it is numeric.
enum class TagKind {
    Length,    ///< Millimetres internally; divided by 25.4 for inch posts.
    FeedRate,  ///< mm/min internally; divided by 25.4 for inch posts.
    Angle,     ///< Degrees; never unit-converted.
    Number,    ///< Plain number (spindle speed, tool number, line number, seconds…).
    Text,      ///< String value (tool name, file name, date…).
};

/// Static description of a tag the interpreter knows how to evaluate.
struct TagInfo {
    std::string_view tag;
    TagKind kind;
    /// True for X, Y, Z, A: the only tags that may be incremental.
    bool axis;
};

/// Looks up a tag (case-sensitive, as written in posts). `nullptr` when the tag is unknown.
const TagInfo* findTag(std::string_view tag);

/// Parses a `[+]W.D` format string.
Result<NumberFormat> parseNumberFormat(std::string_view text, int line = 0);

/// Parses the bracketed specification `[tag|mode|prefix|format]` or
/// `[tag|mode|prefix|format|multiplier]` (brackets included). Unknown tags are an error.
Result<VarDef> parseVarSpec(std::string_view spec, int line = 0);

/// Renders `value` with `format`: rounded half away from zero, no `-0`, locale independent.
std::string formatNumber(const NumberFormat& format, double value);

/// The implicit definition used when a block references a known tag that has no VAR line:
/// always output, no prefix, three decimals (integers for N, T, TP and S).
VarDef defaultVarDef(std::string_view tag);

}  // namespace carvetto::post
