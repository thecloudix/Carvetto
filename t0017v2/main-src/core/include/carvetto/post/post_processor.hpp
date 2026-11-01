#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "carvetto/geometry/vec3.hpp"
#include "carvetto/post/result.hpp"
#include "carvetto/post/variables.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::post {

/// Output length unit declared by the post's `UNITS` setting.
enum class Units {
    Millimetres,  ///< `UNITS = "MM"`: values are written as computed.
    Inches,       ///< `UNITS = "INCHES"`: lengths and feed rates are divided by 25.4.
};

/// A `ROTARY_WRAP_X` / `ROTARY_WRAP_Y` target: the rotary axis letter and its sign.
struct RotaryWrap {
    char axis{'A'};
    bool negate{false};

    bool operator==(const RotaryWrap&) const = default;
};

/// The global settings of a post (`KEY = value` lines). Every setting as written is also kept
/// in `raw`; settings Carvetto does not act on yet are only stored there.
struct PostSettings {
    std::string postName;
    std::string fileExtension;
    Units units{Units::Millimetres};
    /// Text appended after each output line (`LINE_ENDING`, `[13][10]` style codes decoded).
    std::string lineEnding{"\n"};
    long long lineNumberStart{0};
    long long lineNumberIncrement{1};
    /// After the line number passes this value it wraps back to `lineNumberStart`.
    long long lineNumberMaximum{999999};
    bool rapidPlungeToStartZ{false};
    /// When set, `[F]` on cutting moves renders the inverse-time value `[FI]` (1/min).
    bool inverseTimeMode{false};
    /// When set, the NEW_SEGMENT block is also emitted before the first toolpath.
    bool firstToolpathNewSegment{false};
    bool laserSupport{false};
    std::optional<RotaryWrap> rotaryWrapX;
    std::optional<RotaryWrap> rotaryWrapY;
    /// `SUBSTITUTE` character pairs (from, to) applied to text variables.
    std::vector<std::pair<char, char>> substitute;
    /// Arcs with a smaller radius are linearised (0 = no minimum).
    double minArcRadius{0.0};
    /// Arcs with a larger radius are linearised (0 = no maximum).
    double maxArcRadius{0.0};
    /// Every global setting as written (value unquoted), keyed by name.
    std::map<std::string, std::string> raw;
};

/// One `begin NAME … end` block: its quoted output lines, unquoted, in order.
struct PostBlock {
    std::string name;
    int line{0};
    std::vector<std::string> lines;
};

/// Job-level inputs the post needs besides the toolpaths. Everything that would otherwise
/// come from the clock or the filesystem is passed in, so output is deterministic.
struct PostContext {
    std::string fileName;             ///< `[TP_FILENAME]`: output file name without extension.
    std::string fileDir;              ///< `[TP_DIR]`.
    std::string fileExt;              ///< `[TP_EXT]`.
    std::string pathName;             ///< `[PATHNAME]`: full output path.
    std::string date;                 ///< `[DATE]`, preformatted by the caller.
    std::string time;                 ///< `[TIME]`, preformatted by the caller.
    std::string product{"Carvetto"};  ///< `[PRODUCT]`.
    std::string fileNotes;            ///< `[FILE_NOTES]`.
    /// Home position (`[XH] [YH] [ZH]`) and the tool position before the first move, mm.
    geometry::Vec3 home{0.0, 0.0, 0.0};
    /// Safe Z (`[SAFEZ]`), mm.
    double safeZ{5.0};
    /// Material extents (`[XMIN]`, `[XMAX]`, `[XLENGTH]`, and Y/Z equivalents), mm.
    geometry::Vec3 materialMin{0.0, 0.0, 0.0};
    geometry::Vec3 materialMax{0.0, 0.0, 0.0};
    /// Cylinder diameter for `ROTARY_WRAP_X/Y`, mm. Must be > 0 when the post wraps.
    double rotaryDiameter{0.0};
    /// Maximum chordal deviation when arcs are linearised, mm.
    double arcTolerance{0.01};
};

/// The generated program and any non-fatal warnings raised while generating it.
struct PostOutput {
    std::string text;
    std::vector<Diagnostic> warnings;
};

/// A parsed `.pp` post-processor template and its interpreter (ADR-0004, ADR-0011).
///
/// Grammar: blank lines and lines starting with `+` are comments; `KEY = value` lines set
/// global settings; `VAR NAME = [tag|mode|prefix|format|multiplier]` defines how a tag is
/// rendered; `begin NAME` … `end` delimits a block whose lines are double-quoted output text
/// containing `[tag]` references. Unknown blocks and settings are warnings; unknown tags are
/// errors naming the line.
///
/// Running walks each toolpath's move list carrying the current position (ADR-0008) and
/// emits HEADER, then per toolpath TOOLCHANGE (+ SPINDLE_ON) or NEW_SEGMENT, a block per move,
/// and FOOTER. Missing optional blocks fall back as documented in ADR-0011.
class PostProcessor {
   public:
    /// Parses a post from its text.
    static Result<PostProcessor> parse(std::string_view source);

    /// The global settings.
    const PostSettings& settings() const { return settings_; }
    /// Warnings raised while parsing (unknown blocks/settings, duplicates).
    const std::vector<Diagnostic>& warnings() const { return warnings_; }
    /// The named block, or `nullptr` when the post does not define it.
    const PostBlock* block(std::string_view name) const;
    /// True when the post defines the named block.
    bool hasBlock(std::string_view name) const { return block(name) != nullptr; }
    /// The VAR definition for a tag, or `nullptr` when there is none.
    const VarDef* var(std::string_view tag) const;

    /// Generates the program for `toolpaths` in order. Toolpaths without moves are skipped.
    Result<PostOutput> run(const std::vector<toolpath::Toolpath>& toolpaths,
                           const PostContext& context) const;

    /// True when a block name belongs to the `.pp` grammar Carvetto recognises.
    static bool isKnownBlock(std::string_view name);

   private:
    PostSettings settings_;
    std::map<std::string, PostBlock, std::less<>> blocks_;
    std::map<std::string, VarDef, std::less<>> vars_;  // keyed by tag
    std::vector<Diagnostic> warnings_;
};

}  // namespace carvetto::post
