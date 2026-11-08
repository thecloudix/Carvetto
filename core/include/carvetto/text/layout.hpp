#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/text/font.hpp"

// Text layout and text on a curve (T-0130, ADR-0033). Pure functions of (font bytes, UTF-8 text,
// style, limits): no locale, no system fonts, no hinting; millimetres in, millimetres out.

namespace carvetto::text {

/// Horizontal alignment of each line about its anchor (x = 0 for TextBlock, the start offset
/// along the path for text on a curve).
enum class Alignment { Left, Centre, Right };

/// Base direction of a line; Auto lets HarfBuzz derive it from the first strong character.
enum class TextDirection { Auto, LeftToRight, RightToLeft };

/// How to set a block of text.
struct TextStyle {
    double sizeMm{10.0};         ///< The em size in mm (1 em = unitsPerEm font units).
    Alignment alignment{Alignment::Left};
    double letterSpacingMm{0.0}; ///< Extra mm after every glyph (negative tightens).
    double lineSpacing{1.0};     ///< Multiplier of the font's line height between lines.
    bool kerning{true};          ///< Apply the font's kerning ('kern' feature, GPOS or kern table).
    TextDirection direction{TextDirection::Auto};
    /// BCP 47 language tag for shaping ("en"). Fixed rather than taken from the locale so the
    /// result never depends on the machine. At most 35 ASCII characters.
    std::string language{"en"};
};

/// Work and size bounds; any breach refuses the whole layout with a status (no partial output).
/// Each value is clamped to the kAbsolute* ceiling below, so callers cannot disable the bounds.
struct TextLimits {
    std::size_t maxTextBytes{1u << 20};
    std::size_t maxCodePoints{200'000};
    std::size_t maxLines{10'000};
    std::size_t maxGlyphs{200'000};
    std::size_t maxSpans{1'000'000};          ///< Spans emitted, and (separately) spans decoded from the font.
    std::size_t maxGlyphPoints{10'000};       ///< Outline points of one glyph; larger glyphs are skipped.
    std::size_t maxPathSpans{100'000};        ///< Spans of the guide contour of text on a curve.
    std::uint64_t maxWork{100'000'000};       ///< Deterministic work meter, see TextLayout::work.
    double maxSizeMm{10'000.0};               ///< Largest accepted TextStyle::sizeMm (and |spacing| per glyph).
};
inline constexpr std::size_t kAbsoluteMaxTextBytes = 64u << 20;
inline constexpr std::size_t kAbsoluteMaxSpans = 16'000'000;
inline constexpr std::uint64_t kAbsoluteMaxWork = 4'000'000'000ull;

/// Outcome of a layout.
enum class TextStatus {
    Ok,                ///< Done; an empty string is Ok with no glyphs.
    InvalidFont,       ///< The Font is not valid (default-constructed or moved from).
    InvalidStyle,      ///< Non-finite or out-of-range size, spacing, line spacing, or bad language tag.
    InvalidPath,       ///< The guide contour is empty, non-finite or has zero length.
    TextTooLong,       ///< More bytes than the limit.
    TooManyCodePoints,
    TooManyLines,
    TooManyGlyphs,     ///< Shaping produced more glyphs than the limit.
    TooManySpans,      ///< Decoded or emitted spans exceeded the limit.
    PathTooComplex,    ///< The guide contour has more spans than the limit.
    WorkLimit,         ///< The work meter exceeded maxWork.
};

/// One positioned glyph. Its contours are `contours[firstContour, firstContour + contourCount)`
/// of the layout.
struct PlacedGlyph {
    std::uint32_t glyphId{0};
    std::uint32_t cluster{0};       ///< Byte offset in the input text of the characters it came from.
    std::uint32_t line{0};
    bool missing{false};            ///< Glyph 0 (.notdef): the font has no glyph for the character.
    geometry::Vec2 origin;          ///< Baseline origin in mm after shaping offsets.
    geometry::Vec2 baselineDir{1.0, 0.0};  ///< Unit direction of the baseline (tangent for text on a curve).
    double advance{0.0};            ///< Shaped advance in mm (excluding letter spacing).
    std::size_t firstContour{0};
    std::size_t contourCount{0};
};

/// One line of a block.
struct TextLine {
    std::size_t firstGlyph{0};
    std::size_t glyphCount{0};
    geometry::Vec2 baselineStart;   ///< Where the line's first glyph origin is (alignment applied).
    double widthMm{0.0};            ///< Advances plus letter spacing between glyphs.
};

/// The result of laying out text. Contours are closed, outer ones counter-clockwise (positive
/// area) and holes clockwise, consistently for TrueType and CFF fonts; y is up and the first
/// baseline is y = 0 (later lines go down).
struct TextLayout {
    TextStatus status{TextStatus::Ok};
    std::string message;
    std::vector<PlacedGlyph> glyphs;
    std::vector<TextLine> lines;
    std::vector<geometry::Contour> contours;
    geometry::Box2 bounds;           ///< Exact bounds of all contours (empty when there are none).
    std::uint64_t work{0};           ///< Work meter: code points + 4 per glyph + outline points + spans.
    std::size_t skippedGlyphs{0};    ///< Glyphs whose outline the font would not give or was too big.
    bool replacedInvalidUtf8{false}; ///< The text had invalid UTF-8 (shown as U+FFFD, normally .notdef).
    bool truncated{false};           ///< Text on a curve: glyphs past the end of an open path were dropped.

    bool ok() const { return status == TextStatus::Ok; }
};

/// Lays out `utf8` as a multi-line block. Lines break at "\n", "\r\n" and "\r". Each line is shaped
/// separately (HarfBuzz, with the style's direction/language/kerning); the first baseline is at y = 0.
TextLayout layoutText(const Font& font, std::string_view utf8, const TextStyle& style = {},
                      const TextLimits& limits = {});

/// Where along the guide contour the text goes.
struct PathTextOptions {
    /// Arc length (mm from the contour's start) of the anchor that `TextStyle::alignment` refers
    /// to: the text starts there (Left), is centred on it (Centre) or ends there (Right).
    double startOffsetMm{0.0};
};

/// Lays out one line of text along `path` (arc-length parametrised, lines/arcs/Beziers). Each glyph
/// is moved rigidly: its advance midpoint goes to the path point at that arc length and its
/// baseline is rotated to the path tangent there; the glyph's up direction is the path's left
/// side (reverse the contour for the other side). Newlines become spaces. On a closed path the
/// text wraps around; on an open one glyphs whose midpoint is off either end are dropped and
/// `truncated` is set. No trigonometry is used for lines and Beziers, so those results are
/// bit-identical across platforms (arcs use sin/cos like the rest of geometry).
TextLayout layoutTextOnPath(const Font& font, std::string_view utf8, const TextStyle& style,
                            const geometry::Contour& path, const PathTextOptions& options = {},
                            const TextLimits& limits = {});

}  // namespace carvetto::text
