#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"

// Font loading and glyph outlines of the text module (T-0130, ADR-0033).
//
// Fonts come only from bytes or an explicit file path; core never enumerates system fonts, so the
// same bytes always give the same contours. FreeType parses the font (hinting off, unscaled so
// coordinates are integers in font units); HarfBuzz shapes (see layout.hpp). Both are private
// dependencies: no FreeType or HarfBuzz header is included from here.

namespace carvetto::text {

/// Why a font was refused (or Ok).
enum class FontStatus {
    Ok,
    Empty,              ///< No bytes.
    TooLarge,           ///< More bytes than FontLimits::maxFileBytes.
    UnsupportedFormat,  ///< Not a TrueType/OpenType (sfnt) file: Type 1, bitmap, WOFF, ... are refused.
    Corrupt,            ///< FreeType or HarfBuzz could not make sense of the tables, or a limit was exceeded.
    IoError,            ///< The file could not be opened or read.
};

/// Bounds applied while loading a font.
struct FontLimits {
    std::size_t maxFileBytes{32u * 1024u * 1024u};  ///< Never above kAbsoluteMaxFontBytes.
    std::uint32_t maxGlyphs{65535};                 ///< Most glyphs in the font.
};
/// Hard ceiling of FontLimits::maxFileBytes whatever the caller asks for.
inline constexpr std::size_t kAbsoluteMaxFontBytes = 256u * 1024u * 1024u;

/// Vertical metrics, all in font units (divide by `unitsPerEm`; y up).
struct FontMetrics {
    int unitsPerEm{1000};
    int ascender{800};
    int descender{-200};  ///< Negative below the baseline.
    int lineHeight{1000};  ///< Baseline-to-baseline distance of the font (ascender - descender + line gap).
    std::uint32_t glyphCount{0};
};

/// Why a glyph has no (complete) outline.
enum class OutlineStatus {
    Ok,         ///< Outline decoded (an empty one for spaces).
    LoadFailed, ///< FreeType refused the glyph (bad glyph id, corrupt data, bitmap-only glyph).
    TooComplex, ///< More points than the caller allowed.
};

/// One glyph's outline in font units: closed contours of lines and cubic Beziers (quadratics are
/// elevated exactly), y up, outer contours counter-clockwise and holes clockwise whatever the
/// font format. Coordinates are integers except the controls of elevated curves ((p0 + 2q) / 3).
struct GlyphOutline {
    OutlineStatus status{OutlineStatus::Ok};
    std::vector<geometry::Contour> contours;
    std::size_t pointCount{0};  ///< Outline points FreeType reported (work accounting).
};

class Font;

/// Result of loading a font: `font` is valid only when `status == FontStatus::Ok`.
struct FontLoad;

/// An immutable parsed font. Move-only. Not thread-safe: one Font must not be used from two
/// threads at once (outline decoding and shaping reuse internal FreeType/HarfBuzz state).
class Font {
public:
    Font();
    ~Font();
    Font(Font&&) noexcept;
    Font& operator=(Font&&) noexcept;
    Font(const Font&) = delete;
    Font& operator=(const Font&) = delete;

    /// Parses font bytes (copied). Face 0 of a collection.
    static FontLoad load(std::span<const std::uint8_t> bytes, const FontLimits& limits = {});
    /// Reads a file and parses it; the size is checked before reading.
    static FontLoad loadFile(const std::filesystem::path& path, const FontLimits& limits = {});

    bool valid() const { return impl_ != nullptr; }
    const FontMetrics& metrics() const;
    /// Family name from the name table ("" when absent). Informational; layout never uses it.
    const std::string& familyName() const;
    /// Glyph id for a code point; 0 (.notdef) when the font has none.
    std::uint32_t glyphIndex(char32_t codePoint) const;
    /// Outline of a glyph in font units, at most `maxPoints` outline points.
    GlyphOutline outline(std::uint32_t glyphId, std::size_t maxPoints = 20000) const;

    struct Impl;  // private to core/src/text
    Impl* impl() const { return impl_.get(); }

private:
    std::unique_ptr<Impl> impl_;
};

struct FontLoad {
    FontStatus status{FontStatus::Empty};
    std::string message;
    Font font;
    bool ok() const { return status == FontStatus::Ok; }
};

}  // namespace carvetto::text
