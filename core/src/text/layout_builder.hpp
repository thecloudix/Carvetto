#pragma once

// Shared machinery of layoutText and layoutTextOnPath (T-0130, ADR-0033): input validation,
// UTF-8 scanning, HarfBuzz shaping, the glyph outline cache, the work meter and the limits.

#include <cstddef>
#include <cstdint>
#include <map>
#include <string_view>
#include <vector>

#include "carvetto/geometry/transform2.hpp"
#include "carvetto/text/layout.hpp"

namespace carvetto::text::detail {

/// One glyph out of HarfBuzz, in font units.
struct ShapedGlyph {
    std::uint32_t glyphId{0};
    std::uint32_t cluster{0};  // byte offset into the whole text
    std::int64_t advance{0};
    std::int64_t xOffset{0};
    std::int64_t yOffset{0};
};

/// A line of the input: byte range without its terminator.
struct LineRange {
    std::size_t begin{0};
    std::size_t length{0};
};

class LayoutBuilder {
public:
    LayoutBuilder(const Font& font, const TextStyle& style, const TextLimits& limits, TextLayout& out);

    /// Checks font, style and limits (not the text). False with `out.status` set on failure.
    bool validate();
    /// Refuses (TextTooLong) text of more than the byte limit; call before copying or scanning it.
    bool checkTextSize(std::size_t bytes);
    /// Splits `text` at "\n", "\r\n" and "\r", counting code points and applying the size limits.
    /// False with `out.status` set on failure.
    bool scan(std::string_view text, std::vector<LineRange>& lines);
    /// Shapes `line` (which starts at byte `base` of the whole text), appending to `glyphs`.
    bool shape(std::string_view line, std::size_t base, std::vector<ShapedGlyph>& glyphs);
    /// Adds `n` to the work meter. False with WorkLimit set when it exceeds the limit.
    bool addWork(std::uint64_t n);
    /// Appends the glyph's outline, scaled to mm (local = units * size / unitsPerEm) and mapped by
    /// `placement`, to the layout; fills the glyph's contour range. False on a limit breach.
    bool emit(PlacedGlyph& glyph, const geometry::Transform2& placement);
    /// Marks the failure and clears partial output.
    bool fail(TextStatus status, const char* message);
    /// Finalises bounds.
    void finish();

    /// font units -> mm, the one conversion used everywhere: (units * size) / unitsPerEm.
    double toMm(std::int64_t units) const;
    double toMm(double units) const;

    const Font& font() const { return font_; }
    const TextStyle& style() const { return style_; }
    const FontMetrics& metrics() const { return font_.metrics(); }
    std::size_t maxPathSpans() const { return maxPathSpans_; }

private:
    const GlyphOutline* outlineFor(std::uint32_t glyphId);

    const Font& font_;
    const TextStyle& style_;
    TextLayout& out_;
    std::size_t maxTextBytes_;
    std::size_t maxCodePoints_;
    std::size_t maxLines_;
    std::size_t maxGlyphs_;
    std::size_t maxSpans_;
    std::size_t maxGlyphPoints_;
    std::size_t maxPathSpans_;
    std::uint64_t maxWork_;
    double maxSize_;
    std::size_t shapedTotal_{0};
    std::size_t decodedSpans_{0};
    std::size_t emittedSpans_{0};
    std::map<std::uint32_t, GlyphOutline> cache_;
};

}  // namespace carvetto::text::detail
