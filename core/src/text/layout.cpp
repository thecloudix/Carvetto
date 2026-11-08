#include "carvetto/text/layout.hpp"

#include "layout_builder.hpp"

namespace carvetto::text {

using geometry::Transform2;
using geometry::Vec2;

TextLayout layoutText(const Font& font, std::string_view utf8, const TextStyle& style, const TextLimits& limits) {
    TextLayout out;
    detail::LayoutBuilder b(font, style, limits, out);
    std::vector<detail::LineRange> lines;
    if (!b.validate() || !b.scan(utf8, lines)) {
        return out;
    }
    // Baseline-to-baseline distance of one line, in mm.
    const double pitch = b.toMm(static_cast<double>(b.metrics().lineHeight)) * style.lineSpacing;

    std::vector<detail::ShapedGlyph> shaped;
    for (std::size_t li = 0; li < lines.size(); ++li) {
        shaped.clear();
        const detail::LineRange& range = lines[li];
        if (!b.shape(utf8.substr(range.begin, range.length), range.begin, shaped)) {
            return out;
        }
        std::int64_t totalUnits = 0;
        for (const detail::ShapedGlyph& g : shaped) {
            totalUnits += g.advance;
        }
        const double count = static_cast<double>(shaped.size());
        const double width = shaped.empty() ? 0.0 : b.toMm(totalUnits) + (count - 1.0) * style.letterSpacingMm;
        double x0 = 0.0;
        if (style.alignment == Alignment::Centre) {
            x0 = -width / 2.0;
        } else if (style.alignment == Alignment::Right) {
            x0 = -width;
        }
        const double baselineY = -static_cast<double>(li) * pitch;

        TextLine line;
        line.firstGlyph = out.glyphs.size();
        line.glyphCount = shaped.size();
        line.baselineStart = Vec2{x0, baselineY};
        line.widthMm = width;

        std::int64_t pen = 0;
        double index = 0.0;
        for (const detail::ShapedGlyph& g : shaped) {
            PlacedGlyph pg;
            pg.glyphId = g.glyphId;
            pg.cluster = g.cluster;
            pg.line = static_cast<std::uint32_t>(li);
            pg.missing = g.glyphId == 0;
            pg.advance = b.toMm(g.advance);
            pg.origin = Vec2{x0 + (b.toMm(pen) + index * style.letterSpacingMm) + b.toMm(g.xOffset),
                             baselineY + b.toMm(g.yOffset)};
            if (!b.emit(pg, Transform2::translation(pg.origin))) {
                return out;
            }
            out.glyphs.push_back(pg);
            pen += g.advance;
            index += 1.0;
        }
        out.lines.push_back(line);
    }
    b.finish();
    return out;
}

}  // namespace carvetto::text
