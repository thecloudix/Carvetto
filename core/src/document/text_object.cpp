#include "carvetto/document/text_object.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace carvetto::document {

namespace {

using geometry::Transform2;
using geometry::Vec2;

bool finite(double v) {
    return std::isfinite(v);
}

bool placementFinite(const Transform2& m) {
    return finite(m.a) && finite(m.b) && finite(m.c) && finite(m.d) && finite(m.tx) && finite(m.ty);
}

text::Alignment toAlignment(TextAlign a) {
    switch (a) {
        case TextAlign::Centre:
            return text::Alignment::Centre;
        case TextAlign::Right:
            return text::Alignment::Right;
        case TextAlign::Left:
            break;
    }
    return text::Alignment::Left;
}

text::TextStyle toStyle(const TextParams& p) {
    text::TextStyle s;
    s.sizeMm = p.sizeMm;
    s.alignment = toAlignment(p.align);
    s.letterSpacingMm = p.letterSpacingMm;
    s.lineSpacing = p.lineSpacing;
    s.kerning = p.kerning;
    return s;
}

TextBuild refused(text::TextStatus status, std::string message) {
    TextBuild b;
    b.status = status;
    b.message = std::move(message);
    return b;
}

// Splits `utf8` at LF/CRLF/CR into paragraphs.
std::vector<std::string_view> paragraphs(std::string_view utf8) {
    std::vector<std::string_view> out;
    std::size_t begin = 0;
    for (std::size_t i = 0; i < utf8.size(); ++i) {
        if (utf8[i] == '\n' || utf8[i] == '\r') {
            out.push_back(utf8.substr(begin, i - begin));
            if (utf8[i] == '\r' && i + 1 < utf8.size() && utf8[i + 1] == '\n') {
                ++i;
            }
            begin = i + 1;
        }
    }
    out.push_back(utf8.substr(begin));
    return out;
}

double lineWidth(const text::TextLayout& l) {
    return l.lines.empty() ? 0.0 : l.lines.front().widthMm;
}

}  // namespace

std::string textParamsProblem(const TextParams& p) {
    if (p.text.size() > kMaxTextObjectBytes) {
        return "the text is longer than " + std::to_string(kMaxTextObjectBytes) + " bytes";
    }
    if (!finite(p.sizeMm) || !(p.sizeMm > 0.0)) {
        return "the text size must be a finite number greater than zero";
    }
    if (!finite(p.letterSpacingMm) || !finite(p.lineSpacing) || p.lineSpacing < 0.0) {
        return "the spacing must be finite (line spacing not negative)";
    }
    if (!finite(p.boxWidthMm) || !finite(p.boxHeightMm) || p.boxWidthMm < 0.0 || p.boxHeightMm < 0.0) {
        return "the box size must be finite and not negative";
    }
    if (!finite(p.startOffsetMm) || !placementFinite(p.placement)) {
        return "the placement must be finite";
    }
    if (p.mode == TextMode::Box && !(p.boxWidthMm > 0.0)) {
        return "text in a box needs a box width greater than zero";
    }
    if (p.mode == TextMode::Curve && p.guide.spans.empty()) {
        return "text on a curve needs a guide curve";
    }
    return {};
}

TextBuild buildTextContours(const text::Font& font, const TextParams& params, const text::TextLimits& limits) {
    if (const std::string problem = textParamsProblem(params); !problem.empty()) {
        return refused(text::TextStatus::InvalidStyle, problem);
    }
    const text::TextStyle style = toStyle(params);
    TextBuild out;
    std::vector<geometry::Contour> local;

    const auto take = [&](text::TextLayout&& l, Vec2 shift) -> bool {
        if (!l.ok()) {
            out = refused(l.status, l.message.empty() ? "the text could not be laid out" : l.message);
            return false;
        }
        out.missingGlyphs = std::any_of(l.glyphs.begin(), l.glyphs.end(),
                                        [](const text::PlacedGlyph& g) { return g.missing; });
        out.truncated = l.truncated;
        const Transform2 move = Transform2::translation(shift);
        for (geometry::Contour& c : l.contours) {
            if (shift.x != 0.0 || shift.y != 0.0) {
                c.transform(move);
            }
            local.push_back(std::move(c));
        }
        return true;
    };

    if (params.mode == TextMode::Point) {
        if (!take(text::layoutText(font, params.text, style, limits), Vec2{0.0, 0.0})) {
            return out;
        }
    } else if (params.mode == TextMode::Curve) {
        text::PathTextOptions opt;
        opt.startOffsetMm = params.startOffsetMm;
        if (!take(text::layoutTextOnPath(font, params.text, style, params.guide, opt, limits), Vec2{0.0, 0.0})) {
            return out;
        }
    } else {
        // Box: greedy wrap by measured word widths.
        text::TextStyle measure = style;
        measure.alignment = text::Alignment::Left;
        const text::TextLayout spaceLayout = text::layoutText(font, " ", measure, limits);
        if (!spaceLayout.ok()) {
            return refused(spaceLayout.status, "the text could not be measured");
        }
        const double spaceAdv = spaceLayout.glyphs.empty() ? 0.0 : spaceLayout.glyphs.front().advance;
        const double join = spaceAdv + 2.0 * params.letterSpacingMm;
        std::vector<std::string_view> paras = paragraphs(params.text);
        std::size_t wordCount = 0;
        for (std::string_view para : paras) {
            wordCount += static_cast<std::size_t>(std::count(para.begin(), para.end(), ' ')) + 1;
        }
        if (wordCount > kMaxBoxWords) {
            return refused(text::TextStatus::WorkLimit,
                           "the text in a box has more than " + std::to_string(kMaxBoxWords) + " words");
        }
        std::string wrapped;
        bool overflow = false;
        bool firstPara = true;
        for (std::string_view para : paras) {
            if (!firstPara) {
                wrapped += '\n';
            }
            firstPara = false;
            double current = 0.0;
            bool lineEmpty = true;
            std::size_t begin = 0;
            while (true) {
                const std::size_t end = para.find(' ', begin);
                const std::string_view word = para.substr(begin, end == std::string_view::npos ? end : end - begin);
                const text::TextLayout wl = text::layoutText(font, word, measure, limits);
                if (!wl.ok()) {
                    return refused(wl.status, "the text could not be measured");
                }
                const double w = lineWidth(wl);
                if (w > params.boxWidthMm) {
                    overflow = true;
                }
                if (lineEmpty) {
                    current = w;
                    lineEmpty = false;
                } else if (current + join + w <= params.boxWidthMm) {
                    wrapped += ' ';
                    current += join + w;
                } else {
                    wrapped += '\n';
                    current = w;
                }
                wrapped.append(word);
                if (end == std::string_view::npos) {
                    break;
                }
                begin = end + 1;
            }
        }
        text::TextLayout l = text::layoutText(font, wrapped, style, limits);
        if (!l.ok()) {
            return refused(l.status, "the text could not be laid out");
        }
        const double ascender =
            static_cast<double>(font.metrics().ascender) * params.sizeMm / static_cast<double>(font.metrics().unitsPerEm);
        const double x = params.align == TextAlign::Centre ? params.boxWidthMm / 2.0
                         : params.align == TextAlign::Right ? params.boxWidthMm
                                                            : 0.0;
        const double top = -ascender;
        // The last baseline plus the descender must fit in the box height.
        if (!l.lines.empty() && params.boxHeightMm > 0.0) {
            const double descender = static_cast<double>(font.metrics().descender) * params.sizeMm /
                                     static_cast<double>(font.metrics().unitsPerEm);
            const double lastBaseline = top + l.lines.back().baselineStart.y;
            if (lastBaseline + descender < -params.boxHeightMm) {
                overflow = true;
            }
        }
        if (!take(std::move(l), Vec2{x, top})) {
            return out;
        }
        out.overflow = overflow;
    }

    const bool mirrored = params.placement.a * params.placement.d - params.placement.b * params.placement.c < 0.0;
    for (geometry::Contour& c : local) {
        c.transform(params.placement);
        if (mirrored) {
            c.reverse();
        }
    }
    out.contours = std::move(local);
    return out;
}

}  // namespace carvetto::document
