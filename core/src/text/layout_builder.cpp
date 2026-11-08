#include "layout_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <variant>

#include <hb.h>

#include "font_impl.hpp"

namespace carvetto::text::detail {

using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Transform2;
using geometry::Vec2;

namespace {

constexpr double kMaxLineSpacing = 100.0;
constexpr std::size_t kMaxLanguageLength = 35;

// Length of the UTF-8 sequence starting at s[i], or 0 when it is invalid (then one byte is consumed).
std::size_t sequenceLength(std::string_view s, std::size_t i) {
    const auto b = [&](std::size_t k) { return static_cast<unsigned char>(s[k]); };
    const unsigned char c = b(i);
    if (c < 0x80) {
        return 1;
    }
    std::size_t n = 0;
    unsigned char lo = 0x80;
    unsigned char hi = 0xBF;
    if (c >= 0xC2 && c <= 0xDF) {
        n = 2;
    } else if (c >= 0xE0 && c <= 0xEF) {
        n = 3;
        if (c == 0xE0) {
            lo = 0xA0;
        } else if (c == 0xED) {
            hi = 0x9F;  // no surrogates
        }
    } else if (c >= 0xF0 && c <= 0xF4) {
        n = 4;
        if (c == 0xF0) {
            lo = 0x90;
        } else if (c == 0xF4) {
            hi = 0x8F;  // nothing above U+10FFFF
        }
    } else {
        return 0;
    }
    if (i + n > s.size()) {
        return 0;
    }
    if (b(i + 1) < lo || b(i + 1) > hi) {
        return 0;
    }
    for (std::size_t k = 2; k < n; ++k) {
        if ((b(i + k) & 0xC0) != 0x80) {
            return 0;
        }
    }
    return n;
}

}  // namespace

LayoutBuilder::LayoutBuilder(const Font& font, const TextStyle& style, const TextLimits& limits, TextLayout& out)
    : font_(font),
      style_(style),
      out_(out),
      maxTextBytes_(std::min(limits.maxTextBytes, kAbsoluteMaxTextBytes)),
      maxCodePoints_(std::min(limits.maxCodePoints, kAbsoluteMaxSpans)),
      maxLines_(std::min(limits.maxLines, kAbsoluteMaxSpans)),
      maxGlyphs_(std::min(limits.maxGlyphs, kAbsoluteMaxSpans)),
      maxSpans_(std::min(limits.maxSpans, kAbsoluteMaxSpans)),
      maxGlyphPoints_(std::min(limits.maxGlyphPoints, kAbsoluteMaxSpans)),
      maxPathSpans_(std::min(limits.maxPathSpans, kAbsoluteMaxSpans)),
      maxWork_(std::min(limits.maxWork, kAbsoluteMaxWork)),
      maxSize_(limits.maxSizeMm > 0.0 && limits.maxSizeMm < 1e6 ? limits.maxSizeMm : 1e6) {}

bool LayoutBuilder::fail(TextStatus status, const char* message) {
    out_.status = status;
    out_.message = message;
    out_.glyphs.clear();
    out_.lines.clear();
    out_.contours.clear();
    out_.bounds = geometry::Box2{};
    out_.skippedGlyphs = 0;
    out_.truncated = false;
    return false;
}

bool LayoutBuilder::validate() {
    if (!font_.valid()) {
        return fail(TextStatus::InvalidFont, "the font is not valid");
    }
    const double size = style_.sizeMm;
    if (!std::isfinite(size) || size <= 0.0 || size > maxSize_) {
        return fail(TextStatus::InvalidStyle, "text size must be finite, positive and within the limit");
    }
    if (!std::isfinite(style_.letterSpacingMm) || std::fabs(style_.letterSpacingMm) > maxSize_) {
        return fail(TextStatus::InvalidStyle, "letter spacing must be finite and within the size limit");
    }
    if (!std::isfinite(style_.lineSpacing) || style_.lineSpacing < 0.0 || style_.lineSpacing > kMaxLineSpacing) {
        return fail(TextStatus::InvalidStyle, "line spacing must be between 0 and 100");
    }
    const std::string& lang = style_.language;
    bool langOk = !lang.empty() && lang.size() <= kMaxLanguageLength;
    for (const char ch : lang) {
        const bool alnum = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9');
        langOk = langOk && (alnum || ch == '-');
    }
    if (!langOk) {
        return fail(TextStatus::InvalidStyle, "language must be a short BCP 47 tag");
    }
    return true;
}

bool LayoutBuilder::addWork(std::uint64_t n) {
    out_.work += n;
    if (out_.work > maxWork_) {
        return fail(TextStatus::WorkLimit, "text work limit exceeded");
    }
    return true;
}

bool LayoutBuilder::checkTextSize(std::size_t bytes) {
    if (bytes > maxTextBytes_) {
        return fail(TextStatus::TextTooLong, "text is longer than the limit");
    }
    return true;
}

bool LayoutBuilder::scan(std::string_view text, std::vector<LineRange>& lines) {
    if (text.size() > maxTextBytes_) {
        return fail(TextStatus::TextTooLong, "text is longer than the limit");
    }
    std::size_t codePoints = 0;
    std::size_t lineBegin = 0;
    std::size_t i = 0;
    lines.clear();
    while (i < text.size()) {
        const char c = text[i];
        if (c == '\n' || c == '\r') {
            lines.push_back({lineBegin, i - lineBegin});
            if (lines.size() >= maxLines_) {
                return fail(TextStatus::TooManyLines, "text has more lines than the limit");
            }
            i += (c == '\r' && i + 1 < text.size() && text[i + 1] == '\n') ? std::size_t{2} : std::size_t{1};
            lineBegin = i;
            continue;
        }
        std::size_t n = sequenceLength(text, i);
        if (n == 0) {
            out_.replacedInvalidUtf8 = true;
            n = 1;
        }
        i += n;
        ++codePoints;
        if (codePoints > maxCodePoints_) {
            return fail(TextStatus::TooManyCodePoints, "text has more characters than the limit");
        }
    }
    lines.push_back({lineBegin, text.size() - lineBegin});
    if (lines.size() > maxLines_) {
        return fail(TextStatus::TooManyLines, "text has more lines than the limit");
    }
    return addWork(codePoints);
}

bool LayoutBuilder::shape(std::string_view line, std::size_t base, std::vector<ShapedGlyph>& glyphs) {
    if (line.empty()) {
        return true;
    }
    Font::Impl* impl = font_.impl();
    hb_buffer_t* buf = hb_buffer_create();
    hb_buffer_add_utf8(buf, line.data(), static_cast<int>(line.size()), 0, static_cast<int>(line.size()));
    switch (style_.direction) {
        case TextDirection::LeftToRight: hb_buffer_set_direction(buf, HB_DIRECTION_LTR); break;
        case TextDirection::RightToLeft: hb_buffer_set_direction(buf, HB_DIRECTION_RTL); break;
        case TextDirection::Auto: break;
    }
    // A fixed language: guess_segment_properties would otherwise fall back to the process locale.
    hb_buffer_set_language(buf, hb_language_from_string(style_.language.c_str(), -1));
    hb_buffer_guess_segment_properties(buf);

    hb_feature_t kernOff{};
    kernOff.tag = HB_TAG('k', 'e', 'r', 'n');
    kernOff.value = 0;
    kernOff.start = HB_FEATURE_GLOBAL_START;
    kernOff.end = HB_FEATURE_GLOBAL_END;
    // Explicit shaper list: never HB_SHAPER_LIST (also compiled out with HB_NO_GETENV) or the build's defaults.
    static const char* const kShapers[] = {"ot", nullptr};
    hb_shape_full(impl->hbFont, buf, style_.kerning ? nullptr : &kernOff, style_.kerning ? 0u : 1u, kShapers);

    unsigned int n = 0;
    const hb_glyph_info_t* infos = hb_buffer_get_glyph_infos(buf, &n);
    const hb_glyph_position_t* positions = hb_buffer_get_glyph_positions(buf, &n);
    if (!hb_buffer_allocation_successful(buf) || infos == nullptr || positions == nullptr) {
        hb_buffer_destroy(buf);
        return fail(TextStatus::TooManyGlyphs, "shaping failed");
    }
    shapedTotal_ += n;
    if (shapedTotal_ > maxGlyphs_) {
        hb_buffer_destroy(buf);
        return fail(TextStatus::TooManyGlyphs, "text produces more glyphs than the limit");
    }
    for (unsigned int k = 0; k < n; ++k) {
        ShapedGlyph g;
        g.glyphId = infos[k].codepoint;
        g.cluster = static_cast<std::uint32_t>(base + infos[k].cluster);
        g.advance = positions[k].x_advance;
        g.xOffset = positions[k].x_offset;
        g.yOffset = positions[k].y_offset;
        glyphs.push_back(g);
    }
    hb_buffer_destroy(buf);
    return addWork(4ull * n);
}

double LayoutBuilder::toMm(std::int64_t units) const {
    return toMm(static_cast<double>(units));
}

double LayoutBuilder::toMm(double units) const {
    return (units * style_.sizeMm) / static_cast<double>(font_.metrics().unitsPerEm);
}

const GlyphOutline* LayoutBuilder::outlineFor(std::uint32_t glyphId) {
    const auto it = cache_.find(glyphId);
    if (it != cache_.end()) {
        return &it->second;
    }
    GlyphOutline o = font_.outline(glyphId, maxGlyphPoints_);
    if (!addWork(o.pointCount)) {
        return nullptr;
    }
    for (const Contour& c : o.contours) {
        decodedSpans_ += c.spans.size();
    }
    if (decodedSpans_ > maxSpans_) {
        fail(TextStatus::TooManySpans, "glyph outlines exceed the span limit");
        return nullptr;
    }
    return &cache_.emplace(glyphId, std::move(o)).first->second;
}

bool LayoutBuilder::emit(PlacedGlyph& glyph, const Transform2& placement) {
    glyph.firstContour = out_.contours.size();
    glyph.contourCount = 0;
    const GlyphOutline* o = outlineFor(glyph.glyphId);
    if (o == nullptr) {
        return false;
    }
    if (o->status != OutlineStatus::Ok) {
        ++out_.skippedGlyphs;
        return true;
    }
    const auto map = [&](Vec2 p) { return placement.apply({toMm(p.x), toMm(p.y)}); };
    for (const Contour& src : o->contours) {
        emittedSpans_ += src.spans.size();
        if (emittedSpans_ > maxSpans_) {
            return fail(TextStatus::TooManySpans, "text produces more spans than the limit");
        }
        if (!addWork(src.spans.size())) {
            return false;
        }
        Contour dst;
        dst.closed = src.closed;
        dst.spans.reserve(src.spans.size());
        for (const geometry::Span& s : src.spans) {
            if (const auto* l = std::get_if<LineSpan>(&s)) {
                dst.spans.emplace_back(LineSpan{map(l->start), map(l->end)});
            } else if (const auto* b = std::get_if<BezierSpan>(&s)) {
                dst.spans.emplace_back(BezierSpan{map(b->p0), map(b->p1), map(b->p2), map(b->p3)});
            }
        }
        out_.contours.push_back(std::move(dst));
        ++glyph.contourCount;
    }
    return true;
}

void LayoutBuilder::finish() {
    geometry::Box2 box;
    for (const Contour& c : out_.contours) {
        box.expand(c.bounds());
    }
    out_.bounds = box;
}

}  // namespace carvetto::text::detail
