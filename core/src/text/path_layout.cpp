#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <variant>

#include "carvetto/text/layout.hpp"
#include "layout_builder.hpp"

// Text on a curve (T-0130, ADR-0033). The guide contour is parametrised by arc length: lines
// exactly, arcs by angle (their parameter is linear in length), Beziers through a table of
// 64 parameter steps per span (cumulative Gauss-Legendre arc lengths, inverted by interpolation). Tangents come from the exact derivative. Lines and Beziers use only
// + - * / and sqrt, which are correctly rounded everywhere, so their output is bit-identical on
// every platform.

namespace carvetto::text {

using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::LineSpan;
using geometry::Transform2;
using geometry::Vec2;

namespace {

constexpr std::size_t kTableSegments = 64;

Vec2 unit(Vec2 v, Vec2 fallback) {
    const double l2 = v.x * v.x + v.y * v.y;
    if (!(l2 > 0.0) || !std::isfinite(l2)) {
        return fallback;
    }
    const double l = std::sqrt(l2);
    return {v.x / l, v.y / l};
}

double chordLength(Vec2 a, Vec2 b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

double derivativeSpeed(const BezierSpan& b, double t) {
    const Vec2 d = b.derivativeAt(t);
    return std::sqrt(d.x * d.x + d.y * d.y);
}

// Arc length of `b` between parameters t0 and t1 (3-point Gauss-Legendre on the speed).
double speedIntegral(const BezierSpan& b, double t0, double t1) {
    constexpr double kNode = 0.7745966692414834;  // sqrt(3/5)
    const double mid = 0.5 * (t0 + t1);
    const double half = 0.5 * (t1 - t0);
    return half * (5.0 * derivativeSpeed(b, mid - half * kNode) + 8.0 * derivativeSpeed(b, mid) +
                   5.0 * derivativeSpeed(b, mid + half * kNode)) / 9.0;
}

struct Piece {
    const geometry::Span* span{nullptr};
    double start{0.0};
    double length{0.0};
    std::size_t table{0};  // first of kTableSegments + 1 cumulative arc lengths (Beziers only)
};

struct Sample {
    Vec2 point;
    Vec2 tangent{1.0, 0.0};
};

class PathSampler {
public:
    /// Builds the table; false when the path is unusable (empty, non-finite or zero length).
    bool build(const geometry::Contour& path) {
        closed_ = path.closed;
        for (const geometry::Span& s : path.spans) {
            double len = 0.0;
            std::size_t table = 0;
            if (const auto* l = std::get_if<LineSpan>(&s)) {
                len = chordLength(l->start, l->end);
            } else if (const auto* a = std::get_if<ArcSpan>(&s)) {
                len = a->radius * std::fabs(a->sweepAngle());
            } else {
                const auto& b = std::get<BezierSpan>(s);
                table = tables_.size();
                len = bezierTable(b);
            }
            if (!std::isfinite(len) || len < 0.0) {
                return false;
            }
            if (len > 0.0) {
                pieces_.push_back({&s, length_, len, table});
                length_ += len;
            } else {
                tables_.resize(table);  // drop the table of a zero-length Bezier
            }
        }
        return !pieces_.empty() && std::isfinite(length_) && length_ > 0.0;
    }

    double length() const { return length_; }
    bool closed() const { return closed_; }

    Sample at(double s) const {
        // Last piece whose start <= s.
        auto it = std::upper_bound(pieces_.begin(), pieces_.end(), s,
                                   [](double v, const Piece& p) { return v < p.start; });
        if (it != pieces_.begin()) {
            --it;
        }
        const Piece& p = *it;
        const double f = std::clamp((s - p.start) / p.length, 0.0, 1.0);
        Sample out;
        if (const auto* l = std::get_if<LineSpan>(p.span)) {
            out.point = geometry::lerp(l->start, l->end, f);
            out.tangent = unit(l->end - l->start, {1.0, 0.0});
        } else if (const auto* a = std::get_if<ArcSpan>(p.span)) {
            out.point = a->pointAt(f);
            out.tangent = a->tangentAt(f);
        } else {
            const auto& b = std::get<BezierSpan>(*p.span);
            const double* cum = &tables_[p.table];
            const double target = f * cum[kTableSegments];
            // First table entry >= target.
            const double* hi = std::lower_bound(cum, cum + kTableSegments + 1, target);
            std::size_t i = static_cast<std::size_t>(hi - cum);
            i = std::clamp<std::size_t>(i, 1, kTableSegments);
            const double lo = cum[i - 1];
            const double span = cum[i] - lo;
            const double frac = span > 0.0 ? (target - lo) / span : 0.0;
            const double t0 = static_cast<double>(i - 1) / static_cast<double>(kTableSegments);
            double t = (static_cast<double>(i - 1) + frac) / static_cast<double>(kTableSegments);
            // Refine inside the cell: Newton on the exact arc length from the cell start.
            for (int iteration = 0; iteration < 3; ++iteration) {
                const double speed = derivativeSpeed(b, t);
                if (!(speed > 0.0)) {
                    break;
                }
                t -= (speedIntegral(b, t0, t) - (target - lo)) / speed;
                t = std::clamp(t, t0, static_cast<double>(i) / static_cast<double>(kTableSegments));
            }
            out.point = b.pointAt(t);
            const double segments = static_cast<double>(kTableSegments);
            const Vec2 secant = b.pointAt(static_cast<double>(i) / segments) -
                                b.pointAt(static_cast<double>(i - 1) / segments);
            out.tangent = unit(b.derivativeAt(t), unit(secant, {1.0, 0.0}));
        }
        return out;
    }

private:
    // Appends the cumulative arc-length table of `b` (64 equal parameter steps, each integrated
    // with 3-point Gauss-Legendre on |B'(t)|, which is accurate to ~h^6); returns the total length.
    double bezierTable(const BezierSpan& b) {
        const double step = 1.0 / static_cast<double>(kTableSegments);
        double cum = 0.0;
        tables_.push_back(0.0);
        for (std::size_t i = 0; i < kTableSegments; ++i) {
            cum += speedIntegral(b, static_cast<double>(i) * step, static_cast<double>(i + 1) * step);
            tables_.push_back(cum);
        }
        return cum;
    }

    std::vector<Piece> pieces_;
    std::vector<double> tables_;
    double length_{0.0};
    bool closed_{false};
};

}  // namespace

TextLayout layoutTextOnPath(const Font& font, std::string_view utf8, const TextStyle& style,
                            const geometry::Contour& path, const PathTextOptions& options,
                            const TextLimits& limits) {
    TextLayout out;
    detail::LayoutBuilder b(font, style, limits, out);
    if (!b.validate()) {
        return out;
    }
    if (!std::isfinite(options.startOffsetMm)) {
        b.fail(TextStatus::InvalidStyle, "the start offset must be finite");
        return out;
    }
    if (path.spans.empty()) {
        b.fail(TextStatus::InvalidPath, "the guide path is empty");
        return out;
    }
    if (path.spans.size() > b.maxPathSpans()) {
        b.fail(TextStatus::PathTooComplex, "the guide path has more spans than the limit");
        return out;
    }
    PathSampler sampler;
    if (!b.addWork(static_cast<std::uint64_t>(path.spans.size()) * (kTableSegments + 1))) {
        return out;
    }
    if (!sampler.build(path)) {
        b.fail(TextStatus::InvalidPath, "the guide path is non-finite or has zero length");
        return out;
    }

    // One line: line breaks become spaces.
    std::string text(utf8);
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\r' && i + 1 < text.size() && text[i + 1] == '\n') {
            text.erase(i, 1);
        }
        if (text[i] == '\r' || text[i] == '\n') {
            text[i] = ' ';
        }
    }
    std::vector<detail::LineRange> lines;
    if (!b.scan(text, lines)) {
        return out;
    }
    std::vector<detail::ShapedGlyph> shaped;
    if (!b.shape(text, 0, shaped)) {
        return out;
    }

    std::int64_t totalUnits = 0;
    for (const detail::ShapedGlyph& g : shaped) {
        totalUnits += g.advance;
    }
    const double count = static_cast<double>(shaped.size());
    const double width = shaped.empty() ? 0.0 : b.toMm(totalUnits) + (count - 1.0) * style.letterSpacingMm;
    double s0 = options.startOffsetMm;
    if (style.alignment == Alignment::Centre) {
        s0 -= width / 2.0;
    } else if (style.alignment == Alignment::Right) {
        s0 -= width;
    }
    const double pathLength = sampler.length();

    TextLine line;
    line.widthMm = width;
    line.firstGlyph = 0;
    std::int64_t pen = 0;
    double index = 0.0;
    bool haveStart = false;
    for (const detail::ShapedGlyph& g : shaped) {
        const double advance = b.toMm(g.advance);
        double mid = s0 + (b.toMm(pen) + index * style.letterSpacingMm) + advance / 2.0;
        pen += g.advance;
        index += 1.0;
        if (sampler.closed()) {
            mid = std::fmod(mid, pathLength);  // exact in IEEE arithmetic
            if (mid < 0.0) {
                mid += pathLength;
            }
        } else if (mid < 0.0 || mid > pathLength) {
            out.truncated = true;
            continue;
        }
        if (!b.addWork(8)) {
            return out;
        }
        const Sample at = sampler.at(mid);
        const Vec2 d = at.tangent;
        // Glyph-local (x along the baseline, y up) -> world: rotate to the tangent, put the advance
        // midpoint on the path point; shaping offsets move the glyph inside its own frame.
        const double lx = b.toMm(g.xOffset) - advance / 2.0;
        const double ly = b.toMm(g.yOffset);
        const Vec2 origin{at.point.x + (d.x * lx - d.y * ly), at.point.y + (d.y * lx + d.x * ly)};

        PlacedGlyph pg;
        pg.glyphId = g.glyphId;
        pg.cluster = g.cluster;
        pg.line = 0;
        pg.missing = g.glyphId == 0;
        pg.advance = advance;
        pg.origin = origin;
        pg.baselineDir = d;
        if (!b.emit(pg, Transform2{d.x, -d.y, d.y, d.x, origin.x, origin.y})) {
            return out;
        }
        if (!haveStart) {
            line.baselineStart = origin;
            haveStart = true;
        }
        out.glyphs.push_back(pg);
    }
    line.glyphCount = out.glyphs.size();
    out.lines.push_back(line);
    b.finish();
    return out;
}

}  // namespace carvetto::text
