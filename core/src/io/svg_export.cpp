#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "carvetto/io/svg.hpp"

// SVG export (ADR-0026). Deterministic: fixed attribute order, shortest round-trip numbers.
namespace carvetto::io {

namespace {

using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Span;
using geometry::Vec2;

constexpr double kMaxCoordinate = 1.0e12;

bool finite(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::fabs(p.x) <= kMaxCoordinate &&
           std::fabs(p.y) <= kMaxCoordinate;
}

bool spanFinite(const Span& s) {
    return std::visit(
        [](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                return finite(v.start) && finite(v.end);
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                return finite(v.centre) && std::isfinite(v.radius) && v.radius > 0.0 &&
                       v.radius <= kMaxCoordinate && std::isfinite(v.startAngle) && std::isfinite(v.endAngle);
            } else {
                return finite(v.p0) && finite(v.p1) && finite(v.p2) && finite(v.p3);
            }
        },
        s);
}

std::string number(double v) {
    if (v == 0.0) {
        v = 0.0;  // no "-0"
    }
    std::array<char, 64> buf{};
    const auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), ec == std::errc{} ? ptr : buf.data());
}

std::string escape(std::string_view s) {
    std::string out;
    for (const char c : s) {
        const auto u = static_cast<unsigned char>(c);
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            default:
                // Control characters are not valid in XML 1.0: replace them.
                out += (u < 0x20) ? ' ' : c;
        }
    }
    return out;
}

class PathWriter {
   public:
    PathWriter(double ox, double oy, double height) : ox_(ox), oy_(oy), h_(height) {}

    std::string pt(Vec2 p) const { return number(p.x - ox_) + " " + number(h_ - (p.y - oy_)); }

    std::string path(const Contour& c) const {
        std::string d = "M" + pt(c.startPoint());
        for (const Span& s : c.spans) {
            if (const auto* l = std::get_if<LineSpan>(&s)) {
                d += "L" + pt(l->end);
            } else if (const auto* b = std::get_if<BezierSpan>(&s)) {
                d += "C" + pt(b->p1) + " " + pt(b->p2) + " " + pt(b->p3);
            } else {
                arc(d, std::get<ArcSpan>(s));
            }
        }
        if (c.closed) {
            d += "Z";
        }
        return d;
    }

   private:
    void arcPiece(std::string& d, const ArcSpan& a, double t0, double t1) const {
        const double sweep = a.sweepAngle() * (t1 - t0);
        const std::string r = number(a.radius);
        // Y is flipped on output: a counter-clockwise arc has sweep-flag 0 in the Y-down page.
        d += "A" + r + " " + r + " 0 " + (std::fabs(sweep) > geometry::kPi ? "1" : "0") + " " +
             (a.direction == ArcDirection::CW ? "1" : "0") + " " + pt(a.pointAt(t1));
    }

    void arc(std::string& d, const ArcSpan& a) const {
        // Sweeps above 270 degrees (and full circles inside longer contours) become two pieces,
        // which keeps the large-arc flag unambiguous.
        if (std::fabs(a.sweepAngle()) > 1.5 * geometry::kPi) {
            arcPiece(d, a, 0.0, 0.5);
            arcPiece(d, a, 0.5, 1.0);
        } else {
            arcPiece(d, a, 0.0, 1.0);
        }
    }

    double ox_;
    double oy_;
    double h_;
};

std::string sanitiseLayerName(std::string_view name) {
    std::string s = escape(name);
    return s.empty() ? std::string{"Layer"} : s;
}

}  // namespace

SvgResult<std::vector<std::uint8_t>> exportSvg(std::span<const SvgLayer> layers,
                                               std::span<const SvgContour> contours,
                                               const SvgExportOptions& options) {
    if (!std::isfinite(options.margin) || options.margin < 0.0 || options.margin > kMaxCoordinate) {
        return SvgError{SvgError::Code::InvalidArgument, "margin must be finite and not negative", 0};
    }
    bool any = false;
    double minX = 0.0;
    double minY = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    for (const SvgContour& c : contours) {
        if (c.layer >= layers.size()) {
            return SvgError{SvgError::Code::InvalidArgument, "contour refers to a missing layer", 0};
        }
        for (const Span& s : c.contour.spans) {
            if (!spanFinite(s)) {
                return SvgError{SvgError::Code::InvalidArgument, "contour holds non-finite or out-of-range geometry",
                                0};
            }
        }
        if (c.contour.spans.empty()) {
            continue;
        }
        const geometry::Box2 b = c.contour.bounds();
        if (!any) {
            minX = b.min.x;
            minY = b.min.y;
            maxX = b.max.x;
            maxY = b.max.y;
            any = true;
        } else {
            minX = std::min(minX, b.min.x);
            minY = std::min(minY, b.min.y);
            maxX = std::max(maxX, b.max.x);
            maxY = std::max(maxY, b.max.y);
        }
    }
    const double ox = minX - options.margin;
    const double oy = minY - options.margin;
    const double w = std::max(maxX - minX + 2.0 * options.margin, 1.0);
    const double h = std::max(maxY - minY + 2.0 * options.margin, 1.0);
    const PathWriter pw(ox, oy, h);

    std::string out;
    out += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out += "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\"";
    out += " width=\"" + number(w) + "mm\" height=\"" + number(h) + "mm\" viewBox=\"0 0 " + number(w) + " " + number(h) + "\"";
    out += " data-origin-mm=\"" + number(ox) + " " + number(oy) + "\"";
    out += " fill=\"none\" stroke=\"#000000\" stroke-width=\"0.1\">\n";

    std::set<std::string> used;
    for (std::size_t li = 0; li < layers.size(); ++li) {
        const std::string base = sanitiseLayerName(layers[li].name);
        std::string name = base;
        for (int k = 2; used.count(name) != 0; ++k) {
            name = base + "_" + std::to_string(k);
        }
        used.insert(name);
        out += "<g id=\"layer" + std::to_string(li) + "\" inkscape:groupmode=\"layer\" inkscape:label=\"" + name +
               "\">\n";
        for (const SvgContour& c : contours) {
            if (c.layer != li || c.contour.spans.empty()) {
                continue;
            }
            const auto* arc = std::get_if<ArcSpan>(&c.contour.spans[0]);
            if (c.contour.spans.size() == 1 && arc != nullptr && std::fabs(arc->sweepAngle()) >= geometry::kTwoPi) {
                const Vec2 centre{arc->centre.x - ox, h - (arc->centre.y - oy)};
                out += "<circle cx=\"" + number(centre.x) + "\" cy=\"" + number(centre.y) + "\" r=\"" +
                       number(arc->radius) + "\"/>\n";
            } else {
                out += "<path d=\"" + pw.path(c.contour) + "\"/>\n";
            }
        }
        out += "</g>\n";
    }
    out += "</svg>\n";
    return std::vector<std::uint8_t>(out.begin(), out.end());
}

}  // namespace carvetto::io
