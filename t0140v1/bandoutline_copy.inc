#include <algorithm>
#include <numbers>
#include <optional>
#include "carvetto/geometry/contour.hpp"
namespace carvetto::toolpath { using geometry::Contour; using geometry::Vec2;
std::optional<Contour> bandOutline(const Contour& path, double w, double chordal) {
    struct Piece {
        Vec2 a, b;        // start and end points
        Vec2 ta, tb;      // unit tangents there
        bool arc{false};
        double len{0.0};  // a line's length
        Vec2 c;           // arc centre
        double rho{0.0};  // arc radius
        double a0{0.0};   // start angle
        double sweep{0.0};
    };
    std::vector<Piece> ps;
    for (const geometry::Span& sp : path.spans) {
        if (const auto* l = std::get_if<geometry::LineSpan>(&sp)) {
            const Vec2 d = l->end - l->start;
            const double len = d.length();
            if (!(len > 1e-12)) continue;
            Piece q;
            q.a = l->start;
            q.b = l->end;
            q.ta = d / len;
            q.tb = q.ta;
            q.len = len;
            ps.push_back(q);
        } else if (const auto* a = std::get_if<geometry::ArcSpan>(&sp)) {
            const double sweep = a->sweepAngle();
            if (!(a->radius * std::fabs(sweep) > 1e-12)) continue;
            Piece q;
            q.a = a->startPoint();
            q.b = a->endPoint();
            q.ta = a->tangentAt(0.0);
            q.tb = a->tangentAt(1.0);
            q.arc = true;
            q.c = a->centre;
            q.rho = a->radius;
            q.a0 = a->startAngle;
            q.sweep = sweep;
            ps.push_back(q);
        } else {
            return std::nullopt;
        }
    }
    if (ps.empty()) return std::nullopt;
    std::vector<Vec2> pts;
    // Points of the arc about q of radius r from angle t0 through the signed sweep (both ends).
    auto arcPoints = [&](Vec2 q, double r, double t0, double sweep) {
        std::size_t n = 1;
        if (r > chordal) {
            const double step = 2.0 * std::acos(1.0 - chordal / r);
            n = static_cast<std::size_t>(std::ceil(std::fabs(sweep) / step));
            n = std::clamp(n, std::size_t{1}, std::size_t{100000});
        }
        for (std::size_t k = 0; k <= n; ++k) {
            pts.push_back(q + Vec2::fromAngle(t0 + sweep * static_cast<double>(k) / static_cast<double>(n), r));
        }
    };
    auto angleOf = [](Vec2 v) { return std::atan2(v.y, v.x); };
    // One side of the path, forward: side = -1 the right, +1 the left.
    auto chain = [&](double side) {
        for (std::size_t i = 0; i < ps.size(); ++i) {
            const Piece& q = ps[i];
            if (!q.arc) {
                pts.push_back(q.a + q.ta.perp() * (side * w));
                pts.push_back(q.b + q.tb.perp() * (side * w));
            } else {
                // The centre lies on the left of a CCW arc: there this side's offset is rho - w.
                const bool inner = (q.sweep > 0.0) == (side > 0.0);
                const double r = inner ? q.rho - w : q.rho + w;
                if (r > 0.0) {
                    arcPoints(q.c, r, q.a0, q.sweep);
                } else {
                    pts.push_back(q.c + Vec2::fromAngle(q.a0, r));
                    pts.push_back(q.c);
                    pts.push_back(q.c + Vec2::fromAngle(q.a0 + q.sweep, r));
                }
            }
            if (i + 1 == ps.size()) break;
            // The corner to the next piece: a wedge on its outer side, the vertex on the inner.
            const Vec2 tin = q.tb;
            const Vec2 tout = ps[i + 1].ta;
            const double turn = std::atan2(tin.cross(tout), tin.dot(tout));
            if (std::fabs(turn) <= 1e-12) continue;
            if ((turn > 0.0) == (side > 0.0)) {
                // Leaving the vertex out takes the triangle it spans with the two offset ends off
                // the winding; where both neighbours are lines whose pieces cover that triangle
                // (they reach w sin(turn) back from the vertex) it stays covered by the other, so
                // a polygonised curve needs no spike at each vertex.
                const Piece& next = ps[i + 1];
                const bool covered = !q.arc && !next.arc && std::fabs(turn) < 0.5 * std::numbers::pi &&
                                     std::min(q.len, next.len) >= w * std::sin(std::fabs(turn));
                if (!covered) pts.push_back(q.b);
            } else {
                arcPoints(q.b, w, angleOf(tin.perp() * side), turn);
            }
        }
    };
    chain(-1.0);
    const Piece& last = ps.back();
    arcPoints(last.b, w, angleOf(-last.tb.perp()), std::numbers::pi);
    const std::size_t leftFrom = pts.size();
    chain(1.0);
    std::reverse(pts.begin() + static_cast<std::ptrdiff_t>(leftFrom), pts.end());
    const Piece& first = ps.front();
    arcPoints(first.a, w, angleOf(first.ta.perp()), std::numbers::pi);
    Contour out;
    out.closed = true;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        const Vec2 u = pts[k];
        const Vec2 v = pts[(k + 1) % pts.size()];
        if (u != v) out.spans.push_back(geometry::LineSpan{u, v});
    }
    if (out.spans.size() < 3) return std::nullopt;
    return out;
}
}
