#pragma once

// Text-like inputs for the medial axis scaling tests (T-0152 AC2): glyph-shaped regions built
// from lines, arcs and Beziers (an "O" of two Bezier ellipses, an "E" polygon, a "D" with a
// Bezier bowl and a line/arc counter, a "V" polygon, an "S" of Beziers and lines), and single
// regions with many boundary segments.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::test::medial {

using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Polyline;
using geometry::Vec2;

inline Contour glyphPolygon(std::vector<Vec2> pts, Vec2 o) {
    for (Vec2& q : pts) {
        q = o + q;
    }
    return Contour::fromPolyline(Polyline{std::move(pts), true});
}

/// Ellipse of four cubic Beziers; clockwise when `cw` (a hole).
inline Contour bezierEllipse(Vec2 c, double a, double b, bool cw = false) {
    const double k = 0.5522847498;
    Contour e;
    e.closed = true;
    e.append(BezierSpan{{c.x + a, c.y}, {c.x + a, c.y + k * b}, {c.x + k * a, c.y + b}, {c.x, c.y + b}});
    e.append(BezierSpan{{c.x, c.y + b}, {c.x - k * a, c.y + b}, {c.x - a, c.y + k * b}, {c.x - a, c.y}});
    e.append(BezierSpan{{c.x - a, c.y}, {c.x - a, c.y - k * b}, {c.x - k * a, c.y - b}, {c.x, c.y - b}});
    e.append(BezierSpan{{c.x, c.y - b}, {c.x + k * a, c.y - b}, {c.x + a, c.y - k * b}, {c.x + a, c.y}});
    if (cw) {
        e.reverse();
    }
    return e;
}

/// Appends glyph number `kind` (0..4) with its lower-left corner at `o` (5 x 8 mm); returns
/// the number of spans added.
inline std::size_t appendGlyph(std::vector<Contour>& cs, int kind, Vec2 o) {
    switch (kind) {
        case 0:
            cs.push_back(bezierEllipse(o + Vec2{2.5, 4}, 2.5, 4));
            cs.push_back(bezierEllipse(o + Vec2{2.5, 4}, 1.6, 3.1, true));
            return 8;
        case 1:
            cs.push_back(glyphPolygon({{0, 0}, {5, 0}, {5, 1}, {1.1, 1}, {1.1, 3.5}, {4, 3.5}, {4, 4.5}, {1.1, 4.5},
                                       {1.1, 7}, {5, 7}, {5, 8}, {0, 8}},
                                      o));
            return 12;
        case 2: {
            Contour d;
            d.closed = true;
            d.append(LineSpan{o, o + Vec2{2, 0}});
            d.append(BezierSpan{o + Vec2{2, 0}, o + Vec2{6, 0}, o + Vec2{6, 8}, o + Vec2{2, 8}});
            d.append(LineSpan{o + Vec2{2, 8}, o + Vec2{0, 8}});
            d.append(LineSpan{o + Vec2{0, 8}, o});
            cs.push_back(d);
            Contour h;
            h.closed = true;
            h.append(LineSpan{o + Vec2{1, 1.5}, o + Vec2{1, 6.5}});
            h.append(LineSpan{o + Vec2{1, 6.5}, o + Vec2{2, 6.5}});
            h.append(ArcSpan{o + Vec2{2, 4}, 2.5, geometry::kPi / 2, -geometry::kPi / 2, ArcDirection::CW});
            h.append(LineSpan{o + Vec2{2, 1.5}, o + Vec2{1, 1.5}});
            cs.push_back(h);
            return 8;
        }
        case 3:
            cs.push_back(glyphPolygon({{0, 8}, {1.2, 8}, {2.5, 1.5}, {3.8, 8}, {5, 8}, {3.1, 0}, {1.9, 0}}, o));
            return 7;
        default: {
            Contour s;
            s.closed = true;
            s.append(LineSpan{o, o + Vec2{4, 0}});
            s.append(BezierSpan{o + Vec2{4, 0}, o + Vec2{5.5, 0}, o + Vec2{5.5, 4.5}, o + Vec2{2.5, 4.5}});
            s.append(BezierSpan{o + Vec2{2.5, 4.5}, o + Vec2{1, 4.5}, o + Vec2{1, 7}, o + Vec2{2.5, 7}});
            s.append(LineSpan{o + Vec2{2.5, 7}, o + Vec2{5, 7}});
            s.append(LineSpan{o + Vec2{5, 7}, o + Vec2{5, 8}});
            s.append(LineSpan{o + Vec2{5, 8}, o + Vec2{2.5, 8}});
            s.append(BezierSpan{o + Vec2{2.5, 8}, o + Vec2{0, 8}, o + Vec2{0, 3.5}, o + Vec2{2.5, 3.5}});
            s.append(BezierSpan{o + Vec2{2.5, 3.5}, o + Vec2{4.4, 3.5}, o + Vec2{4.4, 1}, o + Vec2{2.5, 1}});
            s.append(LineSpan{o + Vec2{2.5, 1}, o + Vec2{0, 1}});
            s.append(LineSpan{o + Vec2{0, 1}, o});
            cs.push_back(s);
            return 10;
        }
    }
}

/// Glyph-like regions on a 7 x 9 mm pitch, 200 per row, until at least `segments` spans.
inline std::vector<Contour> glyphPage(std::size_t segments) {
    std::vector<Contour> cs;
    std::size_t n = 0;
    for (std::size_t k = 0; n < segments; ++k) {
        const Vec2 o{static_cast<double>(k % 200) * 7.0, static_cast<double>(k / 200) * 9.0};
        n += appendGlyph(cs, static_cast<int>(k % 5), o);
    }
    return cs;
}

/// One region bounded by a wavy polygon with `n` vertices (radius 100 mm, 40 lobes).
inline std::vector<Contour> wavyRegion(std::size_t n) {
    std::vector<Vec2> p;
    for (std::size_t i = 0; i < n; ++i) {
        const double a = geometry::kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        const double r = 100.0 * (1.0 + 0.05 * std::sin(40.0 * a));
        p.push_back({r * std::cos(a), r * std::sin(a)});
    }
    return {Contour::fromPolyline(Polyline{std::move(p), true})};
}

/// One serpentine strip (a 1 mm wide engraved stroke folded back and forth) with about `n`
/// boundary segments: every fold is drawn with short segments, like a polygonised font outline.
inline std::vector<Contour> serpentineRegion(std::size_t n) {
    // Centre line: horizontal runs of 20 mm joined by half circles of radius 1.5 mm; the region is
    // the centre line offset by +-0.5 mm, both sides polygonised with segments of about 0.25 mm.
    const std::size_t perRun = 80;
    const std::size_t perTurn = 20;
    const std::size_t folds = std::max<std::size_t>(1, n / (2 * (perRun + perTurn)));
    std::vector<Vec2> left;
    std::vector<Vec2> right;
    for (std::size_t f = 0; f < folds; ++f) {
        const double y = 3.0 * static_cast<double>(f);
        const bool forward = f % 2 == 0;
        for (std::size_t i = 0; i < perRun; ++i) {
            const double t = static_cast<double>(i) / perRun;
            const double x = forward ? 20.0 * t : 20.0 * (1.0 - t);
            left.push_back({x, y + (forward ? 0.5 : -0.5)});
            right.push_back({x, y + (forward ? -0.5 : 0.5)});
        }
        if (f + 1 == folds) {
            break;
        }
        const Vec2 c{forward ? 20.0 : 0.0, y + 1.5};
        for (std::size_t i = 0; i < perTurn; ++i) {
            const double a = -geometry::kPi / 2 + geometry::kPi * static_cast<double>(i) / perTurn;
            const Vec2 dir = forward ? Vec2{std::cos(a), std::sin(a)} : Vec2{-std::cos(a), std::sin(a)};
            // The outer side of the turn is the right one when turning left (forward folds).
            left.push_back(c + dir * (forward ? 1.0 : 2.0));
            right.push_back(c + dir * (forward ? 2.0 : 1.0));
        }
    }
    std::vector<Vec2> ring = right;
    ring.insert(ring.end(), left.rbegin(), left.rend());
    return {Contour::fromPolyline(Polyline{std::move(ring), true})};
}

}  // namespace carvetto::test::medial
