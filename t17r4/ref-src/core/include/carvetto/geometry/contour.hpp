#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/transform2.hpp"

namespace carvetto::geometry {

/// An ordered chain of spans (lines, arcs, Beziers), open or closed. Layer-agnostic: the
/// document decides where a contour lives. Consecutive spans are expected to share end
/// points; for a closed contour the last span ends where the first starts (see `isContinuous`).
struct Contour {
    std::vector<Span> spans;
    bool closed{false};

    bool operator==(const Contour&) const = default;

    /// Closed circle of one full-circle arc span.
    static Contour circle(Vec2 centre, double radius, ArcDirection dir = ArcDirection::CCW);
    /// Closed counter-clockwise rectangle of four line spans.
    static Contour rectangle(const Box2& box);
    /// Contour of line spans through the polyline's vertices.
    static Contour fromPolyline(const Polyline& pl);

    bool isEmpty() const { return spans.empty(); }
    std::size_t spanCount() const { return spans.size(); }
    Vec2 startPoint() const;
    Vec2 endPoint() const;
    /// Sum of span lengths.
    double length() const;
    /// Exact union of span bounds.
    Box2 bounds() const;
    /// Point at normalised arc-length parameter t in [0,1] over the whole contour.
    Vec2 pointAt(double t) const;
    /// True when every span starts where the previous ended (and the loop closes when `closed`).
    bool isContinuous(const Tolerance& tol) const;
    /// Reverses direction of travel in place: span order and each span are reversed.
    void reverse();
    Contour reversed() const;
    /// Applies `m` to every span in place (arcs may become Beziers under non-conformal maps).
    void transform(const Transform2& m);
    Contour transformed(const Transform2& m) const;
    /// Polygonises every span so that chordal deviation stays within `tol.chordal`. The
    /// result carries the `closed` flag and omits the duplicated closing vertex.
    Polyline polygonise(const Tolerance& tol) const;
    /// Appends a span, enforcing nothing; callers keep continuity.
    void append(Span s) { spans.push_back(std::move(s)); }
};

}  // namespace carvetto::geometry
