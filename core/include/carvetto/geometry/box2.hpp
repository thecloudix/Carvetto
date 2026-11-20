#pragma once

#include <algorithm>
#include <limits>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

namespace box_detail {
// T-0364: the box arithmetic selects by value, never through std::min/std::max. Those return a
// reference, and MSVC 14.52 /O2 compiles them into a conditional move of pointers to the operands;
// when the operands are the fields of a local Box2 that an earlier expand() just updated (only in
// registers), the select reads the stale memory slot (the initial +-infinity). A Box2 built from
// two segment ends then kept one field of four (seen as empty ring indexes and extents that were
// only the last point, CI runs of T-0359 and PR 231). Same results as std::min/std::max,
// including for NaN (the first argument wins when the comparison is false).
constexpr double minOf(double a, double b) { return b < a ? b : a; }
constexpr double maxOf(double a, double b) { return a < b ? b : a; }
}  // namespace box_detail

/// Axis-aligned bounding box in millimetres. Default-constructed boxes are empty
/// (min > max) and absorb the first point they are expanded by.
struct Box2 {
    Vec2 min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    Vec2 max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};

    constexpr Box2() = default;
    /// Box spanning two corners (any order).
    constexpr Box2(Vec2 a, Vec2 b)
        : min{box_detail::minOf(a.x, b.x), box_detail::minOf(a.y, b.y)},
          max{box_detail::maxOf(a.x, b.x), box_detail::maxOf(a.y, b.y)} {}

    /// Box centred on `centre` with the given full width/height.
    static constexpr Box2 fromCentre(Vec2 centre, double width, double height) {
        const Vec2 half{width * 0.5, height * 0.5};
        return Box2{centre - half, centre + half};
    }

    constexpr bool operator==(const Box2&) const = default;

    /// True when no point has been added (or the box was explicitly emptied).
    constexpr bool isEmpty() const { return min.x > max.x || min.y > max.y; }

    constexpr double width() const { return isEmpty() ? 0.0 : max.x - min.x; }
    constexpr double height() const { return isEmpty() ? 0.0 : max.y - min.y; }
    constexpr Vec2 size() const { return {width(), height()}; }
    constexpr Vec2 centre() const { return isEmpty() ? Vec2{} : (min + max) * 0.5; }
    constexpr double area() const { return width() * height(); }

    /// Grows the box to include `p`.
    constexpr void expand(Vec2 p) {
        const double lx = box_detail::minOf(min.x, p.x);
        const double ly = box_detail::minOf(min.y, p.y);
        const double hx = box_detail::maxOf(max.x, p.x);
        const double hy = box_detail::maxOf(max.y, p.y);
        min = Vec2{lx, ly};
        max = Vec2{hx, hy};
    }
    /// Grows the box to include `other`.
    constexpr void expand(const Box2& other) {
        if (other.isEmpty()) {
            return;
        }
        expand(other.min);
        expand(other.max);
    }
    /// Box grown by `margin` on every side (negative shrinks; may become empty).
    constexpr Box2 inflated(double margin) const {
        if (isEmpty()) {
            return *this;
        }
        Box2 b;
        b.min = {min.x - margin, min.y - margin};
        b.max = {max.x + margin, max.y + margin};
        return b;
    }

    /// True when `p` lies inside or on the boundary (within `tol`).
    constexpr bool contains(Vec2 p, double tol = 0.0) const {
        return !isEmpty() && p.x >= min.x - tol && p.x <= max.x + tol && p.y >= min.y - tol &&
               p.y <= max.y + tol;
    }
    /// True when `other` lies entirely inside this box.
    constexpr bool contains(const Box2& other) const {
        return !other.isEmpty() && contains(other.min) && contains(other.max);
    }
    /// True when the two boxes share at least one point.
    constexpr bool intersects(const Box2& other) const {
        return !isEmpty() && !other.isEmpty() && min.x <= other.max.x && other.min.x <= max.x &&
               min.y <= other.max.y && other.min.y <= max.y;
    }
    /// Smallest box containing both.
    constexpr Box2 united(const Box2& other) const {
        Box2 b = *this;
        b.expand(other);
        return b;
    }
    /// Overlap of both boxes (empty when they do not intersect).
    constexpr Box2 intersected(const Box2& other) const {
        if (!intersects(other)) {
            return Box2{};
        }
        Box2 b;
        b.min = {box_detail::maxOf(min.x, other.min.x), box_detail::maxOf(min.y, other.min.y)};
        b.max = {box_detail::minOf(max.x, other.max.x), box_detail::minOf(max.y, other.max.y)};
        return b;
    }
};

/// True when both boxes are empty, or their corners agree within `tol`.
bool approxEqual(const Box2& a, const Box2& b, double tol = 1e-9);

}  // namespace carvetto::geometry
