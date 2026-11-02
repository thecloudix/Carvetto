#pragma once

#include <algorithm>
#include <limits>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// Axis-aligned bounding box in millimetres. Default-constructed boxes are empty
/// (min > max) and absorb the first point they are expanded by.
struct Box2 {
    Vec2 min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    Vec2 max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};

    constexpr Box2() = default;
    /// Box spanning two corners (any order).
    constexpr Box2(Vec2 a, Vec2 b)
        : min{std::min(a.x, b.x), std::min(a.y, b.y)}, max{std::max(a.x, b.x), std::max(a.y, b.y)} {}

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
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
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
        b.min = {std::max(min.x, other.min.x), std::max(min.y, other.min.y)};
        b.max = {std::min(max.x, other.max.x), std::min(max.y, other.max.y)};
        return b;
    }
};

/// True when both boxes are empty, or their corners agree within `tol`.
bool approxEqual(const Box2& a, const Box2& b, double tol = 1e-9);

}  // namespace carvetto::geometry
