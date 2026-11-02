#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// Ordered vertex list joined by straight segments. For a closed polyline the last vertex
/// connects back to the first; the first vertex is NOT repeated at the end.
struct Polyline {
    std::vector<Vec2> points;
    bool closed{false};

    bool operator==(const Polyline&) const = default;

    bool isEmpty() const { return points.empty(); }
    std::size_t vertexCount() const { return points.size(); }
    /// Number of straight segments (n-1 open, n closed, 0 for fewer than two vertices).
    std::size_t segmentCount() const;
    /// Start vertex of segment i (0-based); for the closing segment of a closed polyline the last vertex.
    Vec2 segmentStart(std::size_t i) const { return points[i]; }
    /// End vertex of segment i, wrapping to the first vertex for the closing segment.
    Vec2 segmentEnd(std::size_t i) const { return points[(i + 1) % points.size()]; }
    /// Sum of segment lengths (including the closing segment when closed).
    double length() const;
    /// Bounding box of the vertices.
    Box2 bounds() const;
    /// Point at normalised arc-length parameter t in [0,1].
    Vec2 pointAt(double t) const;
    /// Reverses the vertex order in place (a closed polyline keeps its first vertex first).
    void reverse();
    Polyline reversed() const;
    /// Applies `m` to every vertex in place.
    void transform(const Transform2& m);
    Polyline transformed(const Transform2& m) const;
    /// Removes consecutive duplicate vertices (within `tol`), and the trailing vertex of a
    /// closed polyline when it duplicates the first.
    void removeDuplicates(double tol);
    /// Signed area by the shoelace formula (positive for counter-clockwise); 0 when open.
    double signedArea() const;
};

}  // namespace carvetto::geometry
