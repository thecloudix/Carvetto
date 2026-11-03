#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// A boundary location of the input: contour index (into the input vector), span index and
/// the span's own parameter t in [0,1] (ADR-0005).
struct MedialBoundaryRef {
    std::size_t contour{0};
    std::size_t span{0};
    double t{0.0};
};

/// A point of the medial axis and the radius of the largest inscribed circle centred there
/// (its distance to the region's boundary).
struct MedialPoint {
    Vec2 point;
    double radius{0.0};
};

/// Junction (three or more branches), end point (one branch) or isolated point (none, e.g. the
/// centre of a circle) of the skeleton. A closed loop without junctions gets one node of degree
/// two, where its branch starts and ends.
struct MedialNode {
    Vec2 point;
    double radius{0.0};
    /// Indices into `MedialAxis::branches` of the branches that start or end here (a closed loop
    /// is listed twice).
    std::vector<std::size_t> branches;
};

/// Polyline of the skeleton between two nodes (equal for a closed loop).
struct MedialBranch {
    std::size_t startNode{0};
    std::size_t endNode{0};
    /// Points from the start node's to the end node's, each with its radius. Linear
    /// interpolation of the radius between consecutive points stays within the chordal tolerance.
    std::vector<MedialPoint> points;
    /// Per segment (points[i] -> points[i+1]): the boundary samples the segment is equidistant
    /// to, [0] on its left and [1] on its right in the direction of travel.
    std::vector<std::array<MedialBoundaryRef, 2>> sides;
};

/// Options for `medialAxis`.
struct MedialAxisOptions {
    /// Which parts of the plane the closed contours enclose (as for point-in-contour queries).
    FillRule fillRule{FillRule::EvenOdd};
    /// Upper bound on boundary samples (memory and time guard, about 1.1 KB of peak memory and
    /// 10-25 us of Release CPU each: the 50,000-segment glyph page needs ~2.3 M samples, ~2.5 GB). 0 = automatic: max(250,000, 64 per boundary piece), where pieces are
    /// lines, arcs of up to 45 degrees and Bezier sub-curves. A matching work budget (300 search
    /// steps per allowed sample) bounds inputs whose queries are unusually expensive. When either
    /// is reached, refinement stops and `MedialAxis::accuracyCapped` is set. Text needs about
    /// 30-50 samples per segment; the budgets are hit by long channels just wider than twice the
    /// chordal tolerance (cost ~ length / width) and by large spiky outlines.
    std::size_t maxSamples{0};
};

/// Inner medial axis (skeleton) of a region, with inscribed-circle radii.
struct MedialAxis {
    std::vector<MedialNode> nodes;
    std::vector<MedialBranch> branches;
    /// Input contours not used as boundary: open, empty or non-finite, zero area, or with the
    /// same side (inside or outside) on both sides under the fill rule.
    std::size_t ignoredContours{0};
    /// True when the sample or work budget (`MedialAxisOptions::maxSamples`) stopped refinement:
    /// the accuracy guarantees may not hold.
    bool accuracyCapped{false};
    /// Number of boundary samples the computation used (diagnostic).
    std::size_t boundarySamples{0};
};

/// Computes the medial axis of the region enclosed by `contours` (closed contours of lines, arcs
/// and Beziers; holes are inner contours, interpreted by `options.fillRule`). Every closed
/// contour that separates inside from outside acts as boundary; contours are expected not to
/// cross each other (crossing input does not fail, but every contour then still acts as a wall).
///
/// Approach (ADR-0023): the boundary is sampled, a Delaunay triangulation of the samples is
/// built with exact predicates and refined until every boundary sub-edge is Gabriel (its
/// diametral circle empty), and the Voronoi vertices inside the region (circumcentres) are
/// measured against the exact boundary: where the empty sample circle exceeds the true distance
/// by more than half the chordal tolerance, the boundary is sampled more densely at the nearest
/// point, until the condition holds everywhere. The skeleton is the dual of the Delaunay edges
/// that join two distinct boundary features between interior triangles.
///
/// Guarantees (unless `accuracyCapped`): every output point's radius is its exact distance to
/// the boundary; at every output point two boundary samples that are not neighbours along the
/// boundary lie within radius + tol.chordal / 2 (so the point is equidistant, within the
/// tolerance, from two boundary features); between consecutive points the radius interpolates
/// linearly within tol.chordal. Parts of the region narrower than twice tol.chordal (every
/// inscribed circle below the tolerance) and the last ~tol.chordal of a sharp corner's branch
/// may have no skeleton. Branches into obtuse convex corners are shortened much more, since
/// the spoke test drops the part where the two walls are nearly one line: about 2 x tol.chordal
/// of branch at 120 degrees, 8 x at 150, ~80 x at 170, and the whole bisector branch near 178
/// degrees. The inscribed circles of the remaining skeleton still reach the boundary to within
/// ~0.34 x tol.chordal, so carved depth stays within tolerance, but callers must not assume
/// that every convex corner has a branch reaching it.
///
/// Coordinates: up to ~1e9 mm (1e-12 of the largest coordinate must stay below tol.chordal / 2);
/// beyond that the result is still computed but `accuracyCapped` is set, since rounding then
/// exceeds the tolerance budget. Output is deterministic.
MedialAxis medialAxis(const std::vector<Contour>& contours, const Tolerance& tol,
                      const MedialAxisOptions& options = {});

}  // namespace carvetto::geometry
