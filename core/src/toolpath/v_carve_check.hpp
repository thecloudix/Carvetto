#pragma once

// Internal to the V-Carve toolpath (T-0153, ADR-0025 §5): the safety net that checks a medial
// axis against the even-odd region before anything is carved on it. In a header of its own so
// the tests can feed it a skeleton computed for the wrong region (fault injection).

#include <cstddef>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/box_index.hpp"

namespace carvetto::toolpath::detail {

/// The region's boundaries polygonised at `tol` (rings indexed by box). `consistent` accepts a
/// medial axis only when every skeleton point whose radius r is above `2·tol.chordal +
/// tol.distance` (the rings' error with a margin) lies inside the even-odd region and has no
/// wall strictly closer than r minus that margin: the inscribed disc must not reach past a wall
/// the skeleton ignored or read on the wrong side. Each point's disc box is queried against the
/// ring index; membership is carried along the skeleton while consecutive points are nearer to
/// each other than the clearance already proven, and counted exactly where it is not.
/// The slack applies to every disc, not only small ones: a wrong skeleton whose discs all reach
/// less than 2c + d past a true wall passes (ADR-0025 §5).
class SkeletonCheck {
   public:
    SkeletonCheck(const std::vector<geometry::Contour>& boundaries, const geometry::Tolerance& tol);
    /// `own`: indices (into the boundaries) of the part the medial axis was computed for: their
    /// distance is the medial radius itself, so only the other rings are measured; empty = all.
    /// As a backstop against a medial axis that silently dropped an own boundary, the own rings
    /// (those enclosing area) are measured too at every node and wherever membership is counted
    /// exactly (T-0164).
    bool consistent(const geometry::MedialAxis& ax, const std::vector<std::size_t>& own = {}) const;

   private:
    /// Even-odd membership of p (exact ray count over the rings it may cross).
    bool inside(geometry::Vec2 p) const;
    /// True when a segment of a ring not marked in `skip` is strictly closer to p than `limit`.
    bool closer(geometry::Vec2 p, double limit, const std::vector<char>& skip) const;
    geometry::Tolerance tol_;
    std::vector<std::vector<geometry::Vec2>> rings_;
    std::vector<geometry::Box2> ringBoxes_;
    /// Per ring: 1 when its boundary encloses no area (the medial axis ignores it: no wall).
    std::vector<char> noArea_;
    BoxIndex ringIndex_;
};

}  // namespace carvetto::toolpath::detail
