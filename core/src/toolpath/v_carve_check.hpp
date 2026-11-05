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

/// The region's boundaries polygonised at `tol` (rings indexed by box).
/// `consistent` accepts a medial axis only when, at sampled skeleton points whose radius r is
/// above `2·tol.chordal + tol.distance` (the rings' error with a margin), the distance to the
/// walls is at least r minus that margin (the inscribed disc must not reach past a wall the
/// skeleton ignored or read on the wrong side), and the point lies inside the even-odd region.
/// Every point of a branch is checked when it has at most 8, otherwise 8 evenly spaced ones plus
/// its deepest and its last; every isolated node too. Only the rings meeting the box of the
/// skeleton's inscribed discs take part (no other ring can contain a point or come near it).
class SkeletonCheck {
   public:
    SkeletonCheck(const std::vector<geometry::Contour>& boundaries, const geometry::Tolerance& tol);
    bool consistent(const geometry::MedialAxis& ax) const;

   private:
    /// Even-odd membership of p over `candidates` (the rings that meet a box containing p).
    bool inside(geometry::Vec2 p, const std::vector<std::size_t>& candidates) const;
    /// True when a segment of the candidate rings is strictly closer to p than `limit`.
    bool closer(geometry::Vec2 p, double limit, const std::vector<std::size_t>& candidates) const;
    bool pointOk(geometry::Vec2 p, double r, const std::vector<std::size_t>& candidates) const;
    geometry::Tolerance tol_;
    std::vector<std::vector<geometry::Vec2>> rings_;
    std::vector<geometry::Box2> ringBoxes_;
    BoxIndex ringIndex_;
};

}  // namespace carvetto::toolpath::detail
