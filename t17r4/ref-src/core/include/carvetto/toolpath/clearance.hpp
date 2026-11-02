#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/path_walk.hpp"

// Shared toolpath machinery (T-0120, ADR-0016): "does this tool-centre path stay at least r away
// from these contours?" Used by the profile toolpath to merge neighbouring offsets, to trim
// stretches that would gouge another vector and to shorten leads; reusable by any operation.

namespace carvetto::toolpath {

/// Distance from points to a fixed set of contours (exact closest points on lines, arcs and
/// Beziers), with a bounding-box prefilter so far-away contours cost one box test.
class ClearanceField {
   public:
    ClearanceField() = default;
    explicit ClearanceField(std::vector<geometry::Contour> obstacles);

    bool isEmpty() const { return obstacles_.empty(); }
    std::size_t size() const { return obstacles_.size(); }
    /// Bounds of all obstacles.
    const geometry::Box2& bounds() const { return all_; }
    /// Distance from `p` to the nearest obstacle (infinity when there are none).
    double distance(geometry::Vec2 p) const;

   private:
    std::vector<geometry::Contour> obstacles_;
    std::vector<geometry::Box2> boxes_;
    geometry::Box2 all_;
};

/// Stretches [a, b] (arc length along `path`, within [u0, u1], u0 <= u1) where the distance to
/// `field` drops below `threshold`, found to within `resolution`.
///
/// Approach: the distance is 1-Lipschitz along the path, so an interval whose end samples both
/// exceed threshold + half its length is certified clear. Other intervals are bisected down to
/// `resolution`. Each returned stretch runs from the last clear sample before a violation to the
/// first clear sample after it, so it covers the violation (removing it errs on the safe side).
std::vector<std::pair<double, double>> findViolations(const ArcLengthPath& path, double u0, double u1,
                                                      const ClearanceField& field, double threshold,
                                                      double resolution);

/// True when the whole path stays at least `threshold` (to within `resolution`) from `field`.
bool isClear(const ArcLengthPath& path, const ClearanceField& field, double threshold, double resolution);

}  // namespace carvetto::toolpath
