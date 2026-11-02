#pragma once

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/path_walk.hpp"

// Shared toolpath machinery (T-0120, ADR-0016): "does this tool-centre path stay at least r away
// from these contours?" Used by the profile toolpath to merge neighbouring offsets, to trim
// stretches that would gouge another vector and to shorten leads; reusable by any operation.

namespace carvetto::toolpath {

/// Distance from points to a fixed set of contours (exact closest points on lines, arcs and
/// Beziers). The obstacles' bounding boxes are kept in a `BoxIndex` (T-0123): a query looks at
/// the boxes within a growing square round the point and stops as soon as the nearest distance
/// found lies inside the square, so far-away contours cost nothing and a field over thousands of
/// vectors answers in time proportional to the vectors near the point. The value is the minimum
/// over all (non-excluded) obstacles, exactly as a scan over all of them would give.
class ClearanceField {
   public:
    ClearanceField() = default;
    explicit ClearanceField(std::vector<geometry::Contour> obstacles);

    bool isEmpty() const { return size() == 0; }
    /// Number of obstacles that count (excluded ones do not).
    std::size_t size() const;
    /// Bounds of all obstacles (excluded ones included: a superset is all callers need).
    const geometry::Box2& bounds() const;
    /// Distance from `p` to the nearest obstacle (infinity when there are none).
    double distance(geometry::Vec2 p) const;
    /// The same field without the obstacles at the given indices (into the constructor's
    /// vector). Cheap: the obstacles and their index are shared, not copied.
    ClearanceField excluding(std::vector<std::size_t> indices) const;

   private:
    struct Data {
        std::vector<geometry::Contour> obstacles;
        BoxIndex index;
    };
    bool skipped(std::size_t i) const;
    std::shared_ptr<const Data> data_;
    std::vector<std::size_t> skip_;  // sorted, unique
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
