#pragma once

#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::toolpath {

/// Parameters for joining contour passes into one continuous move list.
struct LinkingOptions {
    /// Height (absolute Z) at which rapid moves between contours travel.
    double safeZ{5.0};
    /// Chordal tolerance used to polygonise Bezier spans (lines and arcs are emitted exactly).
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};
};

/// Appends the moves that cut one contour at the given Z level: Feed moves for line spans,
/// ArcCW/ArcCCW moves for arc spans, and Feed moves along the polygonised Bezier spans. The
/// tool is assumed to already sit at the contour's start point at `z`; no positioning, plunge
/// or retract is emitted. Zero-length spans produce no move.
void appendContourMoves(const geometry::Contour& contour, double z, const LinkingOptions& options,
                        Toolpath& out);

/// Appends one complete pass over `contour` at `z`: Rapid to safe Z above the start point,
/// Plunge to `z`, the cutting moves, Retract to safe Z above the end point. Does nothing for an
/// empty contour.
void linkContour(const geometry::Contour& contour, double z, const LinkingOptions& options, Toolpath& out);

/// Links each contour in order at a single depth with `linkContour`, so two closed contours
/// produce exactly: rapid, plunge, cuts…, retract, rapid, plunge, cuts…, retract.
void linkContours(const std::vector<geometry::Contour>& contours, double z, const LinkingOptions& options,
                  Toolpath& out);

/// Links each contour in order at every Z level in `levels` (shallowest first, as produced by
/// `PassPlan::levels()`) before moving on to the next contour. Between levels of a closed
/// contour the tool plunges straight to the next level (it is already back at the start point);
/// an open contour is cut in alternating directions so no retract is needed between levels.
void linkContours(const std::vector<geometry::Contour>& contours, const std::vector<double>& levels,
                  const LinkingOptions& options, Toolpath& out);

}  // namespace carvetto::toolpath
