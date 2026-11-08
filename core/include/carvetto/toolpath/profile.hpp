#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/lead_moves.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/pass_plan.hpp"
#include "carvetto/toolpath/ramp_entry.hpp"
#include "carvetto/toolpath/tab_placement.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// 2D Profile toolpath (T-0120, ADR-0016; feature-inventory §5 "2D Profile Toolpath").

namespace carvetto::toolpath {

/// Which side of the vectors the tool runs on. Closed vectors are taken as regions (nesting
/// parity: a vector inside one other vector is a hole), so Outside keeps the region — the tool
/// runs outside outer boundaries and inside holes — and Inside runs inside the region. Open
/// vectors: Outside is the right-hand side of the direction of travel, Inside the left.
enum class ProfileSide {
    Outside,
    Inside,
    On,
    Right = Outside,
    Left = Inside,
};

/// Where each closed cut starts (feature-inventory §5 "Start At" / "Use Vector Start Point").
enum class StartAt {
    /// The path point nearest the vector's own start point.
    VectorStart,
    /// The path point nearest the previous cut's start (the first: nearest `startPosition`).
    Optimise,
    /// The path point nearest an anchor of the path's bounding box.
    ClosestOnBoundingBox,
};

/// Anchor on a bounding box for `StartAt::ClosestOnBoundingBox`.
enum class BoxAnchor { BottomLeft, BottomCentre, BottomRight, CentreLeft, Centre, CentreRight, TopLeft, TopCentre, TopRight };

/// V-bit corner options (feature-inventory §5 "Corners"). Ignored, with a diagnostic, unless the
/// tool is a V-bit.
struct CornerOptions {
    /// Convex corners get a sharp mitre instead of an arc about the vertex.
    bool sharpExternal{false};
    /// On the final level, the tool runs from each inside corner of the path to the vector's
    /// corner while rising to the top of the cut, and back, so the V-bit leaves a sharp corner.
    bool sharpInternal{false};
    /// Mitre length limit for sharp external corners, as a multiple of the offset distance;
    /// sharper corners are squared off.
    double mitreLimit{2.0};
    bool operator==(const CornerOptions&) const = default;
};

/// Limits of a profile toolpath (T-0186, ADR-0016 section 19). A job beyond one of them is refused
/// up front with `WorkLimitExceeded` (a parameter or input out of range) and nothing is produced;
/// none of them is near a realistic job (a 5,000-glyph sign, a 3 m x 1.5 m sheet with 10,000
/// vectors, 2 mm deep in 0.1 mm passes).
inline constexpr std::size_t kProfileMaxLevels = 10000;               ///< depth passes
inline constexpr double kProfileMaxMoves = 20e6;                      ///< estimated moves in the toolpath
inline constexpr double kProfileMaxOffsetMm = 1000.0;                 ///< |allowance|, |last pass allowance|,
                                                                      ///< lead length/radius/overcut, ramp distance, tab length
inline constexpr double kProfileMaxCoordinateMm = 1.0e5;              ///< |x|, |y| of every span's end points (control
                                                                      ///< points for a Bezier)
inline constexpr double kProfileMaxRadiusMm = 1.0e12;                 ///< arc radius (a nearly straight arc has a huge one)
inline constexpr double kProfileMaxLengthMm = 1.0e7;                  ///< total length of all vectors
inline constexpr std::size_t kProfileMaxSpans = 2000000;              ///< spans of all vectors
inline constexpr double kProfileMaxTabs = 5.0e6;                      ///< tabs requested over all vectors
/// Default work limit, in work units (see `ProfileResult::workUnits`); `ToolpathRunControl::workLimit`
/// overrides it. A unit is meant as about 30 ns of Release CPU, but the real time per unit varies 4 to
/// 8x with the shape of the job (4 to 67 ns measured): the CPU spent before a run reaches this limit
/// is from about 10 s to over a minute. It is a stop for hostile jobs, not a time bound (ADR-0016
/// section 19; precise bounding needs T-0228, an interruptible geometry offset).
inline constexpr std::uint64_t kProfileDefaultWorkUnits = 900000000ULL;

/// Parameters of a profile toolpath. Lengths in mm, angles in radians.
struct ProfileParams {
    std::string name{"Profile"};
    ProfileSide side{ProfileSide::Outside};
    CutDirection direction{CutDirection::Climb};
    /// Extra distance between the tool and the vector (negative cuts into it). Must be at least
    /// minus `profileToolRadius` (also with `lastPassAllowance` added); below that the toolpath is
    /// refused with `InvalidParameters` (T-0129). On the vector it must also be at most
    /// `profileToolRadius` (also with `lastPassAllowance` added), or the tool would run entirely
    /// off the vector (T-0137); Inside/Outside (and open Left/Right) have no upper limit.
    double allowance{0.0};
    /// Depths. A non-positive `passDepth` takes the cutting data's pass depth.
    PassPlan passes;
    /// Absolute Z for moves between cuts.
    double safeZ{5.0};

    /// Do Separate Last Pass: the levels are cut `lastPassAllowance` further from the vector,
    /// then one finishing pass at full depth at the final distance.
    bool separateLastPass{false};
    double lastPassAllowance{0.2};
    /// The finishing pass runs in the opposite direction.
    bool lastPassReverse{false};

    TabOptions tabs;
    RampOptions ramp;
    LeadOptions leads;
    OrderOptions order;
    StartAt startAt{StartAt::VectorStart};
    BoxAnchor startAnchor{BoxAnchor::BottomLeft};
    /// Tool position before the first cut (XY), for ordering and `StartAt::Optimise`.
    geometry::Vec2 startPosition{0.0, 0.0};
    CornerOptions corners;

    /// Feeds, speeds, tool number and default pass depth; empty = the tool's default data
    /// (`effectiveCuttingData`). Look up machine/material data with `Tool::cuttingDataFor`.
    std::optional<tooling::CuttingData> cuttingData;
    /// As Pocket: 1e-4 <= chordal <= 1 mm and 1e-6 <= distance <= chordal (one offset grid unit,
    /// T-0142), else InvalidParameters.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const ProfileParams&) const = default;
};

/// Profile diagnostics use the shared operation types (T-0123, operation.hpp); the profile's
/// issues are `ProfileIssue::InvalidParameters` … `ProfileIssue::TabThicknessClamped`.
using ProfileIssue = ToolpathIssue;
/// One diagnostic; `contour` is the input vector index when the issue concerns one.
using ProfileDiagnostic = ToolpathDiagnostic;

/// The computed toolpath and what could not be done as asked.
struct ProfileResult {
    Toolpath toolpath;
    std::vector<ProfileDiagnostic> diagnostics;
    /// Work units the run counted (deterministic: depends only on the input and the parameters).
    /// A refused or cancelled run reports the units counted when it stopped.
    std::uint64_t workUnits{0};
    /// True when the toolpath has moves.
    bool ok() const { return !toolpath.isEmpty(); }
    /// True when some diagnostic has the given issue.
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// Radius used to offset the vectors: half the tool's cutting width at the bottom of the cut
/// (`tooling::cuttingWidthAtDepth`), so a V-bit's top edge lands on the vector.
double profileToolRadius(const tooling::Tool& tool, const PassPlan& passes);

/// Computes a 2D profile toolpath around `contours` (closed and open vectors, any mix).
///
/// Approach (ADR-0016):
/// 1. Closed vectors are oriented by nesting parity (outer CCW, holes CW, islands CCW) so the
///    region is on the left. Each vector is offset exactly with `geometry::offsetOpenContour`
///    (arcs stay arcs) by r + allowance to its side (round joins; sharp mitres for a V-bit with
///    `corners.sharpExternal`).
/// 2. Where the offset of a closed vector comes within r + allowance of another closed vector,
///    those vectors are offset together as regions with `geometry::offsetContours` (overlapping
///    cuts merge; polygonal result). Any remaining stretch of a cut that passes within that
///    distance of another vector is cut away (TrimmedNearVector), so no cut gouges a vector.
/// 3. Direction: climb keeps the vector on the right of travel; conventional on the left.
/// 4. Order (`order`), inner cuts first; start points (`startAt`); tabs, leads, ramps.
/// 5. Each cut is machined through all levels before the next: rapid to safe Z above the entry,
///    plunge (or ramp), cut, retract. With a separate last pass, all roughing cuts come first,
///    then the finishing cuts in the same order.
///
/// Output is deterministic. An empty toolpath always comes with at least one diagnostic.
///
/// Work budget (ADR-0016 section 19): parameters and inputs beyond the `kProfileMax...` limits are
/// refused up front (`WorkLimitExceeded`); the run counts work units and stops with
/// `WorkLimitExceeded` when it passes `control.workLimit` (default `kProfileDefaultWorkUnits`), or
/// with `Cancelled` when `control.cancel` reads true. Either way the toolpath is empty and the
/// diagnostics are only the reason: nothing partial is returned. Runs within the budget give the
/// same result with or without a control.
ProfileResult profileToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                              const ProfileParams& params, const ToolpathRunControl& control);
ProfileResult profileToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                              const ProfileParams& params);

}  // namespace carvetto::toolpath
