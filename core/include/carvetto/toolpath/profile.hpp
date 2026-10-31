#pragma once

#include <cstddef>
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

/// Climb or conventional milling, assuming a clockwise (M3) spindle: climb keeps the vector on
/// the right of the direction of travel (outside profiles run clockwise, inside counter-clockwise).
enum class CutDirection { Climb, Conventional };

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

/// Parameters of a profile toolpath. Lengths in mm, angles in radians.
struct ProfileParams {
    std::string name{"Profile"};
    ProfileSide side{ProfileSide::Outside};
    CutDirection direction{CutDirection::Climb};
    /// Extra distance between the tool and the vector (negative cuts into it).
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

    /// Feeds, speeds, tool number and default pass depth; empty = the tool's default data.
    std::optional<tooling::CuttingData> cuttingData;
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};
};

/// Why part of a profile could not be produced as asked.
enum class ProfileIssue {
    InvalidParameters,      ///< Non-finite or out-of-range parameter; nothing produced.
    NoPasses,               ///< The pass plan yields no levels; nothing produced.
    EmptyContour,           ///< A vector without spans was skipped.
    DiscontinuousContour,   ///< A vector whose spans do not join was skipped.
    ToolTooLarge,           ///< The offset of a vector vanished (e.g. a hole smaller than the tool).
    MergedWithNeighbour,    ///< Offsets of nearby vectors overlapped and were merged (polygonal).
    TrimmedNearVector,      ///< Part of a cut was removed because it passed within the tool radius of another vector.
    LeadShortened,          ///< A lead would have gouged and was shortened and/or made gentler (perpendicular / half sweep).
    LeadDropped,            ///< A lead would have gouged at any length and was left out.
    StartMoved,             ///< A closed cut's start was moved along it so that its lead-in fits.
    TabsNotPlaced,          ///< Some tabs could not be placed (overlap, no straight stretch, path too short).
    CornerNeedsVBit,        ///< Sharp corner options need a V-bit; ignored.
    SpiralNeedsClosed,      ///< Spiral ramp on an open vector; a smooth ramp was used.
    TabThicknessClamped,    ///< Tab thickness exceeded the cut depth and was clamped to it.
};

/// One diagnostic; `contour` is the input vector index when the issue concerns one.
struct ProfileDiagnostic {
    ProfileIssue issue{ProfileIssue::InvalidParameters};
    std::optional<std::size_t> contour;
    std::string message;
};

/// The computed toolpath and what could not be done as asked.
struct ProfileResult {
    Toolpath toolpath;
    std::vector<ProfileDiagnostic> diagnostics;
    /// True when the toolpath has moves.
    bool ok() const { return !toolpath.isEmpty(); }
    /// True when some diagnostic has the given issue.
    bool has(ProfileIssue issue) const;
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
ProfileResult profileToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                              const ProfileParams& params);

}  // namespace carvetto::toolpath
