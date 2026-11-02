#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// What the 2D toolpath operations (Profile, ADR-0016; Pocket, ADR-0017) share in their public
// API (T-0123, ADR-0018): the cut direction, the diagnostics type, and the conversion of a
// tool-database `Tool` with its cutting data into the values a pass needs.

namespace carvetto::toolpath {

/// Climb or conventional milling, assuming a clockwise (M3) spindle. Climb keeps the wall being
/// cut on the right of the direction of travel: outside profiles run clockwise, inside profiles
/// and pocket outer walls counter-clockwise, pocket island walls clockwise. Conventional runs the
/// other way.
enum class CutDirection { Climb, Conventional };

/// Why part of a toolpath could not be produced as asked. Shared by every operation; each
/// value's comment names the operation(s) that report it.
enum class ToolpathIssue {
    InvalidParameters,    ///< Non-finite or out-of-range parameter; nothing produced (all).
    NoPasses,             ///< The pass plan yields no levels; nothing produced (Profile).
    EmptyContour,         ///< A vector without spans was skipped (Profile).
    DiscontinuousContour, ///< A vector whose spans do not join was skipped (Profile).
    ToolTooLarge,         ///< No room for the tool (an offset vanished; a pocket holds none) (all).
    MergedWithNeighbour,  ///< Offsets of nearby vectors overlapped and were merged (Profile).
    TrimmedNearVector,    ///< A stretch of cut too close to another vector was removed (Profile).
    LeadShortened,        ///< A lead would have gouged; shortened and/or made gentler (Profile).
    LeadDropped,          ///< A lead would have gouged at any length and was left out (Profile).
    StartMoved,           ///< A closed cut's start was moved so that its lead-in fits (Profile).
    TabsNotPlaced,        ///< Some tabs could not be placed (Profile).
    CornerNeedsVBit,      ///< Sharp corner options need a V-bit; ignored (Profile).
    SpiralNeedsClosed,    ///< Spiral ramp on an open vector; a smooth ramp was used (Profile).
    TabThicknessClamped,  ///< Tab thickness exceeded the cut depth; clamped (Profile).
    NoTool,               ///< No tool was given; nothing produced (Pocket).
    InvalidTool,          ///< Unusable tool or cutting data (diameter, stepover, pass depth) (Pocket).
    OpenContourIgnored,   ///< Open vectors were ignored: closed ones are needed (Pocket).
    NoRegion,             ///< The closed vectors enclose no area; nothing produced (Pocket).
    WorkLimitExceeded,    ///< Refused: detail, pass depth or stepover too small (Pocket).
    RegionNotKept,        ///< A part could not be kept within the safety margin; not cut (Pocket).
    RegionTrimmed,        ///< A part was trimmed by more than the tolerance (Pocket).
    NoRestMaterial,       ///< A rest tool found nothing left by the earlier tools (Pocket).
    RestUnreachable,      ///< A rest tool cannot reach the material left (Pocket).
};

/// One diagnostic of a toolpath operation.
struct ToolpathDiagnostic {
    ToolpathIssue issue{ToolpathIssue::InvalidParameters};
    /// The input vector index, when the issue concerns one.
    std::optional<std::size_t> contour;
    /// Human-readable explanation (names the vector or tool).
    std::string message;
    /// The index in the operation's tool list, when the issue concerns one tool (Pocket).
    std::optional<std::size_t> tool;

    bool operator==(const ToolpathDiagnostic&) const = default;
};

/// True when some diagnostic in `diagnostics` has the given issue.
bool hasIssue(const std::vector<ToolpathDiagnostic>& diagnostics, ToolpathIssue issue);

/// The cutting data an operation uses: `chosen` when given, else the tool's default data
/// (`Tool::defaultCuttingData`). Callers that know the machine and material look the data up
/// with `Tool::cuttingDataFor` and pass it as `chosen`.
const tooling::CuttingData& effectiveCuttingData(const tooling::Tool& tool,
                                                  const std::optional<tooling::CuttingData>& chosen);

/// The tool snapshot stored in a toolpath: name and diameter from the tool; tool number, feed,
/// plunge rate and spindle speed from the cutting data.
ToolSnapshot toolSnapshot(const tooling::Tool& tool, const tooling::CuttingData& data);

/// What a 2D pass takes from a tool and its cutting data (the Tool → pass tool conversion).
struct PassTool {
    /// Copied into the toolpath (see `toolSnapshot`).
    ToolSnapshot snapshot;
    /// Half the width the tool cuts at the bottom of the cut (`tooling::cuttingWidthAtDepth`):
    /// D/2 for end mills; for a V-bit or ball nose the radius at the material surface, so the
    /// top edge of the cut keeps the distance. This is the radius offsets are computed with.
    double radius{0.0};
    /// Lateral step between neighbouring passes (mm), from the cutting data.
    double stepover{0.0};
    /// Depth of each pass (mm), from the cutting data.
    double passDepth{0.0};

    bool operator==(const PassTool&) const = default;
};

/// Converts `tool` + `data` for a cut whose bottom lies `bottomDepth` mm below the material
/// surface (start depth + cut depth).
PassTool passTool(const tooling::Tool& tool, const tooling::CuttingData& data, double bottomDepth);

}  // namespace carvetto::toolpath
