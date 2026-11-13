#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/v_carve.hpp"

// V-carve inlay toolpath (T-0265, ADR-0054; feature-inventory "V-Carve Inlay Toolpath"): the
// pocket and the plug of the same vectors, both cut with a V-bit so that the plug seats in the
// pocket with a stated glue gap. Built on `vCarvingToolpath` (T-0153): flat areas are cleared by
// clearance tools or by the V-bit itself.

namespace carvetto::toolpath {

/// Which piece of the inlay this toolpath cuts: the V-carved pocket in the base, or the plug.
enum class VCarveInlayPiece { Pocket, Plug };

/// What bounds the area of the plug stock that is carved away round the plug.
/// SheetLimits: the plug's sheet. VectorOffset: the (mirrored) vectors grown by `boundaryOffset`.
/// BoundaryOffset: the bounding box of the vectors grown by `boundaryOffset`.
enum class PlugBoundary { SheetLimits, VectorOffset, BoundaryOffset };

/// Parameters of a V-carve inlay. Lengths in mm.
struct VCarveInlayParams {
    std::string name{"V-Carve Inlay"};
    VCarveInlayPiece piece{VCarveInlayPiece::Pocket};
    /// Depth of the pocket floor below the material surface (> 0). Limited to the depth at which
    /// the V-bit reaches its full width (`CarveDepthLimited`).
    double pocketDepth{3.0};
    /// Vertical gap (>= 0, < the pocket depth) left between the seated plug and the pocket
    /// floor and flanks; the plug then stands this far proud of the surface (to be sanded flush).
    double glueGap{0.1};
    /// Extra depth (>= 0) the plug stock is carved away beyond the pocket depth, so the flat
    /// background round the plug stays clear of the base when it is seated.
    double surfaceClearance{0.0};
    /// Plug only: the sheet the plug is cut on, as a sheet id; 0 = the sheet of the vectors. The
    /// plug is mirrored; on another sheet it is also centred on that sheet.
    std::uint64_t plugSheet{0};
    PlugBoundary plugBoundary{PlugBoundary::BoundaryOffset};
    /// Offset (>= 0) of VectorOffset / BoundaryOffset.
    double boundaryOffset{10.0};
    /// Absolute Z of the moves between cuts (> 0).
    double safeZ{5.0};
    /// Flat-area clearance, as in `VCarvingParams`.
    PocketStrategy clearanceStrategy{PocketStrategy::Offset};
    CutDirection clearanceDirection{CutDirection::Climb};
    double clearanceRasterAngle{0.0};
    bool rampPlunge{false};
    double rampDistance{10.0};
    OrderOptions order;
    geometry::Vec2 startPosition{0.0, 0.0};
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const VCarveInlayParams&) const = default;
};

/// True for the tools the carving tool of a V-carve inlay may be (V-bits only: the flank angle
/// is what makes the pieces mate).
bool vCarveInlayToolAllowed(tooling::ToolType type);

/// The plug as geometry: `region` is what the plug stage V-carves (the plug area's boundary minus
/// the plug's top face), `top` the plug's top face (the vectors mirrored, placed, and inset by
/// (pocket depth - glue gap) x tan(half angle)), `boundary` the outer boundary. All polygonal.
struct VCarvePlug {
    std::vector<geometry::Contour> region;
    std::vector<geometry::Contour> top;
    std::vector<geometry::Contour> boundary;
    /// The depth the plug stage carves flat (pocket depth + surface clearance, limited to the
    /// V-bit's full-width depth) and the effective pocket depth (limited likewise).
    double plugDepth{0.0};
    double pocketDepth{0.0};
    std::vector<ToolpathDiagnostic> diagnostics;
};

/// The plug geometry for `contours` (closed vectors; even-odd region). `plugSheet` is the bounds
/// of the sheet the plug is cut on (needed for SheetLimits and for a plug on another sheet;
/// empty = unknown). Empty `region` with a diagnostic when it cannot be made.
VCarvePlug vCarvePlugGeometry(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                              const VCarveInlayParams& params, const geometry::Box2& plugSheet);

/// Computes the pocket or the plug (`params.piece`). Result as `vCarvingToolpath`: clearance
/// toolpaths first, then the carving tool's. Tool: a V-bit passing `tooling::validate` with feed
/// and plunge rates > 0; clearance tools end mills. Refuses (empty, with a diagnostic) bad
/// parameters, an unusable tool, no closed vector, a glue gap not below the pocket depth, a plug
/// boundary too tight for the plug's flank (it must leave 2 x plug depth x tan(half angle) beyond
/// the plug's top face), or an unknown sheet for SheetLimits / another sheet.
VCarvingResult vCarveInlayToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                                   const std::optional<tooling::CuttingData>& cuttingData,
                                   const std::vector<PocketTool>& clearanceTools, const VCarveInlayParams& params,
                                   const geometry::Box2& plugSheet = geometry::Box2{});

}  // namespace carvetto::toolpath
