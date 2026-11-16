#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/image_engrave.hpp"
#include "carvetto/toolpath/laser.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Laser Sketch Engraving (T-0324, ADR-0105; feature-inventory "Laser Sketch Engraving"): a tone drawing made of
// hatch lines. The source (a bitmap, or a 3D model rendered to a tone image by the relief engine) gives the tone;
// along every hatch line the beam burns a line of up to `lineThickness` wide where the tone is dark, narrower
// where it is lighter and not at all where it is white. The beam power is constant.

namespace carvetto::toolpath {

/// Where the tone comes from.
enum class SketchSource { Bitmap, Model };

/// How a 3D model becomes tone: Depth (the lowest point is black, the highest white) or Shaded (the surface lit
/// from the upper left at 45 degrees: a flat surface is white, a slope facing away from the light dark).
enum class SketchModelTone { Depth, Shaded };

/// The area that is sketched: the whole source (the bitmap's placement, the model's extent) or the closed vectors.
enum class SketchBoundary { Source, Vectors };

/// Order of the hatch lines over the vectors. Together: the vectors' union is one region and every line crosses all
/// of it before the next (a vector inside another adds nothing). Regionally: the same union, but each connected
/// piece of it is sketched completely before the next. BetweenVectors: the even-odd region (a vector inside another
/// is a hole), each connected piece in turn. With the Source boundary the one region is the source's rectangle.
enum class SketchFill { Together, Regionally, BetweenVectors };

/// Parameters of a Laser Sketch Engraving. Lengths in mm, angles in radians, power in percent.
struct LaserSketchParams {
    std::string name{"Laser Sketch"};
    SketchSource source{SketchSource::Bitmap};
    SketchModelTone modelTone{SketchModelTone::Depth};
    SketchBoundary boundary{SketchBoundary::Source};
    /// Light areas burn instead of dark ones.
    bool invert{false};
    /// Beam power in percent in (0, 100].
    double power{100.0};
    /// Speed of every move in mm/min; 0 takes the tool's feed rate (cutting data).
    double moveSpeed{0.0};
    /// Hatch lines run this far past the burned stretch with the beam off (mm, >= 0).
    double overscan{0.0};
    /// Machine Limit Boundary: nothing is burned outside the machine limits given to the operation.
    bool machineLimitBoundary{false};
    /// Width of a hatch line where the tone is fully dark (mm, > 0). The line is drawn with
    /// max(1, round(lineThickness / kerf)) tracks a kerf apart; a lighter tone lights fewer of them (centre first).
    double lineThickness{0.3};
    /// Distance between neighbouring hatch lines (mm, > 0).
    double stepover{1.0};
    /// Angle of the hatch lines from the X axis (radians).
    double hatchAngle{0.0};
    /// A second set of lines at right angles to the first.
    bool crossHatch{false};
    SketchFill fill{SketchFill::Together};
    /// Absolute Z the head rises to after the last line.
    double safeZ{5.0};
    OrderOptions order;
    geometry::Vec2 startPosition{0.0, 0.0};
    /// `chordal`: polygonising curves; `distance`: shorter pieces are dropped.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const LaserSketchParams&) const = default;
};

struct LaserSketchResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// Strokes (runs of beam travel).
    std::size_t strokes{0};
    /// Tracks a full-dark hatch line is drawn with.
    std::size_t tracks{0};
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// The tone image of a relief model: `field` rendered per `tone` (a transparent cell is white). `placement` receives
/// the rectangle the image covers in model mm (cell edges, row 0 at the top = the highest Y). A field wider than the
/// image limits is shrunk by an integer factor (averaging the opaque cells); an empty or all-transparent field gives
/// an empty image.
GrayImage toneFromHeightField(const relief::HeightField& field, SketchModelTone tone, geometry::Box2& placement);

/// Number of tracks of a hatch line of `thickness` drawn by a beam of `kerf`: max(1, round(thickness / kerf)),
/// at most 64.
std::size_t laserSketchTracks(double thickness, double kerf);

/// Tracks lit at a darkness of 0..255 on a line of `tracks` tracks: round(darkness / 255 x tracks).
std::size_t laserSketchLit(int darkness, std::size_t tracks);

/// Computes the sketch of the tone image `tone` (gray values, 0 = black) placed at `placement` (document mm, top row
/// at placement.max.y). Hatch lines lie on the multiples of the stepover across their direction (so a cross hatch
/// and neighbouring runs line up), run as a serpentine through the region; with the Vectors boundary the region is
/// the closed vectors (even-odd or union per `fill`) inset by the beam radius, with Source it is the placement.
/// `machineLimits` (document mm) is honoured when `machineLimitBoundary` is set. The head moves at Z 0 and rises to
/// safe Z at the end. Bad input gives an empty toolpath and a diagnostic. `control.workLimit` (pixel edge crossings;
/// default 50 million) and `control.moveLimit` bound the work; `control.cancel` is polled per line.
LaserSketchResult laserSketchToolpath(const GrayImage& tone, const geometry::Box2& placement, const tooling::Tool& tool,
                                      const std::optional<tooling::CuttingData>& cuttingData,
                                      const LaserSketchParams& params,
                                      const std::vector<geometry::Contour>& contours = {},
                                      const std::optional<geometry::Box2>& machineLimits = std::nullopt,
                                      const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
