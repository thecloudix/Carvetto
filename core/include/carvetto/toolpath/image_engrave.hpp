#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/io/raster.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Image Engrave toolpath (T-0250, ADR-0047; feature-inventory "Image Engrave Toolpath"): a V-bit
// (or engraving bit) runs along lines over a bitmap and its depth follows the image: dark pixels
// cut deep, light pixels shallow, bounded by Max Carving Depth and by the depth at which the cone
// is as wide as the tool.

namespace carvetto::toolpath {

/// A grayscale image: 8-bit, row-major, row 0 at the top, 0 = black, 255 = white.
struct GrayImage {
    int width{0};
    int height{0};
    std::vector<std::uint8_t> pixels;

    bool valid() const {
        return width > 0 && height > 0 && width <= io::kMaxImageEdge && height <= io::kMaxImageEdge &&
               static_cast<std::size_t>(width) * static_cast<std::size_t>(height) <= io::kMaxImagePixels &&
               pixels.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    }
    bool operator==(const GrayImage&) const = default;
};

/// Luma (integer weights 299/587/114) of an RGB image; an invalid image gives an empty one.
GrayImage grayFromRaster(const io::RasterImage& image);

/// Raster: parallel lines at the line angle. Hatch: Raster plus a second pass at right angles.
/// SelectedVectors: the supplied vectors are followed, the depth sampled along them.
enum class PhotoStrategy { Raster, Hatch, SelectedVectors };

/// Parameters of an image engrave. Lengths in mm, angles in radians.
struct ImageEngraveParams {
    std::string name{"Image Engrave"};
    /// Max Carving Depth (> 0): the depth of a fully dark pixel (before the tool limit).
    double maxDepth{2.0};
    /// Dark pixels cut deep by default; true makes light pixels cut deep.
    bool invert{false};
    /// Stepover Retract (>= 0): the tool stays down between one line (or vector) and the next when
    /// the depth change of the link move is at most this; otherwise it retracts to safe Z.
    double stepoverRetract{2.0};
    /// Line Spacing as a percentage of the tool diameter, in (0, 1000].
    double lineSpacingPercent{20.0};
    /// Line Angle: direction of the Raster lines (and of the first Hatch pass), CCW from +X.
    double lineAngle{0.0};
    PhotoStrategy strategy{PhotoStrategy::Raster};
    /// Absolute Z for moves between cuts (> 0).
    double safeZ{5.0};
    /// A straight run of samples whose depth stays within this of a straight ramp is merged (mm, > 0).
    double depthTolerance{0.01};
    /// `chordal`: polygonising selected vectors.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const ImageEngraveParams&) const = default;
};

struct ImageEngraveResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// Lines (or vectors) that carry a cut.
    std::size_t lineCount{0};
    /// The depth limit of the tool (full cone width = tool diameter), mm.
    double toolDepthLimit{0.0};
    /// Deepest cut made (mm below the surface, >= 0).
    double deepest{0.0};
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// V-bit and engraving bit.
bool imageEngraveToolAllowed(tooling::ToolType type);

/// Depth at which the cone of a V-bit or engraving bit is as wide as its diameter:
/// (D - tip) / (2 tan(angle / 2)); 0 for another type or a malformed tool.
double imageEngraveDepthLimit(const tooling::Tool& tool);

/// Depth (mm, >= 0) for a gray value 0..255 (may be fractional): maxDepth * darkness, darkness =
/// 1 - gray/255 (gray/255 when inverted), then clamped to `depthLimit` (ADR-0047).
double imageEngraveDepth(const ImageEngraveParams& params, double depthLimit, double gray);

/// Computes the toolpath. `placement` is the rectangle the image covers in document mm (top row at
/// placement.max.y). `contours` are used by SelectedVectors only (polygonised to
/// `tolerance.chordal`, parts outside the placement are not cut). The depth is bilinearly sampled
/// at the pixel pitch along each line; line pitch is the line spacing, shrunk slightly so the
/// lines tile the placement evenly. Bad input (invalid image, empty placement, non-finite or
/// out-of-range parameters, wrong tool) gives an empty toolpath and a diagnostic, never a crash.
/// `control.cancel` stops the run with `Cancelled`; `control.workLimit` (samples; default 50
/// million) and `control.moveLimit` bound the work.
ImageEngraveResult imageEngrave(const GrayImage& image, const geometry::Box2& placement, const tooling::Tool& tool,
                              const std::optional<tooling::CuttingData>& cuttingData,
                              const ImageEngraveParams& params,
                              const std::vector<geometry::Contour>& contours = {},
                              const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
