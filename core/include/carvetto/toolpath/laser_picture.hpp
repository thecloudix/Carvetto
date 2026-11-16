#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/image_engrave.hpp"
#include "carvetto/toolpath/laser.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Laser Picture toolpath (T-0324, ADR-0104; feature-inventory "Laser Picture"): the beam runs along lines over a
// bitmap (the image pipeline of Image Engrave, T-0250) and its power follows the pixels: dark pixels burn hard,
// light pixels not at all. A dithered image burns full power on the pixels error diffusion makes dark.

namespace carvetto::toolpath {

/// Parameters of a Laser Picture. Lengths in mm, angles in radians, power in percent. The strategy is the Image
/// Engrave one: Raster (parallel lines), Hatch (Raster plus a second pass at right angles), SelectedVectors (the
/// supplied vectors are followed).
struct LaserPictureParams {
    std::string name{"Laser Picture"};
    PhotoStrategy strategy{PhotoStrategy::Raster};
    /// Beam power of a fully dark pixel, percent in (0, 100].
    double power{100.0};
    /// Power modulation: the beam power of a pixel is `minPower` + (`power` - `minPower`) x darkness (a pixel with
    /// darkness 0 is never burned). Off: a pixel at least half dark burns at `power`, others not at all.
    bool modulatePower{true};
    /// Beam power of the faintest burned pixel (percent, 0 <= minPower <= power); only with `modulatePower`.
    double minPower{0.0};
    /// Dither Image: error diffusion (Floyd-Steinberg) makes every pixel fully dark or not at all before the lines are
    /// run, so the beam works at `power` or is off.
    bool dither{false};
    /// Light pixels burn instead of dark ones.
    bool invert{false};
    /// Speed of every move in mm/min; 0 takes the tool's feed rate (cutting data).
    double moveSpeed{0.0};
    /// How many times the whole path is run over (>= 1).
    int passes{1};
    /// Raster and Hatch lines run this far past the burned stretch with the beam off, so the beam is at full speed
    /// while it burns (mm, >= 0).
    double overscan{0.0};
    /// Distance between neighbouring lines (mm, > 0), shrunk slightly so the lines tile the placement evenly.
    double lineSpacing{0.1};
    /// Direction of the Raster lines (and of the first Hatch pass), CCW from +X.
    double lineAngle{0.0};
    /// Absolute Z the head rises to after the last pass.
    double safeZ{5.0};
    /// `chordal`: polygonising selected vectors.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const LaserPictureParams&) const = default;
};

struct LaserPictureResult {
    Toolpath toolpath;
    std::vector<ToolpathDiagnostic> diagnostics;
    /// Strokes (runs of beam travel) per pass.
    std::size_t strokes{0};
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// The darkness image the lines read: 255 - gray (gray when `invert`), then error-diffused to 0 / 255 when `dither`.
GrayImage laserDarkness(const GrayImage& image, bool invert, bool dither);

/// Error diffusion of a darkness image (Floyd-Steinberg, row by row from the top, 7/16 right, 3/16 down-left, 5/16
/// down, 1/16 down-right; a pixel is dark when value plus carried error is at least 128). Integer arithmetic.
GrayImage laserDither(const GrayImage& darkness);

/// Beam power (percent, 0 = off) of a pixel of darkness 0..255 under `params` (see `modulatePower`).
double laserPicturePower(const LaserPictureParams& params, int darkness);

/// Computes the toolpath. `placement` is the rectangle the image covers in document mm (top row at placement.max.y).
/// Lines are placed like Image Engrave's; the power is read from the nearest pixel (exact pixel runs along each
/// line), so a run of one pixel value is one feed move with one power. Beam-off stretches of 5 mm or more split a
/// line (a rapid between the strokes); lines run as a serpentine; the head moves at Z 0 and rises to safe Z at the
/// end. `contours` are used by SelectedVectors only (parts outside the placement are not burned). Bad input gives an
/// empty toolpath and a diagnostic, never a crash. `control.workLimit` (pixel edge crossings; default 50 million)
/// and `control.moveLimit` bound the work; `control.cancel` is polled per line.
LaserPictureResult laserPictureToolpath(const GrayImage& image, const geometry::Box2& placement,
                                        const tooling::Tool& tool,
                                        const std::optional<tooling::CuttingData>& cuttingData,
                                        const LaserPictureParams& params,
                                        const std::vector<geometry::Contour>& contours = {},
                                        const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
