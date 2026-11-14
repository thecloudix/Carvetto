#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "carvetto/document/document.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/viewmodel/relief_render.hpp"

// The 3D view of the composed relief model (T-0299, ADR-0077): a Qt-free software renderer. The
// document's visible model is composed onto a bounded grid (level of detail), drawn as a shaded
// height surface with a z-buffer from an orbit camera, and returned as a `Raster`.

namespace carvetto::viewmodel {

/// Materials of the view (a view setting, not stored in the document).
enum class ModelMaterial { SolidColour, Wood, Metal, Stone, Plastic };
const char* modelMaterialName(ModelMaterial m);
/// 0xAARRGGBB of the material at full light.
std::uint32_t modelMaterialColour(ModelMaterial m);

/// Orthographic orbit camera looking at the centre of the model.
struct OrbitCamera {
    /// Rotation about the vertical axis; 0 looks from the front (-Y) towards +Y, X to the right.
    double azimuthRad{0.6};
    /// Angle above the horizon: 0 side view, pi/2 straight down (X right, Y up).
    double elevationRad{0.9};
    /// Magnification relative to "fit the model in the view" (1 = fit).
    double zoom{1.0};
    /// Pan in pixels.
    double panX{0.0};
    double panY{0.0};

    /// Drag by (dx, dy) pixels: horizontal turns, vertical tilts (elevation clamped to 0..pi/2).
    void orbit(double dxPixels, double dyPixels);
    /// Multiplies the zoom by `factor` (clamped to 0.05..200).
    void zoomBy(double factor);
    void pan(double dxPixels, double dyPixels);
    void reset();
    /// Standard views: top, front (looking from -Y), right (looking from +X).
    void viewTop();
    void viewFront();
    void viewRight();
};

/// The composed model, at most `kMaxSurfaceDimension` cells along the longer side.
struct ModelSurface {
    relief::HeightField field;
    /// Cells per surface cell along each axis (1 = full resolution of the finest component).
    std::size_t stride{1};
    /// Why there is nothing to show (no levels, nothing visible, too large); empty when ok.
    std::string problem;
    bool ok() const { return problem.empty() && !field.empty(); }
};

inline constexpr std::size_t kMaxSurfaceDimension = 512;

/// Composes the visible relief model of `sheet` onto a grid of at most `maxDimension` cells along
/// the longer side (cell size multiplied by an integer stride; coarse cells are exact samples of
/// the composition, so a 2000 x 2000 field costs as much as a 512 x 512 one).
ModelSurface composeModelSurface(const document::Document& doc, document::SheetId sheet,
                                 std::size_t maxDimension = kMaxSurfaceDimension);

struct ModelRenderOptions {
    std::size_t width{640};
    std::size_t height{480};
    ModelMaterial material{ModelMaterial::Wood};
    /// Vertical exaggeration.
    double zScale{1.0};
    std::uint32_t background{0xFFE2E4E8u};
};

/// Draws `surface` from `camera`. Transparent cells leave the background. Deterministic; cost is
/// O(surface cells + pixels it covers).
Raster renderModel(const ModelSurface& surface, const OrbitCamera& camera, const ModelRenderOptions& options = {});

/// Shade (0..1) a surface point with normal (nx, ny, nz) gets from the view's light, for tests.
/// The light sits at the upper left of the viewer.
double modelLightFactor(const OrbitCamera& camera, double nx, double ny, double nz);

}  // namespace carvetto::viewmodel
