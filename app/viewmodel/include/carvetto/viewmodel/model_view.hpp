#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
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
    /// Colour (0xFFRRGGBB) of each surface cell from the components' and levels' appearance (T-0305,
    /// ADR-0095), row by row like `field`; 0 means "the view's material". Empty when no appearance is
    /// used on the sheet (or the work would be too large), so the view's material colours everything.
    std::vector<std::uint32_t> colours;
    /// Why there is nothing to show (no levels, nothing visible, too large); empty when ok.
    std::string problem;
    bool ok() const { return problem.empty() && !field.empty(); }
};

inline constexpr std::size_t kMaxSurfaceDimension = 512;
/// Cells x visible components above which `composeModelSurface` skips the colour map.
inline constexpr std::uint64_t kMaxColourWork = std::uint64_t{1} << 27;

/// 0xFFRRGGBB of an appearance material at full light.
std::uint32_t appearanceMaterialColour(relief::AppearanceMaterial m);
/// 0xFFRRGGBB of a solid colour 0xRRGGBB.
std::uint32_t appearanceSolidColour(std::uint32_t rgb);

/// The colour map of `levels` on `grid` (cell values `surface` of the same grid): per cell the colour of the
/// component that supplies the surface there, found by replaying the composition's combine rules (the
/// topmost component for Add, the winner for Merge High/Low, the one below for Subtract/Multiply), with
/// "Same as Parent" resolved through the level (Colour from Children = mean of the colours of the level's
/// components that have their own). 0 where the view's material applies. Empty when no appearance is used.
/// Level effects clip the map but a mirrored copy keeps the colour of the cell it came from.
std::vector<std::uint32_t> composeModelColours(const std::vector<relief::Level>& levels, const relief::Grid& grid,
                                               const relief::HeightField& composed);

/// Composes the visible relief model of `sheet` onto a grid of at most `maxDimension` cells along
/// the longer side (cell size multiplied by an integer stride; coarse cells are exact samples of
/// the composition, so a 2000 x 2000 field costs as much as a 512 x 512 one).
ModelSurface composeModelSurface(const document::Document& doc, document::SheetId sheet,
                                 std::size_t maxDimension = kMaxSurfaceDimension);

/// A toolpath drawn over the model (T-0303, ADR-0085): polylines in model space (x, y, model height;
/// machine Z minus the toolpath's model Z offset), each run being one connected cutting stretch.
struct ModelPathPoint {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};
struct ModelPath {
    std::vector<std::vector<ModelPathPoint>> runs;
    std::uint32_t colour{0xFFD02020u};
};

struct ModelRenderOptions {
    std::size_t width{640};
    std::size_t height{480};
    ModelMaterial material{ModelMaterial::Wood};
    /// Use `ModelSurface::colours` where they are set (turn off to see the view material alone).
    bool useAppearance{true};
    /// Vertical exaggeration.
    double zScale{1.0};
    std::uint32_t background{0xFFE2E4E8u};
    /// Toolpaths drawn as one-pixel lines over the surface, hidden where the surface is in front.
    std::vector<ModelPath> paths;
};

/// The cutting moves (Feed, Plunge, Arc*) of the 3D rough and 3D finish records among `records`, as
/// model-space paths (Z minus the record's model Z offset), colours cycling per toolpath. Rapids and
/// retracts break a run. At most `maxPoints` points are kept per toolpath (evenly thinned).
std::vector<ModelPath> modelPathsFromToolpaths(const std::vector<document::ToolpathRecord>& records,
                                               std::size_t maxPoints = 200000);

/// Draws `surface` from `camera`. Transparent cells leave the background. Deterministic; cost is
/// O(surface cells + pixels it covers).
Raster renderModel(const ModelSurface& surface, const OrbitCamera& camera, const ModelRenderOptions& options = {});

/// The design-space point of the surface under pixel (px, py) of a render with `options` (width,
/// height, zScale) from `camera`: the first surface cell the view ray hits. Nullopt when the ray
/// misses the model or the surface is empty. Used by the Sculpt mode to place the brush (T-0302).
std::optional<relief::Point> pickModelPoint(const ModelSurface& surface, const OrbitCamera& camera,
                                            const ModelRenderOptions& options, double px, double py);

/// A position in the pixels of a render.
struct ScreenPoint {
    double x{0.0};
    double y{0.0};
};

/// Where the model point (x, y) at model height z (before `zScale`) lands in a render with `options`
/// from `camera` (the inverse of the framing `pickModelPoint` uses). Nullopt for an empty surface or
/// a non-finite input. The camera is orthographic, so a flat rectangle projects to a parallelogram.
std::optional<ScreenPoint> projectModelPoint(const ModelSurface& surface, const OrbitCamera& camera,
                                             const ModelRenderOptions& options, double x, double y, double z);

/// The brush ring cursor (T-0309): a circle of `diameter` mm around the design point `centre`,
/// draped on the surface (each of `segments` points takes the surface height there, the height under
/// the centre where the surface is empty), as pixels. Empty when the surface or input is unusable.
std::vector<ScreenPoint> brushRingOutline(const ModelSurface& surface, const OrbitCamera& camera,
                                          const ModelRenderOptions& options, relief::Point centre, double diameter,
                                          std::size_t segments = 48);

/// Reference image over the model (T-0309): where an image of `imageWidth` x `imageHeight` pixels
/// is laid on the design plane, as a rectangle in mm: the biggest one with the image's aspect ratio
/// that fits the model surface, centred on it. All zero when the surface or image is empty.
struct OverlayPlacement {
    double x{0.0};
    double y{0.0};
    double width{0.0};
    double height{0.0};
};
OverlayPlacement overlayPlacement(const ModelSurface& surface, std::size_t imageWidth, std::size_t imageHeight);

/// Opacity clamped to 0..1 (non-finite: 0.5).
double clampedOverlayOpacity(double opacity);

/// Shade (0..1) a surface point with normal (nx, ny, nz) gets from the view's light, for tests.
/// The light sits at the upper left of the viewer.
double modelLightFactor(const OrbitCamera& camera, double nx, double ny, double nz);

}  // namespace carvetto::viewmodel
