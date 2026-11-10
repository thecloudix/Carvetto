#pragma once

#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/layer.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

namespace carvetto::viewmodel {

/// One polygonised contour ready to stroke, in screen pixels.
struct DrawPath {
    document::ObjectId object{};
    std::vector<geometry::Vec2> pointsPx;
    bool closed{false};
    /// Colour of the object's layer.
    document::Colour colour{};
};

/// The text of a dimension (T-0255). `outlinesPx` are the glyph outlines (closed polygons, fill
/// them non-zero) laid out with the text core; when it is empty (no font could set the text) the
/// widget draws `text` with its own font, centred on `centrePx`, turned by `angle` (radians, screen
/// orientation: y down) at `heightPx` digit height.
struct DrawLabel {
    document::ObjectId object{};
    document::Colour colour{};
    std::string text;
    std::vector<std::vector<geometry::Vec2>> outlinesPx;
    geometry::Vec2 centrePx;
    double angle{0.0};
    double heightPx{0.0};
};

/// Everything the 2D canvas paints for one frame, in screen pixels. The Qt widget only strokes
/// and fills what is listed here.
struct CanvasScene {
    /// Material rectangle of the active sheet (empty if the sheet is missing).
    geometry::Box2 materialPx;
    /// Screen position of the active sheet's XY datum.
    geometry::Vec2 datumPx;
    /// Visible vector contours on the active sheet, bottom layer first, then creation order.
    std::vector<DrawPath> paths;
    /// Dimension texts on visible layers, in the order of `paths`.
    std::vector<DrawLabel> labels;
    /// Keep-out zone outlines and the clearance halo round them (T-0256), empty when the zones are
    /// hidden. `object` is null and `colour` unused for these.
    std::vector<DrawPath> keepOutZones;
    std::vector<DrawPath> keepOutHalo;
};

/// Remembers the halo of the keep-out zones between frames (it only changes with the zones).
class KeepOutHaloCache {
   public:
    /// The halo contours of `doc`, recomputed only when its zones or clearance changed.
    const std::vector<geometry::Contour>& get(const document::Document& doc);

   private:
    std::vector<geometry::Contour> zones_;
    double clearance_{-1.0};
    std::vector<geometry::Contour> halo_;
};

/// Options for building a scene.
struct SceneOptions {
    /// Maximum chordal deviation of drawn curves, in screen pixels.
    double tolerancePx{0.25};
    /// Optional per-widget cache for the keep-out halo; computed afresh each frame when null.
    KeepOutHaloCache* haloCache{nullptr};
};

/// Polygonises `contour` so its deviation on screen stays within `tolerancePx`, and maps the
/// vertices to screen pixels.
std::vector<geometry::Vec2> polygoniseForScreen(const geometry::Contour& contour, const ViewTransform& view,
                                                double tolerancePx);

/// Builds the frame for the active sheet: material rectangle, datum, and every vector object on a
/// visible layer whose bounds touch the visible area (others are culled).
CanvasScene buildCanvasScene(const document::Document& doc, const ViewTransform& view,
                             const SceneOptions& options = {});

/// World rectangle a "zoom to material" fits: the active sheet's material.
geometry::Box2 activeMaterialMm(const document::Document& doc);

}  // namespace carvetto::viewmodel
