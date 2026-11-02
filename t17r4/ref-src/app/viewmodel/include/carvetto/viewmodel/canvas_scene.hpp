#pragma once

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

/// Everything the 2D canvas paints for one frame, in screen pixels. The Qt widget only strokes
/// and fills what is listed here.
struct CanvasScene {
    /// Material rectangle of the active sheet (empty if the sheet is missing).
    geometry::Box2 materialPx;
    /// Screen position of the active sheet's XY datum.
    geometry::Vec2 datumPx;
    /// Visible vector contours on the active sheet, bottom layer first, then creation order.
    std::vector<DrawPath> paths;
};

/// Options for building a scene.
struct SceneOptions {
    /// Maximum chordal deviation of drawn curves, in screen pixels.
    double tolerancePx{0.25};
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
