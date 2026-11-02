#include "carvetto/viewmodel/canvas_scene.hpp"

#include <algorithm>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::viewmodel {

using document::Document;
using geometry::Box2;
using geometry::Vec2;

namespace {

/// Floor on the world-space chordal tolerance so extreme zooms cannot explode vertex counts.
constexpr double kMinChordalMm = 1e-6;

}  // namespace

std::vector<Vec2> polygoniseForScreen(const geometry::Contour& contour, const ViewTransform& view,
                                      double tolerancePx) {
    geometry::Tolerance tol;
    tol.chordal = std::max(std::max(tolerancePx, 1e-3) / view.scale(), kMinChordalMm);
    tol.distance = std::min(tol.distance, tol.chordal);
    const geometry::Polyline pl = contour.polygonise(tol);
    std::vector<Vec2> out;
    out.reserve(pl.points.size());
    for (const Vec2& p : pl.points) {
        out.push_back(view.worldToScreen(p));
    }
    return out;
}

Box2 activeMaterialMm(const Document& doc) {
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    return sheet != nullptr ? sheet->materialBoundsMm() : Box2{};
}

CanvasScene buildCanvasScene(const Document& doc, const ViewTransform& view, const SceneOptions& options) {
    CanvasScene scene;
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    if (sheet == nullptr) {
        return scene;
    }
    scene.materialPx = view.worldToScreen(sheet->materialBoundsMm());
    scene.datumPx = view.worldToScreen(sheet->datumOffsetMm);

    // Small inflation so strokes straddling the viewport edge are kept.
    const Box2 visible = view.visibleWorld().inflated(2.0 / view.scale());
    for (const document::Layer& layer : doc.layers()) {
        if (!layer.visible) {
            continue;
        }
        for (const document::ObjectId id : doc.objectsOnLayer(layer.id)) {
            const document::Object* obj = doc.findObject(id);
            if (obj == nullptr || obj->sheet != sheet->id) {
                continue;
            }
            const document::VectorObject* vec = obj->asVector();
            if (vec == nullptr) {
                continue;
            }
            for (const geometry::Contour& contour : vec->contours) {
                if (contour.isEmpty() || !contour.bounds().intersects(visible)) {
                    continue;
                }
                DrawPath path;
                path.object = id;
                path.closed = contour.closed;
                path.colour = layer.colour;
                path.pointsPx = polygoniseForScreen(contour, view, options.tolerancePx);
                scene.paths.push_back(std::move(path));
            }
        }
    }
    return scene;
}

}  // namespace carvetto::viewmodel
