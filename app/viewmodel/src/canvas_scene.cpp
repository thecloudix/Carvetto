#include "carvetto/viewmodel/canvas_scene.hpp"

#include <algorithm>
#include <vector>

#include "carvetto/document/dimension.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/viewmodel/dimension_text.hpp"
#include "carvetto/viewmodel/keepout.hpp"

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

const std::vector<geometry::Contour>& KeepOutHaloCache::get(const Document& doc) {
    const std::vector<geometry::Contour> zones = keepOutOutlines(doc);
    if (clearance_ != doc.keepOutClearance() || zones != zones_) {
        zones_ = zones;
        clearance_ = doc.keepOutClearance();
        halo_ = keepOutHalo(doc);
    }
    return halo_;
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
            std::vector<geometry::Contour> dimensionContours;
            if (const document::BitmapObject* bmp = obj->asBitmap()) {
                if (!bmp->placementMm.isEmpty() && bmp->placementMm.intersects(visible)) {
                    DrawBitmap bitmap;
                    bitmap.object = id;
                    bitmap.key = bmp->sourceRef;
                    bitmap.rectPx = view.worldToScreen(bmp->placementMm);
                    bitmap.colour = layer.colour;
                    scene.bitmaps.push_back(std::move(bitmap));
                }
                continue;
            }
            const document::VectorObject* vec = obj->asVector();
            if (vec == nullptr) {
                const document::DimensionObject* dim = obj->asDimension();
                if (dim == nullptr) {
                    continue;
                }
                // Dimension lines and arrowheads, then the text as a label.
                const document::DimensionGeometry g = document::dimensionGeometry(doc, *dim);
                dimensionContours = g.contours;
                std::string text = g.valid ? document::dimensionText(doc, *dim, doc.job().displayUnits) : std::string{};
                const double reach = dim->textHeightMm * (1.0 + 0.6 * static_cast<double>(text.size()));
                if (!text.empty() && visible.intersects(Box2(g.textCentre - Vec2{reach, reach},
                                                              g.textCentre + Vec2{reach, reach}))) {
                    DrawLabel label;
                    label.object = id;
                    label.colour = layer.colour;
                    label.text = std::move(text);
                    label.centrePx = view.worldToScreen(g.textCentre);
                    label.angle = -g.textAngle;
                    label.heightPx = dim->textHeightMm * view.scale();
                    for (const geometry::Contour& c : dimensionTextContours(doc, *dim, g, label.text)) {
                        label.outlinesPx.push_back(polygoniseForScreen(c, view, options.tolerancePx));
                    }
                    scene.labels.push_back(std::move(label));
                }
            }
            for (const geometry::Contour& contour : vec != nullptr ? vec->contours : dimensionContours) {
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
    if (doc.keepOutVisible() && !doc.keepOutZones().empty()) {
        auto add = [&](const geometry::Contour& contour, std::vector<DrawPath>& list) {
            if (contour.isEmpty() || !contour.bounds().intersects(visible)) {
                return;
            }
            DrawPath path;
            path.closed = contour.closed;
            path.pointsPx = polygoniseForScreen(contour, view, options.tolerancePx);
            list.push_back(std::move(path));
        };
        for (const document::KeepOutZone& zone : doc.keepOutZones()) {
            add(zone.outline, scene.keepOutZones);
        }
        if (doc.keepOutClearance() > 0.0) {
            KeepOutHaloCache local;
            KeepOutHaloCache& cache = options.haloCache != nullptr ? *options.haloCache : local;
            for (const geometry::Contour& halo : cache.get(doc)) {
                add(halo, scene.keepOutHalo);
            }
        }
    }
    return scene;
}

}  // namespace carvetto::viewmodel
