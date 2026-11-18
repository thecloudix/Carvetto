#include "carvetto/viewmodel/draw_3d.hpp"

#include <algorithm>
#include <cmath>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::viewmodel {

PlanePick pickZeroPlane(const ModelSurface& surface, const OrbitCamera& camera, const ModelRenderOptions& options,
                        double px, double py, bool wrapped) {
    PlanePick r;
    if (wrapped) {
        r.refusal = "The wrapped rotary view cannot be drawn on; switch to the flat view.";
        return r;
    }
    if (!std::isfinite(px) || !std::isfinite(py)) {
        r.refusal = "The pointer position is not a number.";
        return r;
    }
    const auto o = projectModelPoint(surface, camera, options, 0.0, 0.0, 0.0);
    const auto ex = projectModelPoint(surface, camera, options, 1.0, 0.0, 0.0);
    const auto ey = projectModelPoint(surface, camera, options, 0.0, 1.0, 0.0);
    if (!o || !ex || !ey) {
        r.refusal = "There is no model to draw on.";
        return r;
    }
    // Orthographic camera: the plane maps to the screen by an affine map; invert its 2x2 part.
    const double a = ex->x - o->x, c = ex->y - o->y;  // image of the x axis
    const double b = ey->x - o->x, d = ey->y - o->y;  // image of the y axis
    const double det = a * d - b * c;
    const double la = std::hypot(a, c), lb = std::hypot(b, d);
    if (!(la > 0.0) || !(lb > 0.0) || std::abs(det) / (la * lb) < kMinPlaneSine) {
        r.refusal = "The view is too close to edge-on to place points on the plane.";
        return r;
    }
    const double dx = px - o->x, dy = py - o->y;
    const geometry::Vec2 mm{(d * dx - b * dy) / det, (-c * dx + a * dy) / det};
    if (!std::isfinite(mm.x) || !std::isfinite(mm.y)) {
        r.refusal = "The pick does not land on the plane.";
        return r;
    }
    const relief::HeightField& f = surface.field;
    const double spanX = static_cast<double>(f.width() - 1) * f.cellSize();
    const double spanY = static_cast<double>(f.height() - 1) * f.cellSize();
    const double diag = std::max(std::hypot(spanX, spanY), 1e-6);
    const double cx = f.origin().x + 0.5 * spanX, cy = f.origin().y + 0.5 * spanY;
    if (std::hypot(mm.x - cx, mm.y - cy) > kMaxPickDiagonals * diag) {
        r.refusal = "The pick is too far from the model.";
        return r;
    }
    r.ok = true;
    r.mm = mm;
    r.pxPerMm = std::sqrt(std::abs(det));
    return r;
}

ModelSurface drawingSurface(const document::Document& doc, const ModelSurface& surface) {
    if (surface.ok()) {
        return surface;
    }
    ModelSurface out;
    const geometry::Box2 box = doc.job().materialBoundsMm();
    const double w = box.max.x - box.min.x, h = box.max.y - box.min.y;
    if (!(w > 0.0) || !(h > 0.0) || !std::isfinite(w) || !std::isfinite(h)) {
        out.problem = surface.problem.empty() ? "There is no model to draw on." : surface.problem;
        return out;
    }
    relief::Grid g;
    g.cellSize = std::max(w, h) / 64.0;
    g.width = static_cast<std::size_t>(std::ceil(w / g.cellSize)) + 1;
    g.height = static_cast<std::size_t>(std::ceil(h / g.cellSize)) + 1;
    g.origin = {box.min.x, box.min.y};
    if (relief::HeightField::create(g, 0.0f, out.field) != relief::Status::Ok) {
        out.field = relief::HeightField{};
        out.problem = "There is no model to draw on.";
    }
    return out;
}

std::vector<ModelPath> planeVectorPaths(const document::Document& doc, std::uint32_t colour) {
    std::vector<ModelPath> out;
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    if (sheet == nullptr) {
        return out;
    }
    ModelPath path;
    path.colour = colour;
    for (const document::Layer& layer : doc.layers()) {
        if (!layer.visible) {
            continue;
        }
        for (const document::ObjectId id : doc.objectsOnLayer(layer.id)) {
            const document::Object* obj = doc.findObject(id);
            if (obj == nullptr || obj->sheet != sheet->id || obj->asVector() == nullptr) {
                continue;
            }
            for (const geometry::Contour& c : obj->asVector()->contours) {
                if (c.isEmpty()) {
                    continue;
                }
                const geometry::Polyline poly = c.polygonise(geometry::Tolerance{});
                std::vector<ModelPathPoint> run;
                for (const geometry::Vec2& p : poly.points) {
                    run.push_back({p.x, p.y, 0.0});
                }
                if (poly.closed && !run.empty()) {
                    run.push_back(run.front());
                }
                if (run.size() > 1) {
                    path.runs.push_back(std::move(run));
                }
            }
        }
    }
    if (!path.runs.empty()) {
        out.push_back(std::move(path));
    }
    return out;
}

const char* draw3DShapeName(Draw3DShape shape) {
    switch (shape) {
        case Draw3DShape::Line: return "Line";
        case Draw3DShape::Polyline: return "Polyline";
        case Draw3DShape::Rectangle: return "Rectangle";
        case Draw3DShape::Circle: return "Circle";
        case Draw3DShape::Ellipse: return "Ellipse";
        case Draw3DShape::Polygon: return "Polygon";
        case Draw3DShape::Select: return "Select";
        case Draw3DShape::NodeEdit: return "Edit nodes";
    }
    return "";
}

namespace {

ToolKind toolFor(Draw3DShape s) {
    switch (s) {
        case Draw3DShape::Line:
        case Draw3DShape::Polyline: return ToolKind::Polyline;
        case Draw3DShape::Select: return ToolKind::Select;
        case Draw3DShape::NodeEdit: return ToolKind::NodeEdit;
        case Draw3DShape::Rectangle: return ToolKind::Rectangle;
        case Draw3DShape::Circle: return ToolKind::Circle;
        case Draw3DShape::Ellipse: return ToolKind::Ellipse;
        case Draw3DShape::Polygon: return ToolKind::Polygon;
    }
    return ToolKind::Rectangle;
}

}  // namespace

Draw3DController::Draw3DController(document::Document* doc) : doc_(doc), tool_(doc) {
    tool_.setTool(toolFor(shape_));
}

void Draw3DController::setDocument(document::Document* doc) {
    doc_ = doc;
    tool_.setDocument(doc);
    linePoints_ = 0;
}

void Draw3DController::setShape(Draw3DShape shape) {
    shape_ = shape;
    linePoints_ = 0;
    tool_.setTool(toolFor(shape));  // cancels a shape in progress
}

bool Draw3DController::prepare(const Draw3DView& view, double px, double py, geometry::Vec2& pseudo,
                               Draw3DResult& out) {
    if (view.surface == nullptr || view.camera == nullptr || view.options == nullptr) {
        out.refused = true;
        out.message = "There is no 3D view to draw in.";
        return false;
    }
    const PlanePick pick = pickZeroPlane(*view.surface, *view.camera, *view.options, px, py, view.wrapped);
    if (!pick.ok) {
        out.refused = true;
        out.message = pick.refusal;
        return false;
    }
    // The tool works in pixels of a canvas with `pxPerMm` pixels per millimetre and the origin at (0, 0);
    // the pointer it receives is the picked point in those pixels, so the drag threshold stays in screen px.
    ViewTransform vt;
    vt.setViewport(1.0e6, 1.0e6);
    vt.set(pick.pxPerMm, {0.0, 0.0});
    tool_.setView(vt);
    pseudo = vt.worldToScreen(pick.mm);
    return true;
}

Draw3DResult Draw3DController::afterPress(Draw3DResult r) {
    if (shape_ == Draw3DShape::Line && !r.tool.committed && tool_.active()) {
        if (++linePoints_ >= 2) {
            linePoints_ = 0;
            r.tool = tool_.key(ToolKey::Enter);  // a line is a polyline of two points
        }
    }
    if (r.tool.committed || !tool_.active()) {
        linePoints_ = 0;
    }
    if (!r.tool.message.empty()) {
        r.message = r.tool.message;
    }
    return r;
}

Draw3DResult Draw3DController::pointerMove(const Draw3DView& view, double px, double py, Modifiers mods) {
    Draw3DResult r;
    geometry::Vec2 q;
    if (!prepare(view, px, py, q, r)) {
        return r;
    }
    r.tool = tool_.pointerMove(q, mods);
    return r;
}

Draw3DResult Draw3DController::pointerPress(const Draw3DView& view, double px, double py, Modifiers mods) {
    Draw3DResult r;
    geometry::Vec2 q;
    if (!prepare(view, px, py, q, r)) {
        return r;
    }
    r.tool = tool_.pointerPress(q, mods);
    return afterPress(std::move(r));
}

Draw3DResult Draw3DController::pointerRelease(const Draw3DView& view, double px, double py, Modifiers mods) {
    Draw3DResult r;
    geometry::Vec2 q;
    if (!prepare(view, px, py, q, r)) {
        return r;
    }
    r.tool = tool_.pointerRelease(q, mods);
    if (r.tool.committed || !tool_.active()) {
        linePoints_ = 0;
    }
    if (!r.tool.message.empty()) {
        r.message = r.tool.message;
    }
    return r;
}

Draw3DResult Draw3DController::doubleClick(const Draw3DView& view, double px, double py, Modifiers mods) {
    Draw3DResult r;
    geometry::Vec2 q;
    if (!prepare(view, px, py, q, r)) {
        return r;
    }
    r.tool = tool_.doubleClick(q, mods);
    linePoints_ = 0;
    if (!r.tool.message.empty()) {
        r.message = r.tool.message;
    }
    return r;
}

Draw3DResult Draw3DController::key(ToolKey k) {
    Draw3DResult r;
    r.tool = tool_.key(k);
    if (k == ToolKey::Escape || r.tool.committed) {
        linePoints_ = 0;
    }
    return r;
}

}  // namespace carvetto::viewmodel
