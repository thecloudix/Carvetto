#include "carvetto/viewmodel/dimension_tool.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::viewmodel {

using document::DimensionObject;
using geometry::Vec2;

namespace {

void copyOptions(const DimensionOptions& o, DimensionObject& d) {
    d.kind = o.kind;
    d.vertical = o.vertical;
    d.diameter = o.diameter;
    d.decimals = o.decimals;
    d.angleDecimals = o.angleDecimals;
    d.textHeightMm = o.textHeightMm;
    d.useCustomText = o.useCustomText;
    d.customText = o.customText;
    d.fontKey = o.fontKey;
}

DimensionToolResult refused(std::string message) {
    DimensionToolResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

}  // namespace

void DimensionTool::setDocument(document::Document* doc) {
    doc_ = doc;
    picked_ = 0;
    editTarget_.reset();
    drag_.reset();
}

DimensionObject DimensionTool::draft() const {
    DimensionObject d;
    copyOptions(options_, d);
    d.points = points_;
    return d;
}

DimensionToolResult DimensionTool::setOptions(const DimensionOptions& options) {
    DimensionObject probe;
    copyOptions(options, probe);
    if (const std::string problem = document::dimensionProblem(probe); !problem.empty()) {
        return refused(problem);
    }
    DimensionToolResult r;
    if (options.kind != options_.kind) {
        picked_ = 0;
    }
    options_ = options;
    r.draftChanged = true;
    return r;
}

DimensionToolResult DimensionTool::click(Vec2 mm, double snapToleranceMm) {
    if (doc_ == nullptr) {
        return refused("no document");
    }
    cursor_ = mm;
    const std::size_t needed = document::dimensionPointCount(options_.kind);
    DimensionToolResult r;
    if (picked_ < needed) {
        document::DimensionPoint p;
        p.pos = mm;
        if (const auto hit = document::findAnchorNear(*doc_, doc_->activeSheet(), mm, snapToleranceMm)) {
            p.pos = hit->pos;
            p.anchor = hit->anchor;
        }
        points_[picked_] = p;
        ++picked_;
        r.draftChanged = true;
        return r;
    }
    DimensionObject d = draft();
    std::array<Vec2, 3> resolved{};
    for (std::size_t i = 0; i < resolved.size(); ++i) {
        resolved[i] = d.points[i].pos;
    }
    const auto offset = document::offsetForPlacement(d, resolved, mm);
    if (!offset) {
        picked_ = 0;
        return refused("the picked points do not define a dimension");
    }
    d.offsetMm = *offset;
    const auto id = document::addDimension(*doc_, d, doc_->activeSheet(), options_.layer);
    picked_ = 0;
    if (!id) {
        return refused("the dimension could not be added");
    }
    doc_->selection().assign({*id});
    editTarget_.reset();  // handles appear only after a Shift-click, so the next pick is not a handle grab
    r.committed = true;
    r.created = *id;
    r.draftChanged = true;
    return r;
}

document::DimensionGeometry DimensionTool::preview() const {
    document::DimensionGeometry g;
    if (doc_ != nullptr && drag_) {
        return document::dimensionGeometry(*doc_, drag_->current);
    }
    if (doc_ == nullptr || picked_ == 0) {
        return g;
    }
    const std::size_t needed = document::dimensionPointCount(options_.kind);
    if (picked_ < needed) {
        geometry::Contour c;
        c.spans.push_back(geometry::LineSpan{points_[picked_ - 1].pos, cursor_});
        g.contours.push_back(std::move(c));
        return g;
    }
    DimensionObject d = draft();
    std::array<Vec2, 3> resolved{};
    for (std::size_t i = 0; i < resolved.size(); ++i) {
        resolved[i] = d.points[i].pos;
    }
    if (const auto offset = document::offsetForPlacement(d, resolved, cursor_)) {
        d.offsetMm = *offset;
        return document::dimensionGeometry(*doc_, d);
    }
    return g;
}

DimensionToolResult DimensionTool::edit(document::ObjectId id, const DimensionOptions& options) {
    if (doc_ == nullptr) {
        return refused("no document");
    }
    const document::Object* obj = doc_->findObject(id);
    const DimensionObject* cur = obj != nullptr ? obj->asDimension() : nullptr;
    if (cur == nullptr) {
        return refused("the object is not a dimension");
    }
    DimensionObject next = *cur;
    DimensionOptions o = options;
    o.kind = cur->kind;  // the kind is fixed once drawn
    copyOptions(o, next);
    if (o.offsetMm || o.reflex) {
        double offset = o.offsetMm.value_or(next.offsetMm);
        if (!std::isfinite(offset)) {
            return refused("the offset must be a finite number");
        }
        if (cur->kind == document::DimensionKind::Angular) {
            const bool reflex = o.reflex.value_or(cur->offsetMm < 0.0);
            offset = reflex ? -std::fabs(offset) : std::fabs(offset);
        } else if (cur->kind == document::DimensionKind::Radial) {
            offset = std::max(0.0, offset);
        }
        next.offsetMm = offset;
    }
    if (const std::string problem = document::dimensionProblem(next); !problem.empty()) {
        return refused(problem);
    }
    DimensionToolResult r;
    r.committed = doc_->execute(std::make_unique<document::SetDimensionCommand>(id, std::move(next)));
    return r;
}

std::optional<DimensionOptions> DimensionTool::optionsOf(document::ObjectId id) const {
    if (doc_ == nullptr) {
        return std::nullopt;
    }
    const document::Object* obj = doc_->findObject(id);
    const DimensionObject* d = obj != nullptr ? obj->asDimension() : nullptr;
    if (d == nullptr) {
        return std::nullopt;
    }
    DimensionOptions o;
    o.kind = d->kind;
    o.vertical = d->vertical;
    o.diameter = d->diameter;
    o.decimals = d->decimals;
    o.angleDecimals = d->angleDecimals;
    o.textHeightMm = d->textHeightMm;
    o.useCustomText = d->useCustomText;
    o.customText = d->customText;
    o.fontKey = d->fontKey;
    o.offsetMm = d->kind == document::DimensionKind::Angular ? std::fabs(d->offsetMm) : d->offsetMm;
    o.reflex = d->kind == document::DimensionKind::Angular && d->offsetMm < 0.0;
    o.layer = obj->layer;
    return o;
}

std::optional<DimensionObject> DimensionTool::previewObject() const {
    if (doc_ == nullptr) {
        return std::nullopt;
    }
    if (drag_) {
        return drag_->current;
    }
    if (picked_ == 0 || picked_ < document::dimensionPointCount(options_.kind)) {
        return std::nullopt;
    }
    DimensionObject d = draft();
    std::array<Vec2, 3> resolved{};
    for (std::size_t i = 0; i < resolved.size(); ++i) {
        resolved[i] = d.points[i].pos;
    }
    if (const auto offset = document::offsetForPlacement(d, resolved, cursor_)) {
        d.offsetMm = *offset;
        return d;
    }
    return std::nullopt;
}

namespace {

double distanceToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double len2 = ab.dot(ab);
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    return geometry::distance(p, a + ab * t);
}

}  // namespace

std::optional<document::ObjectId> DimensionTool::editTarget() const {
    if (doc_ == nullptr || !editTarget_) {
        return std::nullopt;
    }
    const document::Object* obj = doc_->findObject(*editTarget_);
    return obj != nullptr && obj->asDimension() != nullptr ? editTarget_ : std::nullopt;
}

std::optional<document::ObjectId> DimensionTool::pickAt(Vec2 mm, double toleranceMm) const {
    if (doc_ == nullptr || !std::isfinite(mm.x) || !std::isfinite(mm.y) || !(toleranceMm >= 0.0)) {
        return std::nullopt;
    }
    std::optional<document::ObjectId> best;
    double bestDist = std::numeric_limits<double>::infinity();
    for (const auto& [id, obj] : doc_->objects()) {  // ascending id: ties go to the lower id
        const DimensionObject* dim = obj.asDimension();
        const document::Layer* layer = doc_->findLayer(obj.layer);
        if (dim == nullptr || obj.sheet != doc_->activeSheet() || layer == nullptr || !layer->visible) {
            continue;
        }
        const document::DimensionGeometry g = document::dimensionGeometry(*doc_, *dim);
        if (!g.valid) {
            continue;
        }
        double d = geometry::distance(mm, g.textCentre) - dim->textHeightMm * 1.5;  // the text's box
        geometry::Tolerance tol;
        tol.chordal = std::max(toleranceMm / 4.0, 1e-6);
        tol.distance = std::min(tol.distance, tol.chordal);
        for (const geometry::Contour& c : g.contours) {
            const geometry::Polyline pl = c.polygonise(tol);
            for (std::size_t i = 0; i + 1 < pl.points.size(); ++i) {
                d = std::min(d, distanceToSegment(mm, pl.points[i], pl.points[i + 1]));
            }
        }
        if (d <= toleranceMm && d < bestDist) {
            bestDist = d;
            best = id;
        }
    }
    return best;
}

DimensionToolResult DimensionTool::selectAt(Vec2 mm, double toleranceMm) {
    DimensionToolResult r;
    if (doc_ == nullptr) {
        return refused("no document");
    }
    drag_.reset();
    const auto id = pickAt(mm, toleranceMm);
    editTarget_ = id;
    if (id) {
        doc_->selection().assign({*id});
    }
    r.draftChanged = true;
    return r;
}

std::vector<document::DimensionHandle> DimensionTool::handles() const {
    const auto id = editTarget();
    if (!id) {
        return {};
    }
    return document::dimensionHandles(*doc_, *doc_->findObject(*id)->asDimension());
}

bool DimensionTool::beginHandle(Vec2 mm, double toleranceMm) {
    const auto id = editTarget();
    if (!id || !(toleranceMm >= 0.0)) {
        return false;
    }
    const DimensionObject& dim = *doc_->findObject(*id)->asDimension();
    const std::vector<document::DimensionHandle> hs = document::dimensionHandles(*doc_, dim);
    const document::DimensionHandle* best = nullptr;
    double bestDist = toleranceMm;
    for (const document::DimensionHandle& h : hs) {
        const double d = geometry::distance(mm, h.pos);
        if (d <= bestDist && (best == nullptr || d < bestDist)) {
            bestDist = d;
            best = &h;
        }
    }
    if (best == nullptr) {
        return false;
    }
    drag_ = Drag{*id, *best, dim, dim, toleranceMm};
    return true;
}

DimensionToolResult DimensionTool::dragTo(Vec2 mm) {
    DimensionToolResult r;
    if (!drag_ || doc_ == nullptr) {
        return r;
    }
    Vec2 to = mm;
    std::optional<document::AnchorHit> anchor;
    if (drag_->handle.kind == document::DimensionHandleKind::Point) {
        anchor = document::findAnchorNear(*doc_, doc_->activeSheet(), mm, drag_->toleranceMm);
        if (anchor) {
            to = anchor->pos;
        }
    }
    auto next = document::moveDimensionHandle(*doc_, drag_->original, drag_->handle, to);
    if (next) {
        if (anchor) {
            next->points[drag_->handle.index].anchor = anchor->anchor;
        }
        drag_->current = std::move(*next);
    }
    r.draftChanged = true;
    return r;
}

DimensionToolResult DimensionTool::endDrag(Vec2 mm) {
    DimensionToolResult r;
    if (!drag_ || doc_ == nullptr) {
        return r;
    }
    dragTo(mm);
    Drag d = std::move(*drag_);
    drag_.reset();
    r.draftChanged = true;
    if (d.current == d.original) {
        return r;
    }
    r.committed = doc_->execute(
        std::make_unique<document::SetDimensionCommand>(d.id, std::move(d.current), "Edit dimension"));
    if (!r.committed) {
        r.ok = false;
        r.message = "the dimension could not be changed";
    }
    return r;
}

DimensionToolResult DimensionTool::removeSelected() {
    if (doc_ == nullptr) {
        return refused("no document");
    }
    drag_.reset();
    std::vector<document::ObjectId> ids;
    for (const document::ObjectId id : doc_->selection().ids()) {
        const document::Object* obj = doc_->findObject(id);
        if (obj != nullptr && obj->asDimension() != nullptr) {
            ids.push_back(id);
        }
    }
    if (ids.empty()) {
        return refused("select a dimension first");
    }
    DimensionToolResult r;
    r.committed = doc_->execute(std::make_unique<document::RemoveObjectsCommand>(std::move(ids)));
    if (!r.committed) {
        r.ok = false;
        r.message = "the dimension could not be deleted";
    }
    editTarget_.reset();
    r.draftChanged = true;
    return r;
}

}  // namespace carvetto::viewmodel
