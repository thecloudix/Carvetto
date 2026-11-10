#include "carvetto/viewmodel/dimension_tool.hpp"

#include <memory>
#include <utility>

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
    r.committed = true;
    r.created = *id;
    r.draftChanged = true;
    return r;
}

document::DimensionGeometry DimensionTool::preview() const {
    document::DimensionGeometry g;
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
    o.layer = obj->layer;
    return o;
}

}  // namespace carvetto::viewmodel
