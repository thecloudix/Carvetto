#include "carvetto/viewmodel/relief_modelling.hpp"

#include <cmath>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <sstream>
#include <utility>

#include "carvetto/document/relief_commands.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/viewmodel/vector_selector.hpp"

namespace carvetto::viewmodel {

namespace {

using document::Document;

bool parseFinite(const std::string& text, double& out) {
    const char* begin = text.c_str();
    while (*begin == ' ' || *begin == '\t') {
        ++begin;
    }
    if (*begin == '\0') {
        return false;
    }
    char* end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end == begin) {
        return false;
    }
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    if (*end != '\0' || !std::isfinite(v) || std::fabs(v) > 1.0e9) {
        return false;
    }
    out = v;
    return true;
}

std::string fmt(double v) {
    std::ostringstream os;
    os.precision(6);
    os << v;
    return os.str();
}

ModellingPreview fail(std::string message) {
    ModellingPreview p;
    p.message = std::move(message);
    return p;
}

EditResult refused(std::string message) {
    EditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

/// Reads `text` into `out`; on failure `error` names the field.
bool number(const std::string& text, const char* label, double& out, std::string& error) {
    if (!parseFinite(text, out)) {
        error = std::string(label) + " is not a valid number";
        return false;
    }
    return true;
}

std::string describeField(const relief::Component& c) {
    std::ostringstream os;
    os << c.field.width() << " x " << c.field.height() << " cells, " << fmt(c.field.cellSize()) << " mm";
    float lo = 0.0F;
    float hi = 0.0F;
    if (c.field.minMax(lo, hi)) {
        os << ", height " << fmt(static_cast<double>(lo)) << " to " << fmt(static_cast<double>(hi)) << " mm";
    }
    return os.str();
}

std::vector<geometry::Polyline> selectedClosedPolylines(const Document& doc) {
    const ResolvedVectors rv = resolveVectors(doc, VectorSelector::fromSelection(doc));
    std::vector<geometry::Polyline> out;
    for (const ContourRef& ref : rv.contours) {
        if (ref.contour == nullptr || !ref.contour->closed) {
            continue;
        }
        geometry::Polyline pl = ref.contour->polygonise(geometry::Tolerance::defaults());
        pl.closed = true;
        if (pl.vertexCount() >= 3) {
            out.push_back(std::move(pl));
        }
    }
    return out;
}

const document::BitmapObject* selectedBitmap(const Document& doc) {
    for (const document::ObjectId id : doc.selection().ids()) {
        if (const document::Object* o = doc.findObject(id)) {
            if (const auto* b = std::get_if<document::BitmapObject>(&o->data)) {
                return b;
            }
        }
    }
    return nullptr;
}

std::vector<document::ComponentId> sheetComponents(const Document& doc) {
    std::vector<document::ComponentId> ids;
    for (const document::LevelId l : doc.reliefLevelsOnSheet(doc.activeSheet())) {
        if (const document::ReliefLevel* level = doc.findReliefLevel(l)) {
            for (const document::ReliefComponent& c : level->components) {
                ids.push_back(c.id);
            }
        }
    }
    return ids;
}

}  // namespace

// --- Create Shape ------------------------------------------------------------------------------------

std::size_t selectedShapeVectorCount(const Document& doc) {
    return selectedClosedPolylines(doc).size();
}

ModellingPreview previewCreateShape(const Document& doc, const CreateShapeForm& form) {
    relief::Shape shape;
    std::string error;
    double angleDeg = 0.0;
    double limit = 0.0;
    double smoothing = 0.0;
    if (!number(form.angleDeg, "Angle", angleDeg, error) || !number(form.smoothing, "Smoothing", smoothing, error) ||
        !number(form.scalePercent, "Scale", shape.params.scalePercent, error) ||
        !number(form.baseHeight, "Base Height", shape.params.baseHeight, error) ||
        !number(form.cellSize, "Cell size", shape.cellSize, error) ||
        (form.limitEnabled && !number(form.limit, "Limit", limit, error))) {
        return fail(error);
    }
    shape.params.kind = form.kind;
    shape.params.angleRad = angleDeg * std::numbers::pi / 180.0;
    shape.params.limitEnabled = form.limitEnabled;
    shape.params.limit = limit;
    shape.params.smoothing = smoothing;
    shape.params.preserveInternalCorners = form.preserveInternalCorners;
    shape.params.blendToInnerVectors = form.blendToInnerVectors;
    shape.params.mode = form.mode;
    shape.params.name = form.name;
    if (form.kind == relief::ShapeKind::Custom) {
        // The profile editor is not part of this form; the default is a straight ramp.
        shape.params.profile = {{0.0, 0.0}, {1.0, 1.0}};
    }
    shape.vectors = selectedClosedPolylines(doc);
    if (shape.vectors.empty()) {
        return fail("Select one or more closed vectors first");
    }
    ModellingPreview p;
    const relief::ShapeStatus st = relief::buildShape(shape, nullptr, p.component);
    if (st != relief::ShapeStatus::Ok) {
        return fail(relief::shapeStatusMessage(st));
    }
    p.ok = true;
    p.info = describeField(p.component);
    return p;
}

// --- Bitmap -----------------------------------------------------------------------------------------

BitmapReliefForm defaultBitmapForm(const Document& doc) {
    BitmapReliefForm f;
    if (const document::BitmapObject* b = selectedBitmap(doc)) {
        f.widthMm = fmt(b->placementMm.max.x - b->placementMm.min.x);
        f.cornerX = fmt(b->placementMm.min.x);
        f.cornerY = fmt(b->placementMm.min.y);
    }
    return f;
}

ModellingPreview previewBitmapRelief(const Document& doc, const BitmapReliefForm& form,
                                     const document::BitmapDecoder& decoder) {
    relief::BitmapReliefParams params;
    std::string error;
    if (!number(form.widthMm, "Width", params.widthMm, error) ||
        !number(form.zHeightMm, "Z height", params.zHeightMm, error) ||
        !number(form.cornerX, "Corner X", params.corner.x, error) ||
        !number(form.cornerY, "Corner Y", params.corner.y, error)) {
        return fail(error);
    }
    params.name = form.name;
    params.invert = form.invert;
    params.autoScale = form.autoScale;
    params.mode = form.mode;
    geometry::ShapeResult<relief::GrayImage16> image = std::string("no image");
    if (!form.filePath.empty()) {
        image = relief::loadFileGray16(form.filePath, decoder);
    } else if (const document::BitmapObject* b = selectedBitmap(doc)) {
        image = relief::loadBitmapObjectGray16(doc, *b, decoder);
    } else {
        return fail("Select a bitmap on the sheet or choose an image file");
    }
    if (!image.ok()) {
        return fail(image.error());
    }
    ModellingPreview p;
    const relief::Status st = relief::makeBitmapComponent(image.value(), params, p.component);
    if (st != relief::Status::Ok) {
        return fail(relief::statusMessage(st));
    }
    p.ok = true;
    p.info = describeField(p.component);
    return p;
}

// --- Zero plane ---------------------------------------------------------------------------------------

ModellingPreview previewZeroPlane(const Document& doc, const ZeroPlaneForm& form) {
    double cell = 0.0;
    relief::Point corner;
    std::string error;
    if (!number(form.cellSize, "Cell size", cell, error) || !number(form.cornerX, "Corner X", corner.x, error) ||
        !number(form.cornerY, "Corner Y", corner.y, error)) {
        return fail(error);
    }
    ModellingPreview p;
    const relief::Status st =
        relief::makeZeroPlane(doc.job().sizeMm.x, doc.job().sizeMm.y, cell, corner, p.component);
    if (st != relief::Status::Ok) {
        return fail(relief::statusMessage(st));
    }
    p.component.name = form.name;
    p.ok = true;
    p.info = describeField(p.component);
    return p;
}

// --- Scale Z --------------------------------------------------------------------------------------------

ScaleZPreview previewScaleZ(const Document& doc, const ScaleZForm& form) {
    ScaleZPreview out;
    relief::ScaleZ s;
    s.kind = form.kind;
    s.bothSides = form.bothSides;
    if (!parseFinite(form.value, s.value)) {
        out.message = "Value is not a valid number";
        return out;
    }
    std::vector<document::ComponentId> ids = doc.componentSelection().ids();
    if (ids.empty()) {
        ids = sheetComponents(doc);
    }
    if (ids.empty()) {
        out.message = "There is no model to scale";
        return out;
    }
    for (const document::ComponentId id : ids) {
        const document::ReliefComponent* c = doc.findReliefComponent(id);
        if (c == nullptr) {
            continue;
        }
        relief::Component copy = c->data;
        double factor = 1.0;
        const relief::Status st = relief::scaleModelZ(copy.field, s, &factor);
        if (st != relief::Status::Ok) {
            out.message = "\"" + c->data.name + "\": " + relief::statusMessage(st);
            out.ids.clear();
            out.components.clear();
            return out;
        }
        if (out.ids.empty()) {
            out.factor = factor;
        }
        out.ids.push_back(id);
        out.components.push_back(std::move(copy));
    }
    out.ok = !out.ids.empty();
    if (!out.ok) {
        out.message = "There is no model to scale";
    }
    return out;
}

EditResult applyScaleZ(Document& doc, const ScaleZForm& form) {
    ScaleZPreview p = previewScaleZ(doc, form);
    if (!p.ok) {
        return refused(p.message);
    }
    doc.undoStack().breakCoalescing();
    auto macro = std::make_unique<document::MacroCommand>("Scale model Z");
    for (std::size_t i = 0; i < p.ids.size(); ++i) {
        macro->add(std::make_unique<document::ReplaceReliefComponentCommand>(p.ids[i], std::move(p.components[i])));
    }
    const bool ok = doc.execute(std::move(macro));
    doc.undoStack().breakCoalescing();
    if (!ok) {
        return refused("The scaled model exceeds the document limits");
    }
    EditResult r;
    r.changed = true;
    return r;
}

// --- Session ------------------------------------------------------------------------------------------

document::ComponentId ModellingSession::current(const Document& doc) const {
    return doc.findReliefComponent(current_) != nullptr ? current_ : document::ComponentId{};
}

EditResult ModellingSession::apply(Document& doc, relief::Component component, document::LevelId level) {
    doc.undoStack().breakCoalescing();
    const document::ComponentId existing = current(doc);
    if (!existing.isNull()) {
        if (!doc.execute(std::make_unique<document::ReplaceReliefComponentCommand>(existing, std::move(component)))) {
            doc.undoStack().breakCoalescing();
            return refused("Nothing changed, or the component is not storable");
        }
        doc.undoStack().breakCoalescing();
        EditResult r;
        r.changed = true;
        return r;
    }
    doc.undoStack().beginMacro("Add relief component");
    if (level.isNull() || doc.findReliefLevel(level) == nullptr) {
        const std::vector<document::LevelId> levels = doc.reliefLevelsOnSheet(doc.activeSheet());
        if (levels.empty()) {
            auto add = std::make_unique<document::AddReliefLevelCommand>(doc.activeSheet(), "Level 1");
            const document::AddReliefLevelCommand* raw = add.get();
            if (!doc.execute(std::move(add))) {
                doc.undoStack().endMacro();
                return refused("A level could not be added");
            }
            level = raw->createdId();
        } else {
            level = levels.back();
        }
    }
    auto cmd = std::make_unique<document::AddReliefComponentCommand>(level, std::move(component));
    const document::AddReliefComponentCommand* raw = cmd.get();
    const bool ok = doc.execute(std::move(cmd));
    const document::ComponentId created = ok ? raw->createdId() : document::ComponentId{};
    doc.undoStack().endMacro();
    doc.undoStack().breakCoalescing();
    if (!ok) {
        return refused("The component could not be added (a size limit was reached)");
    }
    current_ = created;
    doc.componentSelection().assign({created});
    EditResult r;
    r.changed = true;
    return r;
}

}  // namespace carvetto::viewmodel
