#include "carvetto/viewmodel/relief_modelling.hpp"

#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <numbers>
#include <sstream>
#include <utility>

#include "carvetto/document/relief_commands.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/relief/derive.hpp"
#include "carvetto/relief/modify.hpp"
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

std::vector<geometry::Polyline> selectedPolylines(const Document& doc, bool closedOnly) {
    const ResolvedVectors rv = resolveVectors(doc, VectorSelector::fromSelection(doc));
    std::vector<geometry::Polyline> out;
    for (const ContourRef& ref : rv.contours) {
        if (ref.contour == nullptr || (closedOnly && !ref.contour->closed)) {
            continue;
        }
        geometry::Polyline pl = ref.contour->polygonise(geometry::Tolerance::defaults());
        pl.closed = ref.contour->closed;
        if (pl.vertexCount() >= (pl.closed ? 3U : 2U)) {
            out.push_back(std::move(pl));
        }
    }
    return out;
}

std::vector<geometry::Polyline> selectedClosedPolylines(const Document& doc) {
    return selectedPolylines(doc, true);
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

bool parseShapeProfile(const std::string& text, std::vector<relief::ProfilePoint>& out, std::string& error) {
    out.clear();
    std::istringstream in(text);
    std::string item;
    while (std::getline(in, item, ';')) {
        const std::size_t comma = item.find(',');
        relief::ProfilePoint pt;
        if (comma == std::string::npos || !parseFinite(item.substr(0, comma), pt.u) ||
            !parseFinite(item.substr(comma + 1), pt.h)) {
            if (item.find_first_not_of(" \t") == std::string::npos) {
                continue;   // trailing separator
            }
            error = "Profile point \"" + item + "\" is not \"depth,height\"";
            return false;
        }
        if (pt.u < 0.0 || pt.u > 1.0 || (!out.empty() && pt.u <= out.back().u)) {
            error = "Profile depths must rise strictly within 0 to 1";
            return false;
        }
        out.push_back(pt);
    }
    if (out.size() < 2) {
        error = "A profile needs at least two points";
        return false;
    }
    return true;
}

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
    if (form.kind == relief::ShapeKind::Custom && !parseShapeProfile(form.profile, shape.params.profile, error)) {
        return fail(error);
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

// --- Modify forms ---------------------------------------------------------------------------------------

namespace {

using Kind = ModifyField::Kind;

ModifyField num(const char* label, const char* def, const char* tip) {
    return {label, Kind::Number, def, {}, tip};
}
ModifyField chk(const char* label, bool def, const char* tip) {
    return {label, Kind::Check, def ? "true" : "false", {}, tip};
}
ModifyField choice(const char* label, std::vector<std::string> choices, const char* tip) {
    ModifyField f{label, Kind::Choice, choices.front(), std::move(choices), tip};
    return f;
}

const std::vector<ModifyField> kNoFields;

std::string value(const ModifyForm& f, const ModifyField& spec) {
    const auto it = f.values.find(spec.label);
    return it != f.values.end() ? it->second : spec.defaultValue;
}

/// Reads every Number field of the tool, in order; false with the field named.
bool readNumbers(const ModifyForm& form, std::map<std::string, double>& out, std::string& error) {
    for (const ModifyField& spec : modifyFields(form.tool)) {
        if (spec.kind != Kind::Number) {
            continue;
        }
        double v = 0.0;
        if (!number(value(form, spec), spec.label.c_str(), v, error)) {
            return false;
        }
        out[spec.label] = v;
    }
    return true;
}

bool checked(const ModifyForm& form, const char* label) {
    const auto it = form.values.find(label);
    if (it != form.values.end()) {
        return it->second == "true";
    }
    for (const ModifyField& spec : modifyFields(form.tool)) {
        if (spec.label == label) {
            return spec.defaultValue == "true";
        }
    }
    return false;
}

std::string chosen(const ModifyForm& form, const char* label) {
    const auto it = form.values.find(label);
    if (it != form.values.end()) {
        return it->second;
    }
    for (const ModifyField& spec : modifyFields(form.tool)) {
        if (spec.label == label) {
            return spec.defaultValue;
        }
    }
    return {};
}

ModifyResult failModify(std::string message) {
    ModifyResult r;
    r.message = std::move(message);
    return r;
}

/// Bounds of the cell-centre rectangle of a field.
void fieldBounds(const relief::HeightField& f, double& x0, double& y0, double& x1, double& y1) {
    x0 = f.origin().x;
    y0 = f.origin().y;
    x1 = x0 + static_cast<double>(f.width() > 0 ? f.width() - 1 : 0) * f.cellSize();
    y1 = y0 + static_cast<double>(f.height() > 0 ? f.height() - 1 : 0) * f.cellSize();
}

std::string statusText(relief::Status s) {
    return relief::statusMessage(s);
}

/// The model a whole-model tool works on: the baked selection, else the composed visible sheet.
bool sourceModel(const Document& doc, relief::HeightField& out, std::string& error) {
    const std::vector<document::ComponentId> ids = modifyTargets(doc);
    const bool selected = !doc.componentSelection().ids().empty();
    if (ids.empty()) {
        error = "There is no model";
        return false;
    }
    if (selected) {
        std::vector<relief::Component> parts;
        for (const document::ComponentId id : ids) {
            if (const document::ReliefComponent* c = doc.findReliefComponent(id)) {
                parts.push_back(c->data);
            }
        }
        relief::Component baked;
        const relief::Status st = relief::bakeComponents(parts, baked);
        if (st != relief::Status::Ok) {
            error = statusText(st);
            return false;
        }
        out = std::move(baked.field);
        return true;
    }
    const std::vector<relief::Level> levels = doc.reliefModel(doc.activeSheet());
    relief::Grid grid;
    relief::Status st = relief::unionGrid(levels, grid);
    if (st == relief::Status::Ok) {
        st = relief::compose(levels, grid, out);
    }
    if (st != relief::Status::Ok) {
        error = statusText(st);
        return false;
    }
    return true;
}

}  // namespace

const char* modifyToolName(ModifyTool tool) {
    switch (tool) {
        case ModifyTool::ReplaceBelow: return "Replace Below";
        case ModifyTool::ResetByVectors: return "Reset Inside/Outside Vectors";
        case ModifyTool::Offset: return "Offset Model";
        case ModifyTool::Smooth: return "Smooth";
        case ModifyTool::Emboss: return "Emboss";
        case ModifyTool::Slice: return "Slice Model";
        case ModifyTool::Draft: return "Add Draft";
        case ModifyTool::Split: return "Split Component";
        case ModifyTool::VectorBoundary: return "Create Vector Boundary";
        case ModifyTool::CrossSection: return "Create Cross Section Vector";
        case ModifyTool::Distort: return "Distort Component";
    }
    return "";
}

const char* modifyHelp(ModifyTool tool) {
    switch (tool) {
        case ModifyTool::ReplaceBelow:
        case ModifyTool::Offset:
        case ModifyTool::Smooth:
        case ModifyTool::Emboss:
        case ModifyTool::Distort:
            return "Changes the selected components in place, or every component when none is selected.";
        case ModifyTool::ResetByVectors:
            return "Select closed vectors; cells inside (or outside) them become transparent in the selected "
                   "components (all when none is selected).";
        case ModifyTool::Slice:
        case ModifyTool::Draft:
            return "Adds new components made from the selected components baked together, or from the whole "
                   "model when none is selected.";
        case ModifyTool::Split:
            return "Select one vector (closed: inside / outside; open: left / right of its direction). Each "
                   "target becomes part A and a new part B is added.";
        case ModifyTool::VectorBoundary:
            return "Adds the outline of the selected components (the whole model when none) as a vector.";
        case ModifyTool::CrossSection:
            return "Adds the height profile along a line as a vector, placed at the start point "
                   "(distance along the line is X, height is Y).";
    }
    return "";
}

const std::vector<ModifyField>& modifyFields(ModifyTool tool) {
    static const std::vector<ModifyField> replaceBelow = {
        num("Threshold", "0", "Cells strictly below this height are replaced (mm)"),
        choice("Replace with", {"Transparent", "Plane"}, "What replaced cells become"),
        num("Plane height", "0", "Height of the replacement plane (mm)"),
        num("Lower by", "0", "Subtracted from every opaque cell afterwards (mm)"),
    };
    static const std::vector<ModifyField> reset = {
        choice("Region", {"Inside", "Outside"}, "Which side of the selected vectors becomes transparent"),
    };
    static const std::vector<ModifyField> offset = {
        num("Distance", "1", "Positive grows the model, negative shrinks it (mm)"),
        chk("Clip below zero", false, "Heights below 0 become 0 afterwards"),
    };
    static const std::vector<ModifyField> smooth = {
        num("Percent", "50", "0 to 100"),
        num("Radius", "2", "Smoothing radius at 100 % (mm)"),
        chk("Preserve transparency", true, "Transparent cells stay transparent"),
    };
    static const std::vector<ModifyField> emboss = {
        num("Scale", "0.5", "Height scale of the broad shape; detail is kept"),
        num("Detail smoothness %", "50", "0 to 100: how coarse the broad shape is"),
        num("Radius", "2", "Smoothing radius at 100 % (mm)"),
    };
    static const std::vector<ModifyField> slice = {
        num("Thickness", "1", "Slice thickness (mm)"),
        choice("Anchor", {"Top", "Bottom"}, "Bands start at the model top (going down) or bottom"),
        num("Count", "0", "Number of slices; 0 covers the whole model"),
        chk("Boundary vectors", false, "Also add the footprint of each slice as a vector"),
    };
    static const std::vector<ModifyField> draft = {
        num("Minimum draft angle", "5", "Degrees from vertical, 0 to under 90"),
    };
    static const std::vector<ModifyField> cross = {
        num("Start X", "0", "mm"),   num("Start Y", "0", "mm"),
        num("End X", "10", "mm"),    num("End Y", "0", "mm"),
        num("Spacing", "0.5", "Sample spacing along the line (mm)"),
        num("Simplify tolerance", "0.01", "Vertices within this of a straight line are dropped (mm); 0 keeps all"),
    };
    static const std::vector<ModifyField> distort = {
        num("Left-bottom X", "0", "mm"),  num("Left-bottom Y", "0", "mm"),
        num("Right-bottom X", "10", "mm"), num("Right-bottom Y", "0", "mm"),
        num("Right-top X", "10", "mm"),   num("Right-top Y", "10", "mm"),
        num("Left-top X", "0", "mm"),     num("Left-top Y", "10", "mm"),
    };
    switch (tool) {
        case ModifyTool::ReplaceBelow: return replaceBelow;
        case ModifyTool::ResetByVectors: return reset;
        case ModifyTool::Offset: return offset;
        case ModifyTool::Smooth: return smooth;
        case ModifyTool::Emboss: return emboss;
        case ModifyTool::Slice: return slice;
        case ModifyTool::Draft: return draft;
        case ModifyTool::CrossSection: return cross;
        case ModifyTool::Distort: return distort;
        case ModifyTool::Split:
        case ModifyTool::VectorBoundary: break;
    }
    return kNoFields;
}

std::vector<document::ComponentId> modifyTargets(const Document& doc) {
    std::vector<document::ComponentId> ids;
    for (const document::ComponentId id : doc.componentSelection().ids()) {
        if (doc.findReliefComponent(id) != nullptr) {
            ids.push_back(id);
        }
    }
    return ids.empty() ? sheetComponents(doc) : ids;
}

ModifyForm defaultModifyForm(const Document& doc, ModifyTool tool) {
    ModifyForm f;
    f.tool = tool;
    for (const ModifyField& spec : modifyFields(tool)) {
        f.values[spec.label] = spec.defaultValue;
    }
    const std::vector<document::ComponentId> ids = modifyTargets(doc);
    const document::ReliefComponent* first = ids.empty() ? nullptr : doc.findReliefComponent(ids.front());
    if (first != nullptr && (tool == ModifyTool::CrossSection || tool == ModifyTool::Distort)) {
        double x0 = 0.0;
        double y0 = 0.0;
        double x1 = 0.0;
        double y1 = 0.0;
        fieldBounds(first->data.field, x0, y0, x1, y1);
        if (tool == ModifyTool::CrossSection) {
            f.values["Start X"] = fmt(x0);
            f.values["Start Y"] = fmt((y0 + y1) / 2.0);
            f.values["End X"] = fmt(x1);
            f.values["End Y"] = fmt((y0 + y1) / 2.0);
        } else {
            f.values["Left-bottom X"] = fmt(x0);
            f.values["Left-bottom Y"] = fmt(y0);
            f.values["Right-bottom X"] = fmt(x1);
            f.values["Right-bottom Y"] = fmt(y0);
            f.values["Right-top X"] = fmt(x1);
            f.values["Right-top Y"] = fmt(y1);
            f.values["Left-top X"] = fmt(x0);
            f.values["Left-top Y"] = fmt(y1);
        }
    }
    return f;
}

const relief::HeightField* ModifyResult::previewField() const {
    if (!replacements.empty()) {
        return &replacements.front().field;
    }
    if (!additions.empty()) {
        return &additions.front().field;
    }
    return nullptr;
}

ModifyResult previewModify(const Document& doc, const ModifyForm& form) {
    std::map<std::string, double> n;
    std::string error;
    if (!readNumbers(form, n, error)) {
        return failModify(error);
    }
    ModifyResult out;
    const std::vector<document::ComponentId> targets = modifyTargets(doc);
    if (targets.empty()) {
        return failModify("There is no model");
    }
    const auto perComponent = [&](auto&& op) -> bool {
        for (const document::ComponentId id : targets) {
            const document::ReliefComponent* c = doc.findReliefComponent(id);
            if (c == nullptr) {
                continue;
            }
            relief::Component copy = c->data;
            relief::HeightField result;
            const relief::Status st = op(c->data.field, result);
            if (st != relief::Status::Ok) {
                out.message = "\"" + c->data.name + "\": " + statusText(st);
                return false;
            }
            copy.field = std::move(result);
            out.replaceIds.push_back(id);
            out.replacements.push_back(std::move(copy));
        }
        return !out.replacements.empty();
    };
    bool ok = false;
    switch (form.tool) {
        case ModifyTool::ReplaceBelow: {
            relief::ReplaceBelowParams p;
            p.threshold = n["Threshold"];
            p.with = chosen(form, "Replace with") == "Plane" ? relief::ReplaceWith::Plane
                                                              : relief::ReplaceWith::Transparent;
            p.planeHeight = n["Plane height"];
            p.lowerBy = n["Lower by"];
            ok = perComponent([&](const relief::HeightField& in, relief::HeightField& o) {
                return relief::replaceBelow(in, p, o);
            });
            break;
        }
        case ModifyTool::ResetByVectors: {
            const std::vector<geometry::Polyline> vectors = selectedClosedPolylines(doc);
            if (vectors.empty()) {
                return failModify("Select one or more closed vectors first");
            }
            const relief::ResetRegion region =
                chosen(form, "Region") == "Outside" ? relief::ResetRegion::Outside : relief::ResetRegion::Inside;
            ok = perComponent([&](const relief::HeightField& in, relief::HeightField& o) {
                return relief::resetByVectors(in, vectors, region, o);
            });
            break;
        }
        case ModifyTool::Offset: {
            relief::OffsetParams p;
            p.distance = n["Distance"];
            p.clipToZero = checked(form, "Clip below zero");
            ok = perComponent([&](const relief::HeightField& in, relief::HeightField& o) {
                return relief::offsetModel(in, p, o);
            });
            break;
        }
        case ModifyTool::Smooth: {
            relief::SmoothParams p;
            p.percent = n["Percent"];
            p.radiusMm = n["Radius"];
            p.preserveTransparency = checked(form, "Preserve transparency");
            ok = perComponent([&](const relief::HeightField& in, relief::HeightField& o) {
                return relief::smooth(in, p, o);
            });
            break;
        }
        case ModifyTool::Emboss: {
            relief::EmbossParams p;
            p.scale = n["Scale"];
            p.detailSmoothnessPercent = n["Detail smoothness %"];
            p.radiusMm = n["Radius"];
            ok = perComponent([&](const relief::HeightField& in, relief::HeightField& o) {
                return relief::emboss(in, p, o);
            });
            break;
        }
        case ModifyTool::Distort: {
            relief::Quad q;
            q.p00 = {n["Left-bottom X"], n["Left-bottom Y"]};
            q.p10 = {n["Right-bottom X"], n["Right-bottom Y"]};
            q.p11 = {n["Right-top X"], n["Right-top Y"]};
            q.p01 = {n["Left-top X"], n["Left-top Y"]};
            for (const document::ComponentId id : targets) {
                const document::ReliefComponent* c = doc.findReliefComponent(id);
                if (c == nullptr) {
                    continue;
                }
                relief::Component res;
                const relief::DeriveStatus st = relief::distortComponent(c->data, q, nullptr, res);
                if (st != relief::DeriveStatus::Ok) {
                    return failModify("\"" + c->data.name + "\": " + relief::deriveStatusMessage(st));
                }
                out.replaceIds.push_back(id);
                out.replacements.push_back(std::move(res));
            }
            ok = !out.replacements.empty();
            break;
        }
        case ModifyTool::Split: {
            const std::vector<geometry::Polyline> cutters = selectedPolylines(doc, false);
            if (cutters.empty()) {
                return failModify("Select a vector to split with");
            }
            for (const document::ComponentId id : targets) {
                const document::ReliefComponent* c = doc.findReliefComponent(id);
                if (c == nullptr) {
                    continue;
                }
                relief::Component a;
                relief::Component b;
                const relief::DeriveStatus st = relief::splitComponent(c->data, cutters.front(), a, b);
                if (st != relief::DeriveStatus::Ok) {
                    return failModify("\"" + c->data.name + "\": " + relief::deriveStatusMessage(st));
                }
                out.replaceIds.push_back(id);
                out.replacements.push_back(std::move(a));
                out.additions.push_back(std::move(b));
            }
            ok = !out.replacements.empty();
            break;
        }
        case ModifyTool::Slice: {
            relief::HeightField model;
            if (!sourceModel(doc, model, error)) {
                return failModify(error);
            }
            relief::SliceParams p;
            p.thickness = n["Thickness"];
            p.fromTop = chosen(form, "Anchor") != "Bottom";
            if (n["Count"] < 0.0 || n["Count"] > 1.0e6) {
                return failModify("Count is not valid");
            }
            p.count = static_cast<std::size_t>(n["Count"]);
            p.boundaryVectors = checked(form, "Boundary vectors");
            std::vector<relief::Slice> slices;
            const relief::DeriveStatus st = relief::sliceModel(model, p, nullptr, slices);
            if (st != relief::DeriveStatus::Ok) {
                return failModify(relief::deriveStatusMessage(st));
            }
            for (relief::Slice& s : slices) {
                s.component.name = "Slice " + std::to_string(out.additions.size() + 1);
                out.additions.push_back(std::move(s.component));
                for (geometry::Polyline& pl : s.boundary) {
                    out.vectors.push_back(std::move(pl));
                }
            }
            ok = !out.additions.empty();
            break;
        }
        case ModifyTool::Draft: {
            relief::HeightField model;
            if (!sourceModel(doc, model, error)) {
                return failModify(error);
            }
            relief::Component c;
            const relief::DeriveStatus st =
                relief::addDraft(model, n["Minimum draft angle"] * std::numbers::pi / 180.0, nullptr, c);
            if (st != relief::DeriveStatus::Ok) {
                return failModify(relief::deriveStatusMessage(st));
            }
            out.additions.push_back(std::move(c));
            ok = true;
            break;
        }
        case ModifyTool::VectorBoundary: {
            std::vector<const relief::Component*> parts;
            for (const document::ComponentId id : targets) {
                if (const document::ReliefComponent* c = doc.findReliefComponent(id)) {
                    parts.push_back(&c->data);
                }
            }
            const relief::DeriveStatus st = relief::vectorBoundary(parts, out.vectors);
            if (st != relief::DeriveStatus::Ok) {
                return failModify(relief::deriveStatusMessage(st));
            }
            ok = !out.vectors.empty();
            break;
        }
        case ModifyTool::CrossSection: {
            relief::HeightField model;
            if (!sourceModel(doc, model, error)) {
                return failModify(error);
            }
            const relief::Point a{n["Start X"], n["Start Y"]};
            const relief::Point b{n["End X"], n["End Y"]};
            const relief::DeriveStatus st =
                relief::crossSection(model, a, b, n["Spacing"], n["Simplify tolerance"], out.vectors);
            if (st != relief::DeriveStatus::Ok) {
                return failModify(relief::deriveStatusMessage(st));
            }
            for (geometry::Polyline& pl : out.vectors) {   // (distance, height) -> sheet, at the start point
                for (auto& v : pl.points) {
                    v.x += a.x;
                    v.y += a.y;
                }
            }
            ok = !out.vectors.empty();
            break;
        }
    }
    if (!ok) {
        out.replaceIds.clear();
        out.replacements.clear();
        out.additions.clear();
        out.vectors.clear();
        out.message = out.message.empty() ? "The tool produced nothing" : out.message;
        return out;
    }
    std::ostringstream os;
    os << modifyToolName(form.tool) << ": ";
    if (!out.replacements.empty()) {
        os << out.replacements.size() << " component(s) changed";
    }
    if (!out.additions.empty()) {
        os << (out.replacements.empty() ? "" : ", ") << out.additions.size() << " added";
    }
    if (!out.vectors.empty()) {
        os << (out.replacements.empty() && out.additions.empty() ? "" : ", ") << out.vectors.size()
           << " vector contour(s) added";
    }
    out.info = os.str();
    out.ok = true;
    return out;
}

EditResult applyModify(Document& doc, const ModifyForm& form) {
    ModifyResult p = previewModify(doc, form);
    if (!p.ok) {
        return refused(p.message);
    }
    // Added components go above the first target, in its level.
    document::LevelId level;
    std::size_t index = 0;
    if (!p.replaceIds.empty()) {
        if (const auto loc = doc.locateReliefComponent(p.replaceIds.front())) {
            level = loc->level;
            index = loc->index + 1;
        }
    }
    if (level.isNull()) {
        const std::vector<document::LevelId> levels = doc.reliefLevelsOnSheet(doc.activeSheet());
        if (!levels.empty()) {
            level = levels.back();
            index = doc.findReliefLevel(level)->components.size();
        }
    }
    doc.undoStack().breakCoalescing();
    auto macro = std::make_unique<document::MacroCommand>(modifyToolName(form.tool));
    for (std::size_t i = 0; i < p.replaceIds.size(); ++i) {
        macro->add(std::make_unique<document::ReplaceReliefComponentCommand>(p.replaceIds[i],
                                                                              std::move(p.replacements[i])));
    }
    std::vector<const document::AddReliefComponentCommand*> adds;
    for (std::size_t i = 0; i < p.additions.size() && !level.isNull(); ++i) {
        auto add = std::make_unique<document::AddReliefComponentCommand>(level, std::move(p.additions[i]),
                                                                          index + i);
        adds.push_back(add.get());
        macro->add(std::move(add));
    }
    if (!p.additions.empty() && level.isNull()) {
        return refused("There is no level to add to");
    }
    if (!p.vectors.empty()) {
        document::Object obj;
        obj.layer = doc.defaultLayer();
        obj.sheet = doc.activeSheet();
        document::VectorObject vec;
        for (const geometry::Polyline& pl : p.vectors) {
            vec.contours.push_back(geometry::Contour::fromPolyline(pl));
        }
        obj.data = std::move(vec);
        macro->add(std::make_unique<document::AddObjectCommand>(std::move(obj)));
    }
    const bool ok = doc.execute(std::move(macro));
    doc.undoStack().breakCoalescing();
    if (!ok) {
        return refused("The result exceeds the document limits");
    }
    if (!adds.empty()) {
        std::vector<document::ComponentId> created;
        for (const auto* a : adds) {
            created.push_back(a->createdId());
        }
        doc.componentSelection().assign(created);
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
