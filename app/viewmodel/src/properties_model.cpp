#include "carvetto/viewmodel/properties_model.hpp"

#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/object_property_commands.hpp"
#include "carvetto/document/text_commands.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/transform_ops.hpp"
#include "carvetto/viewmodel/units_format.hpp"

namespace carvetto::viewmodel {

namespace {

using document::CreationParams;
using document::Document;
using document::Object;
using document::ObjectId;

constexpr double kRadToDeg = 180.0 / std::numbers::pi;
constexpr double kDegToRad = std::numbers::pi / 180.0;

const std::vector<std::string>& cornerChoices() {
    static const std::vector<std::string> v{"Square", "Rounded", "Chamfered"};
    return v;
}
const std::vector<std::string>& alignChoices() {
    static const std::vector<std::string> v{"Left", "Centre", "Right"};
    return v;
}
const std::vector<std::string>& onOffChoices() {
    static const std::vector<std::string> v{"Off", "On"};
    return v;
}

EditResult refused(std::string message) {
    EditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

EditResult done(bool changed) {
    EditResult r;
    r.changed = changed;
    return r;
}

/// The single selected object, or null.
const Object* singleObject(const Document& doc) {
    const ObjectId id = doc.selection().single();
    return id.isNull() ? nullptr : doc.findObject(id);
}

const document::VectorObject* singleVector(const Document& doc) {
    const Object* obj = singleObject(doc);
    return obj != nullptr ? obj->asVector() : nullptr;
}

std::string angleText(double radians) {
    return formatFixed(radians * kRadToDeg, 2);
}

PropertyField field(std::string id, std::string label, PropertyKind kind, std::string text) {
    PropertyField f;
    f.id = std::move(id);
    f.label = std::move(label);
    f.kind = kind;
    f.text = std::move(text);
    return f;
}

PropertyField lengthField(const std::string& id, const std::string& label, double mm, document::Units units,
                          bool negative = false) {
    PropertyField f = field(id, label, PropertyKind::Length, formatLength(mm, units));
    f.allowNegative = negative;
    f.suffix = std::string(document::unitSuffix(units));
    return f;
}

PropertyField angleField(const std::string& id, const std::string& label, double radians) {
    PropertyField f = field(id, label, PropertyKind::Angle, angleText(radians));
    f.allowNegative = true;
    f.allowZero = true;
    f.suffix = "deg";
    return f;
}

PropertyField countField(const std::string& id, const std::string& label, int value) {
    return field(id, label, PropertyKind::Count, std::to_string(value));
}

PropertyField choiceField(const std::string& id, const std::string& label, const std::vector<std::string>& choices,
                          int current) {
    PropertyField f = field(id, label, PropertyKind::Choice, {});
    f.choices = choices;
    f.choice = current;
    if (current >= 0 && static_cast<std::size_t>(current) < choices.size()) {
        f.text = choices[static_cast<std::size_t>(current)];
    }
    return f;
}

std::string kindTitle(const Object& obj) {
    if (obj.isGroup()) {
        return "Group";
    }
    if (obj.isBitmap()) {
        return "Bitmap";
    }
    const document::VectorObject* v = obj.asVector();
    if (v == nullptr) {
        return "Object";
    }
    struct Title {
        const char* operator()(const document::NoParams&) const { return "Vector"; }
        const char* operator()(const document::RectangleParams&) const { return "Rectangle"; }
        const char* operator()(const document::CircleParams&) const { return "Circle"; }
        const char* operator()(const document::EllipseParams&) const { return "Ellipse"; }
        const char* operator()(const document::PolygonParams&) const { return "Polygon"; }
        const char* operator()(const document::StarParams&) const { return "Star"; }
        const char* operator()(const document::TextParams&) const { return "Text"; }
    };
    return std::visit(Title{}, v->params);
}

/// True when the object is on an existing, unlocked layer.
bool layerOpen(const Document& doc, const Object& obj) {
    const document::Layer* layer = doc.findLayer(obj.layer);
    return layer != nullptr && !layer->locked;
}

bool selectionOpen(const Document& doc) {
    for (const ObjectId id : doc.selection().ids()) {
        const Object* obj = doc.findObject(id);
        if (obj == nullptr || !layerOpen(doc, *obj)) {
            return false;
        }
    }
    return !doc.selection().empty();
}

/// Runs `cmd` as one undo step (a separate step even after an edit of the same thing).
EditResult run(Document& doc, std::unique_ptr<document::Command> cmd, const char* failure) {
    doc.undoStack().breakCoalescing();
    const bool ok = doc.execute(std::move(cmd));
    doc.undoStack().breakCoalescing();
    return ok ? done(true) : refused(failure);
}

/// Parses a number for a field; fills `message` on error.
std::optional<double> number(const std::string& text, FieldKind kind, document::Units units, bool negative,
                             bool zero, std::string* message) {
    const ParsedNumber p = parseNumber(text, kind, units);
    if (!p.ok) {
        *message = p.error.empty() ? "not a number" : p.error;
        return std::nullopt;
    }
    if (!negative && p.value < 0.0) {
        *message = "the value cannot be negative";
        return std::nullopt;
    }
    if (!zero && p.value == 0.0) {
        *message = "the value cannot be zero";
        return std::nullopt;
    }
    return p.value;
}

std::optional<int> choiceIndex(const std::string& text, std::size_t count) {
    if (text.empty() || text.size() > 6) {
        return std::nullopt;
    }
    int v = 0;
    for (const char c : text) {
        if (c < '0' || c > '9') {
            return std::nullopt;
        }
        v = v * 10 + (c - '0');
    }
    return static_cast<std::size_t>(v) < count ? std::optional<int>(v) : std::nullopt;
}

int cornerIndex(document::RectangleParams::CornerStyle s) {
    return static_cast<int>(s);
}

EditResult editParams(Document& doc, const Object& obj, CreationParams params) {
    if (!layerOpen(doc, obj)) {
        return refused("the layer of this object is locked");
    }
    return run(doc, std::make_unique<document::SetShapeParamsCommand>(obj.id, std::move(params), "Edit " + kindTitle(obj)),
               "the shape cannot take that value");
}

EditResult editText(Document& doc, const Object& obj, document::TextParams params) {
    if (!layerOpen(doc, obj)) {
        return refused("the layer of this object is locked");
    }
    if (const std::string why = document::textParamsProblem(params); !why.empty()) {
        return refused(why);
    }
    const document::FontAsset* asset = doc.findFont(params.fontKey);
    if (asset == nullptr) {
        return refused("the font of this text is not embedded in the document");
    }
    text::FontLoad load = text::Font::load(asset->bytes);
    if (!load.ok()) {
        return refused("the font of this text cannot be read");
    }
    const document::TextBuild build = document::buildTextContours(load.font, params);
    if (!build.ok()) {
        return refused(build.message.empty() ? "the text cannot be laid out" : build.message);
    }
    return run(doc, std::make_unique<document::SetTextCommand>(obj.id, std::move(params), build.contours),
               "the text cannot take that value");
}

}  // namespace

PropertyView describeSelection(const Document& doc) {
    PropertyView view;
    const std::size_t count = doc.selection().size();
    if (count == 0) {
        view.title = "No selection";
        return view;
    }
    const document::Units units = doc.job().displayUnits;
    const Object* one = singleObject(doc);
    view.title = count == 1 ? (one != nullptr ? kindTitle(*one) : std::string("Object"))
                            : std::to_string(count) + " objects";
    const bool open = selectionOpen(doc);

    if (one != nullptr) {
        PropertyField name = field("name", "Name", PropertyKind::Text, one->name);
        name.editable = open;
        view.fields.push_back(std::move(name));
    }

    // Layer: the layer names; -1 when the selection spans several.
    {
        std::vector<std::string> names;
        for (const document::Layer& l : doc.layers()) {
            names.push_back(l.name);
        }
        int current = -1;
        bool mixed = false;
        for (const ObjectId id : doc.selection().ids()) {
            const Object* obj = doc.findObject(id);
            const auto idx = obj != nullptr ? doc.layerIndex(obj->layer) : std::nullopt;
            const int i = idx ? static_cast<int>(*idx) : -1;
            if (current == -1 && !mixed) {
                current = i;
            } else if (i != current) {
                mixed = true;
                current = -1;
            }
        }
        PropertyField layer = choiceField("layer", "Layer", names, current);
        layer.editable = open;
        view.fields.push_back(std::move(layer));
    }

    const std::optional<geometry::Box2> bounds = transformBounds(doc);
    const bool targets = bounds.has_value() && !bounds->isEmpty();
    const document::VectorObject* vec = singleVector(doc);
    const CreationParams* params = vec != nullptr ? &vec->params : nullptr;
    const bool isCircle = params != nullptr && std::holds_alternative<document::CircleParams>(*params);

    if (targets) {
        view.fields.push_back(lengthField("x", "X", bounds->min.x, units, true));
        view.fields.push_back(lengthField("y", "Y", bounds->min.y, units, true));
    }

    // Per-kind parameters replace the generic size/rotation fields.
    bool sized = false;
    bool rotated = false;
    if (params != nullptr && open) {
        if (const auto* rect = std::get_if<document::RectangleParams>(params)) {
            view.fields.push_back(lengthField("width", "Width", rect->widthMm, units));
            view.fields.push_back(lengthField("height", "Height", rect->heightMm, units));
            view.fields.push_back(choiceField("corners", "Corners", cornerChoices(), cornerIndex(rect->corners)));
            PropertyField r = lengthField("cornerRadius", "Corner size", rect->cornerRadiusMm, units);
            r.allowZero = true;
            view.fields.push_back(std::move(r));
            sized = true;
        } else if (const auto* circ = std::get_if<document::CircleParams>(params)) {
            view.fields.push_back(lengthField("diameter", "Diameter", 2.0 * circ->radiusMm, units));
            sized = true;
        } else if (const auto* ell = std::get_if<document::EllipseParams>(params)) {
            view.fields.push_back(lengthField("width", "Width", ell->widthMm, units));
            view.fields.push_back(lengthField("height", "Height", ell->heightMm, units));
            view.fields.push_back(angleField("rotation", "Rotation", ell->angle));
            sized = true;
            rotated = true;
        } else if (const auto* poly = std::get_if<document::PolygonParams>(params)) {
            view.fields.push_back(countField("sides", "Sides", poly->sides));
            view.fields.push_back(lengthField("radius", "Radius", poly->radiusMm, units));
            view.fields.push_back(angleField("rotation", "Rotation", poly->angle));
            sized = true;
            rotated = true;
        } else if (const auto* star = std::get_if<document::StarParams>(params)) {
            view.fields.push_back(countField("points", "Points", star->points));
            view.fields.push_back(lengthField("outerRadius", "Outer radius", star->outerRadiusMm, units));
            view.fields.push_back(lengthField("innerRadius", "Inner radius", star->innerRadiusMm, units));
            view.fields.push_back(angleField("rotation", "Rotation", star->angle));
            sized = true;
            rotated = true;
        } else if (const auto* txt = std::get_if<document::TextParams>(params)) {
            PropertyField t = field("text", "Text", PropertyKind::Text, txt->text);
            t.multiline = true;
            view.fields.push_back(std::move(t));
            view.fields.push_back(lengthField("size", "Size", txt->sizeMm, units));
            view.fields.push_back(choiceField("align", "Align", alignChoices(), static_cast<int>(txt->align)));
            PropertyField ls = lengthField("letterSpacing", "Letter spacing", txt->letterSpacingMm, units, true);
            ls.allowZero = true;
            view.fields.push_back(std::move(ls));
            PropertyField lsp = field("lineSpacing", "Line spacing", PropertyKind::Percent,
                                      formatFixed(txt->lineSpacing * 100.0, 1));
            lsp.suffix = "%";
            view.fields.push_back(std::move(lsp));
            view.fields.push_back(choiceField("kerning", "Kerning", onOffChoices(), txt->kerning ? 1 : 0));
        }
    }
    if (targets && !sized && !(params != nullptr && std::holds_alternative<document::TextParams>(*params))) {
        view.fields.push_back(lengthField("width", "Width", bounds->width(), units));
        view.fields.push_back(lengthField("height", "Height", bounds->height(), units));
    }
    if (targets && !rotated && !isCircle) {
        view.fields.push_back(angleField("rotation", "Rotation", selectionOrientation(doc)));
    }
    return view;
}

EditResult setProperty(Document& doc, const std::string& id, const std::string& text) {
    if (doc.selection().empty()) {
        return refused("nothing is selected");
    }
    const document::Units units = doc.job().displayUnits;
    const Object* one = singleObject(doc);
    std::string why;

    if (id == "name") {
        if (one == nullptr) {
            return refused("select a single object to rename it");
        }
        if (!layerOpen(doc, *one)) {
            return refused("the layer of this object is locked");
        }
        if (const std::string problem = document::nameProblem(text, false); !problem.empty()) {
            return refused(problem);
        }
        if (one->name == text) {
            return done(false);
        }
        return run(doc, std::make_unique<document::RenameObjectCommand>(one->id, text), "the name cannot be set");
    }

    if (id == "layer") {
        const auto index = choiceIndex(text, doc.layers().size());
        if (!index) {
            return refused("pick one of the layers");
        }
        const document::Layer& target = doc.layers()[static_cast<std::size_t>(*index)];
        if (!selectionOpen(doc)) {
            return refused("an object is on a locked layer");
        }
        if (target.locked) {
            return refused("the layer \"" + target.name + "\" is locked");
        }
        std::vector<ObjectId> ids;
        bool already = true;
        for (const ObjectId oid : doc.selection().ids()) {
            ids.push_back(oid);
            const Object* obj = doc.findObject(oid);
            if (obj != nullptr && obj->layer != target.id) {
                already = false;
            }
        }
        if (already) {
            return done(false);
        }
        return run(doc, std::make_unique<document::MoveObjectsToLayerCommand>(std::move(ids), target.id),
                   "the objects cannot be moved to that layer");
    }

    // Per-kind parameters of a single shape.
    const document::VectorObject* vec = singleVector(doc);
    if (one != nullptr && vec != nullptr) {
        if (const auto* rect = std::get_if<document::RectangleParams>(&vec->params)) {
            document::RectangleParams n = *rect;
            if (id == "width" || id == "height" || id == "cornerRadius") {
                const bool zero = id == "cornerRadius";
                const auto v = number(text, FieldKind::Length, units, false, zero, &why);
                if (!v) {
                    return refused(why);
                }
                (id == "width" ? n.widthMm : id == "height" ? n.heightMm : n.cornerRadiusMm) = *v;
                return editParams(doc, *one, n);
            }
            if (id == "corners") {
                const auto i = choiceIndex(text, cornerChoices().size());
                if (!i) {
                    return refused("pick one of the corner styles");
                }
                n.corners = static_cast<document::RectangleParams::CornerStyle>(*i);
                return editParams(doc, *one, n);
            }
        } else if (const auto* circ = std::get_if<document::CircleParams>(&vec->params)) {
            if (id == "diameter") {
                const auto v = number(text, FieldKind::Length, units, false, false, &why);
                if (!v) {
                    return refused(why);
                }
                document::CircleParams n = *circ;
                n.radiusMm = *v * 0.5;
                return editParams(doc, *one, n);
            }
        } else if (const auto* ell = std::get_if<document::EllipseParams>(&vec->params)) {
            document::EllipseParams n = *ell;
            if (id == "width" || id == "height") {
                const auto v = number(text, FieldKind::Length, units, false, false, &why);
                if (!v) {
                    return refused(why);
                }
                (id == "width" ? n.widthMm : n.heightMm) = *v;
                return editParams(doc, *one, n);
            }
            if (id == "rotation") {
                const auto v = number(text, FieldKind::Angle, units, true, true, &why);
                if (!v) {
                    return refused(why);
                }
                n.angle = *v * kDegToRad;
                return editParams(doc, *one, n);
            }
        } else if (const auto* poly = std::get_if<document::PolygonParams>(&vec->params)) {
            document::PolygonParams n = *poly;
            if (id == "sides") {
                const auto v = number(text, FieldKind::Count, units, false, false, &why);
                if (!v) {
                    return refused(why);
                }
                n.sides = static_cast<int>(*v);
                return editParams(doc, *one, n);
            }
            if (id == "radius") {
                const auto v = number(text, FieldKind::Length, units, false, false, &why);
                if (!v) {
                    return refused(why);
                }
                n.radiusMm = *v;
                return editParams(doc, *one, n);
            }
            if (id == "rotation") {
                const auto v = number(text, FieldKind::Angle, units, true, true, &why);
                if (!v) {
                    return refused(why);
                }
                n.angle = *v * kDegToRad;
                return editParams(doc, *one, n);
            }
        } else if (const auto* star = std::get_if<document::StarParams>(&vec->params)) {
            document::StarParams n = *star;
            if (id == "points") {
                const auto v = number(text, FieldKind::Count, units, false, false, &why);
                if (!v) {
                    return refused(why);
                }
                n.points = static_cast<int>(*v);
                return editParams(doc, *one, n);
            }
            if (id == "outerRadius" || id == "innerRadius") {
                const auto v = number(text, FieldKind::Length, units, false, false, &why);
                if (!v) {
                    return refused(why);
                }
                (id == "outerRadius" ? n.outerRadiusMm : n.innerRadiusMm) = *v;
                return editParams(doc, *one, n);
            }
            if (id == "rotation") {
                const auto v = number(text, FieldKind::Angle, units, true, true, &why);
                if (!v) {
                    return refused(why);
                }
                n.angle = *v * kDegToRad;
                return editParams(doc, *one, n);
            }
        } else if (const auto* txt = std::get_if<document::TextParams>(&vec->params)) {
            document::TextParams n = *txt;
            if (id == "text") {
                if (text.size() > document::kMaxTextObjectBytes) {
                    return refused("the text is too long");
                }
                n.text = text;
                return editText(doc, *one, std::move(n));
            }
            if (id == "size") {
                const auto v = number(text, FieldKind::Length, units, false, false, &why);
                if (!v) {
                    return refused(why);
                }
                n.sizeMm = *v;
                return editText(doc, *one, std::move(n));
            }
            if (id == "letterSpacing") {
                const auto v = number(text, FieldKind::Length, units, true, true, &why);
                if (!v) {
                    return refused(why);
                }
                n.letterSpacingMm = *v;
                return editText(doc, *one, std::move(n));
            }
            if (id == "lineSpacing") {
                const auto v = number(text, FieldKind::Percent, units, false, false, &why);
                if (!v) {
                    return refused(why);
                }
                n.lineSpacing = *v / 100.0;
                return editText(doc, *one, std::move(n));
            }
            if (id == "align") {
                const auto i = choiceIndex(text, alignChoices().size());
                if (!i) {
                    return refused("pick one of the alignments");
                }
                n.align = static_cast<document::TextAlign>(*i);
                return editText(doc, *one, std::move(n));
            }
            if (id == "kerning") {
                const auto i = choiceIndex(text, onOffChoices().size());
                if (!i) {
                    return refused("pick Off or On");
                }
                n.kerning = *i == 1;
                return editText(doc, *one, std::move(n));
            }
        }
    }

    // Generic geometry: position, size and rotation of the selection.
    const std::optional<geometry::Box2> bounds = transformBounds(doc);
    if (!bounds || bounds->isEmpty()) {
        return refused("nothing in the selection can be edited (hidden, locked or on another sheet)");
    }
    auto report = [](const TransformResult& r) {
        if (!r.ok) {
            return refused(r.message.empty() ? "the edit was refused" : r.message);
        }
        return done(r.objects > 0);
    };
    doc.undoStack().breakCoalescing();
    struct Break {
        Document& d;
        ~Break() { d.undoStack().breakCoalescing(); }
    } breaker{doc};

    if (id == "x" || id == "y") {
        const auto v = number(text, FieldKind::Length, units, true, true, &why);
        if (!v) {
            return refused(why);
        }
        MoveOptions o;
        o.absolute = true;
        o.anchor = HandleKind::BottomLeft;
        o.xMm = id == "x" ? *v : bounds->min.x;
        o.yMm = id == "y" ? *v : bounds->min.y;
        return report(moveSelection(doc, o));
    }
    if (id == "width" || id == "height") {
        const auto v = number(text, FieldKind::Length, units, false, false, &why);
        if (!v) {
            return refused(why);
        }
        const auto f = scaleFactorsForSize(*bounds, id == "width" ? std::optional<double>(*v) : std::nullopt,
                                           id == "height" ? std::optional<double>(*v) : std::nullopt, false);
        if (!f) {
            return refused("the selection cannot be scaled to that size");
        }
        ScaleOptions o;
        o.mode = ScaleMode::Selection;
        o.anchor = HandleKind::BottomLeft;
        o.scaleX = f->x;
        o.scaleY = f->y;
        return report(scaleSelection(doc, o));
    }
    if (id == "rotation") {
        const auto v = number(text, FieldKind::Angle, units, true, true, &why);
        if (!v) {
            return refused(why);
        }
        RotateOptions o;
        o.absolute = true;
        o.angleRad = *v * kDegToRad;
        return report(rotateSelection(doc, o));
    }
    return refused("there is no such property for this selection");
}

}  // namespace carvetto::viewmodel
