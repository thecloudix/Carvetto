#include "carvetto/viewmodel/transform_form.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "carvetto/viewmodel/units_format.hpp"

namespace carvetto::viewmodel {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxAngleDeg = 36000.0;

std::string trimmed(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

NumericField makeField(const char* id, const char* label, FieldKind kind, bool allowNegative, bool allowZero) {
    NumericField f;
    f.id = id;
    f.label = label;
    f.kind = kind;
    f.allowNegative = allowNegative;
    f.allowZero = allowZero;
    return f;
}

std::string degreesText(double deg) { return formatFixed(deg, 2); }

}  // namespace

TransformForm::TransformForm(TransformFormKind kind, document::Units units) : kind_(kind), units_(units) {}

const TransformForm::Entry* TransformForm::entry(const std::string& id) const {
    for (const auto& [key, e] : entries_) {
        if (key == id) {
            return &e;
        }
    }
    return nullptr;
}

void TransformForm::clear() { entries_.clear(); }

std::vector<NumericField> TransformForm::fields(const document::Document& doc) const {
    std::vector<NumericField> list;
    const std::optional<geometry::Box2> bounds = transformBounds(doc);
    const auto show = [&](NumericField f, const std::optional<std::string>& live) {
        if (const Entry* e = entry(f.id)) {
            f.text = e->text;
            f.typed = true;
        } else if (live.has_value()) {
            f.text = *live;
        }
        list.push_back(std::move(f));
    };
    const auto lengthText = [&](double mm) { return formatLength(mm, units_); };
    switch (kind_) {
        case TransformFormKind::Move: {
            std::optional<std::string> x = std::string("0");
            std::optional<std::string> y = x;
            if (options_.absolute) {
                x = y = std::nullopt;
                if (bounds.has_value()) {
                    const geometry::Vec2 p = anchorPoint(*bounds, options_.anchor);
                    x = lengthText(p.x);
                    y = lengthText(p.y);
                }
            }
            show(makeField("x", options_.absolute ? "X position" : "X offset", FieldKind::Length, true, true), x);
            show(makeField("y", options_.absolute ? "Y position" : "Y offset", FieldKind::Length, true, true), y);
            break;
        }
        case TransformFormKind::Scale: {
            std::optional<std::string> w;
            std::optional<std::string> h;
            if (bounds.has_value()) {
                w = lengthText(bounds->width());
                h = lengthText(bounds->height());
            }
            show(makeField("width", "Width", FieldKind::Length, false, false), w);
            show(makeField("height", "Height", FieldKind::Length, false, false), h);
            break;
        }
        case TransformFormKind::Rotate: {
            std::optional<std::string> angle = std::string("0.00");
            if (options_.absolute) {
                angle = degreesText(selectionOrientation(doc) * 180.0 / kPi);
            }
            show(makeField("angle", "Angle", FieldKind::Angle, true, true), angle);
            if (options_.useCoordinates) {
                std::optional<std::string> px;
                std::optional<std::string> py;
                if (bounds.has_value()) {
                    const geometry::Vec2 p = anchorPoint(*bounds, options_.pivotAnchor);
                    px = lengthText(p.x);
                    py = lengthText(p.y);
                }
                show(makeField("pivotX", "Pivot X", FieldKind::Length, true, true), px);
                show(makeField("pivotY", "Pivot Y", FieldKind::Length, true, true), py);
            }
            break;
        }
        case TransformFormKind::Mirror:
            break;
    }
    return list;
}

FieldResult TransformForm::setField(const std::string& id, const std::string& text) {
    FieldResult out;
    const auto fail = [&](std::string message) {
        out.ok = false;
        out.message = std::move(message);
        return out;
    };
    const std::vector<std::string> known = [&]() -> std::vector<std::string> {
        switch (kind_) {
            case TransformFormKind::Move: return {"x", "y"};
            case TransformFormKind::Scale: return {"width", "height"};
            case TransformFormKind::Rotate: return {"angle", "pivotX", "pivotY"};
            case TransformFormKind::Mirror: return {};
        }
        return {};
    }();
    if (std::find(known.begin(), known.end(), id) == known.end()) {
        return fail("unknown field");
    }
    const std::string t = trimmed(text);
    if (t.empty()) {
        entries_.erase(std::remove_if(entries_.begin(), entries_.end(), [&](const auto& kv) { return kv.first == id; }),
                       entries_.end());
        return out;
    }
    Entry e;
    e.text = t;
    if (kind_ == TransformFormKind::Scale && (t.back() == 'S' || t.back() == 's')) {
        const std::string number = trimmed(t.substr(0, t.size() - 1));
        const bool plain = !number.empty() &&
                           number.find_first_not_of("0123456789+-.eE ") == std::string::npos;
        const ParsedNumber p = plain ? parseNumber(number, FieldKind::Percent, units_) : ParsedNumber{};
        if (!p.ok) {
            return fail(plain ? p.error : "not a number");
        }
        if (!(p.value >= kMinScaleFactor) || p.value > kMaxScaleFactor) {
            return fail("the scale factor must be between 0.000001 and 1000000");
        }
        e.value = p.value;
        e.factor = true;
    } else {
        const FieldKind kind = (id == "angle") ? FieldKind::Angle : FieldKind::Length;
        const ParsedNumber p = parseNumber(t, kind, units_);
        if (!p.ok) {
            return fail(p.error);
        }
        if (kind_ == TransformFormKind::Scale && !(p.value > 0.0)) {
            return fail("the size must be greater than zero");
        }
        if (id == "angle" && std::fabs(p.value) > kMaxAngleDeg) {
            return fail("the angle is too large");
        }
        e.value = p.value;
    }
    for (auto& kv : entries_) {
        if (kv.first == id) {
            kv.second = std::move(e);
            return out;
        }
    }
    entries_.emplace_back(id, std::move(e));
    return out;
}

TransformResult TransformForm::apply(document::Document& doc) const {
    TransformResult refusal;
    switch (kind_) {
        case TransformFormKind::Move: {
            const std::optional<geometry::Box2> bounds = transformBounds(doc);
            MoveOptions mo;
            mo.absolute = options_.absolute;
            mo.anchor = options_.anchor;
            if (mo.absolute && bounds.has_value()) {
                const geometry::Vec2 now = anchorPoint(*bounds, options_.anchor);
                mo.xMm = now.x;
                mo.yMm = now.y;
            }
            if (const Entry* x = entry("x")) {
                mo.xMm = x->value;
            }
            if (const Entry* y = entry("y")) {
                mo.yMm = y->value;
            }
            return moveSelection(doc, mo);
        }
        case TransformFormKind::Scale: {
            const std::optional<geometry::Box2> bounds = transformBounds(doc);
            if (!bounds.has_value()) {
                refusal.message = "nothing to transform: select objects on unlocked, visible layers";
                return refusal;
            }
            const Entry* w = entry("width");
            const Entry* h = entry("height");
            if (w == nullptr && h == nullptr) {
                refusal.message = "enter a size or a scale factor";
                return refusal;
            }
            const auto factorOf = [&](const Entry* e, double extent) -> std::optional<double> {
                if (e == nullptr) {
                    return std::nullopt;
                }
                if (e->factor) {
                    return e->value;
                }
                return extent > 1e-9 ? std::optional<double>(e->value / extent) : std::nullopt;
            };
            std::optional<double> fx = factorOf(w, bounds->width());
            std::optional<double> fy = factorOf(h, bounds->height());
            if ((w != nullptr && !fx.has_value()) || (h != nullptr && !fy.has_value())) {
                refusal.message = "the selection has no size on that axis to scale to a size";
                return refusal;
            }
            if (options_.linkXY) {
                if (fx.has_value()) {
                    fy = fx;
                } else {
                    fx = fy;
                }
            }
            ScaleOptions so;
            so.mode = options_.scaleMode;
            so.anchor = options_.anchor;
            so.scaleX = fx.value_or(1.0);
            so.scaleY = fy.value_or(1.0);
            return scaleSelection(doc, so);
        }
        case TransformFormKind::Rotate: {
            RotateOptions ro;
            ro.absolute = options_.absolute;
            ro.angleRad = 0.0;
            if (const Entry* a = entry("angle")) {
                ro.angleRad = a->value * kPi / 180.0;
            } else if (options_.absolute) {
                ro.angleRad = selectionOrientation(doc);  // unchanged orientation: nothing to do
            }
            ro.pivotAnchor = options_.pivotAnchor;
            if (options_.useCoordinates) {
                const std::optional<geometry::Box2> bounds = transformBounds(doc);
                geometry::Vec2 p = bounds.has_value() ? anchorPoint(*bounds, options_.pivotAnchor) : geometry::Vec2{};
                if (const Entry* px = entry("pivotX")) {
                    p.x = px->value;
                }
                if (const Entry* py = entry("pivotY")) {
                    p.y = py->value;
                }
                ro.pivotMm = p;
            }
            return rotateSelection(doc, ro);
        }
        case TransformFormKind::Mirror: {
            MirrorOptions mo;
            mo.axis = options_.axis;
            mo.createCopy = options_.createCopy;
            mo.useRotatedBounds = options_.useRotatedBounds;
            mo.aboutMaterialCentre = options_.aboutMaterialCentre;
            mo.aboutSelectedLine = options_.aboutSelectedLine;
            return mirrorSelection(doc, mo);
        }
    }
    return refusal;
}

}  // namespace carvetto::viewmodel
