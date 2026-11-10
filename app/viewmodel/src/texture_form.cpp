#include "carvetto/viewmodel/texture_form.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <memory>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/units_format.hpp"
#include "carvetto/viewmodel/vector_ops.hpp"

namespace carvetto::viewmodel {

namespace {

constexpr double kDegToRad = geometry::kPi / 180.0;

NumericField field(const char* id, const char* label, FieldKind kind, bool negative, bool zero, std::string text) {
    NumericField f;
    f.id = id;
    f.label = label;
    f.kind = kind;
    f.allowNegative = negative;
    f.allowZero = zero;
    f.text = std::move(text);
    f.typed = true;
    return f;
}

std::string trimmed(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

}  // namespace

TextureForm::TextureForm(document::Units units) : units_(units) { reset(); }

void TextureForm::reset() {
    params_ = geometry::VectorTextureParams{};
    params_.angle = 0.0;
    params_.maxSpacing = 5.0;
    params_.spacingVariation = 0.0;
    params_.wavelength = 10.0;
    params_.amplitude = 1.0;
    params_.noise = 0.0;
    params_.seed = 1;
    layer_.reset();
}

std::vector<NumericField> TextureForm::fields() const {
    const auto len = [&](double mm) { return formatLength(mm, units_); };
    return {
        field("angle", "Angle", FieldKind::Angle, true, true, formatFixed(params_.angle / kDegToRad, 2)),
        field("spacing", "Max spacing", FieldKind::Length, false, false, len(params_.maxSpacing)),
        field("variation", "Spacing variation", FieldKind::Percent, false, true,
              formatFixed(params_.spacingVariation * 100.0, 1)),
        field("wavelength", "Wavelength", FieldKind::Length, false, false, len(params_.wavelength)),
        field("amplitude", "Amplitude", FieldKind::Length, false, true, len(params_.amplitude)),
        field("noise", "Noise", FieldKind::Percent, false, true, formatFixed(params_.noise * 100.0, 1)),
        field("seed", "Seed", FieldKind::Count, false, true, std::to_string(params_.seed)),
    };
}

FieldResult TextureForm::setField(const std::string& id, const std::string& text) {
    FieldResult out;
    const auto fail = [&](std::string message) {
        out.ok = false;
        out.message = std::move(message);
        return out;
    };
    if (id == "seed") {
        const std::string t = trimmed(text);
        if (t.empty()) return fail("enter a whole number");
        if (t.find_first_not_of("0123456789") != std::string::npos || t.size() > 20) {
            return fail("seed must be a whole number of 0 or more");
        }
        std::uint64_t v = 0;
        const auto res = std::from_chars(t.data(), t.data() + t.size(), v);
        if (res.ec != std::errc() || res.ptr != t.data() + t.size()) return fail("seed is too large");
        params_.seed = v;
        return out;
    }
    struct Spec {
        const char* id;
        FieldKind kind;
        double lo;
        double hi;
        bool loExclusive;
        const char* what;
    };
    static const Spec specs[] = {
        {"angle", FieldKind::Angle, -90.0, 90.0, false, "angle must be within -90 and 90 degrees"},
        {"spacing", FieldKind::Length, 0.0, 1e9, true, "spacing must be positive"},
        {"variation", FieldKind::Percent, 0.0, 100.0, false, "spacing variation must be within 0 and 100%"},
        {"wavelength", FieldKind::Length, 0.0, 1e9, true, "wavelength must be positive"},
        {"amplitude", FieldKind::Length, 0.0, 1e9, false, "amplitude must not be negative"},
        {"noise", FieldKind::Percent, 0.0, 100.0, false, "noise must be within 0 and 100%"},
    };
    const Spec* spec = nullptr;
    for (const Spec& s : specs) {
        if (id == s.id) spec = &s;
    }
    if (spec == nullptr) return fail("unknown field");
    const ParsedNumber n = parseNumber(text, spec->kind, units_);
    if (!n.ok) return fail(n.error);
    if (n.value > spec->hi || n.value < spec->lo || (spec->loExclusive && n.value <= spec->lo)) {
        return fail(spec->what);
    }
    if (id == "angle") params_.angle = n.value * kDegToRad;
    else if (id == "spacing") params_.maxSpacing = n.value;
    else if (id == "variation") params_.spacingVariation = n.value / 100.0;
    else if (id == "wavelength") params_.wavelength = n.value;
    else if (id == "amplitude") params_.amplitude = n.value;
    else params_.noise = n.value / 100.0;
    return out;
}

document::LayerId TextureForm::effectiveLayer(const document::Document& doc) const {
    if (layer_.has_value() && doc.findLayer(*layer_) != nullptr) return *layer_;
    return doc.activeLayer();
}

TexturePreview TextureForm::preview(const document::Document& doc) const {
    TexturePreview out;
    const SelectionRegion region = selectedClosedRegion(doc);
    if (!region.ok) {
        out.message = region.message;
        return out;
    }
    geometry::Box2 box = region.contours.front().bounds();
    for (const geometry::Contour& c : region.contours) box.expand(c.bounds());
    out.region = box;
    // Estimated samples (an upper bound from the box diagonal) before doing any work.
    const double diag = std::hypot(box.width(), box.height()) + 2.0 * params_.amplitude;
    const double pitch = params_.maxSpacing * (1.0 - 0.9 * params_.spacingVariation);
    const double step = params_.amplitude > 0.0 ? params_.wavelength / 24.0 : diag;
    const double estimate = (diag / pitch + 2.0) * (diag / step + 2.0);
    if (estimate > kTexturePreviewMaxVertices) {
        out.skipped = true;
        out.message = "Too many lines for a live preview; the texture is still created when you press OK.";
        return out;
    }
    auto r = geometry::generateVectorTextureInRegion(region.contours, params_);
    if (!r.ok()) {
        out.message = r.error();
        return out;
    }
    out.contours = std::move(r.value());
    for (const geometry::Contour& c : out.contours) out.vertexCount += c.spans.size() + 1;
    if (out.contours.empty()) {
        out.message = "No texture line falls inside the selected region.";
        return out;
    }
    out.ok = true;
    return out;
}

TextureResult TextureForm::apply(document::Document& doc) const {
    TextureResult out;
    const SelectionRegion region = selectedClosedRegion(doc);
    if (!region.ok) {
        out.message = region.message;
        return out;
    }
    const document::LayerId layerId = effectiveLayer(doc);
    const document::Layer* layer = doc.findLayer(layerId);
    if (layer == nullptr || layer->locked) {
        out.message = "The target layer is locked.";
        return out;
    }
    auto r = geometry::generateVectorTextureInRegion(region.contours, params_);
    if (!r.ok()) {
        out.message = r.error();
        return out;
    }
    if (r.value().empty()) {
        out.message = "No texture line falls inside the selected region.";
        return out;
    }
    if (!doc.idGenerator().canAllocate(1)) {
        out.message = "The document has no free object ids left.";
        return out;
    }
    doc.undoStack().breakCoalescing();
    auto cmd = std::make_unique<document::AddObjectCommand>(
        document::Object::vector(std::move(r.value()), layerId, doc.activeSheet()));
    const document::AddObjectCommand* raw = cmd.get();
    if (!doc.execute(std::move(cmd))) {
        out.message = "The texture could not be added.";
        return out;
    }
    out.created = raw->id();
    doc.selection().assign({raw->id()});
    out.ok = true;
    return out;
}

}  // namespace carvetto::viewmodel
