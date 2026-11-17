#include "carvetto/viewmodel/rotary_layout_form.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/viewmodel/units_format.hpp"

namespace carvetto::viewmodel {

namespace {

constexpr double kDegPerTurn = 360.0;

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

std::string rotaryProblem(const document::Document& doc) {
    if (doc.job().type != document::JobType::Rotary) {
        return "This layout wraps round a cylinder: the job must be a rotary job.";
    }
    return {};
}

}  // namespace

RotaryLayoutForm::RotaryLayoutForm(RotaryLayoutKind kind, document::Units units) : kind_(kind), units_(units) {
    reset();
}

void RotaryLayoutForm::reset() {
    count_ = kind_ == RotaryLayoutKind::Fluting ? 4 : 1;
    offset_ = 0.0;
    width_ = 4.0;
    depth_ = 2.0;
    shape_ = 0;
    mode_ = 0;
    pitch_ = 25.0;
    spacing_ = 25.0;
    twistDeg_ = kDegPerTurn;
    direction_ = 0;
    layer_.reset();
}

void RotaryLayoutForm::startFrom(const document::Document& doc) {
    if (doc.job().type != document::JobType::Rotary) {
        return;
    }
    alongStart_ = 0.0;
    alongEnd_ = doc.job().lengthMm();
}

std::vector<NumericField> RotaryLayoutForm::fields() const {
    const auto len = [&](double mm) { return formatLength(mm, units_); };
    std::vector<NumericField> out;
    if (kind_ == RotaryLayoutKind::Fluting) {
        out.push_back(field("flutes", "Number of flutes", FieldKind::Count, false, false, std::to_string(count_)));
    } else {
        out.push_back(field("strands", "Number of strands", FieldKind::Count, false, false, std::to_string(count_)));
    }
    out.push_back(field("alongStart", "Start along the axis", FieldKind::Length, true, true, len(alongStart_)));
    out.push_back(field("alongEnd", "End along the axis", FieldKind::Length, true, true, len(alongEnd_)));
    out.push_back(field("offset", "Offset round the circumference", FieldKind::Length, true, true, len(offset_)));
    if (kind_ == RotaryLayoutKind::Fluting) {
        if (shape_ != 0) {
            out.push_back(field("width", "Flute width", FieldKind::Length, false, false, len(width_)));
        }
        out.push_back(field("depth", "Flute depth", FieldKind::Length, false, false, len(depth_)));
    } else if (mode_ == 0) {
        out.push_back(field("pitch", "Spiral pitch (along the axis per turn)", FieldKind::Length, false, false, len(pitch_)));
    } else if (mode_ == 1) {
        out.push_back(field("spacing", "Spacing between strands", FieldKind::Length, false, false, len(spacing_)));
    } else {
        out.push_back(field("twist", "Total twist (degrees)", FieldKind::Angle, false, false, formatFixed(twistDeg_, 2)));
    }
    return out;
}

std::vector<ChoiceField> RotaryLayoutForm::choices() const {
    std::vector<ChoiceField> out;
    if (kind_ == RotaryLayoutKind::Fluting) {
        out.push_back({"shape", "Flute drawn as", {"Centre line", "Rectangle", "Rounded (cove) ends"}, shape_});
    } else {
        out.push_back({"mode", "Advance given as", {"Spiral pitch", "Spacing between strands", "Total twist"}, mode_});
        out.push_back({"direction", "Twist direction", {"Right hand", "Left hand"}, direction_});
    }
    return out;
}

FieldResult RotaryLayoutForm::setChoice(const std::string& id, int option) {
    FieldResult out;
    const auto fail = [&](std::string message) {
        out.ok = false;
        out.message = std::move(message);
        return out;
    };
    if (kind_ == RotaryLayoutKind::Fluting && id == "shape") {
        if (option < 0 || option > 2) {
            return fail("not a flute shape");
        }
        shape_ = option;
    } else if (kind_ == RotaryLayoutKind::Spiral && id == "mode") {
        if (option < 0 || option > 2) {
            return fail("not a way to give the advance");
        }
        mode_ = option;
    } else if (kind_ == RotaryLayoutKind::Spiral && id == "direction") {
        if (option < 0 || option > 1) {
            return fail("not a twist direction");
        }
        direction_ = option;
    } else {
        return fail("unknown choice");
    }
    return out;
}

FieldResult RotaryLayoutForm::setField(const std::string& id, const std::string& text) {
    FieldResult out;
    const auto fail = [&](std::string message) {
        out.ok = false;
        out.message = std::move(message);
        return out;
    };
    const bool fluting = kind_ == RotaryLayoutKind::Fluting;
    struct Spec {
        FieldKind kind;
        double lo;
        double hi;
        bool loExclusive;
        const char* what;
    };
    Spec spec{FieldKind::Length, -1e5, 1e5, false, "the value is out of range"};
    if (id == (fluting ? "flutes" : "strands")) {
        spec = {FieldKind::Count, 1.0, static_cast<double>(geometry::kMaxLayoutStrands), false,
                "the number must be between 1 and 1000"};
    } else if (id == "alongStart" || id == "alongEnd" || id == "offset") {
        spec = {FieldKind::Length, -1e5, 1e5, false, "the position must be within 100 m"};
    } else if (fluting && id == "width" && shape_ != 0) {
        spec = {FieldKind::Length, 0.0, 1e5, true, "the width must be positive"};
    } else if (fluting && id == "depth") {
        spec = {FieldKind::Length, 0.0, 1e5, true, "the depth must be positive"};
    } else if (!fluting && ((id == "pitch" && mode_ == 0) || (id == "spacing" && mode_ == 1))) {
        spec = {FieldKind::Length, 0.0, 1e5, true, "the value must be positive"};
    } else if (!fluting && id == "twist" && mode_ == 2) {
        spec = {FieldKind::Angle, 0.0, 3.6e6, true, "the twist must be positive"};
    } else {
        return fail("unknown field");
    }
    const ParsedNumber n = parseNumber(text, spec.kind, units_);
    if (!n.ok) {
        return fail(n.error);
    }
    if (n.value > spec.hi || n.value < spec.lo || (spec.loExclusive && n.value <= spec.lo)) {
        return fail(spec.what);
    }
    if (id == "flutes" || id == "strands") {
        count_ = static_cast<int>(n.value);
    } else if (id == "alongStart") {
        alongStart_ = n.value;
    } else if (id == "alongEnd") {
        alongEnd_ = n.value;
    } else if (id == "offset") {
        offset_ = n.value;
    } else if (id == "width") {
        width_ = n.value;
    } else if (id == "depth") {
        depth_ = n.value;
    } else if (id == "pitch") {
        pitch_ = n.value;
    } else if (id == "spacing") {
        spacing_ = n.value;
    } else {
        twistDeg_ = n.value;
    }
    return out;
}

document::LayerId RotaryLayoutForm::effectiveLayer(const document::Document& doc) const {
    if (layer_.has_value() && doc.findLayer(*layer_) != nullptr) {
        return *layer_;
    }
    return doc.activeLayer();
}

geometry::WrappedFlutingParams RotaryLayoutForm::flutingParams(const document::Document& doc) const {
    geometry::WrappedFlutingParams p;
    p.alongX = doc.job().rotaryAxis == document::RotaryAxis::X;
    p.diameter = doc.job().diameterMm;
    p.flutes = count_;
    p.alongStart = alongStart_;
    p.alongEnd = alongEnd_;
    p.circStart = 0.0;
    p.offset = offset_;
    p.shape = shape_ == 0   ? geometry::FluteShape::CentreLine
              : shape_ == 1 ? geometry::FluteShape::Rectangle
                            : geometry::FluteShape::Stadium;
    p.width = width_;
    p.depth = depth_;
    return p;
}

geometry::WrappedSpiralParams RotaryLayoutForm::spiralParams(const document::Document& doc) const {
    geometry::WrappedSpiralParams p;
    p.alongX = doc.job().rotaryAxis == document::RotaryAxis::X;
    p.diameter = doc.job().diameterMm;
    p.flip = doc.job().flipDesign;
    p.strands = count_;
    p.alongStart = alongStart_;
    p.alongEnd = alongEnd_;
    p.circStart = 0.0;
    p.offset = offset_;
    p.mode = mode_ == 0   ? geometry::SpiralPitchMode::Pitch
             : mode_ == 1 ? geometry::SpiralPitchMode::Spacing
                          : geometry::SpiralPitchMode::Turns;
    p.value = mode_ == 0 ? pitch_ : mode_ == 1 ? spacing_ : twistDeg_ / kDegPerTurn;
    p.twist = direction_ == 0 ? geometry::SpiralTwist::RightHand : geometry::SpiralTwist::LeftHand;
    return p;
}

RotaryLayoutPreview RotaryLayoutForm::preview(const document::Document& doc) const {
    RotaryLayoutPreview out;
    if (const std::string bad = rotaryProblem(doc); !bad.empty()) {
        out.message = bad;
        return out;
    }
    if (kind_ == RotaryLayoutKind::Fluting) {
        geometry::WrappedFlutingResult r = geometry::wrappedFlutingLayout(flutingParams(doc));
        if (!r.ok()) {
            out.message = r.error;
            return out;
        }
        out.contours = std::move(r.contours);
        out.pitch = r.spacing;
        for (const int i : r.crossingSeam) {
            out.warnings.push_back("flute " + std::to_string(i + 1) +
                                   " crosses the edge of the turn: change the offset so it stays inside the design area");
        }
    } else {
        geometry::WrappedSpiralResult r = geometry::wrappedSpiralLayout(spiralParams(doc));
        if (!r.ok()) {
            out.message = r.error;
            return out;
        }
        out.contours = std::move(r.contours);
        out.pitch = r.pitch;
        out.turns = r.turns;
    }
    out.ok = true;
    return out;
}

RotaryLayoutResult RotaryLayoutForm::apply(document::Document& doc) const {
    RotaryLayoutResult out;
    RotaryLayoutPreview made = preview(doc);
    if (!made.ok) {
        out.message = made.message;
        return out;
    }
    const document::LayerId layerId = effectiveLayer(doc);
    const document::Layer* layer = doc.findLayer(layerId);
    if (layer == nullptr || layer->locked) {
        out.message = "The target layer is locked.";
        return out;
    }
    if (!doc.idGenerator().canAllocate(1)) {
        out.message = "The document has no free object ids left.";
        return out;
    }
    out.vectors = made.contours.size();
    doc.undoStack().breakCoalescing();
    auto cmd = std::make_unique<document::AddObjectCommand>(
        document::Object::vector(std::move(made.contours), layerId, doc.activeSheet()));
    const document::AddObjectCommand* raw = cmd.get();
    if (!doc.execute(std::move(cmd))) {
        out.message = "The layout could not be added.";
        out.vectors = 0;
        return out;
    }
    out.created = raw->id();
    doc.selection().assign({raw->id()});
    out.warnings = std::move(made.warnings);
    out.ok = true;
    return out;
}

}  // namespace carvetto::viewmodel
