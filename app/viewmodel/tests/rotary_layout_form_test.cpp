// T-0322: the Wrapped Fluting and Spiral Layout forms: fields, choices, refusals, preview, and apply as one undo
// step on a rotary job; a flat job is refused.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <variant>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/rotary_layout_form.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using Catch::Approx;
using document::Document;
using document::Job;
using document::RotaryAxis;
using document::Units;

namespace {
const NumericField* find(const std::vector<NumericField>& fields, const std::string& id) {
    for (const NumericField& f : fields) {
        if (f.id == id) {
            return &f;
        }
    }
    return nullptr;
}
constexpr double kPi = geometry::kPi;
}  // namespace

TEST_CASE("fluting layout form: fields, choices and refusals", "[rotary][layout][form]") {
    Document doc(Job::rotary(120.0, 40.0, Units::Millimetres));
    RotaryLayoutForm form(RotaryLayoutKind::Fluting, Units::Millimetres);
    form.startFrom(doc);
    CHECK(find(form.fields(), "flutes") != nullptr);
    CHECK(find(form.fields(), "width") == nullptr);  // centre lines have no width
    CHECK(std::stod(find(form.fields(), "alongEnd")->text) == Approx(120.0));
    REQUIRE(form.setChoice("shape", 2).ok);
    CHECK(find(form.fields(), "width") != nullptr);
    CHECK_FALSE(form.setChoice("shape", 3).ok);
    CHECK_FALSE(form.setChoice("mode", 0).ok);  // a spiral choice
    CHECK_FALSE(form.setField("pitch", "5").ok);
    REQUIRE(form.setField("flutes", "6").ok);
    REQUIRE(form.setField("width", "5").ok);
    REQUIRE(form.setField("offset", "-2").ok);
    CHECK_FALSE(form.setField("flutes", "0").ok);
    CHECK_FALSE(form.setField("flutes", "1001").ok);
    CHECK_FALSE(form.setField("flutes", "2.5").ok);
    CHECK_FALSE(form.setField("width", "0").ok);
    CHECK_FALSE(form.setField("depth", "-1").ok);
    CHECK_FALSE(form.setField("alongEnd", "abc").ok);
    // A refused entry changes nothing.
    CHECK(find(form.fields(), "flutes")->text == "6");
    const RotaryLayoutPreview pv = form.preview(doc);
    REQUIRE(pv.ok);
    CHECK(pv.contours.size() == 6);
    CHECK(pv.pitch == Approx(40.0 * kPi / 6.0));
    // Inches: lengths are typed in display units.
    RotaryLayoutForm inch(RotaryLayoutKind::Fluting, Units::Inches);
    inch.startFrom(doc);
    REQUIRE(inch.setField("alongEnd", "2").ok);
    CHECK(inch.flutingParams(doc).alongEnd == Approx(50.8));
    REQUIRE(inch.setField("alongEnd", "30mm").ok);
    CHECK(inch.flutingParams(doc).alongEnd == Approx(30.0));
}

TEST_CASE("fluting layout form: apply is one undo step and selects the vectors", "[rotary][layout][form]") {
    Document doc(Job::rotary(120.0, 40.0, Units::Millimetres, RotaryAxis::Y));
    doc.undoStack().clear();
    RotaryLayoutForm form(RotaryLayoutKind::Fluting, Units::Millimetres);
    form.startFrom(doc);
    REQUIRE(form.setField("flutes", "8").ok);
    const std::size_t objects = doc.objects().size();
    const RotaryLayoutResult r = form.apply(doc);
    INFO(r.message);
    REQUIRE(r.ok);
    CHECK(r.vectors == 8);
    REQUIRE(r.created.has_value());
    CHECK(doc.objects().size() == objects + 1);
    CHECK(doc.selection().ids().size() == 1);
    const document::Object* obj = doc.findObject(*r.created);
    REQUIRE(obj != nullptr);
    const auto& contours = std::get<document::VectorObject>(obj->data).contours;
    REQUIRE(contours.size() == 8);
    // Along Y: the flutes run along Y at evenly spaced X.
    const auto& line = std::get<geometry::LineSpan>(contours[3].spans[0]);
    CHECK(line.start.y == 0.0);
    CHECK(line.end.y == 120.0);
    CHECK(line.start.x == Approx(3.0 * 40.0 * kPi / 8.0));
    REQUIRE(doc.undo());
    CHECK(doc.objects().size() == objects);
    REQUIRE(doc.redo());
    CHECK(doc.objects().size() == objects + 1);
}

TEST_CASE("fluting layout form: flutes across the seam are reported, bad input changes nothing", "[rotary][layout][form]") {
    Document doc(Job::rotary(100.0, 40.0, Units::Millimetres));
    RotaryLayoutForm form(RotaryLayoutKind::Fluting, Units::Millimetres);
    form.startFrom(doc);
    REQUIRE(form.setChoice("shape", 1).ok);
    REQUIRE(form.setField("width", "6").ok);
    const RotaryLayoutPreview pv = form.preview(doc);
    REQUIRE(pv.ok);
    REQUIRE(pv.warnings.size() == 1);  // flute 1 sits on the start of the turn
    CHECK(pv.warnings[0].find("flute 1") != std::string::npos);
    REQUIRE(form.setField("width", "100").ok);  // the form accepts it; the layout refuses
    const RotaryLayoutPreview wide = form.preview(doc);
    CHECK_FALSE(wide.ok);
    CHECK_FALSE(wide.message.empty());
    const std::size_t objects = doc.objects().size();
    CHECK_FALSE(form.apply(doc).ok);
    CHECK(doc.objects().size() == objects);
    CHECK_FALSE(doc.undoStack().canUndo());
}

TEST_CASE("spiral layout form: modes, direction and a spiral on the job", "[rotary][layout][form]") {
    Document doc(Job::rotary(100.0, 40.0, Units::Millimetres));
    RotaryLayoutForm form(RotaryLayoutKind::Spiral, Units::Millimetres);
    form.startFrom(doc);
    CHECK(find(form.fields(), "strands") != nullptr);
    CHECK(find(form.fields(), "pitch") != nullptr);
    CHECK(find(form.fields(), "spacing") == nullptr);
    CHECK(form.choices().size() == 2);
    REQUIRE(form.setField("strands", "2").ok);
    REQUIRE(form.setField("pitch", "50").ok);
    RotaryLayoutPreview pv = form.preview(doc);
    REQUIRE(pv.ok);
    CHECK(pv.pitch == 50.0);
    CHECK(pv.turns == Approx(2.0));
    // Spacing: pitch = spacing x strands.
    REQUIRE(form.setChoice("mode", 1).ok);
    CHECK(find(form.fields(), "spacing") != nullptr);
    CHECK(find(form.fields(), "pitch") == nullptr);
    CHECK_FALSE(form.setField("pitch", "5").ok);
    REQUIRE(form.setField("spacing", "10").ok);
    pv = form.preview(doc);
    REQUIRE(pv.ok);
    CHECK(pv.pitch == Approx(20.0));
    // Total twist: 720 degrees over the length is two turns.
    REQUIRE(form.setChoice("mode", 2).ok);
    REQUIRE(form.setField("twist", "720").ok);
    CHECK_FALSE(form.setField("twist", "0").ok);
    pv = form.preview(doc);
    REQUIRE(pv.ok);
    CHECK(pv.turns == Approx(2.0));
    CHECK(pv.pitch == Approx(50.0));
    // Direction changes the geometry.
    const auto first = pv.contours;
    REQUIRE(form.setChoice("direction", 1).ok);
    CHECK_FALSE(form.setChoice("direction", 2).ok);
    CHECK(form.preview(doc).contours != first);
    const geometry::WrappedSpiralParams sp = form.spiralParams(doc);
    CHECK(sp.twist == geometry::SpiralTwist::LeftHand);
    CHECK(sp.mode == geometry::SpiralPitchMode::Turns);
    CHECK(sp.value == Approx(2.0));
    CHECK(sp.strands == 2);
    CHECK(sp.alongX);
    CHECK_FALSE(sp.flip);
    // Flip Design is the job's.
    Document flipped(Job::rotary(100.0, 40.0, Units::Millimetres, RotaryAxis::X, document::ZZero::CylinderAxis, true));
    CHECK(form.spiralParams(flipped).flip);
    const RotaryLayoutResult r = form.apply(doc);
    REQUIRE(r.ok);
    CHECK(r.vectors >= 2);
    REQUIRE(doc.undo());
}

TEST_CASE("layout forms refuse a flat job", "[rotary][layout][form]") {
    Document flat;
    for (const RotaryLayoutKind kind : {RotaryLayoutKind::Fluting, RotaryLayoutKind::Spiral}) {
        RotaryLayoutForm form(kind, Units::Millimetres);
        form.startFrom(flat);  // changes nothing
        const RotaryLayoutPreview pv = form.preview(flat);
        CHECK_FALSE(pv.ok);
        CHECK(pv.message.find("rotary") != std::string::npos);
        CHECK_FALSE(form.apply(flat).ok);
        CHECK(flat.objects().empty());
    }
}

TEST_CASE("layout forms: the target layer, locked layers and reset", "[rotary][layout][form]") {
    Document doc(Job::rotary(100.0, 40.0, Units::Millimetres));
    RotaryLayoutForm form(RotaryLayoutKind::Fluting, Units::Millimetres);
    form.startFrom(doc);
    CHECK(form.effectiveLayer(doc) == doc.activeLayer());
    form.setTargetLayer(document::LayerId{9999});  // gone: the active layer
    CHECK(form.effectiveLayer(doc) == doc.activeLayer());
    REQUIRE(form.setField("flutes", "3").ok);
    form.reset();
    CHECK(find(form.fields(), "flutes")->text == "4");
    CHECK_FALSE(form.targetLayer().has_value());
    RotaryLayoutForm spiral(RotaryLayoutKind::Spiral, Units::Millimetres);
    CHECK(find(spiral.fields(), "strands")->text == "1");
}

TEST_CASE("T-0340: a Fluting toolpath form takes its depth from the selected layout", "[rotary][layout][session]") {
    Document doc(Job::rotary(120.0, 40.0, Units::Millimetres));
    ToolpathSession session(&doc);
    RotaryLayoutForm form(RotaryLayoutKind::Fluting, Units::Millimetres);
    form.startFrom(doc);
    REQUIRE(form.setField("depth", "3.5").ok);
    const RotaryLayoutResult made = form.apply(doc);
    REQUIRE(made.ok);
    REQUIRE(made.created.has_value());
    const auto depthOf = [&]() {
        const ToolpathForm f = session.newForm(document::ToolpathOperation::Fluting);
        return std::get<toolpath::FlutingParams>(f.definition().params).fluteDepth;
    };
    const double dflt = [&]() {
        doc.selection().clear();
        return depthOf();
    }();
    CHECK(dflt != Approx(3.5));
    // Not remembered until the owner records it; then only with the layout selected.
    doc.selection().assign({*made.created});
    CHECK(depthOf() == Approx(dflt));
    session.setLayoutFlutingDepth(*made.created, 3.5);
    CHECK(depthOf() == Approx(3.5));
    doc.selection().clear();
    CHECK(depthOf() == Approx(dflt));
    // Other operations are untouched and a deleted layout is forgotten.
    doc.selection().assign({*made.created});
    CHECK(session.layoutFlutingDepth(doc.selection().ids()).has_value());
    REQUIRE(doc.undo());
    CHECK_FALSE(session.layoutFlutingDepth({*made.created}).has_value());
}
