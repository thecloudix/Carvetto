// Weld / Subtract / Overlap on the selection (T-0247): acceptance scenarios on top of the T-0207
// operations: circles, text counters, exact results on squares, one undo step.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/vector_ops.hpp"

using namespace carvetto;
using Catch::Approx;
using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;
using viewmodel::BooleanKind;

namespace {

ObjectId add(Document& doc, Contour c) {
    return doc.insertObject(Object::vector({std::move(c)}, doc.activeLayer(), doc.activeSheet()));
}

double totalArea(const Document& doc, const std::vector<ObjectId>& ids) {
    double a = 0.0;
    for (ObjectId id : ids) {
        for (const Contour& c : doc.findObject(id)->asVector()->contours) {
            a += geometry::signedArea(c);
        }
    }
    return a;
}

std::size_t contourCount(const Document& doc, const std::vector<ObjectId>& ids, bool holes) {
    std::size_t n = 0;
    for (ObjectId id : ids) {
        for (const Contour& c : doc.findObject(id)->asVector()->contours) {
            if ((geometry::signedArea(c) < 0.0) == holes) {
                ++n;
            }
        }
    }
    return n;
}

}  // namespace

TEST_CASE("weld of two overlapping circles is one contour", "[weld_subtract]") {
    Document doc;
    const ObjectId a = add(doc, Contour::circle({0, 0}, 10));
    const ObjectId b = add(doc, Contour::circle({12, 0}, 10));
    doc.selection().assign({a, b});
    const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 1);
    const auto& cs = doc.findObject(r.created[0])->asVector()->contours;
    REQUIRE(cs.size() == 1);
    CHECK(cs[0].closed);
    // Lens area for two radius-10 circles 12 apart: 2 r^2 acos(d/2r) - (d/2) sqrt(4r^2 - d^2).
    const double lens = 2.0 * 100.0 * std::acos(0.6) - 6.0 * std::sqrt(400.0 - 144.0);
    CHECK(geometry::signedArea(cs[0]) == Approx(2.0 * 3.14159265358979323846 * 100.0 - lens).epsilon(2e-3));
    CHECK(doc.undoStack().count() == 1);
    REQUIRE(doc.undo());
    CHECK(doc.hasObject(a));
    CHECK(doc.hasObject(b));
}

TEST_CASE("weld of text keeps the counters", "[weld_subtract][text]") {
    std::ifstream in(std::string(CARVETTO_TEXT_FIXTURE_DIR) + "/CarvettoTest-Regular.ttf", std::ios::binary);
    REQUIRE(in.good());
    const std::vector<std::uint8_t> font((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    Document doc(document::Job::singleSided(1000.0, 1000.0, 18.0, document::Units::Millimetres));
    viewmodel::ViewTransform view;
    view.setViewport(1000.0, 1000.0);
    view.set(2.0, {100.0, 900.0});
    viewmodel::ToolController tool(&doc);
    tool.setView(view);
    tool.setTool(viewmodel::ToolKind::Text);
    REQUIRE(tool.textEditor().setFont(font, "Test").ok);
    const Vec2 at = view.worldToScreen({100.0, 100.0});
    tool.pointerMove(at);
    tool.pointerPress(at);
    tool.pointerRelease(at);
    tool.textEditor().insertText("TOI");
    REQUIRE(tool.key(viewmodel::ToolKey::Enter).committed);
    REQUIRE(doc.objectCount() == 1);
    const ObjectId text = doc.objectIds().front();
    const auto textContours = doc.findObject(text)->asVector()->contours;
    double textArea = 0.0;
    std::size_t holes = 0;
    Box2 box;
    for (const Contour& c : textContours) {
        textArea += geometry::signedArea(c);
        if (geometry::signedArea(c) < 0.0) {
            ++holes;
        }
        box.expand(c.bounds());
    }
    REQUIRE(holes >= 1);  // the O has a counter

    // A bar that sticks out to the left of the text and overlaps it slightly.
    const ObjectId bar = add(doc, Contour::rectangle(Box2{{box.min.x - 5.0, box.min.y}, {box.min.x + 0.5, box.max.y}}));
    doc.selection().assign({text, bar});
    const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    REQUIRE(r.ok);
    CHECK(contourCount(doc, r.created, true) == holes);
    const double welded = totalArea(doc, r.created);
    CHECK(welded > textArea);
    CHECK(welded < textArea + 5.5 * box.height() + 1e-6);
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("subtract and overlap are exact on squares, one undo step each", "[weld_subtract]") {
    Document doc;
    const ObjectId a = add(doc, Contour::rectangle(Box2{{0, 0}, {10, 10}}));
    const ObjectId b = add(doc, Contour::rectangle(Box2{{5, 5}, {15, 15}}));

    const std::size_t base = doc.undoStack().count();
    doc.selection().assign({a, b});
    const auto sub = viewmodel::applyBoolean(doc, BooleanKind::Subtract);
    REQUIRE(sub.ok);
    CHECK(doc.undoStack().count() == base + 1);
    REQUIRE(sub.created.size() == 1);
    const auto& l = doc.findObject(sub.created[0])->asVector()->contours;
    REQUIRE(l.size() == 1);
    CHECK(l[0].spans.size() == 6);  // an L shape
    CHECK(totalArea(doc, sub.created) == Approx(75.0).margin(1e-9));
    CHECK(l[0].bounds().min.x == Approx(0.0));
    CHECK(l[0].bounds().max.x == Approx(10.0));
    REQUIRE(doc.undo());
    CHECK(doc.hasObject(a));
    CHECK(doc.hasObject(b));

    doc.selection().assign({a, b});
    const auto ov = viewmodel::applyBoolean(doc, BooleanKind::Intersect);
    REQUIRE(ov.ok);
    CHECK(doc.undoStack().count() == base + 1);
    REQUIRE(ov.created.size() == 1);
    const Box2 ob = doc.findObject(ov.created[0])->asVector()->contours[0].bounds();
    CHECK(ob.min.x == Approx(5.0));
    CHECK(ob.min.y == Approx(5.0));
    CHECK(ob.max.x == Approx(10.0));
    CHECK(ob.max.y == Approx(10.0));
    CHECK(totalArea(doc, ov.created) == Approx(25.0).margin(1e-9));
}
