#include "carvetto/viewmodel/tools.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <variant>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/shapes.hpp"

using namespace carvetto::document;
using namespace carvetto::geometry;
using namespace carvetto::viewmodel;
using Catch::Matchers::WithinAbs;

namespace {

struct Fixture {
    Document doc;
    ViewTransform view;
    ToolController tool;
    explicit Fixture(Units units = Units::Millimetres)
        : doc(Job::singleSided(1000.0, 1000.0, 18.0, units)) {
        view.setViewport(1000.0, 1000.0);
        view.set(2.0, {100.0, 900.0});  // 2 px per mm
        tool.setDocument(&doc);
        tool.setView(view);
    }
    Vec2 px(Vec2 mm) const { return view.worldToScreen(mm); }

    ToolResult click(Vec2 mm, Modifiers m = {}) {
        tool.pointerMove(px(mm), m);
        const ToolResult p = tool.pointerPress(px(mm), m);
        const ToolResult r = tool.pointerRelease(px(mm), m);
        ToolResult out = r;
        out.committed = p.committed || r.committed;
        out.selectionChanged = p.selectionChanged || r.selectionChanged;
        if (out.message.empty()) {
            out.message = p.message;
        }
        return out;
    }
    ToolResult drag(Vec2 a, Vec2 b, Modifiers m = {}) {
        tool.pointerMove(px(a), m);
        tool.pointerPress(px(a), m);
        tool.pointerMove(px((a + b) * 0.5), m);
        tool.pointerMove(px(b), m);
        return tool.pointerRelease(px(b), m);
    }
    const Object& last() const {
        const auto& objs = doc.objects();
        return objs.rbegin()->second;
    }
    template <class P>
    const P& params() const {
        const auto* v = last().asVector();
        REQUIRE(v != nullptr);
        const P* p = std::get_if<P>(&v->params);
        REQUIRE(p != nullptr);
        return *p;
    }
    /// The object's contours equal a fresh build from its stored parameters.
    void requireRebuilds() const {
        const auto* v = last().asVector();
        REQUIRE(v != nullptr);
        const auto built = buildContours(v->params);
        REQUIRE(built.ok());
        REQUIRE(built.value() == v->contours);
    }
    void requireOneUndoStep(std::size_t objectsBefore) {
        REQUIRE(doc.undoStack().count() == 1);
        REQUIRE(doc.objectCount() == objectsBefore + 1);
        const Document* unused = &doc;
        (void)unused;
        REQUIRE(doc.undo());
        REQUIRE(doc.objectCount() == objectsBefore);
        REQUIRE(doc.redo());
        REQUIRE(doc.objectCount() == objectsBefore + 1);
    }
};

Contour need(const ShapeResult<Contour>& r) {
    INFO((r.ok() ? std::string() : r.error()));
    REQUIRE(r.ok());
    return r.value();
}

}  // namespace

TEST_CASE("rectangle: drag creates the T-0200 contour in one undo step", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Rectangle);
    const ToolResult r = f.drag({10, 20}, {50, 60});
    REQUIRE(r.committed);
    const auto& p = f.params<RectangleParams>();
    CHECK(p.centreMm == Vec2{30, 40});
    CHECK(p.widthMm == 40.0);
    CHECK(p.heightMm == 40.0);
    CHECK(p.corners == RectangleParams::CornerStyle::Square);
    const auto expected = buildRectangle({30, 40}, 40, 40, RectCorner::Square, 0.0);
    REQUIRE(expected.ok());
    REQUIRE(f.last().asVector()->contours.size() == 1);
    CHECK(f.last().asVector()->contours[0] == expected.value());
    f.requireRebuilds();
    f.requireOneUndoStep(0);
    CHECK_FALSE(f.tool.active());
}

TEST_CASE("rectangle: dragging in any direction, rounded and chamfered corners", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Rectangle);
    f.tool.options().cornerStyle = RectangleParams::CornerStyle::Rounded;
    f.tool.options().cornerSizeMm = 5.0;
    REQUIRE(f.drag({100, 100}, {60, 70}).committed);  // up-left in screen terms: negative dx, dy
    auto p = f.params<RectangleParams>();
    CHECK(p.centreMm == Vec2{80, 85});
    CHECK(p.widthMm == 40.0);
    CHECK(p.heightMm == 30.0);
    CHECK(p.cornerRadiusMm == 5.0);
    f.requireRebuilds();
    CHECK(f.last().asVector()->contours[0] ==
          need(buildRectangle({80, 85}, 40, 30, RectCorner::Rounded, 5.0)));

    f.tool.options().cornerStyle = RectangleParams::CornerStyle::Chamfered;
    f.tool.options().cornerSizeMm = 100.0;  // clamped to half the short side while dragging
    REQUIRE(f.drag({0, 0}, {20, 10}).committed);
    p = f.params<RectangleParams>();
    CHECK(p.corners == RectangleParams::CornerStyle::Chamfered);
    CHECK(p.cornerRadiusMm == 5.0);
    f.requireRebuilds();
    CHECK(f.doc.undoStack().count() == 2);
}

TEST_CASE("click-click completes a shape, a zero-size drag is refused", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Circle);
    CHECK_FALSE(f.click({50, 50}).committed);
    CHECK(f.tool.active());
    const ToolPreview pv = [&] {
        f.tool.pointerMove(f.px({80, 50}));
        return f.tool.preview();
    }();
    CHECK(pv.valid);
    REQUIRE(pv.contours.size() == 1);
    CHECK(pv.contours[0] == need(buildCircle({50, 50}, 30)));
    const ToolResult r = f.click({80, 50});
    REQUIRE(r.committed);
    CHECK(f.params<CircleParams>().centreMm == Vec2{50, 50});
    CHECK(f.params<CircleParams>().radiusMm == 30.0);
    f.requireRebuilds();
    f.requireOneUndoStep(0);

    // Completing at the anchor itself: no shape, nothing recorded, tool keeps waiting.
    Fixture g;
    g.tool.setTool(ToolKind::Circle);
    g.click({10, 10});
    const ToolResult bad = g.click({10, 10});
    CHECK_FALSE(bad.committed);
    CHECK(g.doc.undoStack().count() == 0);
    CHECK(g.tool.active());
}

TEST_CASE("escape cancels without touching the document", "[viewmodel][tool]") {
    for (ToolKind k : {ToolKind::Rectangle, ToolKind::Circle, ToolKind::Ellipse, ToolKind::Polygon,
                       ToolKind::Star, ToolKind::Arc, ToolKind::Polyline}) {
        Fixture f;
        f.tool.setTool(k);
        f.click({10, 10});
        f.tool.pointerMove(f.px({40, 30}));
        CHECK(f.tool.active());
        f.tool.key(ToolKey::Escape);
        CHECK_FALSE(f.tool.active());
        CHECK(f.doc.objectCount() == 0);
        CHECK(f.doc.undoStack().count() == 0);
        CHECK(f.tool.preview().contours.empty());
        CHECK_FALSE(f.tool.preview().valid);
        // And the tool still works afterwards.
        CHECK_FALSE(f.tool.active());
    }
    // Switching tools cancels too.
    Fixture g;
    g.tool.setTool(ToolKind::Polyline);
    g.click({1, 1});
    g.tool.setTool(ToolKind::Rectangle);
    CHECK_FALSE(g.tool.active());
}

TEST_CASE("ellipse, polygon and star match the builders", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Ellipse);
    REQUIRE(f.drag({10, 10}, {70, 40}).committed);
    CHECK(f.params<EllipseParams>().centreMm == Vec2{40, 25});
    CHECK(f.params<EllipseParams>().widthMm == 60.0);
    CHECK(f.params<EllipseParams>().heightMm == 30.0);
    CHECK(f.last().asVector()->contours[0] == need(buildEllipse({40, 25}, 60, 30, 0.0)));
    f.requireRebuilds();

    f.tool.setTool(ToolKind::Polygon);
    f.tool.options().polygonSides = 7;
    REQUIRE(f.drag({200, 200}, {200, 230}).committed);  // vertex 0 straight up
    const auto& pg = f.params<PolygonParams>();
    CHECK(pg.sides == 7);
    CHECK(pg.centreMm == Vec2{200, 200});
    CHECK_THAT(pg.radiusMm, WithinAbs(30.0, 1e-9));
    CHECK_THAT(pg.angle, WithinAbs(kPi / 2, 1e-12));
    CHECK(f.last().asVector()->contours[0] == need(buildPolygon({200, 200}, 7, pg.radiusMm, pg.angle)));
    f.requireRebuilds();

    f.tool.setTool(ToolKind::Star);
    f.tool.options().starPoints = 6;
    f.tool.options().starInnerPercent = 40.0;
    REQUIRE(f.drag({300, 300}, {330, 300}).committed);
    const auto& st = f.params<StarParams>();
    CHECK(st.points == 6);
    CHECK_THAT(st.outerRadiusMm, WithinAbs(30.0, 1e-9));
    CHECK_THAT(st.innerRadiusMm, WithinAbs(12.0, 1e-9));
    CHECK_THAT(st.angle, WithinAbs(0.0, 1e-12));
    f.requireRebuilds();
    CHECK(f.doc.undoStack().count() == 3);
}

TEST_CASE("arc: three points, centre/start/end, start/end/radius, start/end/height", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Arc);

    f.tool.options().arcMode = ArcMode::ThreePoint;
    f.click({0, 0});
    f.click({10, 10});
    const ToolResult r3 = f.click({20, 0});
    REQUIRE(r3.committed);
    CHECK(f.last().asVector()->contours[0] == need(buildArcThreePoint({0, 0}, {10, 10}, {20, 0})));
    CHECK(std::holds_alternative<NoParams>(f.last().asVector()->params));

    f.tool.options().arcMode = ArcMode::CentreStartEnd;
    f.tool.options().arcDirection = ArcDirection::CW;
    f.click({100, 100});
    f.click({110, 100});
    REQUIRE(f.click({100, 110}).committed);
    CHECK(f.last().asVector()->contours[0] ==
          need(buildArcCentreStartEnd({100, 100}, {110, 100}, {100, 110}, ArcDirection::CW)));

    // Start/end/radius: the pointer picks the side; the radius is typed.
    f.tool.options().arcMode = ArcMode::StartEndRadius;
    f.tool.options().arcLarge = false;
    f.click({200, 0});
    f.click({240, 0});
    f.tool.pointerMove(f.px({220, -30}));  // right of start->end
    REQUIRE(f.tool.setField("radius", "25").ok);
    REQUIRE(f.tool.key(ToolKey::Enter).committed);
    CHECK(f.last().asVector()->contours[0] ==
          need(buildArcStartEndRadius({200, 0}, {240, 0}, 25, ArcDirection::CCW, false)));
    // Without a typed radius the arc passes through the pointer (circle through the three points).
    f.click({200, 100});
    f.click({240, 100});
    f.tool.pointerMove(f.px({220, 120}));  // left of start->end, 20 above the chord
    const ToolPreview pv = f.tool.preview();
    REQUIRE(pv.valid);
    const auto* arc = std::get_if<ArcSpan>(&pv.contours[0].spans[0]);
    REQUIRE(arc != nullptr);
    CHECK(arc->direction == ArcDirection::CW);
    CHECK_THAT(arc->radius, WithinAbs((400.0 + 400.0) / 40.0, 1e-9));  // (c/2)^2 + h^2) / 2h
    f.tool.key(ToolKey::Escape);

    // Start/end/height: signed distance of the pointer from the chord, or typed.
    f.tool.options().arcMode = ArcMode::StartEndHeight;
    f.click({300, 0});
    f.click({340, 0});
    f.tool.pointerMove(f.px({320, 15}));
    REQUIRE(f.click({320, 15}).committed);  // the third click completes it
    CHECK(f.last().asVector()->contours[0] == need(buildArcStartEndHeight({300, 0}, {340, 0}, 15.0)));
    f.click({400, 0});
    f.click({440, 0});
    f.tool.pointerMove(f.px({420, 15}));
    REQUIRE(f.tool.setField("height", "-12").ok);
    REQUIRE(f.tool.key(ToolKey::Enter).committed);
    CHECK(f.last().asVector()->contours[0] == need(buildArcStartEndHeight({400, 0}, {440, 0}, -12.0)));
    CHECK(f.doc.undoStack().count() == 5);
    CHECK(f.doc.objectCount() == 5);
    // Collinear three points are refused and the tool keeps its two points.
    f.tool.options().arcMode = ArcMode::ThreePoint;
    f.click({0, 500});
    f.click({10, 500});
    const ToolResult bad = f.click({20, 500});
    CHECK_FALSE(bad.committed);
    CHECK_FALSE(bad.message.empty());
    CHECK(f.doc.objectCount() == 5);
    CHECK(f.tool.active());
}

TEST_CASE("polyline: points, double-click, Enter, close, backspace", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Polyline);
    f.click({0, 0});
    f.click({20, 0});
    f.click({20, 20});
    f.tool.pointerMove(f.px({0, 20}));
    ToolPreview pv = f.tool.preview();
    REQUIRE(pv.contours.size() == 1);
    CHECK(pv.contours[0].spans.size() == 3);  // two placed plus the rubber band
    CHECK(pv.anchorsMm.size() == 3);
    // Double-click finishes with a point at the click.
    f.tool.pointerMove(f.px({0, 20}));
    f.tool.pointerPress(f.px({0, 20}));
    f.tool.pointerRelease(f.px({0, 20}));
    const ToolResult r = f.tool.doubleClick(f.px({0, 20}));
    REQUIRE(r.committed);
    const Contour& c = f.last().asVector()->contours[0];
    CHECK_FALSE(c.closed);
    REQUIRE(c.spans.size() == 3);  // the double-click point duplicates the last: not added twice
    CHECK(c.spans[2] == Span{LineSpan{{20, 20}, {0, 20}}});
    CHECK(std::holds_alternative<NoParams>(f.last().asVector()->params));
    CHECK(f.doc.undoStack().count() == 1);

    // Tab closes (three or more points).
    f.click({100, 0});
    f.click({120, 0});
    CHECK_FALSE(f.tool.key(ToolKey::Close).committed);
    CHECK(f.tool.active());
    f.click({120, 20});
    REQUIRE(f.tool.key(ToolKey::Close).committed);
    const Contour& closed = f.last().asVector()->contours[0];
    CHECK(closed.closed);
    CHECK(closed.spans.size() == 3);
    CHECK(closed.isContinuous(Tolerance::defaults()));

    // Backspace removes the last point; Enter finishes; fewer than two points is refused.
    f.click({200, 0});
    f.click({210, 0});
    f.click({210, 10});
    f.tool.key(ToolKey::Backspace);
    REQUIRE(f.tool.key(ToolKey::Enter).committed);
    CHECK(f.last().asVector()->contours[0].spans.size() == 1);
    f.click({300, 300});
    CHECK_FALSE(f.tool.key(ToolKey::Enter).committed);
    CHECK(f.doc.undoStack().count() == 3);
}

TEST_CASE("modifiers: Alt centre-out, Ctrl square/circle, Alt circle diameter", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Rectangle);
    Modifiers alt;
    alt.alt = true;
    REQUIRE(f.drag({50, 50}, {70, 60}, alt).committed);
    CHECK(f.params<RectangleParams>().centreMm == Vec2{50, 50});
    CHECK(f.params<RectangleParams>().widthMm == 40.0);
    CHECK(f.params<RectangleParams>().heightMm == 20.0);

    Modifiers ctrl;
    ctrl.ctrl = true;
    REQUIRE(f.drag({0, 0}, {30, 10}, ctrl).committed);
    CHECK(f.params<RectangleParams>().widthMm == 30.0);
    CHECK(f.params<RectangleParams>().heightMm == 30.0);
    CHECK(f.params<RectangleParams>().centreMm == Vec2{15, 15});

    Modifiers both;
    both.alt = both.ctrl = true;
    REQUIRE(f.drag({0, 0}, {-10, 5}, both).committed);  // square, centre-out
    CHECK(f.params<RectangleParams>().widthMm == 20.0);
    CHECK(f.params<RectangleParams>().heightMm == 20.0);
    CHECK(f.params<RectangleParams>().centreMm == Vec2{0, 0});

    f.tool.setTool(ToolKind::Ellipse);
    REQUIRE(f.drag({100, 100}, {140, 120}, ctrl).committed);
    CHECK(f.params<EllipseParams>().widthMm == 40.0);
    CHECK(f.params<EllipseParams>().heightMm == 40.0);
    CHECK(f.last().asVector()->contours[0].spans.size() == 1);  // exact circle
    REQUIRE(f.drag({500, 500}, {510, 505}, alt).committed);
    CHECK(f.params<EllipseParams>().centreMm == Vec2{500, 500});
    CHECK(f.params<EllipseParams>().widthMm == 20.0);

    f.tool.setTool(ToolKind::Circle);
    REQUIRE(f.drag({0, 200}, {40, 200}, alt).committed);  // diameter drag
    CHECK(f.params<CircleParams>().centreMm == Vec2{20, 200});
    CHECK(f.params<CircleParams>().radiusMm == 20.0);
}

TEST_CASE("modifiers: Shift constrains the polyline to 45 degrees and disables snapping", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Polyline);
    Modifiers shift;
    shift.shift = true;
    f.click({0, 0});
    f.tool.pointerMove(f.px({30, 4}), shift);
    CHECK_THAT(f.tool.pointMm().y, WithinAbs(0.0, 1e-9));
    CHECK_THAT(f.tool.pointMm().x, WithinAbs(std::hypot(30.0, 4.0) * std::cos(std::atan2(4.0, 30.0)), 1e-9));
    f.tool.pointerMove(f.px({30, 28}), shift);
    CHECK_THAT(f.tool.pointMm().x, WithinAbs(f.tool.pointMm().y, 1e-9));
    f.tool.pointerMove(f.px({-3, 40}), shift);
    CHECK_THAT(f.tool.pointMm().x, WithinAbs(0.0, 1e-9));  // snaps to straight up
    CHECK_THAT(f.tool.pointMm().y, WithinAbs(40.0, 1e-9));
    CHECK_FALSE(f.tool.snap().has_value());
}

TEST_CASE("snapping: tools receive the T-0203 best snap, or the raw point", "[viewmodel][tool][snap]") {
    Fixture f;
    Contour line;
    line.append(LineSpan{{50, 50}, {150, 50}});
    f.doc.insertObject(Object::vector(line, f.doc.activeLayer(), f.doc.activeSheet()));
    f.tool.setTool(ToolKind::Rectangle);

    // 4 px (2 mm) from the end point: snapped; the exposed snap equals the library's best().
    const Vec2 nearPx = f.px({52, 51});
    f.tool.pointerMove(nearPx);
    REQUIRE(f.tool.snap().has_value());
    const auto expected = SnapIndex(f.doc).best(f.view, nearPx, f.tool.snapSettings());
    REQUIRE(expected.has_value());
    CHECK(f.tool.snap()->kind == expected->kind);
    CHECK(f.tool.snap()->pointMm == expected->pointMm);
    CHECK(f.tool.pointMm() == expected->pointMm);
    CHECK(f.tool.snap()->kind == SnapKind::Endpoint);
    CHECK(f.tool.pointMm() == Vec2{50, 50});

    // Far from everything: the raw point and no snap.
    f.tool.pointerMove(f.px({400, 400}));
    CHECK_FALSE(f.tool.snap().has_value());
    CHECK_THAT(f.tool.pointMm().x, WithinAbs(400.0, 1e-9));

    // The drawn shape uses the snapped corner.
    f.tool.pointerPress(nearPx);
    const ToolResult r = f.tool.pointerRelease(f.px({250, 120}));
    REQUIRE(r.committed);
    CHECK(f.params<RectangleParams>().centreMm == Vec2{150, 85});

    // Shift: snapping off for that event.
    Modifiers shift;
    shift.shift = true;
    f.tool.pointerMove(nearPx, shift);
    CHECK_FALSE(f.tool.snap().has_value());
    CHECK_THAT(f.tool.pointMm().x, WithinAbs(52.0, 1e-9));

    // Disabled kinds never snap; the grid snaps when enabled.
    f.tool.snapSettings().disableAll();
    f.tool.pointerMove(nearPx);
    CHECK_FALSE(f.tool.snap().has_value());
    f.tool.snapSettings().setEnabled(SnapKind::Grid, true);
    f.tool.snapSettings().gridSpacingMm = 10.0;
    f.tool.pointerMove(f.px({53, 48}));
    REQUIRE(f.tool.snap().has_value());
    CHECK(f.tool.snap()->kind == SnapKind::Grid);
    CHECK(f.tool.pointMm() == Vec2{50, 50});
}

TEST_CASE("the snap index follows document edits (undo, new objects)", "[viewmodel][tool][snap]") {
    Fixture f;
    f.tool.setTool(ToolKind::Circle);
    REQUIRE(f.drag({100, 100}, {120, 100}).committed);
    const Vec2 q = f.px({120, 100}) + Vec2{3, 3};
    f.tool.pointerMove(q);
    REQUIRE(f.tool.snap().has_value());  // end/quadrant of the new circle
    REQUIRE(f.doc.undo());
    f.tool.pointerMove(q);
    CHECK_FALSE(f.tool.snap().has_value());
    REQUIRE(f.doc.redo());
    f.tool.pointerMove(q);
    CHECK(f.tool.snap().has_value());
}

TEST_CASE("numeric entry: parsing and validation", "[viewmodel][tool]") {
    CHECK(parseNumber("12.5", FieldKind::Length, Units::Millimetres).value == 12.5);
    CHECK_THAT(parseNumber("1", FieldKind::Length, Units::Inches).value, WithinAbs(25.4, 1e-12));
    CHECK_THAT(parseNumber(" 2 in ", FieldKind::Length, Units::Millimetres).value, WithinAbs(50.8, 1e-12));
    CHECK(parseNumber("7mm", FieldKind::Length, Units::Inches).value == 7.0);
    CHECK(parseNumber("-3", FieldKind::Length, Units::Millimetres).value == -3.0);
    CHECK(parseNumber("45deg", FieldKind::Angle, Units::Millimetres).value == 45.0);
    CHECK(parseNumber("50%", FieldKind::Percent, Units::Millimetres).value == 50.0);
    CHECK(parseNumber("6", FieldKind::Count, Units::Millimetres).value == 6.0);
    for (const char* bad : {"", "  ", "abc", "1.2.3", "1e999", "nan", "inf", "-inf", "0x10", "12 cm", "--1", "1,5",
                            "1e12", "mm"}) {
        INFO(bad);
        CHECK_FALSE(parseNumber(bad, FieldKind::Length, Units::Millimetres).ok);
    }
    CHECK_FALSE(parseNumber("2.5", FieldKind::Count, Units::Millimetres).ok);
    CHECK_FALSE(parseNumber("5 mm", FieldKind::Angle, Units::Millimetres).ok);
}

TEST_CASE("numeric entry: bad input is rejected without side effects", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Rectangle);
    f.click({10, 10});
    REQUIRE(f.tool.setField("width", "30").ok);
    const ToolPreview before = f.tool.preview();
    for (const char* bad : {"abc", "0", "-5", "1e999", ""}) {
        if (std::string(bad).empty()) {
            continue;
        }
        const FieldResult r = f.tool.setField("width", bad);
        INFO(bad);
        CHECK_FALSE(r.ok);
        CHECK_FALSE(r.message.empty());
    }
    CHECK_FALSE(f.tool.setField("nonsense", "1").ok);
    CHECK_FALSE(f.tool.setField("corner", "-1").ok);
    const ToolPreview after = f.tool.preview();
    CHECK(after.contours == before.contours);
    bool typedWidth = false;
    for (const NumericField& nf : f.tool.fields()) {
        if (nf.id == "width") {
            typedWidth = nf.typed;
            CHECK(nf.text == "30.00");
        }
    }
    CHECK(typedWidth);
    CHECK(f.doc.objectCount() == 0);
    // An empty entry clears the override.
    CHECK(f.tool.setField("width", "").ok);

    f.tool.setTool(ToolKind::Polygon);
    CHECK_FALSE(f.tool.setField("sides", "2").ok);
    CHECK_FALSE(f.tool.setField("sides", "3.5").ok);
    CHECK(f.tool.setField("sides", "8").ok);
    f.tool.setTool(ToolKind::Star);
    CHECK_FALSE(f.tool.setField("inner", "100").ok);
    CHECK_FALSE(f.tool.setField("inner", "0").ok);
    CHECK(f.tool.setField("inner", "35").ok);
}

TEST_CASE("numeric entry completes shapes in display units", "[viewmodel][tool]") {
    Fixture f(Units::Inches);
    f.tool.setTool(ToolKind::Rectangle);
    f.click({10, 10});
    REQUIRE(f.tool.setField("width", "2").ok);   // inches
    REQUIRE(f.tool.setField("height", "1").ok);
    REQUIRE(f.tool.key(ToolKey::Enter).committed);
    CHECK_THAT(f.params<RectangleParams>().widthMm, WithinAbs(50.8, 1e-12));
    CHECK_THAT(f.params<RectangleParams>().heightMm, WithinAbs(25.4, 1e-12));
    // Cursor was at the anchor: positive direction, centre offset by half the size.
    CHECK_THAT(f.params<RectangleParams>().centreMm.x, WithinAbs(10.0 + 25.4, 1e-12));
    f.requireRebuilds();

    // Typed centre without any click.
    f.tool.setTool(ToolKind::Circle);
    REQUIRE(f.tool.setField("x", "1").ok);
    REQUIRE(f.tool.setField("y", "2").ok);
    REQUIRE(f.tool.setField("radius", "0.5").ok);
    REQUIRE(f.tool.commitNumeric().committed);
    CHECK_THAT(f.params<CircleParams>().centreMm.x, WithinAbs(25.4, 1e-12));
    CHECK_THAT(f.params<CircleParams>().centreMm.y, WithinAbs(50.8, 1e-12));
    CHECK_THAT(f.params<CircleParams>().radiusMm, WithinAbs(12.7, 1e-12));

    f.tool.setTool(ToolKind::Star);
    f.tool.setField("x", "0");
    f.tool.setField("y", "0");
    f.tool.setField("points", "7");
    f.tool.setField("radius", "10mm");
    f.tool.setField("inner", "25");
    f.tool.setField("angle", "90");
    REQUIRE(f.tool.commitNumeric().committed);
    const auto& st = f.params<StarParams>();
    CHECK(st.points == 7);
    CHECK_THAT(st.innerRadiusMm, WithinAbs(2.5, 1e-12));
    CHECK_THAT(st.angle, WithinAbs(kPi / 2, 1e-12));
    CHECK(f.doc.undoStack().count() == 3);

    // Nothing typed, nothing placed: refused, no side effect.
    f.tool.setTool(ToolKind::Rectangle);
    CHECK_FALSE(f.tool.commitNumeric().committed);
    CHECK(f.doc.undoStack().count() == 3);
}

TEST_CASE("numeric entry: polyline next point by X/Y and by length/angle", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Polyline);
    REQUIRE(f.tool.setField("x", "5").ok);
    CHECK_FALSE(f.tool.commitNumeric().committed);  // needs Y too
    CHECK(f.tool.active() == false);
    REQUIRE(f.tool.setField("y", "5").ok);
    f.tool.commitNumeric();
    CHECK(f.tool.active());
    REQUIRE(f.tool.setField("length", "10").ok);
    REQUIRE(f.tool.setField("angle", "90").ok);
    f.tool.commitNumeric();
    REQUIRE(f.tool.setField("length", "10").ok);
    REQUIRE(f.tool.setField("angle", "0").ok);
    f.tool.commitNumeric();
    REQUIRE(f.tool.key(ToolKey::Enter).committed);
    const Contour& c = f.last().asVector()->contours[0];
    REQUIRE(c.spans.size() == 2);
    const auto& l0 = std::get<LineSpan>(c.spans[0]);
    const auto& l1 = std::get<LineSpan>(c.spans[1]);
    CHECK(l0.start == Vec2{5, 5});
    CHECK_THAT(l0.end.x, WithinAbs(5.0, 1e-9));
    CHECK_THAT(l0.end.y, WithinAbs(15.0, 1e-9));
    CHECK_THAT(l1.end.x, WithinAbs(15.0, 1e-9));
}

TEST_CASE("locked or hidden active layer refuses drawing", "[viewmodel][tool]") {
    Fixture f;
    f.tool.setTool(ToolKind::Rectangle);
    f.doc.findLayer(f.doc.activeLayer())->locked = true;
    const ToolResult r = f.click({10, 10});
    CHECK_FALSE(r.message.empty());
    CHECK_FALSE(f.tool.active());
    f.doc.findLayer(f.doc.activeLayer())->locked = false;
    f.doc.findLayer(f.doc.activeLayer())->visible = false;
    CHECK_FALSE(f.click({10, 10}).message.empty());
    CHECK(f.doc.undoStack().count() == 0);
}

TEST_CASE("created objects survive a .carv round trip and rebuild identically", "[viewmodel][tool]") {
    Fixture f;
    f.tool.options().cornerStyle = RectangleParams::CornerStyle::Rounded;
    f.tool.options().cornerSizeMm = 4.0;
    f.tool.setTool(ToolKind::Rectangle);
    REQUIRE(f.drag({10, 10}, {60, 40}).committed);
    f.tool.setTool(ToolKind::Star);
    REQUIRE(f.drag({100, 100}, {120, 110}).committed);
    f.tool.setTool(ToolKind::Ellipse);
    REQUIRE(f.drag({200, 100}, {260, 140}).committed);
    f.tool.setTool(ToolKind::Polygon);
    REQUIRE(f.drag({300, 100}, {320, 130}).committed);
    f.tool.setTool(ToolKind::Circle);
    REQUIRE(f.drag({400, 100}, {420, 130}).committed);

    const auto bytes = saveCarv(f.doc);
    REQUIRE(bytes.ok());
    const auto loaded = loadCarv(bytes.value());
    REQUIRE(loaded.ok());
    const Document& d2 = loaded.value().document;
    REQUIRE(d2.objectCount() == f.doc.objectCount());
    for (const auto& [id, obj] : f.doc.objects()) {
        const Object* other = d2.findObject(id);
        REQUIRE(other != nullptr);
        CHECK(other->asVector()->params == obj.asVector()->params);
        const auto rebuilt = buildContours(other->asVector()->params);
        REQUIRE(rebuilt.ok());
        CHECK(rebuilt.value() == obj.asVector()->contours);
    }
}

TEST_CASE("select tool: click, modifiers and marquee", "[viewmodel][tool]") {
    Fixture f;
    Contour a = Contour::circle({50, 50}, 10);
    Contour b = Contour::circle({150, 50}, 10);
    const ObjectId ia = f.doc.insertObject(Object::vector(a, f.doc.activeLayer(), f.doc.activeSheet()));
    const ObjectId ib = f.doc.insertObject(Object::vector(b, f.doc.activeLayer(), f.doc.activeSheet()));
    f.tool.setTool(ToolKind::Select);

    ToolResult r = f.click({60, 50});  // on the outline of a
    CHECK(r.selectionChanged);
    CHECK(f.doc.selection().ids() == std::vector<ObjectId>{ia});
    Modifiers shift;
    shift.shift = true;
    f.click({160, 50}, shift);
    CHECK(f.doc.selection().size() == 2);
    Modifiers ctrl;
    ctrl.ctrl = true;
    f.click({60, 50}, ctrl);
    CHECK(f.doc.selection().ids() == std::vector<ObjectId>{ib});
    f.click({300, 300});  // empty: clears
    CHECK(f.doc.selection().empty());

    // Marquee left to right selects objects wholly inside; right to left also those it crosses.
    f.drag({30, 30}, {100, 70});
    CHECK(f.doc.selection().ids() == std::vector<ObjectId>{ia});
    f.tool.pointerMove(f.px({200, 70}));
    f.tool.pointerPress(f.px({200, 70}));
    f.tool.pointerMove(f.px({145, 40}));
    CHECK(f.tool.preview().marqueePx.has_value());
    f.tool.pointerRelease(f.px({145, 40}));  // crosses b only
    CHECK(f.doc.selection().ids() == std::vector<ObjectId>{ib});
    CHECK_FALSE(f.tool.preview().marqueePx.has_value());
    CHECK(f.doc.undoStack().count() == 0);  // selection is not an undo step
}

TEST_CASE("scripted event sequences are deterministic", "[viewmodel][tool]") {
    auto run = [] {
        Fixture f;
        f.tool.setTool(ToolKind::Star);
        f.drag({10, 10}, {40, 25});
        f.tool.setTool(ToolKind::Polyline);
        f.click({0, 0});
        f.click({30, 5});
        f.tool.key(ToolKey::Enter);
        return saveCarv(f.doc).value();
    };
    CHECK(run() == run());
}
