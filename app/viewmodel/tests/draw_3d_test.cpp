#include "carvetto/viewmodel/draw_3d.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;
using carvetto::geometry::Box2;
using Catch::Approx;

namespace {

struct Fixture {
    Document doc{Job::singleSided(200.0, 100.0, 18.0, Units::Millimetres)};
    ModelSurface surface;
    OrbitCamera camera;
    ModelRenderOptions options;
    Draw3DController draw;
    std::size_t base{0};
    std::size_t steps() const { return doc.undoStack().count() - base; }

    Fixture() {
        LevelId level;
        REQUIRE(addReliefLevel(doc, doc.activeSheet(), {}, &level).changed);
        relief::Component c;
        c.name = "C";
        REQUIRE(relief::HeightField::create(relief::Grid{101, 51, 2.0, {0.0, 0.0}}, 3.0f, c.field) ==
                relief::Status::Ok);
        REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(level, std::move(c))));
        surface = composeModelSurface(doc, doc.activeSheet());
        REQUIRE(surface.ok());
        draw.setDocument(&doc);
        base = doc.undoStack().count();
    }
    Draw3DView view(bool wrapped = false) const { return Draw3DView{&surface, &camera, &options, wrapped}; }
    ScreenPoint screen(double x, double y) const {
        const auto p = projectModelPoint(surface, camera, options, x, y, 0.0);
        REQUIRE(p.has_value());
        return *p;
    }
    // Press at plane point (x, y), release at (x2, y2).
    Draw3DResult drag(double x, double y, double x2, double y2) {
        const ScreenPoint a = screen(x, y), b = screen(x2, y2);
        draw.pointerMove(view(), a.x, a.y);
        draw.pointerPress(view(), a.x, a.y);
        draw.pointerMove(view(), 0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
        draw.pointerMove(view(), b.x, b.y);
        return draw.pointerRelease(view(), b.x, b.y);
    }
};

}  // namespace

TEST_CASE("pickZeroPlane inverts the projection of plane points", "[viewmodel][draw3d]") {
    Fixture f;
    for (const double azimuth : {0.0, 0.6, 2.0, -1.2}) {
        f.camera.azimuthRad = azimuth;
        for (const auto& pt : {std::pair{10.0, 20.0}, std::pair{150.0, 70.0}, std::pair{-30.0, 120.0}}) {
            const ScreenPoint s = f.screen(pt.first, pt.second);
            const PlanePick p = pickZeroPlane(f.surface, f.camera, f.options, s.x, s.y);
            REQUIRE(p.ok);
            CHECK(p.mm.x == Approx(pt.first).margin(1e-6));
            CHECK(p.mm.y == Approx(pt.second).margin(1e-6));
            CHECK(p.pxPerMm > 0.0);
        }
    }
}

TEST_CASE("pickZeroPlane refuses what is not on the plane", "[viewmodel][draw3d]") {
    Fixture f;
    // Grazing view.
    f.camera.elevationRad = 0.02;
    CHECK_FALSE(pickZeroPlane(f.surface, f.camera, f.options, 300, 240).ok);
    f.camera.viewFront();
    const PlanePick edgeOn = pickZeroPlane(f.surface, f.camera, f.options, 300, 240);
    CHECK_FALSE(edgeOn.ok);
    CHECK_FALSE(edgeOn.refusal.empty());
    // A pixel that lands far beyond the model.
    f.camera.viewTop();
    CHECK(pickZeroPlane(f.surface, f.camera, f.options, 320, 240).ok);
    CHECK_FALSE(pickZeroPlane(f.surface, f.camera, f.options, 1.0e7, 240).ok);
    CHECK_FALSE(pickZeroPlane(f.surface, f.camera, f.options, std::nan(""), 240).ok);
    // The wrapped rotary view.
    CHECK_FALSE(pickZeroPlane(f.surface, f.camera, f.options, 320, 240, true).ok);
    // No model at all.
    CHECK_FALSE(pickZeroPlane(ModelSurface{}, f.camera, f.options, 320, 240).ok);
}

TEST_CASE("a rectangle drawn in the 3D view is a document vector at the picked coordinates", "[viewmodel][draw3d]") {
    Fixture f;
    f.draw.setShape(Draw3DShape::Rectangle);
    const Draw3DResult r = f.drag(20.0, 30.0, 80.0, 70.0);
    REQUIRE(r.tool.committed);
    CHECK(f.steps() == 1);
    const Object& o = f.doc.objects().rbegin()->second;
    REQUIRE(o.asVector() != nullptr);
    const auto* p = std::get_if<RectangleParams>(&o.asVector()->params);
    REQUIRE(p != nullptr);
    CHECK(p->centreMm.x == Approx(50.0).margin(1e-6));
    CHECK(p->centreMm.y == Approx(50.0).margin(1e-6));
    CHECK(p->widthMm == Approx(60.0).margin(1e-6));
    CHECK(p->heightMm == Approx(40.0).margin(1e-6));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.objects().empty());
}

TEST_CASE("a circle drawn click-click from a tilted view; undo removes it", "[viewmodel][draw3d]") {
    Fixture f;
    f.draw.setShape(Draw3DShape::Circle);
    const ScreenPoint c = f.screen(100.0, 50.0), e = f.screen(130.0, 50.0);
    f.draw.pointerPress(f.view(), c.x, c.y);
    f.draw.pointerRelease(f.view(), c.x, c.y);
    CHECK(f.draw.active());
    f.draw.pointerMove(f.view(), e.x, e.y);
    CHECK_FALSE(f.draw.preview().contours.empty());
    CHECK(f.draw.preview().valid);
    f.draw.pointerPress(f.view(), e.x, e.y);
    const Draw3DResult r = f.draw.pointerRelease(f.view(), e.x, e.y);
    CHECK(f.steps() == 1);
    CHECK((r.tool.committed || f.doc.objects().size() == 1));
    REQUIRE(f.doc.objects().size() == 1);
    const auto* p = std::get_if<CircleParams>(&f.doc.objects().rbegin()->second.asVector()->params);
    REQUIRE(p != nullptr);
    CHECK(p->centreMm.x == Approx(100.0).margin(1e-6));
    CHECK(p->centreMm.y == Approx(50.0).margin(1e-6));
    CHECK(p->radiusMm == Approx(30.0).margin(1e-6));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.objects().empty());
}

TEST_CASE("a line, polyline, ellipse and polygon each take one undo step", "[viewmodel][draw3d]") {
    Fixture f;
    f.draw.setShape(Draw3DShape::Line);
    const ScreenPoint a = f.screen(10, 10), b = f.screen(90, 40);
    f.draw.pointerPress(f.view(), a.x, a.y);
    f.draw.pointerRelease(f.view(), a.x, a.y);
    f.draw.pointerPress(f.view(), b.x, b.y);
    f.draw.pointerRelease(f.view(), b.x, b.y);
    CHECK(f.doc.objects().size() == 1);
    CHECK(f.steps() == 1);

    f.draw.setShape(Draw3DShape::Polyline);
    for (const auto& pt : {std::pair{10.0, 60.0}, std::pair{60.0, 90.0}, std::pair{100.0, 60.0}}) {
        const ScreenPoint s = f.screen(pt.first, pt.second);
        f.draw.pointerPress(f.view(), s.x, s.y);
        f.draw.pointerRelease(f.view(), s.x, s.y);
    }
    f.draw.key(ToolKey::Enter);
    CHECK(f.doc.objects().size() == 2);
    CHECK(f.steps() == 2);

    f.draw.setShape(Draw3DShape::Ellipse);
    CHECK(f.drag(110, 20, 170, 50).tool.committed);
    f.draw.setShape(Draw3DShape::Polygon);
    CHECK(f.drag(150, 70, 180, 70).tool.committed);
    CHECK(f.doc.objects().size() == 4);
    CHECK(f.steps() == 4);
    REQUIRE(f.doc.undo());
    CHECK(f.doc.objects().size() == 3);
}

TEST_CASE("refused picks draw nothing", "[viewmodel][draw3d]") {
    Fixture f;
    f.draw.setShape(Draw3DShape::Rectangle);
    // Wrapped view.
    const ScreenPoint a = f.screen(20, 30), b = f.screen(80, 70);
    Draw3DResult r = f.draw.pointerPress(f.view(true), a.x, a.y);
    CHECK(r.refused);
    CHECK_FALSE(r.message.empty());
    f.draw.pointerRelease(f.view(true), b.x, b.y);
    CHECK(f.doc.objects().empty());
    CHECK_FALSE(f.draw.active());
    // Edge-on view.
    f.camera.viewFront();
    r = f.draw.pointerPress(f.view(), 300, 240);
    CHECK(r.refused);
    CHECK(f.steps() == 0);
    // No view.
    CHECK(f.draw.pointerPress(Draw3DView{}, 1, 1).refused);
}

TEST_CASE("drawingSurface gives an empty job a plane and planeVectorPaths lists drawn vectors", "[viewmodel][draw3d]") {
    Document doc(Job::singleSided(200.0, 100.0, 18.0, Units::Millimetres));
    const ModelSurface none = composeModelSurface(doc, doc.activeSheet());
    REQUIRE_FALSE(none.ok());
    const ModelSurface plane = drawingSurface(doc, none);
    REQUIRE(plane.ok());
    OrbitCamera cam;
    cam.viewTop();
    ModelRenderOptions opt;
    Draw3DController draw(&doc);
    draw.setShape(Draw3DShape::Rectangle);
    const auto s0 = projectModelPoint(plane, cam, opt, 40, 20, 0.0);
    const auto s1 = projectModelPoint(plane, cam, opt, 90, 60, 0.0);
    REQUIRE((s0 && s1));
    const Draw3DView v{&plane, &cam, &opt, false};
    draw.pointerPress(v, s0->x, s0->y);
    draw.pointerMove(v, s1->x, s1->y);
    CHECK(draw.pointerRelease(v, s1->x, s1->y).tool.committed);
    const std::vector<ModelPath> paths = planeVectorPaths(doc);
    REQUIRE(paths.size() == 1);
    CHECK(paths[0].runs.size() == 1);
    CHECK(paths[0].runs[0].front().z == 0.0);
}

namespace {

Box2 vectorBounds(const Document& doc) {
    REQUIRE(doc.objects().size() == 1);
    const auto* v = doc.objects().begin()->second.asVector();
    REQUIRE(v != nullptr);
    Box2 b = v->contours.front().bounds();
    for (const auto& c : v->contours) {
        b.expand(c.bounds());
    }
    return b;
}

void click(Fixture& f, double x, double y) {
    const ScreenPoint s = f.screen(x, y);
    f.draw.pointerMove(f.view(), s.x, s.y);
    f.draw.pointerPress(f.view(), s.x, s.y);
    f.draw.pointerRelease(f.view(), s.x, s.y);
}

}  // namespace

TEST_CASE("a selected vector is moved by dragging in the 3D view like on the 2D canvas", "[viewmodel][draw3d]") {
    Fixture f;
    f.draw.setShape(Draw3DShape::Rectangle);
    REQUIRE(f.drag(20.0, 30.0, 80.0, 70.0).tool.committed);
    f.draw.setShape(Draw3DShape::Select);
    CHECK(f.draw.editing());
    f.doc.selection().clear();
    click(f, 20.0, 50.0);  // on the left edge
    REQUIRE(f.doc.selection().size() == 1);
    const std::size_t before = f.doc.undoStack().count();
    // Drag the centre handle by (+30, +10) mm from a tilted view.
    f.camera.azimuthRad = 0.7;
    const Draw3DResult r = f.drag(50.0, 50.0, 80.0, 60.0);
    CHECK(r.refused == false);
    const Box2 b = vectorBounds(f.doc);
    CHECK(b.min.x == Approx(50.0).margin(1e-6));
    CHECK(b.min.y == Approx(40.0).margin(1e-6));
    CHECK(b.max.x == Approx(110.0).margin(1e-6));
    CHECK(b.max.y == Approx(80.0).margin(1e-6));
    CHECK(f.doc.undoStack().count() == before + 1);  // the whole drag is one undo step
    REQUIRE(f.doc.undo());
    const Box2 b0 = vectorBounds(f.doc);
    CHECK(b0.min.x == Approx(20.0).margin(1e-6));
    CHECK(b0.max.y == Approx(70.0).margin(1e-6));
}

TEST_CASE("a corner handle scales the selection in the 3D view", "[viewmodel][draw3d]") {
    Fixture f;
    f.draw.setShape(Draw3DShape::Rectangle);
    REQUIRE(f.drag(20.0, 30.0, 80.0, 70.0).tool.committed);
    f.draw.setShape(Draw3DShape::Select);
    f.doc.selection().clear();
    click(f, 20.0, 50.0);
    REQUIRE(f.doc.selection().size() == 1);
    const std::size_t before = f.doc.undoStack().count();
    f.drag(80.0, 70.0, 140.0, 110.0);  // top-right corner handle: width 60 -> 120, height 40 -> 80
    const Box2 b = vectorBounds(f.doc);
    CHECK((b.max.x - b.min.x) == Approx(120.0).margin(1e-6));
    CHECK((b.max.y - b.min.y) == Approx(80.0).margin(1e-6));
    CHECK(f.doc.undoStack().count() == before + 1);
}

TEST_CASE("node editing from the 3D view moves one node in one undo step; Escape reverts", "[viewmodel][draw3d]") {
    Fixture f;
    f.draw.setShape(Draw3DShape::Rectangle);
    REQUIRE(f.drag(20.0, 30.0, 80.0, 70.0).tool.committed);
    f.draw.setShape(Draw3DShape::NodeEdit);
    REQUIRE(f.draw.editing());
    f.doc.selection().clear();
    click(f, 20.0, 50.0);  // picks the object
    REQUIRE(f.doc.selection().size() == 1);
    const std::size_t before = f.doc.undoStack().count();
    f.drag(80.0, 70.0, 100.0, 90.0);
    const Box2 b = vectorBounds(f.doc);
    CHECK(b.max.x == Approx(100.0).margin(1e-6));
    CHECK(b.max.y == Approx(90.0).margin(1e-6));
    CHECK(b.min.x == Approx(20.0).margin(1e-6));
    CHECK(f.doc.undoStack().count() == before + 1);
    REQUIRE(f.doc.undo());
    CHECK(vectorBounds(f.doc).max.x == Approx(80.0).margin(1e-6));

    // Escape during a drag restores the geometry exactly.
    const ScreenPoint a = f.screen(80.0, 70.0), c = f.screen(120.0, 100.0);
    f.draw.pointerMove(f.view(), a.x, a.y);
    f.draw.pointerPress(f.view(), a.x, a.y);
    f.draw.pointerMove(f.view(), c.x, c.y);
    CHECK(vectorBounds(f.doc).max.x == Approx(120.0).margin(1e-6));
    f.draw.key(ToolKey::Escape);
    CHECK(vectorBounds(f.doc).max.x == Approx(80.0).margin(1e-6));
}

TEST_CASE("editing in the wrapped or edge-on 3D view is refused", "[viewmodel][draw3d]") {
    Fixture f;
    f.draw.setShape(Draw3DShape::Rectangle);
    REQUIRE(f.drag(20.0, 30.0, 80.0, 70.0).tool.committed);
    f.draw.setShape(Draw3DShape::Select);
    const ScreenPoint a = f.screen(50.0, 50.0);
    CHECK(f.draw.pointerPress(f.view(true), a.x, a.y).refused);
    f.camera.viewFront();
    CHECK(f.draw.pointerPress(f.view(), 300, 240).refused);
    CHECK(f.doc.objects().size() == 1);
}
