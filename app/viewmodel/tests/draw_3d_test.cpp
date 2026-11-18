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
