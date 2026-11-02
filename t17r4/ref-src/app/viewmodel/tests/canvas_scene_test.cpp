#include "carvetto/viewmodel/canvas_scene.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <memory>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Vec2;
using carvetto::viewmodel::activeMaterialMm;
using carvetto::viewmodel::buildCanvasScene;
using carvetto::viewmodel::polygoniseForScreen;
using carvetto::viewmodel::ViewTransform;
using Catch::Matchers::WithinAbs;

namespace {

ViewTransform fittedView(const Document& doc) {
    ViewTransform view;
    view.setViewport(1000.0, 800.0);
    view.zoomToFit(activeMaterialMm(doc));
    return view;
}

ObjectId addCircle(Document& doc, Vec2 c, double r) {
    auto cmd = std::make_unique<AddObjectCommand>(
        Object::vector(Contour::circle(c, r), doc.activeLayer(), doc.activeSheet(), CircleParams{c, r}));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

}  // namespace

TEST_CASE("scene shows the material rectangle and datum in screen pixels", "[viewmodel][scene]") {
    Document doc(Job::singleSided(10.0, 10.0, 0.5, Units::Inches, Datum::Centre));
    const ViewTransform view = fittedView(doc);
    const auto scene = buildCanvasScene(doc, view);
    CHECK_THAT(scene.materialPx.height(), WithinAbs(720.0, 1e-9));
    CHECK_THAT(scene.materialPx.min.x, WithinAbs(140.0, 1e-9));
    // Centre datum at document (0,0) lands in the middle of the viewport.
    CHECK_THAT(scene.datumPx.x, WithinAbs(500.0, 1e-9));
    CHECK_THAT(scene.datumPx.y, WithinAbs(400.0, 1e-9));
    CHECK(scene.paths.empty());
}

TEST_CASE("scene polygonises document contours and follows undo", "[viewmodel][scene]") {
    Document doc(Job::singleSided(254.0, 254.0, 18.0, Units::Millimetres));
    const ViewTransform view = fittedView(doc);
    const ObjectId id = addCircle(doc, {127.0, 127.0}, 50.0);
    auto scene = buildCanvasScene(doc, view);
    REQUIRE(scene.paths.size() == 1);
    const auto& path = scene.paths.front();
    CHECK(path.object == id);
    CHECK(path.closed);
    REQUIRE(path.pointsPx.size() > 16);
    const Vec2 centrePx = view.worldToScreen({127.0, 127.0});
    for (const Vec2& p : path.pointsPx) {
        CHECK_THAT((p - centrePx).length(), WithinAbs(50.0 * view.scale(), 1e-6));
    }
    REQUIRE(doc.undo());
    scene = buildCanvasScene(doc, view);
    CHECK(scene.paths.empty());
    REQUIRE(doc.redo());
    CHECK(buildCanvasScene(doc, view).paths.size() == 1);
}

TEST_CASE("screen polygonisation honours the pixel tolerance", "[viewmodel][scene]") {
    const Contour circle = Contour::circle({0.0, 0.0}, 100.0);
    for (const double scale : {0.05, 1.0, 20.0}) {
        ViewTransform view;
        view.setViewport(800.0, 600.0);
        view.set(scale, {400.0, 300.0});
        const auto pts = polygoniseForScreen(circle, view, 0.25);
        REQUIRE(pts.size() >= 3);
        const double rPx = 100.0 * scale;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2 a = pts[i];
            const Vec2 b = pts[(i + 1) % pts.size()];
            const Vec2 mid = (a + b) * 0.5;
            const double sagitta = rPx - (mid - Vec2{400.0, 300.0}).length();
            REQUIRE(sagitta <= 0.25 + 1e-9);
        }
    }
}

TEST_CASE("scene skips hidden layers, other sheets and off-screen objects", "[viewmodel][scene]") {
    Document doc(Job::singleSided(100.0, 100.0, 10.0, Units::Millimetres));
    ViewTransform view = fittedView(doc);
    addCircle(doc, {50.0, 50.0}, 10.0);
    addCircle(doc, {5000.0, 5000.0}, 10.0);  // far outside the view
    CHECK(buildCanvasScene(doc, view).paths.size() == 1);

    Layer hidden = doc.layers().front();
    hidden.visible = false;
    REQUIRE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(hidden)));
    CHECK(buildCanvasScene(doc, view).paths.empty());

    hidden.visible = true;
    hidden.colour = Colour{200, 10, 20};
    REQUIRE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(hidden)));
    auto scene = buildCanvasScene(doc, view);
    REQUIRE(scene.paths.size() == 1);
    CHECK(scene.paths.front().colour == Colour{200, 10, 20});

    auto addSheet = std::make_unique<AddSheetCommand>(Sheet::fromJob(doc.job(), "Sheet 2"), true);
    REQUIRE(doc.execute(std::move(addSheet)));
    CHECK(buildCanvasScene(doc, view).paths.empty());
}
