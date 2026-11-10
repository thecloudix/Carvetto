// T-0256: keep-out zones in the app: viewmodel commands, routed rapids in the preview and the job
// summary, canvas scene drawing.

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/keepout.hpp"
#include "carvetto/viewmodel/time_readout.hpp"
#include "carvetto/viewmodel/toolpath_preview.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using toolpath::Move;

namespace {

/// A toolpath record with hand-made moves: one rapid from the first point to the second across x = 50.
document::ToolpathId addRecord(document::Document& doc, std::vector<Move> moves, const std::string& name = "t") {
    document::ToolpathRecord rec;
    rec.name = name;
    rec.moves = document::shareMoves(std::move(moves));
    auto cmd = std::make_unique<document::AddToolpathCommand>(std::move(rec));
    const document::AddToolpathCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

std::vector<Move> crossing() {
    return {Move::rapid({0, 20, 10}), Move::rapid({100, 20, 10}), Move::plunge({100, 20, -1}),
            Move::feed({100, 30, -1})};
}

void selectOnly(document::Document& doc, document::ObjectId id) {
    doc.selection().assign({id});
}

}  // namespace

TEST_CASE("keep-out: create, clearance, clear are one undo step each", "[keepout][viewmodel]") {
    document::Document doc;
    CHECK_FALSE(canCreateKeepOutFromSelection(doc));
    CHECK_FALSE(createKeepOutFromSelection(doc).ok);

    const document::ObjectId a = addVector(doc, square(40, 10, 20));
    selectOnly(doc, a);
    CHECK(canCreateKeepOutFromSelection(doc));
    const std::size_t before = doc.undoStack().count();
    const KeepOutResult made = createKeepOutFromSelection(doc);
    REQUIRE(made.ok);
    CHECK(made.count == 1);
    CHECK(doc.undoStack().count() == before + 1);
    CHECK(doc.keepOutZones().size() == 1);

    CHECK(setKeepOutClearanceMm(doc, 3.0).ok);
    CHECK(doc.keepOutClearance() == 3.0);
    CHECK_FALSE(setKeepOutClearanceMm(doc, 3.0).ok);
    CHECK_FALSE(setKeepOutClearanceMm(doc, -1.0).ok);
    CHECK_FALSE(setKeepOutClearanceMm(doc, 1e9).ok);
    CHECK(doc.keepOutClearance() == 3.0);

    CHECK(setKeepOutVisible(doc, false));
    CHECK_FALSE(setKeepOutVisible(doc, false));
    CHECK(describeKeepOut(doc).find("hidden") != std::string::npos);

    const KeepOutResult cleared = clearKeepOutZones(doc);
    REQUIRE(cleared.ok);
    CHECK(cleared.count == 1);
    CHECK(doc.keepOutZones().empty());
    CHECK_FALSE(clearKeepOutZones(doc).ok);
    REQUIRE(doc.undo());
    CHECK(doc.keepOutZones().size() == 1);
    REQUIRE(doc.undo());  // clearance
    CHECK(doc.keepOutClearance() == 0.0);
    REQUIRE(doc.undo());  // create
    CHECK(doc.keepOutZones().empty());
}

TEST_CASE("keep-out: canvas scene draws zones and the halo, hidden zones not", "[keepout][viewmodel]") {
    document::Document doc;
    const document::ObjectId a = addVector(doc, square(40, 10, 20));
    selectOnly(doc, a);
    REQUIRE(createKeepOutFromSelection(doc).ok);
    ViewTransform view;
    view.setViewport(400, 300);
    view.zoomToFit(activeMaterialMm(doc), 0.05);

    CanvasScene scene = buildCanvasScene(doc, view);
    CHECK(scene.keepOutZones.size() == 1);
    CHECK(scene.keepOutHalo.empty());  // clearance 0

    REQUIRE(setKeepOutClearanceMm(doc, 4.0).ok);
    KeepOutHaloCache cache;
    SceneOptions options;
    options.haloCache = &cache;
    scene = buildCanvasScene(doc, view, options);
    REQUIRE(scene.keepOutHalo.size() == 1);
    // The halo encloses the zone: it is larger on screen.
    double zoneW = 0, haloW = 0;
    auto width = [](const DrawPath& p) {
        double lo = 1e18, hi = -1e18;
        for (const geometry::Vec2& q : p.pointsPx) {
            lo = std::min(lo, q.x);
            hi = std::max(hi, q.x);
        }
        return hi - lo;
    };
    zoneW = width(scene.keepOutZones[0]);
    haloW = width(scene.keepOutHalo[0]);
    CHECK(haloW > zoneW + 2.0 * 4.0 * view.scale() - 1.0);

    REQUIRE(setKeepOutVisible(doc, false));
    scene = buildCanvasScene(doc, view, options);
    CHECK(scene.keepOutZones.empty());
    CHECK(scene.keepOutHalo.empty());
}

TEST_CASE("keep-out: preview and summary use the routed rapids", "[keepout][viewmodel]") {
    document::Document doc;
    addRecord(doc, crossing());
    ToolpathSession session(&doc);
    MachineSetup setup;
    setup.rapidRateMmPerMin = 6000.0;
    REQUIRE(session.setMachineSetup(setup));

    PreviewModel model;
    model.setEnabled(true);
    model.sync(session);
    const JobSummary plain = session.summary();
    CHECK(plain.keepOutRouted == 0);
    REQUIRE(model.scene().toolpaths.size() == 1);
    CHECK(model.scene().toolpaths[0].routedMoves == model.scene().toolpaths[0].moves);
    const std::size_t plainPoints = model.scene().toolpaths[0].geometry->points;

    const document::ObjectId z = addVector(doc, square(40, 10, 20));
    selectOnly(doc, z);
    REQUIRE(createKeepOutFromSelection(doc).ok);
    session.invalidate();
    model.sync(session);
    const PreviewToolpath& tp = model.scene().toolpaths[0];
    CHECK(tp.routing.routed == 1);
    CHECK(model.scene().routing.routed == 1);
    CHECK(tp.routedMoves != tp.moves);
    CHECK(tp.geometry->points > plainPoints);
    CHECK(model.scene().totalMoves == 4);  // playback still indexes the record's moves

    // No drawn rapid point lies inside the zone's interior.
    ViewTransform view;
    view.setViewport(800, 600);
    view.zoomToFit(geometry::Box2{{-10, -10}, {110, 60}}, 0.0);
    const PreviewDrawList list = model.draw(view);
    for (const PreviewBatch& b : list.batches) {
        if (b.style.cls != MoveClass::Rapid) {
            continue;
        }
        std::size_t at = 0;
        for (const std::uint32_t run : b.runLengths) {
            for (std::size_t i = 0; i + 1 < run; ++i) {
                const geometry::Vec2 p = view.screenToWorld(b.points[at + i]);
                const geometry::Vec2 q = view.screenToWorld(b.points[at + i + 1]);
                const geometry::Vec2 m{(p.x + q.x) / 2, (p.y + q.y) / 2};
                CHECK_FALSE((m.x > 40.1 && m.x < 59.9 && m.y > 10.1 && m.y < 29.9));
            }
            at += run;
        }
    }

    const JobSummary routed = session.summary();
    CHECK(routed.keepOutRouted == 1);
    CHECK(routed.moves == plain.moves);
    CHECK(routed.time.rapid > plain.time.rapid);  // the detour is longer
    const TimeReadout readout = makeTimeReadout(routed);
    bool mentioned = false;
    for (const std::string& w : readout.warnings) {
        mentioned = mentioned || w.find("keep-out") != std::string::npos;
    }
    CHECK(mentioned);

    // Clear zones: back to the straight rapids.
    REQUIRE(clearKeepOutZones(doc).ok);
    session.invalidate();
    model.sync(session);
    CHECK(model.scene().toolpaths[0].routedMoves == model.scene().toolpaths[0].moves);
    CHECK(session.summary().time.rapid == plain.time.rapid);
}

TEST_CASE("keep-out: unroutable rapids are reported", "[keepout][viewmodel]") {
    document::Document doc;
    // The second point is inside the zone.
    addRecord(doc, {Move::rapid({0, 20, 10}), Move::rapid({50, 20, 10}), Move::feed({50, 25, 10})});
    const document::ObjectId z = addVector(doc, square(40, 10, 20));
    selectOnly(doc, z);
    REQUIRE(createKeepOutFromSelection(doc).ok);
    ToolpathSession session(&doc);
    const JobSummary s = session.summary();
    CHECK(s.keepOutUnroutable == 1);
    CHECK(s.keepOutRouted == 0);
    const TimeReadout r = makeTimeReadout(s);
    bool mentioned = false;
    for (const std::string& w : r.warnings) {
        mentioned = mentioned || w.find("cannot be routed") != std::string::npos;
    }
    CHECK(mentioned);
}

TEST_CASE("keep-out: routing is cached and follows the zones", "[keepout][viewmodel]") {
    document::Document doc;
    const document::ToolpathId id = addRecord(doc, crossing());
    const document::ObjectId z = addVector(doc, square(40, 10, 20));
    selectOnly(doc, z);
    REQUIRE(createKeepOutFromSelection(doc).ok);
    KeepOutRouter router;
    const RoutedJob& first = router.route(doc, {id});
    REQUIRE(first.toolpaths.size() == 1);
    const document::SharedMoves routed = first.toolpaths[0].routed;
    CHECK(first.toolpaths[0].changed());
    CHECK(router.route(doc, {id}).toolpaths[0].routed == routed);  // cache hit
    REQUIRE(setKeepOutClearanceMm(doc, 5.0).ok);
    CHECK(router.route(doc, {id}).toolpaths[0].routed != routed);   // new zone set
}
