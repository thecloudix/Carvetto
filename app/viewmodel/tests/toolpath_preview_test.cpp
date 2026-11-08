// T-0223: the 2D toolpath preview model (move classification, polylines, visibility, draw list with
// culling and decimation, staleness, tool position) and the time readout.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "carvetto/viewmodel/time_readout.hpp"
#include "carvetto/viewmodel/toolpath_preview.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using toolpath::Move;
using toolpath::Vec3;

namespace {

ViewTransform viewOf(double w, double h, double scale, geometry::Vec2 origin) {
    ViewTransform v;
    v.setViewport(w, h);
    v.set(scale, origin);
    return v;
}

std::size_t pointsOf(const PreviewDrawList& list) {
    std::size_t n = 0;
    for (const PreviewBatch& b : list.batches) {
        n += b.points.size();
    }
    return n;
}

const PreviewBatch* batchOf(const PreviewDrawList& list, MoveClass cls) {
    for (const PreviewBatch& b : list.batches) {
        if (b.style.cls == cls) {
            return &b;
        }
    }
    return nullptr;
}

double nowSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

TEST_CASE("preview: moves are classified", "[preview]") {
    const Vec3 a{0, 0, 5};
    CHECK(classify(Move::rapid({10, 0, 5}), a) == MoveClass::Rapid);
    CHECK(classify(Move::feed({10, 0, 5}), a) == MoveClass::Feed);
    CHECK(classify(Move::plunge({0, 0, -1}), a) == MoveClass::Plunge);
    CHECK(classify(Move::retract({0, 0, 5}), {0, 0, -1}) == MoveClass::Retract);
    CHECK(classify(Move::feed({10, 0, 3}), a) == MoveClass::Ramp);   // sloped feed
    CHECK(classify(Move::feed({0, 0, 3}), a) == MoveClass::Feed);    // vertical feed has no XY travel
    CHECK(classify(Move::arcCW({10, 0, 5}, {5, 0}), {0, 0, 5}) == MoveClass::Feed);
    CHECK(classify(Move::arcCCW({10, 0, 4}, {5, 0}), {0, 0, 5}) == MoveClass::Ramp);  // helix
    CHECK_FALSE(classify(Move::dwell(a, 1.0), a).has_value());
    // Every class has its own colour key and a name.
    for (std::size_t i = 0; i < kMoveClassCount; ++i) {
        for (std::size_t j = i + 1; j < kMoveClassCount; ++j) {
            CHECK_FALSE(colourOf(static_cast<MoveClass>(i)) == colourOf(static_cast<MoveClass>(j)));
        }
        CHECK(std::string(nameOf(static_cast<MoveClass>(i))).size() > 2);
    }
}

TEST_CASE("preview: polylines per class, arcs, dots", "[preview]") {
    std::vector<Move> moves;
    moves.push_back(Move::rapid({0, 0, 10}));            // 0: the start
    moves.push_back(Move::rapid({20, 0, 10}));           // 1
    moves.push_back(Move::plunge({20, 0, -1}));          // 2: vertical -> dot
    moves.push_back(Move::feed({40, 0, -1}));            // 3
    moves.push_back(Move::feed({40, 20, -1}));           // 4: same class continues the polyline
    moves.push_back(Move::arcCCW({20, 20, -1}, {30, 20}));  // 5: half circle r=10
    moves.push_back(Move::dwell({20, 20, -1}, 2.0));     // 6
    moves.push_back(Move::retract({20, 20, 10}));        // 7: dot
    const auto geo = buildPreviewGeometry(moves);
    REQUIRE(geo != nullptr);
    CHECK(geo->movesByClass[static_cast<std::size_t>(MoveClass::Rapid)] == 2);
    CHECK(geo->movesByClass[static_cast<std::size_t>(MoveClass::Feed)] == 3);
    // chunks: rapid, plunge dot, feed(+arc), retract dot
    REQUIRE(geo->chunks.size() == 4);
    CHECK(geo->chunks[0].cls == MoveClass::Rapid);
    CHECK(geo->chunks[0].points.size() == 2);
    CHECK(geo->chunks[1].cls == MoveClass::Plunge);
    CHECK(geo->chunks[1].points.size() == 1);
    const PreviewChunk& feed = geo->chunks[2];
    CHECK(feed.cls == MoveClass::Feed);
    CHECK(feed.firstMove == 3);
    CHECK(feed.lastMove == 5);
    // Start (20,0) is not repeated: (20,0),(40,0),(40,20), then the arc points to (20,20).
    CHECK(feed.points.front().x == 20.0);
    CHECK(feed.points[2].y == 20.0);
    CHECK(feed.points.back().x == 20.0);
    CHECK(feed.points.back().y == 20.0);
    CHECK(feed.points.size() > 10);
    // The arc bulges to x = 40 on the right? No: CCW from (40,20) about (30,20) goes through (30,30).
    CHECK(feed.bounds.max.y == Catch::Approx(30.0).margin(1e-9));
    for (std::size_t i = 3; i < feed.points.size(); ++i) {
        CHECK(std::hypot(feed.points[i].x - 30.0, feed.points[i].y - 20.0) == Catch::Approx(10.0).margin(1e-9));
    }
    CHECK(geo->chunks[3].cls == MoveClass::Retract);
    CHECK(geo->bounds.max.y == Catch::Approx(30.0).margin(1e-9));
}

TEST_CASE("preview: full circle, CW arc and hostile moves", "[preview]") {
    std::vector<Move> moves;
    moves.push_back(Move::rapid({10, 0, 0}));
    moves.push_back(Move::arcCW({10, 0, 0}, {0, 0}));  // full circle, clockwise
    moves.push_back(Move::feed({std::numeric_limits<double>::quiet_NaN(), 0, 0}));  // skipped
    moves.push_back(Move::arcCCW({10, 0, 0}, {-1e9, 0}));  // huge radius: bounded points
    const auto geo = buildPreviewGeometry(moves);
    REQUIRE(geo->chunks.size() >= 1);
    const PreviewChunk& c = geo->chunks.front();
    CHECK(c.points.size() > 8);
    CHECK(c.bounds.min.x == Catch::Approx(-10.0).margin(0.02));
    CHECK(c.bounds.max.y == Catch::Approx(10.0).margin(0.02));
    // clockwise from (10,0): the second point has negative y
    CHECK(c.points[1].y < 0.0);
    for (const PreviewChunk& ch : geo->chunks) {
        CHECK(ch.points.size() <= kPreviewMaxArcPoints + kPreviewChunkPoints);
    }
    CHECK(buildPreviewGeometry({})->chunks.empty());
}

TEST_CASE("preview: long runs are split into chunks", "[preview]") {
    std::vector<Move> moves;
    moves.push_back(Move::rapid({0, 0, 0}));
    for (int i = 1; i <= 1000; ++i) {
        moves.push_back(Move::feed({static_cast<double>(i), 0, 0}));
    }
    const auto geo = buildPreviewGeometry(moves);
    CHECK(geo->chunks.size() >= 1000 / kPreviewChunkPoints);
    std::size_t covered = 0;
    for (const PreviewChunk& c : geo->chunks) {
        CHECK(c.points.size() <= kPreviewChunkPoints + 1);
        covered += c.lastMove - c.firstMove + 1;
    }
    CHECK(covered == 1000);
}

namespace {

struct PreviewFixture {
    document::Document doc;
    tooling::ToolDatabase db{cliToolDatabase()};
    ToolpathSession session{&doc};
    PreviewModel model;
    document::ToolpathId id;
    document::ObjectId square30;

    PreviewFixture() {
        session.setToolDatabase(db);
        square30 = addVector(doc, square(50, 50, 30));
        addVector(doc, square(150, 50, 30));  // a second contour: rapid moves between them
        const ToolpathOutcome out = session.add(profileDefinition(db, 3.0));
        REQUIRE(out.ok);
        id = out.ids.front();
        model.sync(session);
        model.setEnabled(true);
    }
};

}  // namespace

TEST_CASE("preview: scene follows the session; staleness and visibility", "[preview]") {
    PreviewFixture f;
    REQUIRE(f.model.scene().toolpaths.size() == 1);
    const PreviewToolpath& tp = f.model.scene().toolpaths.front();
    CHECK(tp.computed);
    CHECK_FALSE(tp.stale);
    CHECK(tp.moveCount == f.doc.toolpaths().front().moveCount());
    CHECK(f.model.scene().totalMoves == tp.moveCount);
    CHECK(f.model.scene().bounds.contains(geometry::Vec2{65, 65}));

    const ViewTransform view = viewOf(800, 600, 4.0, {-100, 500});
    PreviewDrawList list = f.model.draw(view);
    CHECK(pointsOf(list) > 4);
    CHECK(list.stats.chunksDrawn > 0);
    CHECK(batchOf(list, MoveClass::Feed) != nullptr);

    // Same move list: the geometry object is reused (no rebuild).
    const auto before = tp.geometry;
    const std::uint64_t rev = f.model.revision();
    f.model.sync(f.session);
    CHECK(f.model.scene().toolpaths.front().geometry == before);
    CHECK(f.model.revision() == rev);

    // Visibility per toolpath.
    f.model.setVisible(f.id, false);
    CHECK(f.model.draw(view).batches.empty());
    f.model.setVisible(f.id, true);
    CHECK_FALSE(f.model.draw(view).batches.empty());
    // Selected only.
    f.model.setSelectedOnly(true);
    CHECK(f.model.draw(view).batches.empty());
    f.model.setSelected(f.id);
    list = f.model.draw(view);
    REQUIRE_FALSE(list.batches.empty());
    CHECK(list.batches.front().style.highlighted);
    f.model.setSelectedOnly(false);
    // Preview off draws nothing.
    f.model.setEnabled(false);
    CHECK(f.model.draw(view).batches.empty());
    f.model.setEnabled(true);

    // Editing the vector makes the toolpath stale: still drawn, flagged.
    REQUIRE(f.doc.execute(std::make_unique<document::TransformObjectsCommand>(
        std::vector<document::ObjectId>{f.square30}, geometry::Transform2::translation({5, 0}))));
    f.session.invalidate();
    f.model.sync(f.session);
    CHECK(f.model.scene().toolpaths.front().stale);
    CHECK(f.model.staleDrawn() == 1);
    list = f.model.draw(view);
    REQUIRE_FALSE(list.batches.empty());
    CHECK(list.batches.front().style.stale);
    // Recalculating replaces the moves: new geometry, no longer stale.
    REQUIRE(f.session.recalculate(f.id).ok);
    f.model.sync(f.session);
    CHECK_FALSE(f.model.scene().toolpaths.front().stale);
    CHECK(f.model.scene().toolpaths.front().geometry != before);

    // Disabled toolpaths are not drawn nor counted.
    f.session.setEnabled(f.id, false);
    f.model.sync(f.session);
    CHECK(f.model.draw(view).batches.empty());
    CHECK(f.model.scene().totalMoves == 0);
    CHECK_FALSE(f.model.positionAfter(3).has_value());

    // A different document: nothing of the old scene remains.
    f.model.clear();
    CHECK(f.model.scene().toolpaths.empty());
}

TEST_CASE("preview: culling, decimation and played/pending split", "[preview]") {
    PreviewFixture f;
    // Zoomed far away from the toolpath: everything is culled.
    PreviewDrawList list = f.model.draw(viewOf(400, 300, 4.0, {-5000, 5000}));
    CHECK(list.batches.empty());
    CHECK(list.stats.chunksTotal > 0);
    CHECK(list.stats.chunksDrawn == 0);
    // Zoomed far out: points collapse (decimation) but the shape stays.
    const ViewTransform far = viewOf(400, 300, 0.05, {0, 300});
    list = f.model.draw(far);
    CHECK(list.stats.pointsOut <= list.stats.pointsIn);
    CHECK(pointsOf(list) >= 2);
    // Played / pending split: ranges of the job-wide move list, styled by the option.
    const ViewTransform view = viewOf(800, 600, 4.0, {-100, 500});
    PreviewDrawOptions opt;
    opt.played = false;
    PreviewDrawList all = f.model.draw(view, opt);
    REQUIRE_FALSE(all.batches.empty());
    for (const PreviewBatch& b : all.batches) {
        CHECK_FALSE(b.style.played);
    }
    const std::size_t half = f.model.scene().totalMoves / 2;
    opt = PreviewDrawOptions{};
    opt.toMove = half;
    const PreviewDrawList first = f.model.draw(view, opt);
    opt.fromMove = half;
    opt.toMove = static_cast<std::size_t>(-1);
    const PreviewDrawList second = f.model.draw(view, opt);
    CHECK(first.stats.chunksDrawn + second.stats.chunksDrawn == all.stats.chunksDrawn);
    CHECK(first.stats.chunksDrawn > 0);
    CHECK(second.stats.chunksDrawn > 0);
    opt.fromMove = f.model.scene().totalMoves + 10;
    CHECK(f.model.draw(view, opt).batches.empty());
    // Hiding a class.
    opt = PreviewDrawOptions{};
    opt.classVisible[static_cast<std::size_t>(MoveClass::Feed)] = false;
    CHECK(batchOf(f.model.draw(view, opt), MoveClass::Feed) == nullptr);
}

TEST_CASE("preview: tool position after a move", "[preview]") {
    PreviewFixture f;
    const auto& moves = *f.doc.toolpaths().front().moves;
    const auto first = f.model.positionAfter(0);
    REQUIRE(first.has_value());
    CHECK(first->x == moves.front().to.x);
    const auto last = f.model.positionAfter(moves.size());
    REQUIRE(last.has_value());
    CHECK(last->x == moves.back().to.x);
    CHECK(last->y == moves.back().to.y);
    const auto beyond = f.model.positionAfter(moves.size() + 100);
    CHECK(beyond->x == last->x);
    const auto mid = f.model.positionAfter(5);
    CHECK(mid->x == moves[4].to.x);
}

TEST_CASE("preview: 1M-move toolpath builds and paints within bounds", "[preview][perf]") {
    // A raster of 1000 rows x 1000 feed moves over 300 x 300 mm, built as a shared move list in a
    // record of a document so the model syncs it exactly like a real toolpath.
    std::vector<Move> moves;
    moves.reserve(1'002'000);
    moves.push_back(Move::rapid({0, 0, 5}));
    for (int row = 0; row < 1000; ++row) {
        const double y = row * 0.3;
        const bool fwd = row % 2 == 0;
        for (int i = 0; i < 1000; ++i) {
            const double x = (fwd ? i + 1 : 999 - i) * 0.3;
            moves.push_back(Move::feed({x, y, -1}));
        }
        moves.push_back(Move::feed({fwd ? 300.0 : 0.0, y + 0.3, -1}));
    }
    const double t0 = nowSeconds();
    const auto geo = buildPreviewGeometry(moves);
    const double build = nowSeconds() - t0;

    document::Document doc;
    ToolpathSession session(&doc);
    document::ToolpathRecord rec;
    rec.name = "big";
    rec.tool.diameter = 3;
    rec.tool.feedRate = 1000;
    rec.tool.plungeRate = 300;
    rec.moves = document::shareMoves(std::move(moves));
    REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(rec)));
    PreviewModel model;
    model.setEnabled(true);
    const double s0 = nowSeconds();
    model.sync(session);
    const double sync = nowSeconds() - s0;
    REQUIRE(model.scene().totalMoves > 1'000'000);

    const ViewTransform whole = viewOf(1200, 900, 3.0, {50, 900});
    const ViewTransform zoomed = viewOf(1200, 900, 60.0, {-3000, 6000});
    const double d0 = nowSeconds();
    const PreviewDrawList a = model.draw(whole);
    const double drawWhole = nowSeconds() - d0;
    const double d1 = nowSeconds();
    const PreviewDrawList b = model.draw(zoomed);
    const double drawZoom = nowSeconds() - d1;
    std::printf("preview perf: %zu moves, geometry %.3f s, sync %.3f s, draw whole %.4f s (%zu -> %zu pts), "
                "draw zoomed %.4f s (%zu -> %zu pts)\n",
                model.scene().totalMoves, build, sync, drawWhole, a.stats.pointsIn, a.stats.pointsOut, drawZoom,
                b.stats.pointsIn, b.stats.pointsOut);
    // Output is bounded by the screen (a few pixels per row), not by the move count.
    CHECK(a.stats.pointsOut < a.stats.pointsIn * 6 / 10);
    CHECK(b.stats.chunksDrawn < a.stats.chunksTotal);
    CHECK(b.stats.pointsOut < a.stats.pointsIn);
#ifdef NDEBUG
    CHECK(drawWhole < 0.25);
    CHECK(drawZoom < 0.25);
#else
    CHECK(drawWhole < 5.0);
#endif
}

// --- time readout -------------------------------------------------------------------------------

TEST_CASE("time readout: durations", "[readout]") {
    CHECK(formatDuration(0.0) == "0 s");
    CHECK(formatDuration(12.4) == "12 s");
    CHECK(formatDuration(247.0) == "4 min 07 s");
    CHECK(formatDuration(3723.0) == "1 h 02 min 03 s");
    CHECK(formatDuration(-1.0) == "unknown");
    CHECK(formatDuration(std::numeric_limits<double>::infinity()) == "unknown");
    CHECK(formatDuration(std::numeric_limits<double>::quiet_NaN()) == "unknown");
    CHECK(formatDuration(1e300) == "unknown");
}

TEST_CASE("time readout: from the job summary", "[readout]") {
    JobSummary empty;
    CHECK(makeTimeReadout(empty).headline.find("no toolpaths") != std::string::npos);

    PreviewFixture f;
    // No rapid rate: the number is flagged as without rapids.
    TimeReadout r = makeTimeReadout(f.session.summary());
    CHECK(r.incomplete);
    CHECK(r.headline.find("without rapids") != std::string::npos);
    REQUIRE_FALSE(r.warnings.empty());
    REQUIRE(r.perToolpath.size() == 1);
    CHECK(r.perToolpath.front().find("moves)") != std::string::npos);
    // With a rapid rate: complete, and larger.
    const double without = f.session.summary().time.total();
    REQUIRE(f.session.setMachineSetup(MachineSetup{3000.0, std::numeric_limits<double>::infinity()}));
    const JobSummary with = f.session.summary();
    r = makeTimeReadout(with);
    CHECK_FALSE(r.incomplete);
    CHECK(r.headline.find("without") == std::string::npos);
    CHECK(with.time.total() > without);
    CHECK(r.headline == "Estimated machining time: " + formatDuration(with.time.total()));
    CHECK(r.breakdown.find("rapids " + formatDuration(with.time.rapid)) != std::string::npos);
    CHECK(r.warnings.empty());

    // Stale toolpath: warned about.
    REQUIRE(f.doc.execute(std::make_unique<document::TransformObjectsCommand>(
        std::vector<document::ObjectId>{f.square30}, geometry::Transform2::translation({5, 0}))));
    f.session.invalidate();
    r = makeTimeReadout(f.session.summary());
    bool warned = false;
    for (const std::string& w : r.warnings) {
        warned = warned || w.find("recalculation") != std::string::npos;
    }
    CHECK(warned);

    // A disabled toolpath leaves nothing to estimate.
    f.session.setEnabled(f.id, false);
    r = makeTimeReadout(f.session.summary());
    CHECK(r.headline.find("no computed, enabled") != std::string::npos);

    // Unknown time (feed rate 0) is reported, not shown as a number.
    JobSummary bad;
    bad.toolpaths = bad.computed = bad.included = 1;
    bad.time.feed = std::numeric_limits<double>::infinity();
    r = makeTimeReadout(bad);
    CHECK(r.headline == "Estimated machining time: unknown");
    CHECK(r.incomplete);
}
