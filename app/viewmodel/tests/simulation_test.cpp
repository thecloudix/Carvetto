// T-0223: the simulation setup (stock, tools, bounds), the off-thread controller (frames equal a
// direct core run, seek forward/back, cancel, supersede, failure), staleness against the document
// (edit / recalculate / replace / destroy while running: must be ASan clean), the playback model and
// the relief raster.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/viewmodel/playback.hpp"
#include "carvetto/viewmodel/relief_render.hpp"
#include "carvetto/viewmodel/simulation_controller.hpp"
#include "carvetto/viewmodel/simulation_setup.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using toolpath::Move;

namespace {

struct SimFixture {
    std::unique_ptr<document::Document> doc = std::make_unique<document::Document>();
    tooling::ToolDatabase db{cliToolDatabase()};
    ToolpathSession session{doc.get()};
    document::ObjectId vec;
    document::ToolpathId id;

    SimFixture() {
        session.setToolDatabase(db);
        vec = addVector(*doc, square(50, 50, 30));
        const ToolpathOutcome out = session.add(profileDefinition(db, 3.0));
        REQUIRE(out.ok);
        id = out.ids.front();
    }
};

/// A zig-zag of `rows` rows over a 60 x 60 mm area at depth z, as a toolpath record whose tool is
/// not in the database (so the shape is approximated by a flat cutter).
document::ToolpathRecord zigzagRecord(int rows, double z) {
    std::vector<Move> moves;
    moves.push_back(Move::rapid({0, 0, 5}));
    for (int r = 0; r < rows; ++r) {
        const double y = 60.0 * r / rows;
        moves.push_back(Move::feed({r % 2 == 0 ? 60.0 : 0.0, y, z}));
        moves.push_back(Move::feed({r % 2 == 0 ? 60.0 : 0.0, y + 60.0 / rows, z}));
    }
    document::ToolpathRecord rec;
    rec.name = "zigzag";
    rec.tool.diameter = 3.0;
    rec.tool.feedRate = 1000.0;
    rec.tool.plungeRate = 300.0;
    rec.moves = document::shareMoves(std::move(moves));
    return rec;
}

template <typename Pred>
bool waitFor(Pred pred, int timeoutMs = 120000) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < end) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

bool waitReadyAt(SimulationController& c, std::size_t move, int timeoutMs = 120000) {
    return waitFor(
        [&] {
            const auto f = c.frame();
            return f && f->move == move && c.progress().state == SimState::Ready;
        },
        timeoutMs);
}

/// The raster the core produces directly for `setup` stopped after `move` moves.
Raster directRelief(const SimulationSetup& setup, std::size_t move, const ReliefOptions& options = {}) {
    simulate::Simulator sim(setup.stock, setup.options);
    REQUIRE(sim.ok());
    for (std::size_t i = 0; i < setup.toolpaths.size(); ++i) {
        toolpath::Toolpath path;
        path.name = setup.toolpaths[i].name;
        path.tool = setup.toolpaths[i].tool;
        path.moves = *setup.toolpaths[i].moves;
        REQUIRE(sim.addToolpath(path, setup.toolpaths[i].shape, static_cast<std::uint32_t>(i + 1)));
    }
    REQUIRE(sim.advanceToMove(move));
    return renderRelief(sim.heightfield(), options);
}

}  // namespace

// --- setup --------------------------------------------------------------------------------------

TEST_CASE("simulation setup: stock, tools and notes", "[simulation]") {
    SimFixture f;
    const SimulationSetup s = makeSimulationSetup(f.session);
    REQUIRE(s.ok);
    CHECK(s.toolpaths.size() == 1);
    CHECK(s.toolpaths.front().id == f.id);
    CHECK(s.toolpaths.front().shape.valid());
    CHECK_FALSE(s.toolpaths.front().shapeApproximated);
    CHECK(s.totalMoves == f.doc->toolpaths().front().moveCount());
    CHECK(s.stock.width == 300.0);
    CHECK(s.stock.height == 300.0);
    CHECK(s.stock.thickness == 18.0);
    CHECK(s.stock.topZ == 0.0);
    CHECK(s.stock.cellSize == simulate::cellSizeFor(simulate::Quality::Medium));
    CHECK_FALSE(s.coarsened);
    CHECK(s.rapidRateAssumed);  // no machine setup
    CHECK(s.options.rapidRate == kDefaultPlaybackRapidRate);
    CHECK(simulate::checkStock(s.stock).ok());

    // The machine's rapid rate wins; an explicit request wins over both.
    REQUIRE(f.session.setMachineSetup(MachineSetup{4000.0, 500.0}));
    SimulationSetup s2 = makeSimulationSetup(f.session);
    CHECK_FALSE(s2.rapidRateAssumed);
    CHECK(s2.options.rapidRate == 4000.0);
    CHECK(s2.options.acceleration == 500.0);
    SimulationRequest req;
    req.rapidRateMmPerMin = 1234.0;
    CHECK(makeSimulationSetup(f.session, req).options.rapidRate == 1234.0);

    // Z zero at the machine bed: the top of the stock is at the thickness.
    document::Job job = f.doc->job();
    job.zZero = document::ZZero::MachineBed;
    f.doc->setJob(job);
    CHECK(makeSimulationSetup(f.session).stock.topZ == 18.0);
}

TEST_CASE("simulation setup: quality is bounded by the cell cap", "[simulation]") {
    document::Document doc(document::Job::singleSided(3000, 3000, 20, document::Units::Millimetres));
    ToolpathSession session(&doc);
    REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(zigzagRecord(10, -1.0))));
    SimulationRequest req;
    req.quality = simulate::Quality::Fine;
    const SimulationSetup s = makeSimulationSetup(session, req);
    REQUIRE(s.ok);
    CHECK(s.coarsened);
    CHECK(s.stock.cellSize > simulate::cellSizeFor(simulate::Quality::Fine));
    CHECK(simulate::checkStock(s.stock).cells() <= kSimulationMaxCells);
    bool noted = false;
    for (const std::string& n : s.notes) {
        noted = noted || n.find("instead of") != std::string::npos;
    }
    CHECK(noted);
    // The tool is not in the database: approximated, with a note.
    CHECK(s.toolpaths.front().shapeApproximated);
    CHECK(s.toolpaths.front().shape.valid());
}

TEST_CASE("simulation setup: refusals", "[simulation]") {
    {
        document::Document doc;
        ToolpathSession session(&doc);
        const SimulationSetup s = makeSimulationSetup(session);
        CHECK_FALSE(s.ok);
        CHECK(s.problem.find("no computed") != std::string::npos);
    }
    {
        ToolpathSession none(nullptr);
        CHECK_FALSE(makeSimulationSetup(none).ok);
    }
    {
        document::Document doc(document::Job::rotary(100, 20, document::Units::Millimetres));
        ToolpathSession session(&doc);
        const SimulationSetup s = makeSimulationSetup(session);
        CHECK_FALSE(s.ok);
        CHECK(s.problem.find("Rotary") != std::string::npos);
    }
    {
        SimFixture f;
        SimulationRequest req;
        req.maxMoves = 3;
        const SimulationSetup s = makeSimulationSetup(f.session, req);
        CHECK_FALSE(s.ok);
        CHECK(s.problem.find("Too many moves") != std::string::npos);
        SimulationController c;
        CHECK_FALSE(c.start(s));
        CHECK_FALSE(c.active());
    }
    {
        // A disabled toolpath is not simulated.
        SimFixture f;
        f.session.setEnabled(f.id, false);
        CHECK_FALSE(makeSimulationSetup(f.session).ok);
    }
}

TEST_CASE("simulation setup: currency against the document", "[simulation]") {
    SimFixture f;
    const SimulationSetup s = makeSimulationSetup(f.session);
    REQUIRE(s.ok);
    CHECK(checkSimulationCurrent(s, f.session).current);

    // The vector moved: the toolpath is stale, the simulation shows its old moves.
    REQUIRE(f.doc->execute(std::make_unique<document::TransformObjectsCommand>(
        std::vector<document::ObjectId>{f.vec}, geometry::Transform2::translation({5, 0}))));
    f.session.invalidate();
    SimulationValidity v = checkSimulationCurrent(s, f.session);
    CHECK_FALSE(v.current);
    CHECK(v.reason.find("recalculation") != std::string::npos);
    // Recalculated: new moves, the old simulation no longer matches.
    REQUIRE(f.session.recalculate(f.id).ok);
    v = checkSimulationCurrent(s, f.session);
    CHECK_FALSE(v.current);
    CHECK(v.reason.find("changed") != std::string::npos);
    CHECK(checkSimulationCurrent(makeSimulationSetup(f.session), f.session).current);

    // Undo returns to the old moves: identity matches the first simulation again only if the very
    // same move list comes back (it does: the undo history shares it).
    const SimulationSetup now = makeSimulationSetup(f.session);
    REQUIRE(f.doc->undo());  // the recalculation
    CHECK_FALSE(checkSimulationCurrent(now, f.session).current);
    REQUIRE(f.doc->undo());  // the move of the vector: the toolpath is current again
    f.session.invalidate();
    // Disabling, material change and a replaced document.
    const SimulationSetup again = makeSimulationSetup(f.session);
    REQUIRE(again.ok);
    f.session.setEnabled(f.id, false);
    CHECK_FALSE(checkSimulationCurrent(again, f.session).current);
    f.session.setEnabled(f.id, true);
    CHECK(checkSimulationCurrent(again, f.session).current);
    document::Job job = f.doc->job();
    job.sizeMm.z = 25.0;
    f.doc->setJob(job);
    v = checkSimulationCurrent(again, f.session);
    CHECK_FALSE(v.current);
    CHECK(v.reason.find("material") != std::string::npos);

    // A new document with a same-id toolpath is never mistaken for the old one.
    SimFixture other;
    CHECK(other.id == f.id);
    const SimulationSetup mine = makeSimulationSetup(f.session);
    CHECK_FALSE(checkSimulationCurrent(mine, other.session).current);
    other.session.setDocument(nullptr);
    CHECK_FALSE(checkSimulationCurrent(mine, other.session).current);
}

// --- controller ---------------------------------------------------------------------------------

TEST_CASE("simulation controller: frames equal a direct core run", "[simulation]") {
    SimFixture f;
    SimulationRequest req;
    req.quality = simulate::Quality::Coarse;
    const SimulationSetup setup = makeSimulationSetup(f.session, req);
    REQUIRE(setup.ok);
    SimulationController c;
    CHECK(c.progress().state == SimState::Idle);
    REQUIRE(c.start(setup));
    REQUIRE(waitReadyAt(c, 0));
    const auto meta = c.meta();
    REQUIRE(meta);
    CHECK(meta->moveCount == setup.totalMoves);
    CHECK(meta->moveEndTimes->size() == setup.totalMoves);
    CHECK(meta->totalSeconds > 0.0);
    CHECK(meta->cellSize == setup.stock.cellSize);
    CHECK(meta->heightfieldBytes > 0);
    auto first = c.frame();
    CHECK(first->cutCells == 0);
    CHECK(first->relief->pixels == directRelief(setup, 0).pixels);

    // All the way.
    c.seek(setup.totalMoves);
    REQUIRE(waitReadyAt(c, setup.totalMoves));
    auto last = c.frame();
    CHECK(last->cutCells > 0);
    CHECK(last->seconds == Catch::Approx(meta->totalSeconds));
    CHECK(last->relief->pixels == directRelief(setup, setup.totalMoves).pixels);
    CHECK(last->relief->pixels != first->relief->pixels);
    CHECK(last->serial > first->serial);
    CHECK(c.progress().fraction() == 1.0);

    // Backwards (reset + replay) and forwards again, to a middle move.
    const std::size_t mid = setup.totalMoves / 2;
    c.seek(mid);
    REQUIRE(waitReadyAt(c, mid));
    CHECK(c.frame()->relief->pixels == directRelief(setup, mid).pixels);
    c.seek(mid + 7);
    REQUIRE(waitReadyAt(c, mid + 7));
    CHECK(c.frame()->relief->pixels == directRelief(setup, mid + 7).pixels);
    // Clamped past the end.
    c.seek(setup.totalMoves + 1000);
    REQUIRE(waitReadyAt(c, setup.totalMoves));

    // Another colouring re-shades without re-simulating.
    ReliefOptions byToolpath;
    byToolpath.colourBy = ReliefOptions::ColourBy::Toolpath;
    c.setReliefOptions(byToolpath);
    REQUIRE(waitFor([&] { return c.frame()->serial > last->serial + 2 && c.progress().state == SimState::Ready; }));
    CHECK(c.frame()->relief->pixels == directRelief(setup, setup.totalMoves, byToolpath).pixels);
    CHECK(c.frame()->toolpathIds.size() == 1);
    CHECK(c.frame()->toolpathIds.front() == f.id.value);
}

TEST_CASE("simulation controller: cancel and superseded requests", "[simulation]") {
    document::Document doc;
    ToolpathSession session(&doc);
    REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(zigzagRecord(4000, -1.0))));
    SimulationRequest req;
    req.quality = simulate::Quality::Coarse;
    const SimulationSetup setup = makeSimulationSetup(session, req);
    REQUIRE(setup.ok);
    REQUIRE(setup.totalMoves > 8000);

    SimulationController c;
    REQUIRE(c.start(setup));
    REQUIRE(waitReadyAt(c, 0));
    c.setChunkDelayForTest(15);
    c.seek(setup.totalMoves);
    REQUIRE(waitFor([&] { return c.progress().movesDone > 0; }));
    CHECK(c.progress().state == SimState::Working);
    CHECK(c.progress().fraction() < 1.0);
    c.cancel();
    REQUIRE(waitFor([&] { return c.progress().state == SimState::Ready; }));
    const SimProgress p = c.progress();
    CHECK(p.cancelled);
    CHECK(p.movesDone < setup.totalMoves);
    CHECK(c.frame()->move == p.movesDone);
    // The stopped state is exactly a direct run to that move.
    CHECK(c.frame()->relief->pixels == directRelief(setup, p.movesDone).pixels);

    // The latest request wins: seek far, then back to a near one while it is working.
    const std::uint64_t serialBefore = c.frame()->serial;
    c.seek(setup.totalMoves);
    REQUIRE(waitFor([&] { return c.progress().state == SimState::Working; }));
    c.seek(p.movesDone + 3);
    c.setChunkDelayForTest(0);
    REQUIRE(waitReadyAt(c, p.movesDone + 3));
    CHECK_FALSE(c.progress().cancelled);
    // The abandoned run to the end published nothing: exactly one new frame.
    CHECK(c.frame()->serial == serialBefore + 1);

    // A cancel issued while the worker is still preparing applies to the run requested before it started.
    {
        SimulationController early;
        REQUIRE(early.start(setup));
        early.seek(setup.totalMoves);
        early.cancel();  // state is Preparing right after start()
        REQUIRE(waitFor([&] { return early.progress().state == SimState::Ready && early.frame(); }));
        REQUIRE(waitFor([&] { return early.progress().cancelled || early.frame()->move == setup.totalMoves; }));
        CHECK(early.progress().cancelled);
        CHECK(early.frame()->move < setup.totalMoves);
    }

    // Cancel while idle changes nothing.
    c.cancel();
    CHECK(c.progress().state == SimState::Ready);
    // Stop in the middle of work joins promptly and leaves no state.
    c.setChunkDelayForTest(15);
    c.seek(setup.totalMoves);
    c.stop();
    CHECK_FALSE(c.active());
    CHECK(c.frame() == nullptr);
    CHECK(c.progress().state == SimState::Idle);
}

TEST_CASE("simulation controller: failures are reported, not thrown", "[simulation]") {
    SimFixture f;
    const SimulationSetup setup = makeSimulationSetup(f.session);
    REQUIRE(setup.ok);
    SimulationController c;
    simulate::detail::failNextAllocation(1);
    REQUIRE(c.start(setup));
    REQUIRE(waitFor([&] { return c.progress().state == SimState::Failed; }));
    CHECK_FALSE(c.error().empty());
    c.seek(5);  // ignored
    CHECK(c.progress().state == SimState::Failed);
    // A new start recovers.
    REQUIRE(c.start(setup));
    REQUIRE(waitReadyAt(c, 0));
    CHECK(c.error().empty());
}

TEST_CASE("simulation controller: document edited, replaced or destroyed mid-run", "[simulation]") {
    SimFixture f;
    // Make the run long enough to overlap the edits.
    REQUIRE(f.doc->execute(std::make_unique<document::AddToolpathCommand>(zigzagRecord(3000, -2.0))));
    f.session.invalidate();
    SimulationRequest req;
    req.quality = simulate::Quality::Coarse;
    const SimulationSetup setup = makeSimulationSetup(f.session, req);
    REQUIRE(setup.ok);
    REQUIRE(setup.toolpaths.size() == 2);

    SimulationController c;
    REQUIRE(c.start(setup));
    REQUIRE(waitReadyAt(c, 0));
    c.setChunkDelayForTest(5);
    c.seek(setup.totalMoves);
    REQUIRE(waitFor([&] { return c.progress().movesDone > 0; }));

    // Edit the vector, recalculate the first toolpath (its move list is replaced and the old one
    // would be freed if the simulation did not share it), then undo and redo, all mid-run.
    REQUIRE(f.doc->execute(std::make_unique<document::TransformObjectsCommand>(
        std::vector<document::ObjectId>{f.vec}, geometry::Transform2::translation({3, 0}))));
    f.session.invalidate();
    CHECK_FALSE(checkSimulationCurrent(setup, f.session).current);
    REQUIRE(f.session.recalculate(f.id).ok);
    f.doc->undoStack().clear();  // drops the history's references to the old moves
    CHECK_FALSE(checkSimulationCurrent(setup, f.session).current);

    // Replace the whole document and session state while it still runs.
    f.session.setDocument(nullptr);
    f.doc.reset();
    // The run goes on with its copies and completes at the full state.
    c.setChunkDelayForTest(0);
    REQUIRE(waitReadyAt(c, setup.totalMoves));
    CHECK(c.frame()->relief->pixels == directRelief(setup, setup.totalMoves).pixels);
    // The result of the old document is stale against any session.
    SimFixture other;
    CHECK_FALSE(checkSimulationCurrent(setup, other.session).current);
    // And the controller can be destroyed while working.
    c.setChunkDelayForTest(20);
    c.seek(0);
    c.seek(setup.totalMoves);
}

// --- playback -----------------------------------------------------------------------------------

TEST_CASE("playback: scrub, step and timed play", "[playback]") {
    PlaybackModel p;
    CHECK(p.moveCount() == 0);
    p.play();
    CHECK_FALSE(p.playing());
    CHECK_FALSE(p.tick(1.0));
    p.stepForward();
    CHECK(p.position() == 0);

    // Moves end at 1, 2, 4, 8, 16 s.
    p.reset(std::make_shared<const std::vector<double>>(std::vector<double>{1, 2, 4, 8, 16}));
    CHECK(p.moveCount() == 5);
    CHECK(p.totalSeconds() == 16.0);
    p.setPosition(3);
    CHECK(p.seconds() == 4.0);
    CHECK(p.fraction() == 0.25);
    p.stepBack();
    CHECK(p.position() == 2);
    p.stepBack(100);
    CHECK(p.position() == 0);
    p.stepForward(100);
    CHECK(p.position() == 5);
    CHECK(p.atEnd());
    p.setPosition(1000);
    CHECK(p.position() == 5);

    p.setFraction(0.5);  // t = 8 s: four moves are finished
    CHECK(p.position() == 4);
    p.setFraction(-3.0);
    CHECK(p.position() == 0);
    p.setFraction(std::numeric_limits<double>::quiet_NaN());
    CHECK(p.position() == 0);

    // Play at 1x: after 2.5 s two moves are done; play to the end pauses; play again restarts.
    p.setSpeed(1.0);
    p.play();
    CHECK(p.playing());
    CHECK(p.tick(2.5));
    CHECK(p.position() == 2);
    CHECK_FALSE(p.tick(0.5));  // t = 3.0 s: still two
    p.setSpeed(1e9);           // clamped
    CHECK(p.speed() == 1000.0);
    CHECK(p.tick(1.0));
    CHECK(p.position() == 5);
    CHECK_FALSE(p.playing());
    p.play();
    CHECK(p.position() == 0);
    CHECK(p.playing());
    p.toggle();
    CHECK_FALSE(p.playing());
    // Hostile tick values.
    p.play();
    CHECK_FALSE(p.tick(-1.0));
    CHECK_FALSE(p.tick(std::numeric_limits<double>::infinity()));
    CHECK_FALSE(p.tick(std::numeric_limits<double>::quiet_NaN()));
    p.setSpeed(std::numeric_limits<double>::quiet_NaN());
    CHECK(p.speed() == 1000.0);
}

TEST_CASE("playback: unknown times fall back to moves", "[playback]") {
    PlaybackModel p;
    const double inf = std::numeric_limits<double>::infinity();
    p.reset(std::make_shared<const std::vector<double>>(std::vector<double>{inf, inf, inf, inf}));
    CHECK(p.totalSeconds() == 0.0);
    p.setSpeed(1.0);
    p.play();
    CHECK(p.tick(0.04));  // at least one move per tick
    CHECK(p.position() >= 1);
    p.setFraction(0.5);
    CHECK(p.position() == 2);
    CHECK(p.seconds() == 0.0);
    while (p.playing()) {
        p.tick(1.0);
    }
    CHECK(p.atEnd());
}

// --- relief -------------------------------------------------------------------------------------

TEST_CASE("relief: cut shows darker, through-cut shows the table, passes get colours", "[relief]") {
    simulate::Stock stock;
    stock.width = 40;
    stock.height = 40;
    stock.thickness = 6;
    stock.cellSize = 0.5;
    simulate::Simulator sim(stock);
    REQUIRE(sim.ok());
    toolpath::Toolpath tp;
    tp.moves = {Move::rapid({5, 20, 5}), Move::feed({5, 20, -2}), Move::feed({35, 20, -2}),
                Move::feed({35, 30, -9}), Move::feed({5, 30, -9})};
    auto tool = simulate::ToolShape::flatEndMill(4.0);
    REQUIRE(tool);
    REQUIRE(sim.addToolpath(tp, *tool, 1));
    REQUIRE(sim.advanceAll());

    ReliefOptions opt;
    const Raster r = renderRelief(sim.heightfield(), opt);
    REQUIRE(r.width == sim.heightfield().nx());
    REQUIRE(r.height == sim.heightfield().ny());
    auto at = [&](double x, double y) {
        const std::size_t px = static_cast<std::size_t>(x / stock.cellSize);
        const std::size_t py = r.height - 1 - static_cast<std::size_t>(y / stock.cellSize);  // row 0 = top
        return r.pixels[py * r.width + px];
    };
    auto lum = [](std::uint32_t c) { return ((c >> 16) & 0xFF) + ((c >> 8) & 0xFF) + (c & 0xFF); };
    CHECK((at(20, 5) & 0xFF000000u) == 0xFF000000u);
    CHECK(lum(at(20, 20)) < lum(at(20, 5)));      // the groove is darker than the stock
    CHECK(at(20, 5) == kStockColour);              // flat uncut stock keeps its colour (shade 1)
    CHECK(at(36, 30) != at(20, 5));
    // Through the stock at z = -9 on a 6 mm slab: the table colour appears (shaded) there.
    const std::uint32_t tableish = at(20, 30);
    CHECK(((tableish >> 16) & 0xFF) < ((at(20, 5) >> 16) & 0xFF));

    opt.colourBy = ReliefOptions::ColourBy::Toolpath;
    const Raster byPass = renderRelief(sim.heightfield(), opt);
    CHECK(byPass.pixels != r.pixels);
    CHECK(toolpathColour(1) != toolpathColour(2));
    CHECK(toolpathColour(1) == toolpathColour(1));

    // Subsampling bounds the raster.
    ReliefOptions small;
    small.maxPixels = 16;
    const Raster s = renderRelief(sim.heightfield(), small);
    CHECK(s.width <= 16);
    CHECK(s.height <= 16);
    CHECK(s.stride > 1);
    CHECK(renderRelief(simulate::Heightfield{}).pixels.empty());
}
