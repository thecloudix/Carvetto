// T-0335: a drill bank toolpath through the viewmodel (ADR-0111): the recalculated record carries the spindle sets,
// the preview draws one dot per spindle of a stroke, the simulation setup carries the head and the simulation
// (direct and through the controller, which queues strokes whole) drills one hole per active spindle.

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/viewmodel/relief_render.hpp"
#include "carvetto/viewmodel/simulation_controller.hpp"
#include "carvetto/viewmodel/simulation_setup.hpp"
#include "carvetto/viewmodel/toolpath_preview.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using document::Document;
using document::ToolpathOperation;
using document::Units;

namespace {

/// A document with two rows of three holes at the pitch and a four-spindle drill bank over them (spindle 4 off).
struct BankFixture {
    Document doc;
    tooling::ToolDatabase db{cliToolDatabase()};
    ToolpathSession session{&doc};
    document::ToolpathId id;

    BankFixture() {
        session.setToolDatabase(db);
        for (const double y : {40.0, 120.0}) {
            for (int i = 0; i < 3; ++i) {
                addVector(doc, geometry::Contour::circle({30.0 + 32.0 * i, y}, 1.5));
            }
        }
        ToolpathForm form(ToolpathOperation::DrillBank, Units::Millimetres);
        REQUIRE(form.setTool(db, source("drill-3mm")).ok);
        REQUIRE(form.setField("cutDepth", "8").ok);
        REQUIRE(form.setBankSpindles({4}, {}, {0, 5, 0, 0}).ok);
        const ToolpathOutcome out = session.add(form.definition());
        INFO(out.message);
        REQUIRE(out.ok);
        id = out.ids.front();
    }
};

bool near(double a, double b) {
    return std::fabs(a - b) < 1e-9;
}

}  // namespace

TEST_CASE("bank spindles: the calculated record names the spindles of every stroke", "[toolpath][drill_bank][spindles]") {
    BankFixture f;
    const document::ToolpathRecord& rec = f.doc.toolpaths().front();
    REQUIRE(rec.hasMoves());
    std::size_t rapids = 0;
    for (const toolpath::Move& m : *rec.moves) {
        CHECK(m.bank == 0b0111);
        rapids += m.kind == toolpath::MoveKind::Rapid ? 1 : 0;
    }
    CHECK(rapids == 2);
    const std::vector<toolpath::BankSpindleSpec> head = rec.bankHead();
    REQUIRE(head.size() == 4);
    CHECK(head[1].depth == 5.0);
    CHECK(head[0].depth == 8.0);
    CHECK(rec.toolpath().bankHead == head);
}

TEST_CASE("bank spindles: the preview draws a dot per spindle of a stroke", "[preview][drill_bank][spindles]") {
    BankFixture f;
    PreviewModel model;
    model.sync(f.session);
    model.setEnabled(true);
    REQUIRE(model.scene().toolpaths.size() == 1);
    const PreviewGeometry& geo = *model.scene().toolpaths.front().geometry;
    std::set<std::pair<double, double>> plunges;
    for (const PreviewChunk& c : geo.chunks) {
        if (c.cls == MoveClass::Plunge && c.points.size() == 1) {
            plunges.insert({c.points.front().x, c.points.front().y});
        }
    }
    // Reference spindle (1) at the first hole of each row, spindles 2 and 3 on the next holes; none for the off spindle.
    std::set<std::pair<double, double>> expected;
    for (const double y : {40.0, 120.0}) {
        for (int i = 0; i < 3; ++i) {
            expected.insert({30.0 + 32.0 * i, y});
        }
    }
    CHECK(plunges == expected);
    // Without the head the same moves draw a dot at the reference place only.
    const std::shared_ptr<const PreviewGeometry> plain = buildPreviewGeometry(*model.scene().toolpaths.front().moves);
    std::set<std::pair<double, double>> reference;
    for (const PreviewChunk& c : plain->chunks) {
        if (c.cls == MoveClass::Plunge && c.points.size() == 1) {
            reference.insert({c.points.front().x, c.points.front().y});
        }
    }
    CHECK(reference.size() == 2);
    // Pecks at one place merge: the dot of a spindle is not repeated per pass.
    const std::vector<toolpath::BankSpindleSpec> head = f.doc.toolpaths().front().bankHead();
    std::vector<toolpath::Move> pecked;
    const auto banked = [](toolpath::Move m) {
        m.bank = 0b011;
        return m;
    };
    pecked.push_back(banked(toolpath::Move::rapid({10, 10, 5})));
    for (const double z : {-2.0, -4.0, -6.0}) {
        pecked.push_back(banked(toolpath::Move::plunge({10, 10, z})));
        pecked.push_back(banked(toolpath::Move::retract({10, 10, z + 1})));
    }
    const std::shared_ptr<const PreviewGeometry> pg = buildPreviewGeometry(pecked, nullptr, std::nullopt, &head);
    std::size_t plungeDots = 0;
    std::size_t retractDots = 0;
    for (const PreviewChunk& c : pg->chunks) {
        plungeDots += c.cls == MoveClass::Plunge ? 1 : 0;
        retractDots += c.cls == MoveClass::Retract ? 1 : 0;
    }
    CHECK(plungeDots == 2);
    CHECK(retractDots == 2);
}

TEST_CASE("bank spindles: the simulation setup carries the head and drills every spindle's hole",
          "[simulation][drill_bank][spindles]") {
    BankFixture f;
    const SimulationSetup setup = makeSimulationSetup(f.session);
    REQUIRE(setup.ok);
    REQUIRE(setup.toolpaths.size() == 1);
    CHECK(setup.toolpaths.front().bankHead.size() == 4);
    simulate::Simulator sim(setup.stock, setup.options);
    REQUIRE(sim.ok());
    toolpath::Toolpath path;
    path.name = setup.toolpaths.front().name;
    path.tool = setup.toolpaths.front().tool;
    path.moves = *setup.toolpaths.front().moves;
    path.bankHead = setup.toolpaths.front().bankHead;
    REQUIRE(sim.addToolpath(path, setup.toolpaths.front().shape, 1));
    REQUIRE(sim.advanceAll());
    const auto height = [&sim](double x, double y) {
        std::size_t i = 0;
        std::size_t j = 0;
        REQUIRE(sim.heightfield().cellAt({x, y}, i, j));
        return static_cast<double>(sim.heightfield().height(i, j));
    };
    // Each of the six holes is drilled: 8 mm deep, except spindle 2's (5 mm); the drill's point is a cone, so the
    // cell next to the axis is a fraction of a millimetre above the tip.
    for (const double y : {40.0, 120.0}) {
        CHECK(std::fabs(height(30.0, y) + 8.0) < 0.25);
        CHECK(std::fabs(height(62.0, y) + 5.0) < 0.25);
        CHECK(std::fabs(height(94.0, y) + 8.0) < 0.25);
        CHECK(near(height(126.0, y), 0.0));  // spindle 4 is off
    }
    // The controller (which queues the stroke whole and passes the head) shows the same picture.
    SimulationController c;
    REQUIRE(c.start(setup));
    const std::size_t total = setup.totalMoves;
    c.seek(total);
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    bool ready = false;
    while (std::chrono::steady_clock::now() < end && !ready) {
        const auto fr = c.frame();
        ready = fr && fr->move == total && c.progress().state == SimState::Ready;
        if (!ready) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    REQUIRE(ready);
    CHECK(c.frame()->relief->pixels == renderRelief(sim.heightfield(), {}).pixels);
}

TEST_CASE("bank spindles: a stroke is never split between two queued slices", "[simulation][drill_bank][spindles]") {
    BankFixture f;
    SimulationSetup setup = makeSimulationSetup(f.session);
    REQUIRE(setup.ok);
    // 10922 filler strokes of three moves, then a pecked stroke whose first plunge is the last move of the first
    // 32768-move slice: [rapid, plunge -4, | retract -3, plunge -8, retract]. Spindle B (depth 4) must still be
    // limited by the stroke's own bottom (-8), not by the part queued first.
    std::vector<toolpath::Move> moves;
    const auto banked = [](toolpath::Move m, std::uint64_t set) {
        m.bank = set;
        return m;
    };
    for (int i = 0; i < 10922; ++i) {
        moves.push_back(banked(toolpath::Move::rapid({10, 10, 5}), 0b01));
        moves.push_back(banked(toolpath::Move::plunge({10, 10, -1}), 0b01));
        moves.push_back(banked(toolpath::Move::retract({10, 10, 5}), 0b01));
    }
    REQUIRE(moves.size() == 32766);
    moves.push_back(banked(toolpath::Move::rapid({50, 20, 5}), 0b11));
    moves.push_back(banked(toolpath::Move::plunge({50, 20, -4}), 0b11));
    moves.push_back(banked(toolpath::Move::retract({50, 20, -3}), 0b11));
    moves.push_back(banked(toolpath::Move::plunge({50, 20, -8}), 0b11));
    moves.push_back(banked(toolpath::Move::retract({50, 20, 5}), 0b11));
    SimulationToolpath& tp = setup.toolpaths.front();
    tp.moves = document::shareMoves(std::move(moves));
    tp.bankHead = {{{0.0, 0.0}, 3.0, 8.0}, {{20.0, 0.0}, 3.0, 4.0}};
    setup.totalMoves = tp.moves->size();

    simulate::Simulator sim(setup.stock, setup.options);
    REQUIRE(sim.ok());
    toolpath::Toolpath path;
    path.tool = tp.tool;
    path.moves = *tp.moves;
    path.bankHead = tp.bankHead;
    REQUIRE(sim.addToolpath(path, tp.shape, 1));
    REQUIRE(sim.advanceToMove(32768));
    {
        std::size_t i = 0;
        std::size_t j = 0;
        REQUIRE(sim.heightfield().cellAt({70.0, 20.0}, i, j));
        CHECK(std::fabs(static_cast<double>(sim.heightfield().height(i, j)) + 4.0) < 0.25);  // B is cut to its depth
    }
    SimulationController c;
    REQUIRE(c.start(setup));
    c.seek(32768);
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(300);
    bool ready = false;
    while (std::chrono::steady_clock::now() < end && !ready) {
        const auto fr = c.frame();
        ready = fr && fr->move == 32768 && c.progress().state == SimState::Ready;
        if (!ready) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    REQUIRE(ready);
    CHECK(c.frame()->relief->pixels == renderRelief(sim.heightfield(), {}).pixels);
}

TEST_CASE("bank spindles: a bank with only a row or only a column can be typed", "[toolpath][form][drill_bank][spindles]") {
    ToolpathForm form(ToolpathOperation::DrillBank, Units::Millimetres);
    REQUIRE(form.setField("verticalDrills", "2").ok);
    REQUIRE(form.setField("verticalDrills", "0").ok);  // zero is a count here, not "unset"
    CHECK(std::get<toolpath::DrillBankParams>(form.definition().params).verticalDrills == 0);
    REQUIRE(form.setField("horizontalDrills", "0").ok);
    REQUIRE(form.setField("verticalDrills", "3").ok);
    CHECK(std::get<toolpath::DrillBankParams>(form.definition().params).horizontalDrills == 0);
    CHECK_FALSE(form.setField("horizontalDrills", "33").ok);
    CHECK_FALSE(form.setField("horizontalDrills", "-1").ok);
}
