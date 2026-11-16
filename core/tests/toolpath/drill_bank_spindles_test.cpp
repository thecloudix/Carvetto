// T-0335: the spindle sets a drill bank toolpath carries (ADR-0111): every move of a stroke names the spindles that
// drill in it, the head comes with the toolpath, switched-off and reference spindles, per-spindle depth and diameter,
// and toolpaths of other operations carry nothing.

#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/drill_bank.hpp"
#include "carvetto/toolpath/keepout.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour;
using geometry::Vec2;

namespace {

tooling::Tool drillTool(double diameter = 3.0) {
    tooling::Tool t;
    t.id = "drill";
    t.name = "Drill";
    t.type = tooling::ToolType::Drill;
    t.diameter = diameter;
    t.includedAngle = 118.0 * geometry::kPi / 180.0;
    t.defaultCuttingData.toolNumber = 7;
    t.defaultCuttingData.feedRate = 500.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 10000.0;
    t.defaultCuttingData.passDepth = 2.0;
    return t;
}

DrillBankParams bank(int horizontal, int vertical = 0) {
    DrillBankParams p;
    p.horizontalDrills = horizontal;
    p.verticalDrills = vertical;
    p.pitch = 32.0;
    p.cutDepth = 10.0;
    p.safeZ = 5.0;
    return p;
}

std::vector<Contour> row(double x0, double y, int count) {
    std::vector<Contour> out;
    for (int i = 0; i < count; ++i) {
        out.push_back(Contour::circle({x0 + 32.0 * i, y}, 1.5));
    }
    return out;
}

DrillBankResult run(const std::vector<Contour>& cs, const DrillBankParams& p) {
    return drillBankToolpath(cs, drillTool(), std::nullopt, p);
}

}  // namespace

TEST_CASE("T-0335: every move of a stroke carries the stroke's spindle set; the head comes with the toolpath",
          "[toolpath][drill_bank][spindles]") {
    DrillBankParams p = bank(4);
    p.minHoles = 4;
    // Two strokes of four holes.
    std::vector<Contour> holes = row(10, 10, 4);
    for (const Contour& c : row(10, 90, 4)) {
        holes.push_back(c);
    }
    const DrillBankResult r = run(holes, p);
    REQUIRE(r.ok());
    REQUIRE(r.placements.size() == 2);
    REQUIRE(r.toolpath.bankHead.size() == 4);
    CHECK(r.toolpath.bankHead == drillBankHead(p, 3.0));
    CHECK(r.toolpath.bankHead[0].offset == Vec2{0.0, 0.0});  // the reference spindle (1)
    CHECK(r.toolpath.bankHead[3].offset == Vec2{96.0, 0.0});
    for (const BankSpindleSpec& s : r.toolpath.bankHead) {
        CHECK(s.diameter == 3.0);
        CHECK(s.depth == 10.0);
    }
    for (const Move& m : r.toolpath.moves) {
        CHECK(m.bank == 0b1111);
    }
    // A stroke starts at the rapid that carries its set: one rapid per stroke.
    std::size_t rapids = 0;
    for (const Move& m : r.toolpath.moves) {
        rapids += m.kind == MoveKind::Rapid ? 1 : 0;
    }
    CHECK(rapids == 2);
}

TEST_CASE("T-0335: the spindle set is the spindles that drill, not the reference or the switched-off ones",
          "[toolpath][drill_bank][spindles]") {
    // Spindle 2 is the reference and switched off; spindle 4 is off too. Holes at spindles 1 and 3 only.
    DrillBankParams p = bank(4);
    p.referenceDrill = 2;
    p.inactiveSpindles = {2, 4};
    p.minHoles = 2;
    // Reference (spindle 2) at x = 40: spindle 1 at 8, spindle 3 at 72.
    const std::vector<Contour> holes{Contour::circle({8, 20}, 1.5), Contour::circle({72, 20}, 1.5)};
    const DrillBankResult r = run(holes, p);
    REQUIRE(r.ok());
    REQUIRE(r.placements.size() == 1);
    CHECK(r.placements[0].reference.x == 40.0);
    for (const Move& m : r.toolpath.moves) {
        CHECK(m.bank == 0b0101);
    }
    // The head is the whole head, relative to the reference spindle (number 2).
    REQUIRE(r.toolpath.bankHead.size() == 4);
    CHECK(r.toolpath.bankHead[0].offset == Vec2{-32.0, 0.0});
    CHECK(r.toolpath.bankHead[1].offset == Vec2{0.0, 0.0});
    CHECK(r.toolpath.bankHead[2].offset == Vec2{32.0, 0.0});
    CHECK(r.toolpath.bankHead[3].offset == Vec2{64.0, 0.0});
}

TEST_CASE("T-0335: per-spindle diameters and depths are resolved in the head", "[toolpath][drill_bank][spindles]") {
    DrillBankParams p = bank(2, 1);
    p.spindleDiameters = {5.0, 6.0, 8.0};
    p.spindleDepths = {0.0, 4.0, 12.0};
    p.cutDepth = 7.0;
    const std::vector<BankSpindleSpec> head = drillBankHead(p, 3.0);
    REQUIRE(head.size() == 3);
    CHECK(head[0].diameter == 5.0);
    CHECK(head[0].depth == 7.0);  // 0 = the cut depth
    CHECK(head[1].depth == 4.0);
    CHECK(head[2].diameter == 8.0);
    CHECK(head[2].depth == 12.0);
    CHECK(head[2].offset == Vec2{16.0, 0.0});  // the column at x = 0, the reference at x = -16
    // Without lists: the tool's diameter and the cut depth.
    const std::vector<BankSpindleSpec> plain = drillBankHead(bank(3), 4.5);
    for (const BankSpindleSpec& s : plain) {
        CHECK(s.diameter == 4.5);
        CHECK(s.depth == 10.0);
    }
    // A reference outside the head counts as spindle 1; no spindles, no head.
    DrillBankParams q = bank(3);
    q.referenceDrill = 9;
    CHECK(drillBankHead(q, 3.0)[0].offset == Vec2{0.0, 0.0});
    CHECK(drillBankHead(bank(0, 0), 3.0).empty());
    // The bit of a spindle number; numbers outside 1..64 name none.
    CHECK(drillBankSpindleBit(1) == 1);
    CHECK(drillBankSpindleBit(64) == std::uint64_t{1} << 63);
    CHECK(drillBankSpindleBit(0) == 0);
    CHECK(drillBankSpindleBit(65) == 0);
    CHECK(drillBankSpindleBit(-1) == 0);
}

TEST_CASE("T-0335: the head's moves are unchanged by the spindle sets; Drilling carries none",
          "[toolpath][drill_bank][spindles]") {
    // A one-spindle bank's moves equal Drilling's apart from the spindle set.
    DrillBankParams p = bank(1);
    p.minHoles = 1;
    p.peck = true;
    p.dwellTime = 0.3;
    const std::vector<Contour> hole{Contour::circle({20, 20}, 1.5)};
    const DrillBankResult r = run(hole, p);
    REQUIRE(r.ok());
    const DrillResult d = drillToolpath(hole, drillTool(), std::nullopt, static_cast<const DrillParams&>(p));
    REQUIRE(d.ok());
    REQUIRE(r.toolpath.moves.size() == d.toolpath.moves.size());
    for (std::size_t i = 0; i < d.toolpath.moves.size(); ++i) {
        Move m = r.toolpath.moves[i];
        CHECK(m.bank == 1);
        m.bank = 0;
        CHECK(m == d.toolpath.moves[i]);
        CHECK(d.toolpath.moves[i].bank == 0);
    }
    CHECK(d.toolpath.bankHead.empty());
}

TEST_CASE("T-0335: keep-out routing keeps a stroke's spindle set on the rapid that arrives",
          "[toolpath][drill_bank][spindles][keepout]") {
    Toolpath tp;
    tp.bankHead = {{{0.0, 0.0}, 3.0, 6.0}, {{20.0, 0.0}, 3.0, 6.0}};
    const auto banked = [](Move m) {
        m.bank = 0b11;
        return m;
    };
    tp.append(banked(Move::rapid({-5, 5, 5})));
    tp.append(banked(Move::plunge({-5, 5, -6})));
    tp.append(banked(Move::retract({-5, 5, 5})));
    tp.append(banked(Move::rapid({15, 6, 5})));  // across the zone: routed around it
    tp.append(banked(Move::plunge({15, 6, -6})));
    tp.append(banked(Move::retract({15, 6, 5})));
    KeepOutField field({Contour::rectangle(geometry::Box2{{0, 0}, {10, 10}})}, 0.0);
    const RoutingReport rep = routeRapids(tp, field, geometry::Vec3{-5, 5, 5});
    REQUIRE(rep.routed == 1);
    REQUIRE(tp.moves.size() == 8);
    // The detour legs before the arrival carry no set; the arriving rapid is the stroke's, with its set.
    CHECK(tp.moves[3].bank == 0);
    CHECK(tp.moves[4].bank == 0);
    CHECK(tp.moves[5].kind == MoveKind::Rapid);
    CHECK(tp.moves[5].bank == 0b11);
    CHECK(tp.moves[5].to == geometry::Vec3{15, 6, 5});
    CHECK(tp.moves[6].bank == 0b11);
    CHECK(tp.moves[7].bank == 0b11);
    CHECK(tp.moves[0].bank == 0b11);
}
