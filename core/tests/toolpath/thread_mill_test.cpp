// T-0237: Thread milling toolpath (thread_mill.hpp, ADR-0037): helix pitch / turns of an M6x1
// internal thread, direction combinations, single and multi point cutters, fit tolerance, presets,
// external threads, bore circles, hostile input.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/thread_mill.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Contour;
using geometry::Vec2;

namespace {

tooling::Tool endMill(double diameter = 3.0) {
    tooling::Tool t;
    t.id = "em";
    t.name = "Thread mill";
    t.type = tooling::ToolType::EndMill;
    t.diameter = diameter;
    t.defaultCuttingData.toolNumber = 3;
    t.defaultCuttingData.feedRate = 600.0;
    t.defaultCuttingData.plungeRate = 200.0;
    t.defaultCuttingData.spindleSpeed = 12000.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 0.5;
    return t;
}

Contour hole(Vec2 c = {10, 10}, double r = 2.45) {
    return Contour::circle(c, r);
}

/// M6x1 internal, right hand, cut down, 5 mm long.
ThreadMillParams m6() {
    ThreadMillParams p;
    p.diameter = 6.0;
    p.pitch = 1.0;
    p.threadLength = 5.0;
    return p;
}

/// The helix: full revolution arcs (start and end coincide in XY) in order, with their start Z.
struct Turn {
    double z0;
    double z1;
    MoveKind kind;
};
std::vector<Turn> turnsOf(const Toolpath& tp) {
    std::vector<Turn> out;
    Vec3 at{0, 0, 0};
    for (const Move& m : tp.moves) {
        if (isArc(m.kind) && std::fabs(m.to.x - at.x) < 1e-12 && std::fabs(m.to.y - at.y) < 1e-12 &&
            std::fabs(m.to.z - at.z) > 1e-12) {
            out.push_back({at.z, m.to.z, m.kind});
        }
        at = m.to;
    }
    return out;
}

std::vector<const Move*> arcsOf(const Toolpath& tp) {
    std::vector<const Move*> out;
    for (const Move& m : tp.moves) {
        if (isArc(m.kind)) {
            out.push_back(&m);
        }
    }
    return out;
}

ThreadMillResult run(const ThreadMillParams& p, const std::vector<Contour>& cs = {hole()},
                     const tooling::Tool& tool = endMill()) {
    return threadMillToolpath(cs, tool, std::nullopt, p);
}

}  // namespace

TEST_CASE("thread mill: M6x1 internal advances exactly the pitch per revolution", "[thread_mill]") {
    const ThreadMillResult r = run(m6());
    REQUIRE(r.ok());
    REQUIRE(r.threads.size() == 1);
    CHECK(r.threads[0].centre.x == Approx(10.0).margin(1e-9));
    const std::vector<Turn> turns = turnsOf(r.toolpath);
    REQUIRE(turns.size() == 5);  // 5 mm of pitch 1
    for (std::size_t i = 0; i < turns.size(); ++i) {
        CHECK(std::fabs(turns[i].z1 - turns[i].z0) == Approx(1.0).margin(1e-6));
        CHECK(turns[i].z1 < turns[i].z0);  // cut down
        CHECK(turns[i].kind == MoveKind::ArcCW);  // right hand, down
    }
    CHECK(turns.front().z0 == Approx(0.0).margin(1e-12));
    CHECK(turns.back().z1 == Approx(-5.0).margin(1e-12));
    // The path radius: D/2 - d/2 = 3 - 1.5.
    const Move& first = r.toolpath.moves[3];  // after rapid, plunge, lead-in
    CHECK(first.centre.x == Approx(10.0));
    CHECK(first.to.x == Approx(11.5).margin(1e-9));
    // Starts above, ends at safe Z at the centre.
    CHECK(r.toolpath.moves.front().kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves.front().to.z == 5.0);
    CHECK(r.toolpath.moves.back().kind == MoveKind::Retract);
    CHECK(r.toolpath.moves.back().to.x == Approx(10.0).margin(1e-9));
    CHECK(r.toolpath.moves.back().to.z == 5.0);
    CHECK(r.toolpath.minZ().value() == Approx(-5.0).margin(1e-12));
    CHECK(r.toolpath.tool.feedRate == 600.0);
}

TEST_CASE("thread mill: hand and cut direction choose the rotation and Z direction", "[thread_mill]") {
    struct Case {
        ThreadHand hand;
        ThreadCutDirection cut;
        MoveKind kind;
        bool down;
    };
    const Case cases[] = {{ThreadHand::Right, ThreadCutDirection::Down, MoveKind::ArcCW, true},
                          {ThreadHand::Right, ThreadCutDirection::Up, MoveKind::ArcCCW, false},
                          {ThreadHand::Left, ThreadCutDirection::Down, MoveKind::ArcCCW, true},
                          {ThreadHand::Left, ThreadCutDirection::Up, MoveKind::ArcCW, false}};
    for (const ThreadType type : {ThreadType::Internal, ThreadType::External}) {
        for (const Case& c : cases) {
            ThreadMillParams p = m6();
            p.type = type;
            p.hand = c.hand;
            p.cutDirection = c.cut;
            const ThreadMillResult r = run(p, {Contour::circle({0, 0}, 3.0)});
            INFO("type " << static_cast<int>(type) << " hand " << static_cast<int>(c.hand) << " cut "
                         << static_cast<int>(c.cut));
            REQUIRE(r.ok());
            const std::vector<Turn> turns = turnsOf(r.toolpath);
            REQUIRE(turns.size() == 5);
            for (const Turn& t : turns) {
                CHECK(t.kind == c.kind);
                CHECK(t.z1 - t.z0 == Approx(c.down ? -1.0 : 1.0).margin(1e-6));
            }
            CHECK(turns.front().z0 == Approx(c.down ? 0.0 : -5.0).margin(1e-12));
            CHECK(turns.back().z1 == Approx(c.down ? -5.0 : 0.0).margin(1e-12));
        }
    }
}

TEST_CASE("thread mill: start depth, a fractional last turn and ending on the right point", "[thread_mill]") {
    ThreadMillParams p = m6();
    p.startDepth = 2.0;
    p.threadLength = 5.5;
    const ThreadMillResult r = run(p);
    REQUIRE(r.ok());
    const std::vector<Turn> turns = turnsOf(r.toolpath);
    REQUIRE(turns.size() == 5);
    CHECK(turns.front().z0 == Approx(-2.0).margin(1e-12));
    CHECK(turns.back().z1 == Approx(-7.0).margin(1e-9));
    // The half turn: ends opposite the start, at the bottom of the thread.
    const std::vector<const Move*> arcs = arcsOf(r.toolpath);
    const Move& partial = *arcs[arcs.size() - 2];
    CHECK(partial.to.z == Approx(-7.5).margin(1e-12));
    CHECK(partial.to.x == Approx(8.5).margin(1e-9));
    CHECK(partial.to.y == Approx(10.0).margin(1e-9));
}

TEST_CASE("thread mill: a multi point cutter needs (L - teeth + pitch) of travel", "[thread_mill]") {
    ThreadMillParams p = m6();
    p.cutter = ThreadCutter::MultiPoint;
    p.threadLength = 8.0;
    p.threadedLength = 6.0;
    ThreadMillResult r = run(p);
    REQUIRE(r.ok());
    std::vector<Turn> turns = turnsOf(r.toolpath);
    REQUIRE(turns.size() == 3);
    for (const Turn& t : turns) {
        CHECK(t.z1 - t.z0 == Approx(-1.0).margin(1e-6));
    }
    CHECK(turns.front().z0 == Approx(-5.0).margin(1e-12));  // teeth reach up to +1 above the surface
    CHECK(turns.back().z1 == Approx(-8.0).margin(1e-12));
    p.cutDirection = ThreadCutDirection::Up;
    r = run(p);
    REQUIRE(r.ok());
    turns = turnsOf(r.toolpath);
    REQUIRE(turns.size() == 3);
    CHECK(turns.front().z0 == Approx(-8.0).margin(1e-12));
    CHECK(turns.back().z1 == Approx(-5.0).margin(1e-12));
    // A cutter as long as the thread: one revolution.
    p.threadedLength = 8.0;
    r = run(p);
    REQUIRE(r.ok());
    CHECK(turnsOf(r.toolpath).size() == 1);
    // Longer than the thread, or shorter than a pitch: refused.
    p.threadedLength = 8.5;
    CHECK(run(p).has(ToolpathIssue::InvalidParameters));
    p.threadedLength = 0.5;
    CHECK(run(p).has(ToolpathIssue::InvalidParameters));
}

TEST_CASE("thread mill: fit tolerance moves the path radius", "[thread_mill]") {
    ThreadMillParams p = m6();
    const ThreadMillResult base = run(p);
    p.fitTolerance = 0.1;
    const ThreadMillResult loose = run(p);
    p.fitTolerance = -0.1;
    const ThreadMillResult tight = run(p);
    REQUIRE(base.ok());
    REQUIRE(loose.ok());
    REQUIRE(tight.ok());
    CHECK(loose.toolpath.moves[3].to.x - 10.0 == Approx(1.55).margin(1e-9));
    CHECK(tight.toolpath.moves[3].to.x - 10.0 == Approx(1.45).margin(1e-9));
    CHECK(*threadPathRadius(m6(), 3.0) == Approx(1.5));

    // External: tool outside, root diameter 6 - 2 * 0.6134, positive tolerance cuts thinner.
    ThreadMillParams e = m6();
    e.type = ThreadType::External;
    const double root = 3.0 - threadExternalDepth(1.0);
    CHECK(threadExternalDepth(1.0) == Approx(0.61343).margin(1e-5));
    CHECK(*threadPathRadius(e, 3.0) == Approx(root + 1.5));
    e.fitTolerance = 0.2;
    CHECK(*threadPathRadius(e, 3.0) == Approx(root - 0.1 + 1.5));
}

TEST_CASE("thread mill: internal lead-in and lead-out are tangent half circles", "[thread_mill]") {
    for (const ThreadHand hand : {ThreadHand::Right, ThreadHand::Left}) {
        ThreadMillParams p = m6();
        p.hand = hand;
        p.threadLength = 4.5;
        const ThreadMillResult r = run(p);
        REQUIRE(r.ok());
        const std::vector<const Move*> arcs = arcsOf(r.toolpath);
        const Move& in = *arcs.front();
        CHECK(in.centre.x == Approx(10.75).margin(1e-9));
        CHECK(in.centre.y == Approx(10.0).margin(1e-9));
        CHECK(in.to.z == Approx(0.0).margin(1e-12));  // constant Z
        const Move& out = *arcs.back();
        const Move& last = *(arcs.end() - 2).operator*();
        CHECK(out.to.x == Approx(10.0).margin(1e-9));
        CHECK(out.to.y == Approx(10.0).margin(1e-9));
        CHECK(out.centre.x == Approx(0.5 * (10.0 + last.to.x)).margin(1e-9));
        CHECK(out.centre.y == Approx(0.5 * (10.0 + last.to.y)).margin(1e-9));
        CHECK(out.kind == last.kind);
    }
}

TEST_CASE("thread mill: external threads approach and leave along the tangent", "[thread_mill]") {
    ThreadMillParams p = m6();
    p.type = ThreadType::External;
    const ThreadMillResult r = run(p, {Contour::circle({0, 0}, 3.0)});
    REQUIRE(r.ok());
    const double radius = *threadPathRadius(p, 3.0);
    const std::vector<Move>& m = r.toolpath.moves;
    // Right hand, down: clockwise; starts at (R, 0), heading -y, so it comes from +y.
    CHECK(m[0].kind == MoveKind::Rapid);
    CHECK(m[0].to.x == Approx(radius).margin(1e-9));
    CHECK(m[0].to.y == Approx(3.0).margin(1e-9));  // one tool diameter back
    CHECK(m[1].kind == MoveKind::Plunge);
    CHECK(m[2].kind == MoveKind::Feed);
    CHECK(m[2].to.y == Approx(0.0).margin(1e-9));
    // Whole turns: ends at the start; leaves 3 mm further along -y.
    const Move& leave = m[m.size() - 2];
    CHECK(leave.kind == MoveKind::Feed);
    CHECK(leave.to.x == Approx(radius).margin(1e-9));
    CHECK(leave.to.y == Approx(-3.0).margin(1e-9));
    CHECK(leave.to.z == Approx(-5.0).margin(1e-12));
    CHECK(m.back().kind == MoveKind::Retract);
    CHECK(m.back().to.z == 5.0);
    // Counter-clockwise: mirrored.
    p.cutDirection = ThreadCutDirection::Up;
    const ThreadMillResult up = run(p, {Contour::circle({0, 0}, 3.0)});
    REQUIRE(up.ok());
    CHECK(up.toolpath.moves[0].to.y == Approx(-3.0).margin(1e-9));
    // A fractional end angle (a quarter turn) leaves tangentially.
    p.cutDirection = ThreadCutDirection::Down;
    p.threadLength = 5.25;
    const ThreadMillResult q = run(p, {Contour::circle({0, 0}, 3.0)});
    REQUIRE(q.ok());
    const Move& qEnd = q.toolpath.moves[q.toolpath.moves.size() - 3];
    CHECK(qEnd.to.x == Approx(0.0).margin(1e-9));
    CHECK(qEnd.to.y == Approx(-radius).margin(1e-9));  // clockwise quarter turn from +x
    const Move& qLeave = q.toolpath.moves[q.toolpath.moves.size() - 2];
    CHECK(qLeave.to.x == Approx(-3.0).margin(1e-9));  // heading -x at the bottom of a clockwise circle
    CHECK(qLeave.to.y == Approx(-radius).margin(1e-9));
}

TEST_CASE("thread mill: pitch presets", "[thread_mill]") {
    CHECK(*threadPresetPitch(ThreadPitchPreset::IsoMetricCoarse, 6.0) == 1.0);
    CHECK(*threadPresetPitch(ThreadPitchPreset::IsoMetricCoarse, 10.0) == 1.5);
    CHECK(*threadPresetPitch(ThreadPitchPreset::IsoMetricCoarse, 1.0) == 0.25);
    CHECK(*threadPresetPitch(ThreadPitchPreset::IsoMetricCoarse, 64.0) == 6.0);
    CHECK_FALSE(threadPresetPitch(ThreadPitchPreset::IsoMetricCoarse, 7.0).has_value());
    CHECK_FALSE(threadPresetPitch(ThreadPitchPreset::Custom, 6.0).has_value());
    CHECK_FALSE(threadPresetPitch(ThreadPitchPreset::IsoMetricCoarse, std::nan("")).has_value());
    CHECK(*threadPresetPitch(ThreadPitchPreset::UnifiedCoarse, 6.35) == Approx(1.27));
    CHECK(*threadPresetPitch(ThreadPitchPreset::UnifiedCoarse, 25.4) == Approx(3.175));
    CHECK_FALSE(threadPresetPitch(ThreadPitchPreset::UnifiedCoarse, 6.0).has_value());

    ThreadMillParams p = m6();
    p.pitch = 0.5;  // ignored while a preset is chosen
    p.pitchPreset = ThreadPitchPreset::IsoMetricCoarse;
    REQUIRE(threadPitch(p).has_value());
    CHECK(*threadPitch(p) == 1.0);
    const ThreadMillResult r = run(p);
    REQUIRE(r.ok());
    CHECK(turnsOf(r.toolpath).size() == 5);
    p.diameter = 7.0;
    const ThreadMillResult bad = run(p);
    CHECK_FALSE(bad.ok());
    CHECK(bad.has(ToolpathIssue::InvalidParameters));
}

TEST_CASE("thread mill: bore circles", "[thread_mill]") {
    ThreadMillParams p = m6();
    CHECK(threadBoreDiameter(6.0, 1.0) == Approx(4.9175).margin(1e-4));
    const std::vector<Contour> circles = threadBoreCircles(p, {{1, 2}, {5, 6}});
    REQUIRE(circles.size() == 2);
    const geometry::Box2 b = circles[0].bounds();
    CHECK(b.width() == Approx(4.9175).margin(1e-4));
    CHECK(b.centre().x == Approx(1.0).margin(1e-9));
    p.type = ThreadType::External;
    CHECK(threadBoreCircles(p, {{1, 2}}).empty());
    CHECK(threadBoreDiameter(1.0, 10.0) == 0.0);
    CHECK(threadBoreDiameter(std::nan(""), 1.0) == 0.0);
    CHECK(threadBoreDiameter(6.0, -1.0) == 0.0);
}

TEST_CASE("thread mill: several vectors are ordered, duplicates made once", "[thread_mill]") {
    ThreadMillParams p = m6();
    p.startPosition = {100, 0};
    const std::vector<Contour> cs = {hole({0, 0}), hole({50, 0}), hole({100, 0}), hole({50, 0}, 1.0)};
    const ThreadMillResult r = run(p, cs);
    REQUIRE(r.ok());
    REQUIRE(r.threads.size() == 3);
    CHECK(r.threads[0].contour == 2);  // nearest the start
    CHECK(r.threads[1].contour == 1);
    CHECK(r.threads[2].contour == 0);
    CHECK(r.has(ToolpathIssue::DuplicateHole));
    CHECK(turnsOf(r.toolpath).size() == 15);
    // Every rapid is at safe Z.
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Rapid) {
            CHECK(m.to.z == 5.0);
        }
    }
}

TEST_CASE("thread mill: deterministic", "[thread_mill]") {
    CHECK(run(m6()).toolpath == run(m6()).toolpath);
}

TEST_CASE("thread mill: bad parameters and tools are refused", "[thread_mill]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const auto refused = [](const ThreadMillParams& p, ToolpathIssue issue, const tooling::Tool& tool = endMill()) {
        const ThreadMillResult r = threadMillToolpath({hole()}, tool, std::nullopt, p);
        CHECK_FALSE(r.ok());
        CHECK(r.has(issue));
        CHECK(r.toolpath.moves.empty());
    };
    const ToolpathIssue bad = ToolpathIssue::InvalidParameters;
    ThreadMillParams p = m6();
    p.pitch = nan;
    refused(p, bad);
    p = m6();
    p.pitch = 0.0;
    refused(p, bad);
    p = m6();
    p.pitch = -1.0;
    refused(p, bad);
    p = m6();
    p.diameter = inf;
    refused(p, bad);
    p = m6();
    p.diameter = 0.0;
    refused(p, bad);
    p = m6();
    p.fitTolerance = nan;
    refused(p, bad);
    p = m6();
    p.fitTolerance = 1.5;  // beyond the pitch
    refused(p, bad);
    p = m6();
    p.startDepth = -1.0;
    refused(p, bad);
    p = m6();
    p.threadLength = 0.5;  // below the pitch
    refused(p, bad);
    p = m6();
    p.threadLength = 0.0;
    refused(p, bad);
    p = m6();
    p.safeZ = 0.0;  // not above the start
    refused(p, bad);
    p = m6();
    p.safeZ = nan;
    refused(p, bad);
    p = m6();
    p.maxDepth = 4.0;  // shallower than the thread
    refused(p, bad);
    p = m6();
    p.maxDepth = -1.0;
    refused(p, bad);
    p = m6();
    p.startPosition.x = inf;
    refused(p, bad);
    p = m6();
    p.tolerance.chordal = 0.0;
    refused(p, bad);
    p = m6();
    p.tolerance.distance = nan;
    refused(p, bad);
    p = m6();
    p.pitch = 1e-6;  // too fine
    refused(p, bad);
    p = m6();
    p.pitch = 0.001;
    p.threadLength = 1000.0;  // 1e6 revolutions
    refused(p, ToolpathIssue::WorkLimitExceeded);
    p = m6();
    p.pitchPreset = ThreadPitchPreset::IsoMetricCoarse;
    p.diameter = 6.5;
    refused(p, bad);

    // A max depth equal to the bottom is fine.
    p = m6();
    p.maxDepth = 5.0;
    CHECK(run(p).ok());

    // Tools.
    p = m6();
    tooling::Tool t = endMill();
    t.type = tooling::ToolType::VBit;
    refused(p, ToolpathIssue::InvalidTool, t);
    t = endMill();
    t.diameter = 0.0;
    refused(p, ToolpathIssue::InvalidTool, t);
    t = endMill();
    t.diameter = nan;
    refused(p, ToolpathIssue::InvalidTool, t);
    t = endMill();
    t.defaultCuttingData.feedRate = 0.0;
    refused(p, ToolpathIssue::InvalidTool, t);
    t = endMill();
    t.defaultCuttingData.plungeRate = 0.0;
    refused(p, ToolpathIssue::InvalidTool, t);
    // Too big for the bore (4.9175 mm) and for the thread axis.
    refused(p, ToolpathIssue::ToolTooLarge, endMill(5.0));
    refused(p, ToolpathIssue::ToolTooLarge, endMill(6.0));
    CHECK(run(p, {hole()}, endMill(4.9)).ok());
    // External: a root diameter that vanishes.
    p = m6();
    p.type = ThreadType::External;
    p.diameter = 1.0;
    p.pitch = 1.0;
    refused(p, ToolpathIssue::ToolTooLarge);
}

TEST_CASE("thread mill: hostile vectors", "[thread_mill]") {
    const ThreadMillParams p = m6();
    Contour open = Contour::circle({0, 0}, 2.0);
    open.closed = false;
    ThreadMillResult r = run(p, {open});
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::OpenContourIgnored));
    r = run(p, {Contour{}});
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::EmptyContour));
    r = run(p, {});
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::NoRegion));
    // A line pair enclosing no area.
    Contour flat;
    flat.closed = true;
    flat.spans.push_back(geometry::LineSpan{{0, 0}, {5, 0}});
    flat.spans.push_back(geometry::LineSpan{{5, 0}, {0, 0}});
    r = run(p, {flat});
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::NoRegion));
    // A broken vector next to a good one: the good one is still made.
    Contour gap;
    gap.closed = true;
    gap.spans.push_back(geometry::LineSpan{{0, 0}, {5, 0}});
    gap.spans.push_back(geometry::LineSpan{{7, 7}, {0, 5}});
    r = run(p, {gap, hole()});
    CHECK(r.ok());
    CHECK(r.has(ToolpathIssue::DiscontinuousContour));
    CHECK(r.threads.size() == 1);
    // Huge coordinates stay finite.
    r = run(p, {hole({1e9, -1e9})});
    REQUIRE(r.ok());
    for (const Move& m : r.toolpath.moves) {
        CHECK(std::isfinite(m.to.x));
        CHECK(std::isfinite(m.to.y));
        CHECK(std::isfinite(m.to.z));
    }
}
