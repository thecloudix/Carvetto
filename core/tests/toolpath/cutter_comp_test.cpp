// T-0327 (ADR-0100): controller-side cutter compensation for Profile toolpaths. Covers the derivation of the
// programmed path (cutterCompProgram) on hand-made and Profile-made move lists, the Profile option and its
// refusals, and the post blocks. Tests are deterministic and compare against geometry computed here with the
// independent distance oracle of profile_test_util.hpp.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/cutter_comp.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Vec2;
using carvetto::geometry::Vec3;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr double kR = 3.0;  // the 6 mm end mill

ProfileParams compParams(ProfileSide side = ProfileSide::Outside, CutDirection dir = CutDirection::Climb) {
    ProfileParams p;
    p.side = side;
    p.direction = dir;
    p.passes.cutDepth = 3.0;
    p.passes.passDepth = 4.0;
    p.leads.type = LeadType::Straight;
    p.leads.length = 5.0;
    p.leads.angle = geometry::kPi / 4.0;
    p.leads.leadOut = true;
    p.cutterComp = true;
    return p;
}

Contour square() {
    return Contour::rectangle(Box2{{0, 0}, {40, 30}});
}

Move comp(Move m, CutterComp c) {
    m.comp = c;
    return m;
}

struct Run {
    std::size_t first{0};
    std::size_t last{0};  // last flagged move
};

/// The flagged stretch of a one-cut toolpath.
Run runOf(const std::vector<Move>& moves) {
    Run r;
    bool seen = false;
    for (std::size_t i = 0; i < moves.size(); ++i) {
        if (moves[i].comp != CutterComp::Off) {
            if (!seen) {
                r.first = i;
                seen = true;
            }
            r.last = i;
        }
    }
    REQUIRE(seen);
    return r;
}

std::vector<Move> withoutFlags(std::vector<Move> moves) {
    for (Move& m : moves) {
        m.comp = CutterComp::Off;
    }
    return moves;
}

bool hasIssueOf(const ProfileResult& r, ToolpathIssue issue) {
    return r.has(issue);
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// The derivation on hand-made moves
// ---------------------------------------------------------------------------------------------

TEST_CASE("cutter comp: an inside corner of the tool-centre path meets at the programmed corner", "[toolpath][cuttercomp]") {
    // Tool on the left of a counter-clockwise square programmed 0..10; tool centre path inset by r = 3.
    const std::vector<Move> moves{
        Move::rapid({0, 6, 5}),
        Move::plunge({0, 6, -1}),
        comp(Move::feed({3, 3, -1}), CutterComp::Left),    // engage from the tool side
        comp(Move::feed({7, 3, -1}), CutterComp::Left),
        comp(Move::feed({7, 7, -1}), CutterComp::Left),
        Move::feed({7, 12, -1}),                           // release
    };
    const CutterCompProgram p = cutterCompProgram(moves, kR, {});
    REQUIRE(p.ok());
    REQUIRE(p.moves.size() == moves.size());
    CHECK(p.moves[0].move == moves[0]);
    CHECK(p.moves[1].move == moves[1]);
    CHECK(p.moves[2].move.to.x == 3.0);
    CHECK(p.moves[2].move.to.y == 0.0);  // perpendicular to the next move
    CHECK(p.moves[3].move.to.x == 10.0);  // the corner of the programmed square
    CHECK(p.moves[3].move.to.y == 0.0);
    CHECK(p.moves[4].move.to.x == 10.0);
    CHECK(p.moves[4].move.to.y == 7.0);
    CHECK(p.moves[5].move == moves[5]);
    for (const ProgramMove& m : p.moves) {
        CHECK_FALSE(m.omit);
    }
    // The z of every move is the tool's.
    CHECK(p.moves[3].move.to.z == -1.0);
}

TEST_CASE("cutter comp: arcs keep their centre, change radius by the tool radius, round corners vanish",
          "[toolpath][cuttercomp]") {
    // Tool on the left of a clockwise circle of radius 10 programmed about (0, 0): tool outside, radius 13.
    // Engage with a line to (13, 0) tangent to the circle there (travelling down: clockwise at +x).
    const std::vector<Move> moves{
        Move::rapid({16, 6, 5}),
        Move::plunge({16, 6, -1}),
        comp(Move::feed({13, 0, -1}), CutterComp::Left),
        comp(Move::arcCW({13, 0, -1}, {0, 0}), CutterComp::Left),  // full circle
        Move::feed({16, -6, -1}),
    };
    const CutterCompProgram p = cutterCompProgram(moves, kR, {});
    REQUIRE(p.ok());
    CHECK(p.moves[2].move.to.x == 10.0);
    CHECK(p.moves[2].move.to.y == 0.0);
    CHECK(p.moves[3].move.kind == MoveKind::ArcCW);
    CHECK(p.moves[3].move.centre == Vec2{0, 0});
    CHECK(p.moves[3].move.to.x == 10.0);
    CHECK(std::fabs(p.moves[3].move.to.y) < 1e-12);
    // A round corner: radius r about the corner point, which is one point of the programmed path.
    const std::vector<Move> corner{
        Move::rapid({-3, -10, 5}),
        Move::plunge({-3, -10, -1}),
        comp(Move::feed({-3, 0, -1}), CutterComp::Left),   // up the left of a part whose corner is the origin
        comp(Move::arcCW({0, 3, -1}, {0, 0}), CutterComp::Left),
        comp(Move::feed({10, 3, -1}), CutterComp::Left),
        Move::feed({10, 12, -1}),
    };
    // (tool on the left of an upward line at x = -3 means the programmed line is x = 0)
    const CutterCompProgram q = cutterCompProgram(corner, kR, {});
    REQUIRE(q.ok());
    CHECK(q.moves[3].omit);
    CHECK(q.moves[3].move.to.x == 0.0);
    CHECK(q.moves[3].move.to.y == 0.0);
    CHECK(q.moves[4].move.to.x == 10.0);
    CHECK(q.moves[4].move.to.y == 0.0);
}

TEST_CASE("cutter comp: inside corners between a line and an arc, and between two arcs, meet at the programmed corner",
          "[toolpath][cuttercomp]") {
    using std::cos;
    using std::sin;
    const double deg = geometry::kPi / 180.0;
    // Programmed path: a circle of radius 5 about c1 = (-3, 4) through the origin, counter-clockwise, tool on
    // the left (inside the circle): the tool-centre path is the circle of radius 2 about c1.
    const Vec2 c1{-3, 4};
    const Vec2 toolStart = c1 + Vec2{2 * cos(-150 * deg), 2 * sin(-150 * deg)};
    {
        // ... then straight up the line x = 0 (the tool-centre path x = -3).
        const std::vector<Move> moves{
            Move::rapid({-9, 0, 5}),
            Move::plunge({-9, 0, -1}),
            comp(Move::feed({toolStart.x, toolStart.y, -1}), CutterComp::Left),
            comp(Move::arcCCW({-3, 2, -1}, c1), CutterComp::Left),
            comp(Move::feed({-3, 7, -1}), CutterComp::Left),
            Move::feed({-8, 12, -1}),
        };
        const CutterCompProgram p = cutterCompProgram(moves, kR, {});
        REQUIRE(p.ok());
        const Vec2 engage = c1 + (toolStart - c1) * 2.5;  // radially out to radius 5
        CHECK(std::fabs(p.moves[2].move.to.x - engage.x) < 1e-9);
        CHECK(std::fabs(p.moves[2].move.to.y - engage.y) < 1e-9);
        CHECK(std::fabs(p.moves[3].move.to.x - 0.0) < 1e-9);  // arc meets the line at the origin
        CHECK(std::fabs(p.moves[3].move.to.y - 0.0) < 1e-9);
        CHECK(p.moves[3].move.centre == c1);
        CHECK(std::fabs(p.moves[4].move.to.x - 0.0) < 1e-9);
        CHECK(std::fabs(p.moves[4].move.to.y - 7.0) < 1e-9);
    }
    {
        // A line y = 0 into the arc: tool-centre line y = 3 meets the circle of radius 2 about c1 at x = -3 + sqrt(3).
        const double hit = -3.0 + std::sqrt(3.0);
        const Vec2 arcEnd = c1 + Vec2{2 * cos(60 * deg), 2 * sin(60 * deg)};
        const std::vector<Move> moves{
            Move::rapid({-12, 8, 5}),
            Move::plunge({-12, 8, -1}),
            comp(Move::feed({-10, 3, -1}), CutterComp::Left),
            comp(Move::feed({hit, 3, -1}), CutterComp::Left),
            comp(Move::arcCCW({arcEnd.x, arcEnd.y, -1}, c1), CutterComp::Left),
            Move::feed({arcEnd.x, arcEnd.y + 6, -1}),
        };
        const CutterCompProgram p = cutterCompProgram(moves, kR, {});
        REQUIRE(p.ok());
        CHECK(std::fabs(p.moves[2].move.to.x - (-10.0)) < 1e-9);  // perpendicular to the next move: straight down by r
        CHECK(std::fabs(p.moves[2].move.to.y - 0.0) < 1e-9);
        CHECK(std::fabs(p.moves[3].move.to.x - 0.0) < 1e-9);  // the line meets the arc at the origin
        CHECK(std::fabs(p.moves[3].move.to.y - 0.0) < 1e-9);
        const Vec2 programEnd = c1 + Vec2{5 * cos(60 * deg), 5 * sin(60 * deg)};
        CHECK(std::fabs(p.moves[4].move.to.x - programEnd.x) < 1e-9);
        CHECK(std::fabs(p.moves[4].move.to.y - programEnd.y) < 1e-9);
    }
    {
        // Two counter-clockwise arcs of radius 5, about c1 and c2 = (-4, 3), both through the origin.
        const Vec2 c2{-4, 3};
        // Tool-centre circles (radius 2) meet at the midpoint of the centres plus h along the normal.
        const Vec2 mid = (c1 + c2) * 0.5;
        const double h = std::sqrt(4.0 - 0.5);
        const Vec2 corner = mid + Vec2{1, -1} * (h / std::sqrt(2.0));
        const std::vector<Move> moves{
            Move::rapid({-8, -2, 5}),
            Move::plunge({-8, -2, -1}),
            comp(Move::feed({-3, 2, -1}), CutterComp::Left),
            comp(Move::arcCCW({corner.x, corner.y, -1}, c1), CutterComp::Left),
            comp(Move::arcCCW({-2, 3, -1}, c2), CutterComp::Left),
            Move::feed({-2, 9, -1}),
        };
        const CutterCompProgram p = cutterCompProgram(moves, kR, {});
        REQUIRE(p.ok());
        CHECK(std::fabs(p.moves[2].move.to.x - (-3.0)) < 1e-9);  // c1 + (0, -2) * 2.5
        CHECK(std::fabs(p.moves[2].move.to.y - (-1.0)) < 1e-9);
        CHECK(std::fabs(p.moves[3].move.to.x - 0.0) < 1e-9);
        CHECK(std::fabs(p.moves[3].move.to.y - 0.0) < 1e-9);
        CHECK(std::fabs(p.moves[4].move.to.x - 1.0) < 1e-9);  // c2 + (5, 0)
        CHECK(std::fabs(p.moves[4].move.to.y - 3.0) < 1e-9);
        CHECK(p.moves[3].move.centre == c1);
        CHECK(p.moves[4].move.centre == c2);
    }
    {
        // Arcs about the same centre with different radii cannot meet; circles that do not touch neither.
        // Out along a circle of radius 5 about c1 and back along it the other way: the moved arcs are concentric.
        const std::vector<Move> moves{
            Move::rapid({-8, -2, 5}),
            Move::plunge({-8, -2, -1}),
            comp(Move::feed({-3, -1, -1}), CutterComp::Left),
            comp(Move::arcCCW({-3, 9, -1}, c1), CutterComp::Left),  // radius 5, moved to 8
            comp(Move::arcCW({-3, -1, -1}, c1), CutterComp::Left),  // radius 5, moved to 2
            Move::feed({-2, 9, -1}),
        };
        const CutterCompProgram bad = cutterCompProgram(moves, kR, {});
        CHECK_FALSE(bad.ok());
        CHECK_THAT(bad.error, ContainsSubstring("do not meet"));
    }
}

TEST_CASE("cutter comp: moves without flags are returned unchanged and nothing needs a radius", "[toolpath][cuttercomp]") {
    const std::vector<Move> moves{Move::rapid({1, 2, 5}), Move::feed({3, 4, -1}), Move::arcCW({5, 6, -1}, {4, 4})};
    const CutterCompProgram p = cutterCompProgram(moves, std::nan(""), {});
    REQUIRE(p.ok());
    REQUIRE(p.moves.size() == 3);
    for (std::size_t i = 0; i < moves.size(); ++i) {
        CHECK(p.moves[i].move == moves[i]);
        CHECK_FALSE(p.moves[i].omit);
    }
    CHECK_FALSE(usesCutterComp(moves));
    CHECK(usesCutterComp({comp(Move::feed({0, 0, 0}), CutterComp::Right)}));
}

TEST_CASE("cutter comp: hostile move lists are refused with a message, never written", "[toolpath][cuttercomp][hostile]") {
    const auto good = []() {
        return std::vector<Move>{Move::rapid({0, 6, 5}),
                                 Move::plunge({0, 6, -1}),
                                 comp(Move::feed({3, 3, -1}), CutterComp::Left),
                                 comp(Move::feed({7, 3, -1}), CutterComp::Left),
                                 Move::feed({7, 12, -1})};
    };
    REQUIRE(cutterCompProgram(good(), kR, {}).ok());
    const auto error = [&](std::vector<Move> m, double radius = kR) { return cutterCompProgram(m, radius, {}).error; };

    CHECK_THAT(error(good(), 0.0), ContainsSubstring("tool radius above zero"));
    CHECK_THAT(error(good(), std::nan("")), ContainsSubstring("tool radius above zero"));
    CHECK_THAT(error(good(), -1.0), ContainsSubstring("tool radius above zero"));
    CHECK_THAT(error(good(), std::numeric_limits<double>::infinity()), ContainsSubstring("tool radius above zero"));
    CHECK(cutterCompProgram(good(), 0.0, {}).moves.empty());  // nothing partial

    {  // never released
        std::vector<Move> m = good();
        m.pop_back();
        CHECK_THAT(error(m), ContainsSubstring("never released"));
    }
    {  // side switch without release
        std::vector<Move> m = good();
        m[3].comp = CutterComp::Right;
        CHECK_THAT(error(m), ContainsSubstring("without releasing"));
    }
    {  // released by a vertical retract
        std::vector<Move> m = good();
        m.back() = Move::retract({7, 3, 5});
        CHECK_THAT(error(m), ContainsSubstring("released by a straight move in the plane"));
    }
    {  // released by an arc
        std::vector<Move> m = good();
        m.back() = Move::arcCW({7, 9, -1}, {7, 6});
        CHECK_THAT(error(m), ContainsSubstring("released by a straight move in the plane"));
    }
    {  // released by a rapid
        std::vector<Move> m = good();
        m.back() = Move::rapid({7, 12, 5});
        CHECK_THAT(error(m), ContainsSubstring("released by a straight move in the plane"));
    }
    {  // a rapid inside the run
        std::vector<Move> m = good();
        m[3] = comp(Move::rapid({7, 3, 5}), CutterComp::Left);
        CHECK_THAT(error(m), ContainsSubstring("rapid or dwell"));
    }
    {  // a dwell inside the run
        std::vector<Move> m = good();
        m[3] = comp(Move::dwell({3, 3, -1}, 1.0), CutterComp::Left);
        CHECK_THAT(error(m), ContainsSubstring("rapid or dwell"));
    }
    {  // engaged by an arc
        std::vector<Move> m = good();
        m[2] = comp(Move::arcCW({3, 3, -1}, {0, 3}), CutterComp::Left);
        CHECK_THAT(error(m), ContainsSubstring("engaged by a straight move"));
    }
    {  // engaged by a vertical move
        std::vector<Move> m = good();
        m[2] = comp(Move::feed({0, 6, -2}), CutterComp::Left);
        CHECK_THAT(error(m), ContainsSubstring("engaged by a straight move"));
    }
    {  // an arc smaller than the tool: clockwise with the tool on its left is outside, counter-clockwise inside
        const std::vector<Move> m{Move::rapid({0, 6, 5}),
                                  Move::plunge({0, 6, -1}),
                                  comp(Move::feed({3, 3, -1}), CutterComp::Left),
                                  comp(Move::arcCCW({3, 5, -1}, {3, 4}), CutterComp::Left),  // radius 1 going round inside
                                  comp(Move::feed({3, 9, -1}), CutterComp::Left),
                                  Move::feed({3, 12, -1})};
        // CCW with the tool on the left is inward: radius 1 + 3, fine. Clockwise with the tool on the left is
        // outward of the centre side: radius 1 - 3 < 0.
        CHECK(cutterCompProgram(m, kR, {}).ok());
        std::vector<Move> cw = m;
        cw[3] = comp(Move::arcCW({3, 5, -1}, {3, 4}), CutterComp::Left);
        CHECK_THAT(error(cw), ContainsSubstring("smaller than the tool radius"));
        CHECK_THAT(error(cw), ContainsSubstring("move 4"));
    }
    {  // a direction reversal: the moved moves are parallel and never meet
        const std::vector<Move> m{Move::rapid({0, 6, 5}),
                                  Move::plunge({0, 6, -1}),
                                  comp(Move::feed({3, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({13, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({3, 3, -1}), CutterComp::Left),
                                  Move::feed({0, 6, -1})};
        CHECK_THAT(error(m), ContainsSubstring("do not meet"));
    }
    {  // a very sharp inside corner: the moved moves meet far away
        const std::vector<Move> m{Move::rapid({0, 6, 5}),
                                  Move::plunge({0, 6, -1}),
                                  comp(Move::feed({3, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({13, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({3, 3.01, -1}), CutterComp::Left),
                                  Move::feed({0, 6, -1})};
        CHECK_THAT(error(m), ContainsSubstring("too sharp"));
    }
    {  // a corner that is sharp but not hostile: the moved moves meet 4.9 tool radii from the tool-path corner
        const double turn = 157.0 * geometry::kPi / 180.0;
        const Vec2 d2{std::cos(turn), std::sin(turn)};  // second direction, turned left by 157 degrees
        const std::vector<Move> m{Move::rapid({0, 6, 5}),
                                  Move::plunge({0, 6, -1}),
                                  comp(Move::feed({3, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({13, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({13 + 10 * d2.x, 3 + 10 * d2.y, -1}), CutterComp::Left),
                                  Move::feed({0, 6, -1})};
        const CutterCompProgram p = cutterCompProgram(m, kR, {});
        REQUIRE(p.ok());
        // The programmed corner is on the moved first line (y = 0).
        CHECK(std::fabs(p.moves[3].move.to.y) < 1e-9);
    }
    {  // a run at the start of the list needs the start position
        std::vector<Move> m{comp(Move::feed({3, 3, -1}), CutterComp::Left), comp(Move::feed({7, 3, -1}), CutterComp::Left),
                            Move::feed({7, 12, -1})};
        const CutterCompProgram p = cutterCompProgram(m, kR, {0, 6, -1});
        REQUIRE(p.ok());
        CHECK(p.moves[0].move.to.x == 3.0);
        CHECK(p.moves[0].move.to.y == 0.0);
    }
}

TEST_CASE("cutter comp: vertical moves inside a run keep the programmed position", "[toolpath][cuttercomp]") {
    const std::vector<Move> moves{Move::rapid({0, 6, 5}),
                                  Move::plunge({0, 6, -1}),
                                  comp(Move::feed({3, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({7, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({7, 3, 0}), CutterComp::Left),  // up over a tab
                                  comp(Move::feed({9, 3, 0}), CutterComp::Left),
                                  Move::feed({9, 9, 0})};
    const CutterCompProgram p = cutterCompProgram(moves, kR, {});
    REQUIRE(p.ok());
    CHECK(p.moves[3].move.to.x == 7.0);
    CHECK(p.moves[4].move.to.x == 7.0);  // same XY as before, new z
    CHECK(p.moves[4].move.to.y == 0.0);
    CHECK(p.moves[4].move.to.z == 0.0);
    CHECK(p.moves[5].move.to.x == 9.0);
}

TEST_CASE("cutter comp: a very short move is a move, not a vertical one", "[toolpath][cuttercomp]") {
    const std::vector<Move> moves{Move::rapid({0, 6, 5}),
                                  Move::plunge({0, 6, -1}),
                                  comp(Move::feed({3, 3, -1}), CutterComp::Left),
                                  comp(Move::feed({3.2, 3, -1}), CutterComp::Left),  // 0.2 mm
                                  comp(Move::feed({7, 3, -1}), CutterComp::Left),
                                  Move::feed({7, 12, -1})};
    const CutterCompProgram p = cutterCompProgram(moves, kR, {});
    REQUIRE(p.ok());
    CHECK(p.moves[2].move.to.x == 3.0);  // the engaging move ends perpendicular to the short move after it
    CHECK(std::fabs(p.moves[3].move.to.x - 3.2) < 1e-12);
    CHECK(std::fabs(p.moves[3].move.to.y - 0.0) < 1e-12);
    // A move of 1e-10 mm has no direction: it stays where the programmed path is.
    std::vector<Move> tiny = moves;
    tiny[3] = comp(Move::feed({3.0 + 1e-10, 3, -2}), CutterComp::Left);
    const CutterCompProgram q = cutterCompProgram(tiny, kR, {});
    REQUIRE(q.ok());
    CHECK(q.moves[3].move.to.x == q.moves[2].move.to.x);
    CHECK(q.moves[3].move.to.z == -2.0);
}

// ---------------------------------------------------------------------------------------------
// The Profile option
// ---------------------------------------------------------------------------------------------

TEST_CASE("profile cutter comp: outside square, climb: left, tool-centre path unchanged, programmed path on the vector",
          "[toolpath][cuttercomp][profile]") {
    const std::vector<Contour> vectors{square()};
    const tooling::Tool tool = test::endMill(6.0);
    const ProfileResult r = profileToolpath(vectors, tool, compParams());
    REQUIRE(r.ok());
    ProfileParams plain = compParams();
    plain.cutterComp = false;
    const ProfileResult base = profileToolpath(vectors, tool, plain);
    REQUIRE(base.ok());
    // The simulation sees the same tool-centre path as without the option.
    CHECK(withoutFlags(r.toolpath.moves) == base.toolpath.moves);
    CHECK_FALSE(usesCutterComp(base.toolpath.moves));

    const Run run = runOf(r.toolpath.moves);
    CHECK(r.toolpath.moves[run.first - 1].kind == MoveKind::Plunge);  // engaged right after the plunge
    CHECK(r.toolpath.moves[run.last + 1].comp == CutterComp::Off);    // released by the lead-out
    CHECK(r.toolpath.moves[run.last + 1].kind == MoveKind::Feed);
    for (std::size_t k = run.first; k <= run.last; ++k) {
        CHECK(r.toolpath.moves[k].comp == CutterComp::Left);
    }
    const CutterCompProgram p = cutterCompProgram(r.toolpath.moves, kR, {});
    REQUIRE(p.ok());
    std::size_t omitted = 0;
    for (std::size_t k = run.first; k <= run.last; ++k) {
        const Vec2 at = p.moves[k].move.to.xy();
        CHECK(test::bruteDistance(at, vectors) < 1e-6);  // the programmed path is the vector (allowance 0)
        omitted += p.moves[k].omit ? 1U : 0U;
        // The tool centre never comes nearer the vector than the radius.
        CHECK(test::bruteDistance(r.toolpath.moves[k].to.xy(), vectors) > kR - 1e-6);
    }
    CHECK(omitted == 4);  // the four round corners are points of the programmed path
    // The programmed path has no move outside the run changed.
    for (std::size_t k = 0; k < r.toolpath.moves.size(); ++k) {
        if (k < run.first || k > run.last) {
            CHECK(p.moves[k].move == r.toolpath.moves[k]);
        }
    }
}

TEST_CASE("profile cutter comp: side follows direction for outside, inside and the finishing reverse",
          "[toolpath][cuttercomp][profile]") {
    const std::vector<Contour> vectors{square()};
    const tooling::Tool tool = test::endMill(6.0);
    for (ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
        for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
            INFO("side " << static_cast<int>(side) << " direction " << static_cast<int>(dir));
            const ProfileResult r = profileToolpath(vectors, tool, compParams(side, dir));
            REQUIRE(r.ok());
            const Run run = runOf(r.toolpath.moves);
            CHECK(r.toolpath.moves[run.first].comp == (dir == CutDirection::Climb ? CutterComp::Left : CutterComp::Right));
            const CutterCompProgram p = cutterCompProgram(r.toolpath.moves, kR, {});
            REQUIRE(p.ok());
            for (std::size_t k = run.first; k <= run.last; ++k) {
                CHECK(test::bruteDistance(p.moves[k].move.to.xy(), vectors) < 1e-6);
            }
        }
    }
    // A reversed finishing pass flips the side of that pass only.
    ProfileParams two = compParams();
    two.separateLastPass = true;
    two.lastPassAllowance = 0.2;
    two.lastPassReverse = true;
    const ProfileResult r = profileToolpath(vectors, tool, two);
    REQUIRE(r.ok());
    std::vector<CutterComp> sides;
    CutterComp last = CutterComp::Off;
    for (const Move& m : r.toolpath.moves) {
        if (m.comp != CutterComp::Off && m.comp != last) {
            sides.push_back(m.comp);
        }
        last = m.comp;
    }
    REQUIRE(sides.size() == 2);
    CHECK(sides[0] == CutterComp::Left);
    CHECK(sides[1] == CutterComp::Right);
    CHECK(cutterCompProgram(r.toolpath.moves, kR, {}).ok());
}

TEST_CASE("profile cutter comp: inside square programs the vector's corners, allowance moves the programmed path",
          "[toolpath][cuttercomp][profile]") {
    const std::vector<Contour> vectors{square()};
    const tooling::Tool tool = test::endMill(6.0);
    const ProfileResult r = profileToolpath(vectors, tool, compParams(ProfileSide::Inside));
    REQUIRE(r.ok());
    const CutterCompProgram p = cutterCompProgram(r.toolpath.moves, kR, {});
    REQUIRE(p.ok());
    const Run run = runOf(r.toolpath.moves);
    std::size_t corners = 0;
    for (std::size_t k = run.first; k <= run.last; ++k) {
        const Vec2 at = p.moves[k].move.to.xy();
        for (Vec2 c : {Vec2{0, 0}, Vec2{40, 0}, Vec2{40, 30}, Vec2{0, 30}}) {
            if (geometry::distance(at, c) < 1e-9) {
                ++corners;
            }
        }
        CHECK_FALSE(p.moves[k].omit);
    }
    // The start corner is the tool path's sharp turn where the cut closes; the program meets the other three.
    CHECK(corners >= 3);

    // An allowance of 0.5 keeps the programmed path 0.5 mm off the vector, on the waste side.
    ProfileParams a = compParams();
    a.allowance = 0.5;
    const ProfileResult oa = profileToolpath(vectors, tool, a);
    REQUIRE(oa.ok());
    const CutterCompProgram pa = cutterCompProgram(oa.toolpath.moves, kR, {});
    REQUIRE(pa.ok());
    const Run ra = runOf(oa.toolpath.moves);
    for (std::size_t k = ra.first + 1; k <= ra.last; ++k) {
        if (pa.moves[k].omit) {
            continue;
        }
        const Vec2 at = pa.moves[k].move.to.xy();
        if (k != ra.first) {
            CHECK(std::fabs(test::bruteDistance(at, vectors) - 0.5) < 1e-6);
        }
    }
}

TEST_CASE("profile cutter comp: circles, outside and inside, keep their arcs", "[toolpath][cuttercomp][profile]") {
    const tooling::Tool tool = test::endMill(6.0);
    const std::vector<Contour> circle{Contour::circle({20, 20}, 10)};
    for (ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
        const ProfileResult r = profileToolpath(circle, tool, compParams(side));
        REQUIRE(r.ok());
        const CutterCompProgram p = cutterCompProgram(r.toolpath.moves, kR, {});
        REQUIRE(p.ok());
        const Run run = runOf(r.toolpath.moves);
        std::size_t arcs = 0;
        for (std::size_t k = run.first; k <= run.last; ++k) {
            const Move& m = p.moves[k].move;
            CHECK(test::bruteDistance(m.to.xy(), circle) < 1e-6);
            if (isArc(m.kind)) {
                ++arcs;
                CHECK(m.centre == r.toolpath.moves[k].centre);
            }
        }
        CHECK(arcs >= 1);
    }
}

TEST_CASE("profile cutter comp: tabs and several passes", "[toolpath][cuttercomp][profile]") {
    const std::vector<Contour> vectors{square()};
    const tooling::Tool tool = test::endMill(6.0);
    ProfileParams p = compParams();
    p.passes.cutDepth = 6.0;
    p.passes.passDepth = 2.0;
    p.tabs.enabled = true;
    p.tabs.length = 6.0;
    p.tabs.thickness = 1.5;
    p.tabs.count = 4;
    const ProfileResult r = profileToolpath(vectors, tool, p);
    REQUIRE(r.ok());
    p.cutterComp = false;
    const ProfileResult base = profileToolpath(vectors, tool, p);
    REQUIRE(base.ok());
    CHECK(withoutFlags(r.toolpath.moves) == base.toolpath.moves);
    const CutterCompProgram prog = cutterCompProgram(r.toolpath.moves, kR, {});
    REQUIRE(prog.ok());
    const Run run = runOf(r.toolpath.moves);
    // One run through every level (and over the tabs): moves keep their heights.
    for (std::size_t k = run.first; k <= run.last; ++k) {
        CHECK(prog.moves[k].move.to.z == r.toolpath.moves[k].to.z);
        CHECK(r.toolpath.moves[k].comp == CutterComp::Left);
    }
}

TEST_CASE("profile cutter comp: open vector in one pass, with a smooth ramp", "[toolpath][cuttercomp][profile]") {
    geometry::Polyline pl;
    pl.points = {{0, 0}, {50, 0}};
    const std::vector<Contour> open{Contour::fromPolyline(pl)};
    const tooling::Tool tool = test::endMill(6.0);
    for (ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
        const ProfileResult r = profileToolpath(open, tool, compParams(side));
        REQUIRE(r.ok());
        const CutterCompProgram p = cutterCompProgram(r.toolpath.moves, kR, {});
        REQUIRE(p.ok());
        const Run run = runOf(r.toolpath.moves);
        for (std::size_t k = run.first + 1; k <= run.last; ++k) {
            CHECK(std::fabs(p.moves[k].move.to.y) < 1e-9);  // programmed on the vector
        }
    }
    ProfileParams ramp = compParams();
    ramp.ramp.type = RampType::Smooth;
    ramp.ramp.distance = 20.0;
    ramp.passes.cutDepth = 3.0;
    const ProfileResult r = profileToolpath(open, tool, ramp);
    // Either a compensated toolpath whose program is derivable, or a refusal: never a program that fails.
    if (r.ok()) {
        CHECK(cutterCompProgram(r.toolpath.moves, kR, {}).ok());
    } else {
        CHECK(hasIssueOf(r, ToolpathIssue::CutterCompRefused));
    }
}

TEST_CASE("profile cutter comp: refusals, each with one diagnostic and no moves", "[toolpath][cuttercomp][profile][hostile]") {
    const tooling::Tool tool = test::endMill(6.0);
    const std::vector<Contour> vectors{square()};
    const auto refused = [&](const std::vector<Contour>& v, const tooling::Tool& t, const ProfileParams& p,
                             const std::string& why) {
        INFO(why);
        const ProfileResult r = profileToolpath(v, t, p);
        CHECK_FALSE(r.ok());
        CHECK(r.toolpath.moves.empty());
        CHECK(r.has(ToolpathIssue::CutterCompRefused));
        bool found = false;
        for (const ProfileDiagnostic& d : r.diagnostics) {
            INFO("diagnostic: " << d.message);
            if (d.issue == ToolpathIssue::CutterCompRefused) {
                found = found || d.message.find(why) != std::string::npos;
                CHECK_THAT(d.message, ContainsSubstring("cutter compensation"));
                CHECK_THAT(d.message, ContainsSubstring("nothing is cut"));
            }
        }
        CHECK(found);
    };
    {
        ProfileParams p = compParams(ProfileSide::On);
        refused(vectors, tool, p, "has no side");
    }
    {
        tooling::Tool v = test::endMill(6.0);
        v.type = tooling::ToolType::VBit;
        v.includedAngle = geometry::kPi / 2.0;
        refused(vectors, v, compParams(), "plain end mill");
    }
    {
        tooling::Tool b = test::endMill(6.0);
        b.type = tooling::ToolType::BallNose;
        refused(vectors, b, compParams(), "plain end mill");
    }
    {
        ProfileParams p = compParams();
        p.leads.type = LeadType::None;
        refused(vectors, tool, p, "Straight line with a lead-out");
    }
    {
        ProfileParams p = compParams();
        p.leads.type = LeadType::Circular;
        refused(vectors, tool, p, "Straight line with a lead-out");
    }
    {
        ProfileParams p = compParams();
        p.leads.leadOut = false;
        refused(vectors, tool, p, "Straight line with a lead-out");
    }
    {
        ProfileParams p = compParams();
        p.leads.length = 2.0;
        refused(vectors, tool, p, "at least the tool radius");
    }
    {
        ProfileParams p = compParams();
        p.ramp.type = RampType::ZigZag;
        p.ramp.distance = 10.0;
        refused(vectors, tool, p, "zig-zag");
    }
    {  // a curve
        geometry::BezierSpan bz{{0, 0}, {10, 20}, {30, 20}, {40, 0}};
        Contour c;
        c.spans = {bz, geometry::LineSpan{{40, 0}, {0, 0}}};
        c.closed = true;
        refused({c}, tool, compParams(), "lines and arcs");
    }
    {  // an open vector in several passes reverses at its end
        geometry::Polyline pl;
        pl.points = {{0, 0}, {50, 0}};
        ProfileParams p = compParams();
        p.passes.cutDepth = 8.0;
        p.passes.passDepth = 4.0;
        refused({Contour::fromPolyline(pl)}, tool, p, "several passes");
    }
    {  // a hole the tool fills: radius 3 against a 3 mm tool radius, and 3.5 is allowed
        refused({Contour::circle({20, 20}, 3.0)}, tool, compParams(ProfileSide::Inside), "no path along it");
        CHECK(profileToolpath({Contour::circle({20, 20}, 3.0)}, tool, compParams(ProfileSide::Inside)).diagnostics.size() == 1);
        ProfileParams roomy = compParams(ProfileSide::Inside);
        roomy.leads.length = 3.0;
        roomy.leads.angle = 0.0;
        const ProfileResult ok = profileToolpath({Contour::circle({20, 20}, 6.0)}, tool, roomy);
        for (const ProfileDiagnostic& d : ok.diagnostics) {
            INFO(d.message);
        }
        CHECK(ok.ok());
        // A hole that leaves the tool path 0.5 mm of radius has no room for the 5 mm lead-in.
        refused({Contour::circle({20, 20}, 3.5)}, tool, compParams(ProfileSide::Inside), "lead-in");
        // The allowance counts: radius 4 with +1.5 allowance is not larger than 3 + 1.5.
        ProfileParams p = compParams(ProfileSide::Inside);
        p.allowance = 1.5;
        refused({Contour::circle({20, 20}, 4.0)}, tool, p, "no path along it");
        // Outside, the same circle is convex to the tool: fine.
        CHECK(profileToolpath({Contour::circle({20, 20}, 3.0)}, tool, compParams()).ok());
    }
    {  // the lead-length boundary: exactly the tool radius is allowed
        ProfileParams p = compParams();
        p.leads.length = kR;
        CHECK(profileToolpath(vectors, tool, p).ok());
        p.leads.length = std::nextafter(kR, 0.0);
        refused(vectors, tool, p, "at least the tool radius");
    }
    {  // the last-pass allowance counts towards the arc check: radius 4 against 3 + 0 + 1.5
        ProfileParams p = compParams(ProfileSide::Inside);
        p.separateLastPass = true;
        p.lastPassAllowance = 1.5;
        refused({Contour::circle({20, 20}, 4.0)}, tool, p, "no path along it");
    }
    {  // an inside corner of the tool-centre path that is too sharp: the apex of a 10 degree wedge
        geometry::Polyline wedge;
        wedge.closed = true;
        wedge.points = {{0, 0}, {60, 0}, {60, 10.5}};
        ProfileParams p = compParams(ProfileSide::Inside);
        p.leads.length = 3.0;
        p.leads.angle = 0.0;
        refused({Contour::fromPolyline(wedge)}, tool, p, "too sharp to compensate");
        p.cutterComp = false;
        CHECK(profileToolpath({Contour::fromPolyline(wedge)}, tool, p).ok());
    }
    {  // negative allowances: the lead-in has no room, so the cut is refused (never written)
        ProfileParams p = compParams();
        p.allowance = -2.0;
        const ProfileResult r = profileToolpath(vectors, tool, p);
        CHECK_FALSE(r.ok());
        CHECK(r.has(ToolpathIssue::CutterCompRefused));
    }
    {  // two failing vectors report the first only
        geometry::Polyline a;
        a.points = {{0, 0}, {50, 0}};
        geometry::Polyline b;
        b.points = {{0, 40}, {50, 40}};
        ProfileParams p = compParams();
        p.passes.cutDepth = 8.0;
        p.passes.passDepth = 4.0;
        const ProfileResult r = profileToolpath({Contour::fromPolyline(a), Contour::fromPolyline(b)}, tool, p);
        std::size_t n = 0;
        for (const ProfileDiagnostic& d : r.diagnostics) {
            n += d.issue == ToolpathIssue::CutterCompRefused ? 1U : 0U;
        }
        CHECK(n == 1);
        CHECK(r.toolpath.moves.empty());
    }
    {  // leads with no room at either end of an open vector (the cut runs from x = 50 to x = 0 here): a square
       // beside the end, clear of the tool path, drops the lead or shortens it below the tool radius
        geometry::Polyline a;
        a.points = {{0, 0}, {50, 0}};
        const auto beside = [&](double x0, double x1) {
            return std::vector<Contour>{Contour::fromPolyline(a), Contour::rectangle(Box2{{x0, -12}, {x1, -6}})};
        };
        refused(beside(51, 59), tool, compParams(), "no lead-in fits");
        refused(beside(52, 60), tool, compParams(), "lead-in had to be shortened below the tool radius");
        refused(beside(-9, -1), tool, compParams(), "no lead-out fits");
        refused(beside(-10, -2), tool, compParams(), "lead-out had to be shortened below the tool radius");
        // Without the option the same vectors are cut, with a gentler or shorter lead.
        ProfileParams plain = compParams();
        plain.cutterComp = false;
        CHECK(profileToolpath(beside(-10, -2), tool, plain).ok());
        CHECK(profileToolpath(beside(52, 60), tool, plain).ok());
    }
    {  // neighbours that merge
        const std::vector<Contour> two{Contour::rectangle(Box2{{0, 0}, {10, 10}}),
                                       Contour::rectangle(Box2{{11, 0}, {21, 10}})};
        const ProfileResult r = profileToolpath(two, tool, compParams());
        CHECK_FALSE(r.ok());
        CHECK(r.has(ToolpathIssue::CutterCompRefused));
        // ... and without the option the same vectors give a toolpath.
        ProfileParams plain = compParams();
        plain.cutterComp = false;
        CHECK(profileToolpath(two, tool, plain).ok());
    }
}

TEST_CASE("profile cutter comp: a cut trimmed by a vector it does not merge with stays an exact offset",
          "[toolpath][cuttercomp][profile]") {
    // An open line 4 mm above the square: the cut round the square passes within the tool radius of it, so
    // that stretch is cut away (TrimmedNearVector). The pieces are exact offsets: compensated, each is a run
    // whose programmed path is derivable, and no tool-centre move comes nearer than the radius to either vector.
    geometry::Polyline a;
    a.points = {{-10, 34}, {50, 34}};
    const std::vector<Contour> vectors{square(), Contour::fromPolyline(a)};
    const tooling::Tool tool = test::endMill(6.0);
    const ProfileResult r = profileToolpath(vectors, tool, compParams());
    CHECK(r.has(ToolpathIssue::TrimmedNearVector));
    if (r.ok()) {
        CHECK(cutterCompProgram(r.toolpath.moves, kR, {}).ok());
        for (const Move& m : r.toolpath.moves) {
            if (m.comp != CutterComp::Off) {
                CHECK(test::bruteDistance(m.to.xy(), vectors) > kR - 1e-3);
            }
        }
    } else {
        CHECK(r.has(ToolpathIssue::CutterCompRefused));
        CHECK(r.toolpath.moves.empty());
    }
}

TEST_CASE("profile cutter comp: without the option nothing is flagged", "[toolpath][cuttercomp][profile]") {
    ProfileParams p = compParams();
    p.cutterComp = false;
    const ProfileResult r = profileToolpath({square()}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    CHECK_FALSE(usesCutterComp(r.toolpath.moves));
}

// ---------------------------------------------------------------------------------------------
// The post blocks
// ---------------------------------------------------------------------------------------------

namespace {

const std::string kCompPost = R"PP(
VAR X = [X|A| X|1.3]
VAR Y = [Y|A| Y|1.3]
VAR Z = [Z|A| Z|1.3]
VAR I = [I|A| I|1.3]
VAR J = [J|A| J|1.3]
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin PLUNGE_MOVE
"G1[X][Y][Z] P"
end
begin RETRACT_MOVE
"G1[X][Y][Z] R"
end
begin FEED_MOVE
"G1[X][Y][Z]"
end
begin CW_ARC_MOVE
"G2[X][Y][Z][I][J]"
end
begin CCW_ARC_MOVE
"G3[X][Y][Z][I][J]"
end
begin CUTTER_COMP_LEFT
"G41 D[CUTTER_COMP]"
end
begin CUTTER_COMP_RIGHT
"G42 D[CUTTER_COMP]"
end
begin CUTTER_COMP_OFF
"G40 D[CUTTER_COMP]"
end
)PP";

post::PostProcessor parsePost(const std::string& text) {
    auto r = post::PostProcessor::parse(text);
    if (!r.ok()) {
        FAIL("parse failed: " << r.error().describe());
    }
    return r.value();
}

std::vector<std::string> linesOf(const std::string& text) {
    std::vector<std::string> out;
    std::istringstream in(text);
    std::string l;
    while (std::getline(in, l)) {
        out.push_back(l);
    }
    return out;
}

std::string run(const post::PostProcessor& pp, const Toolpath& tp) {
    auto r = pp.run({tp}, {});
    if (!r.ok()) {
        FAIL("run failed: " << r.error().describe());
    }
    return r.value().text;
}

double word(const std::string& line, char letter) {
    const std::size_t i = line.find(letter);
    return i == std::string::npos ? std::nan("") : std::stod(line.substr(i + 1));
}

}  // namespace

TEST_CASE("post: CUTTER_COMP blocks come right before the engaging move and the releasing move",
          "[toolpath][cuttercomp][post]") {
    const std::vector<Contour> vectors{square()};
    const ProfileResult r = profileToolpath(vectors, test::endMill(6.0), compParams());
    REQUIRE(r.ok());
    const post::PostProcessor pp = parsePost(kCompPost);
    const std::vector<std::string> lines = linesOf(run(pp, r.toolpath));

    std::size_t g41 = lines.size();
    std::size_t g40 = lines.size();
    std::size_t n41 = 0;
    std::size_t n40 = 0;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].rfind("G41", 0) == 0) {
            g41 = i;
            ++n41;
        }
        if (lines[i].rfind("G40", 0) == 0) {
            g40 = i;
            ++n40;
        }
    }
    REQUIRE(n41 == 1);
    REQUIRE(n40 == 1);
    CHECK(lines[g41] == "G41 DLEFT");
    CHECK(lines[g40] == "G40 DOFF");
    CHECK(g41 < g40);
    // The plunge comes first, uncompensated; then the block, then the engaging move.
    CHECK(lines[g41 - 1].find(" P") != std::string::npos);
    const CutterCompProgram p = cutterCompProgram(r.toolpath.moves, kR, {});
    REQUIRE(p.ok());
    const Run rn = runOf(r.toolpath.moves);
    CHECK(std::fabs(word(lines[g41 + 1], 'X') - p.moves[rn.first].move.to.x) < 1e-3);
    CHECK(std::fabs(word(lines[g41 + 1], 'Y') - p.moves[rn.first].move.to.y) < 1e-3);
    // The move before the release is the last compensated one, written with the programmed end point.
    CHECK(std::fabs(word(lines[g40 - 1], 'X') - p.moves[rn.last].move.to.x) < 1e-3);
    CHECK(std::fabs(word(lines[g40 - 1], 'Y') - p.moves[rn.last].move.to.y) < 1e-3);
    // The release move is written at the tool-centre coordinates (no compensation any more).
    CHECK(std::fabs(word(lines[g40 + 1], 'X') - r.toolpath.moves[rn.last + 1].to.x) < 1e-3);
    CHECK(std::fabs(word(lines[g40 + 1], 'Y') - r.toolpath.moves[rn.last + 1].to.y) < 1e-3);
    // Every written position between the blocks is on the vector (the programmed path); the omitted round
    // corners write nothing: 4 fewer lines than moves in the run.
    std::size_t written = g40 - g41 - 1;
    std::size_t omitted = 0;
    for (std::size_t k = rn.first; k <= rn.last; ++k) {
        omitted += p.moves[k].omit ? 1U : 0U;
    }
    CHECK(written == (rn.last - rn.first + 1) - omitted);
}

TEST_CASE("post: conventional cutting writes the right-hand block; arcs are written with their programmed radius",
          "[toolpath][cuttercomp][post]") {
    const std::vector<Contour> circle{Contour::circle({20, 20}, 10)};
    const ProfileResult r = profileToolpath(circle, test::endMill(6.0), compParams(ProfileSide::Outside, CutDirection::Conventional));
    REQUIRE(r.ok());
    const post::PostProcessor pp = parsePost(kCompPost);
    const std::vector<std::string> lines = linesOf(run(pp, r.toolpath));
    bool found42 = false;
    bool arcOnVector = false;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (lines[i] == "G42 DRIGHT") {
            found42 = true;
        }
        if (lines[i].rfind("G2", 0) == 0 || lines[i].rfind("G3", 0) == 0) {
            // I and J are relative to the start of the move: the arc is about (20, 20) with radius 10.
            const double i0 = word(lines[i], 'I');
            const double j0 = word(lines[i], 'J');
            arcOnVector = arcOnVector || std::fabs(std::hypot(i0, j0) - 10.0) < 1e-2;
        }
        CHECK(lines[i].rfind("G41", 0) != 0);
    }
    CHECK(found42);
    CHECK(arcOnVector);
}

TEST_CASE("post: a post without the compensation blocks refuses a compensated toolpath", "[toolpath][cuttercomp][post]") {
    const ProfileResult r = profileToolpath({square()}, test::endMill(6.0), compParams());
    REQUIRE(r.ok());
    for (const char* missing : {"CUTTER_COMP_LEFT", "CUTTER_COMP_OFF"}) {
        std::string text = kCompPost;
        const std::size_t at = text.find(std::string("begin ") + missing);
        REQUIRE(at != std::string::npos);
        text.replace(at, std::string("begin ").size() + std::string(missing).size(),
                     std::string("begin UNUSED_") + missing);
        auto parsed = post::PostProcessor::parse(text);
        REQUIRE(parsed.ok());
        auto out = parsed.value().run({r.toolpath}, {});
        REQUIRE_FALSE(out.ok());
        CHECK_THAT(out.error().describe(), ContainsSubstring(missing));
    }
    // An uncompensated toolpath needs none of them.
    ProfileParams plain = compParams();
    plain.cutterComp = false;
    const ProfileResult b = profileToolpath({square()}, test::endMill(6.0), plain);
    REQUIRE(b.ok());
    auto parsed = post::PostProcessor::parse("VAR X = [X|A| X|1.3]\nbegin RAPID_MOVE\n\"G0[X]\"\nend\nbegin FEED_MOVE\n\"G1[X]\"\nend\n"
                                             "begin PLUNGE_MOVE\n\"G1[X]\"\nend\nbegin RETRACT_MOVE\n\"G1[X]\"\nend\n");
    REQUIRE(parsed.ok());
    CHECK(parsed.value().run({b.toolpath}, {}).ok());
}

TEST_CASE("post: a compensated toolpath whose programmed path cannot be derived is refused", "[toolpath][cuttercomp][post][hostile]") {
    Toolpath tp;
    tp.name = "Hand made";
    tp.tool.name = "T";
    tp.tool.toolNumber = 1;
    tp.tool.diameter = 6.0;
    tp.tool.feedRate = 1000.0;
    tp.tool.plungeRate = 300.0;
    tp.tool.spindleSpeed = 10000.0;
    tp.moves = {Move::rapid({0, 6, 5}), Move::plunge({0, 6, -1}), comp(Move::feed({3, 3, -1}), CutterComp::Left),
                comp(Move::feed({7, 3, -1}), CutterComp::Left)};  // never released
    const post::PostProcessor pp = parsePost(kCompPost);
    auto out = pp.run({tp}, {});
    REQUIRE_FALSE(out.ok());
    CHECK_THAT(out.error().describe(), ContainsSubstring("never released"));
    CHECK_THAT(out.error().describe(), ContainsSubstring("Hand made"));
}

TEST_CASE("post: only the blocks of the sides used are needed; a run at the start uses the position of the post",
          "[toolpath][cuttercomp][post]") {
    const ProfileResult climb = profileToolpath({square()}, test::endMill(6.0), compParams());
    REQUIRE(climb.ok());
    std::string text = kCompPost;
    const std::size_t at = text.find("begin CUTTER_COMP_RIGHT");
    REQUIRE(at != std::string::npos);
    text.replace(at, std::string("begin CUTTER_COMP_RIGHT").size(), "begin UNUSED_RIGHT");
    auto noRight = post::PostProcessor::parse(text);
    REQUIRE(noRight.ok());
    CHECK(noRight.value().run({climb.toolpath}, {}).ok());  // left only
    const ProfileResult conventional =
        profileToolpath({square()}, test::endMill(6.0), compParams(ProfileSide::Outside, CutDirection::Conventional));
    REQUIRE(conventional.ok());
    auto out = noRight.value().run({conventional.toolpath}, {});
    REQUIRE_FALSE(out.ok());
    CHECK_THAT(out.error().describe(), ContainsSubstring("CUTTER_COMP_RIGHT"));

    // A toolpath that starts with the engaging move starts from the post's home position.
    Toolpath tp;
    tp.name = "Starts engaged";
    tp.tool = climb.toolpath.tool;
    tp.moves = {comp(Move::feed({3, 3, -1}), CutterComp::Left), comp(Move::feed({7, 3, -1}), CutterComp::Left),
                Move::feed({7, 12, -1})};
    const post::PostProcessor pp = parsePost(kCompPost);
    post::PostContext far;
    far.home = {0, 6, -1};
    CHECK(pp.run({tp}, far).ok());
    post::PostContext onTarget;
    onTarget.home = {3, 3, -1};  // the engaging move would not move in the plane
    auto bad = pp.run({tp}, onTarget);
    REQUIRE_FALSE(bad.ok());
    CHECK_THAT(bad.error().describe(), ContainsSubstring("engaged by a straight move"));
}

TEST_CASE("post: a post with both jet and compensation blocks writes each toolpath with its own switching",
          "[toolpath][cuttercomp][post][laser]") {
    // T-0327 next to T-0323: the compensated Profile writes G41/G40 and no beam blocks; the laser toolpath
    // after it writes beam blocks and no compensation; a laser move list is not disturbed by the
    // compensation state and vice versa.
    const std::string both = std::string("LASER_SUPPORT = \"YES\"\nVAR POWER = [JET_POWER|A| S|1.0|10]\n") + kCompPost +
                             "begin JET_TOOL_POWER\n\"(power[JET_POWER])\"\nend\n"
                             "begin JET_TOOL_ON\n\"M4\"\nend\n"
                             "begin JET_TOOL_OFF\n\"M5\"\nend\n";
    const std::vector<Contour> vectors{square()};
    const ProfileResult r = profileToolpath(vectors, test::endMill(6.0), compParams());
    REQUIRE(r.ok());
    Toolpath laser;
    laser.name = "Laser";
    laser.tool.name = "Laser";
    laser.tool.toolNumber = 9;
    laser.tool.diameter = 0.2;
    laser.tool.feedRate = 800.0;
    laser.tool.plungeRate = 800.0;
    laser.tool.laser = true;
    laser.moves = {Move::rapid({0, 0, 0}), Move::feed({10, 0, 0}).withJet(50.0), Move::feed({10, 10, 0}).withJet(50.0),
                   Move::rapid({0, 0, 5})};
    const post::PostProcessor pp = parsePost(both);
    auto res = pp.run({r.toolpath, laser}, {});
    REQUIRE(res.ok());
    const std::vector<std::string> lines = linesOf(res.value().text);
    std::size_t g41 = 0, g40 = 0, m4 = 0, m5 = 0;
    std::size_t firstLaser = lines.size();
    for (std::size_t i = 0; i < lines.size(); ++i) {
        g41 += lines[i].rfind("G41", 0) == 0 ? 1U : 0U;
        g40 += lines[i].rfind("G40", 0) == 0 ? 1U : 0U;
        if (lines[i] == "M4") {
            ++m4;
            firstLaser = std::min(firstLaser, i);
        }
        m5 += lines[i] == "M5" ? 1U : 0U;
    }
    CHECK(g41 == 1);
    CHECK(g40 == 1);
    CHECK(m4 == 1);
    CHECK(m5 == 1);
    // The compensation is released before the laser toolpath starts.
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].rfind("G40", 0) == 0) {
            CHECK(i < firstLaser);
        }
    }
}
