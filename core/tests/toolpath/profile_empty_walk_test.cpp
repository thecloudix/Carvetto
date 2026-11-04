// T-0163: a closed cut can collapse to a point when its start is chosen (here: the roughing cut of
// a tiny Bezier blob, On with a separate last pass, polygonised at a chordal tolerance much larger
// than the blob, offsets to a single line that closes onto itself). Its walk then has no spans;
// planning its lead-in used to sort `spans.begin() + 1 .. end()` of an empty list (undefined
// behaviour; a segfault). Found by the T-0137 verifier (harness t0137v1, seg_241_31.scn,
// reduced to the one vector that triggers it).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::BezierSpan;
using carvetto::geometry::Contour;
using carvetto::geometry::Vec2;

namespace {

Contour tinyBlob() {
    Contour c;
    c.closed = true;
    c.spans = {
        BezierSpan{{-0.187086822, -0.120915918}, {-0.187087152, -0.12069675}, {-0.187207274, -0.120374426},
                   {-0.187405755, -0.120228396}},
        BezierSpan{{-0.187405755, -0.120228396}, {-0.187604236, -0.120082366}, {-0.188050888, -0.119920716},
                   {-0.188277708, -0.120039738}},
        BezierSpan{{-0.188277708, -0.120039738}, {-0.188504528, -0.12015876}, {-0.188793055, -0.120695069},
                   {-0.188766163, -0.120946042}},
        BezierSpan{{-0.188766163, -0.120946042}, {-0.18873927, -0.121197014}, {-0.188495223, -0.121448916},
                   {-0.188269986, -0.121547799}},
        BezierSpan{{-0.188269986, -0.121547799}, {-0.188044749, -0.121646683}, {-0.187730812, -0.121651905},
                   {-0.187564689, -0.121547244}},
        BezierSpan{{-0.187564689, -0.121547244}, {-0.187398566, -0.121442583}, {-0.187088059, -0.121145133},
                   {-0.187086822, -0.120915918}},
    };
    return c;
}

tooling::Tool tinyVBit() {
    tooling::Tool t = test::endMill(0.004);
    t.type = tooling::ToolType::VBit;
    t.includedAngle = 30.0 * geometry::kPi / 180.0;
    return t;
}

ProfileParams params(LeadType lead) {
    ProfileParams p;
    p.side = ProfileSide::On;
    // Deep enough that the tool radius at the cut depth (0.002 * tan(15 deg) = 0.000536 mm) stays above
    // the last-pass allowance, which On requires (T-0137).
    p.passes.cutDepth = 0.002;
    p.passes.passDepth = 0.005;
    p.separateLastPass = true;
    p.lastPassAllowance = 0.0003777814985338173;
    p.leads.type = lead;
    p.leads.length = 0.0006;
    p.leads.radius = 0.0003;
    p.startAt = StartAt::Optimise;
    return p;
}

}  // namespace

TEST_CASE("T-0163: lead-in planning skips a closed cut that collapsed to a point",
          "[toolpath][profile][leads][T-0163]") {
    const std::vector<Contour> cs{tinyBlob()};
    const tooling::Tool tool = tinyVBit();
    // Every move stays near the vector: the vector's box grown by the tool radius, the lead and
    // the last-pass allowance (with room to spare), never at a point the empty cut made up.
    geometry::Box2 box = cs[0].bounds();
    const double grow = 0.01;
    for (LeadType lead : {LeadType::Straight, LeadType::Circular}) {
        CAPTURE(static_cast<int>(lead));
        const ProfileResult r = profileToolpath(cs, tool, params(lead));
        REQUIRE_FALSE(r.toolpath.moves.empty());
        for (const Move& m : r.toolpath.moves) {
            REQUIRE(std::isfinite(m.to.x));
            REQUIRE(std::isfinite(m.to.y));
            REQUIRE(std::isfinite(m.to.z));
            if (m.kind == MoveKind::Dwell) {
                continue;
            }
            CHECK(m.to.x >= box.min.x - grow);
            CHECK(m.to.x <= box.max.x + grow);
            CHECK(m.to.y >= box.min.y - grow);
            CHECK(m.to.y <= box.max.y + grow);
        }
        for (const ToolpathDiagnostic& d : r.diagnostics) {
            CHECK(d.issue != ProfileIssue::LeadDropped);
        }
    }
}

TEST_CASE("T-0163: the collapsed cut emits nothing with leads or without", "[toolpath][profile][leads][T-0163]") {
    // The empty cut is skipped by lead planning exactly as emitting skips it, so leads change
    // only how the other (finishing) cut enters: the same cuts are emitted, each ending in one
    // retract to safe Z.
    const std::vector<Contour> cs{tinyBlob()};
    auto retracts = [](const ProfileResult& r) {
        std::size_t n = 0;
        for (const Move& m : r.toolpath.moves) {
            if (m.kind == MoveKind::Retract) {
                ++n;
            }
        }
        return n;
    };
    const ProfileResult none = profileToolpath(cs, tinyVBit(), params(LeadType::None));
    const ProfileResult with = profileToolpath(cs, tinyVBit(), params(LeadType::Straight));
    REQUIRE(retracts(none) > 0);
    CHECK(retracts(with) == retracts(none));
}
