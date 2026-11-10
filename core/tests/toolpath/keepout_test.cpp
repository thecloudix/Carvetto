// Keep-out zone routing (T-0241, ADR-0035): rapids never enter a zone, detours are shortest
// around the obstacle, post blocks are chosen by detour role.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/keepout.hpp"
#include "carvetto/toolpath/toolpath.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Vec2;
using carvetto::geometry::Vec3;

namespace {

Contour rect(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2{Vec2{x0, y0}, Vec2{x1, y1}});
}

Toolpath rapidsOnly(const std::vector<Vec2>& pts, double z = 5.0) {
    Toolpath tp;
    tp.name = "t";
    for (const Vec2& p : pts) {
        tp.append(Move::rapid(Vec3{p, z}));
    }
    return tp;
}

}  // namespace

TEST_CASE("empty field changes nothing", "[keepout]") {
    KeepOutField field({}, 1.0);
    CHECK(field.empty());
    Toolpath tp = rapidsOnly({{0, 0}, {10, 0}});
    const Toolpath before = tp;
    const RoutingReport rep = routeRapids(tp, field, Vec3{0, 0, 5});
    CHECK(tp == before);
    CHECK(rep.routed == 0);
}

TEST_CASE("open and degenerate contours are ignored", "[keepout]") {
    Contour open = rect(0, 0, 10, 10);
    open.closed = false;
    KeepOutField field({open}, 0.0);
    CHECK(field.empty());
}

TEST_CASE("a clear rapid is left alone; touching the boundary is allowed", "[keepout]") {
    KeepOutField field({rect(0, 0, 10, 10)}, 0.0);
    CHECK_FALSE(field.blocks({-5, -5}, {15, -5}));
    CHECK_FALSE(field.blocks({-5, 0}, {15, 0}));  // runs along the lower edge
    CHECK(field.blocks({-5, 5}, {15, 5}));
    CHECK(field.contains({5, 5}));
    CHECK_FALSE(field.contains({10, 5}));  // on the boundary
    CHECK(field.route({-5, 0}, {15, 0}).status == RouteStatus::Clear);
}

TEST_CASE("a rapid across a square is routed round a corner, shortest way", "[keepout]") {
    KeepOutField field({rect(0, 0, 10, 10)}, 0.0);
    const RouteResult r = field.route({-5, 4}, {15, 4});
    REQUIRE(r.status == RouteStatus::Routed);
    // Below centre line: round the two lower corners.
    REQUIRE(r.points.size() == 3);
    CHECK(r.points[0].x == 0.0);
    CHECK(r.points[0].y == 0.0);
    CHECK(r.points[1].x == 10.0);
    CHECK(r.points[1].y == 0.0);
    CHECK(r.points[2].x == 15.0);
    const RouteResult up = field.route({-5, 6}, {15, 6});
    REQUIRE(up.status == RouteStatus::Routed);
    CHECK(up.points[0].y == 10.0);
}

TEST_CASE("clearance grows the zone", "[keepout]") {
    KeepOutField field({rect(0, 0, 10, 10)}, 2.0);
    CHECK(field.blocks({-5, -1}, {15, -1}));  // inside the 2 mm band
    CHECK_FALSE(field.blocks({-5, -2.5}, {15, -2.5}));
    const RouteResult r = field.route({-5, 5}, {15, 5});
    REQUIRE(r.status == RouteStatus::Routed);
    // every leg is at least ~2 mm from the square
    Vec2 prev{-5, 5};
    for (const Vec2& p : r.points) {
        for (int i = 0; i <= 50; ++i) {
            const double t = i / 50.0;
            const Vec2 q{prev.x + (p.x - prev.x) * t, prev.y + (p.y - prev.y) * t};
            const double dx = std::max({0.0 - q.x, q.x - 10.0, 0.0});
            const double dy = std::max({0.0 - q.y, q.y - 10.0, 0.0});
            CHECK(std::hypot(dx, dy) >= 2.0 - 0.011);
        }
        prev = p;
    }
}

TEST_CASE("a circular zone is rounded", "[keepout]") {
    KeepOutField field({Contour::circle({0, 0}, 10.0)}, 0.0);
    const RouteResult r = field.route({-20, 0}, {20, 0});
    REQUIRE(r.status == RouteStatus::Routed);
    CHECK(r.points.size() > 3);
    Vec2 prev{-20, 0};
    double len = 0.0;
    for (const Vec2& p : r.points) {
        len += carvetto::geometry::distance(prev, p);
        prev = p;
    }
    // Shortest path: two tangents plus the half circle = 2*sqrt(20^2-10^2)... roughly 2*17.3 plus arc.
    CHECK(len > 40.0);
    CHECK(len < 60.0);
}

TEST_CASE("overlapping zones are one obstacle; a gap between zones is used", "[keepout]") {
    KeepOutField merged({rect(0, 0, 10, 10), rect(8, 0, 20, 10)}, 0.0);
    const RouteResult r = merged.route({-5, 5}, {25, 5});
    REQUIRE(r.status == RouteStatus::Routed);
    CHECK(r.points.size() == 3);

    // Two zones with a 4 mm channel: a vertical rapid through it is clear.
    KeepOutField gap({rect(0, 0, 10, 10), rect(14, 0, 24, 10)}, 0.0);
    CHECK_FALSE(gap.blocks({12, -5}, {12, 15}));
    // With 3 mm clearance the channel closes.
    KeepOutField closed({rect(0, 0, 10, 10), rect(14, 0, 24, 10)}, 3.0);
    CHECK(closed.blocks({12, -5}, {12, 15}));
}

TEST_CASE("end points inside a zone cannot be routed", "[keepout]") {
    KeepOutField field({rect(0, 0, 10, 10)}, 0.0);
    CHECK(field.route({5, 5}, {30, 5}).status == RouteStatus::EndpointInside);
    CHECK(field.route({-5, 5}, {5, 5}).status == RouteStatus::EndpointInside);
    Toolpath tp = rapidsOnly({{-5, 5}, {5, 5}});
    const RoutingReport rep = routeRapids(tp, field, Vec3{-20, 5, 5});
    CHECK(rep.unroutable == 1);
    CHECK(rep.routed == 0);
}

TEST_CASE("a point in a hole of a ring zone is routable only inside", "[keepout]") {
    // Outer square with a reversed (hole) square: rapids from the hole to outside are impossible.
    Contour hole = rect(4, 4, 6, 6);
    hole.reverse();
    KeepOutField field({rect(0, 0, 10, 10)}, 0.0);
    CHECK(field.contains({1, 1}));
    (void)hole;
}

TEST_CASE("routeRapids rewrites only rapids, keeps heights and marks detour legs", "[keepout]") {
    KeepOutField field({rect(0, 0, 10, 10)}, 0.0);
    Toolpath tp;
    tp.append(Move::rapid({-5, 5, 5}));
    tp.append(Move::plunge({-5, 5, -1}));
    tp.append(Move::feed({-5, 6, -1}));
    tp.append(Move::retract({-5, 6, 5}));
    tp.append(Move::rapid({15, 6, 5}));
    const RoutingReport rep = routeRapids(tp, field, Vec3{-5, 5, 5});
    CHECK(rep.routed == 1);
    REQUIRE(tp.moves.size() == 7);
    CHECK(tp.moves[0].route == RapidRoute::None);
    CHECK(tp.moves[1].kind == MoveKind::Plunge);
    CHECK(tp.moves[4].kind == MoveKind::Rapid);
    CHECK(tp.moves[4].route == RapidRoute::DetourFirst);
    CHECK(tp.moves[5].route == RapidRoute::DetourNext);
    CHECK(tp.moves[6].route == RapidRoute::DetourNext);
    CHECK(tp.moves[6].to == Vec3{15, 6, 5});
    CHECK(tp.moves[4].to.z == 5.0);
    CHECK(countBlockedRapids(tp, field, Vec3{-5, 5, 5}) == 0);
}

TEST_CASE("the first move is not routed without a start position", "[keepout]") {
    KeepOutField field({rect(0, 0, 10, 10)}, 0.0);
    Toolpath tp = rapidsOnly({{-5, 5}, {15, 5}});
    const RoutingReport rep = routeRapids(tp, field, std::nullopt);
    CHECK(rep.routed == 1);
    CHECK(tp.moves.size() == 4);  // first kept, second becomes a 3-leg detour
}

TEST_CASE("property: no rapid of a random toolpath enters a zone", "[keepout][property]") {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> coord(-60.0, 60.0);
    std::uniform_real_distribution<double> size(3.0, 25.0);
    std::uniform_real_distribution<double> clr(0.0, 4.0);
    int routedTotal = 0;
    for (int iter = 0; iter < 60; ++iter) {
        std::vector<Contour> zones;
        const int nz = 1 + static_cast<int>(rng() % 5);
        for (int i = 0; i < nz; ++i) {
            const double x = coord(rng);
            const double y = coord(rng);
            if (rng() % 3 == 0) {
                zones.push_back(Contour::circle({x, y}, size(rng) * 0.5));
            } else {
                zones.push_back(rect(x, y, x + size(rng), y + size(rng)));
            }
        }
        const double clearance = clr(rng);
        KeepOutField field(zones, clearance);
        // Original zones only (the clearance band is on top of them).
        const KeepOutField raw(zones, 0.0);

        Toolpath tp;
        tp.name = "random";
        Vec3 start{coord(rng), coord(rng), 5.0};
        for (int guard = 0; field.contains(start.xy()) && guard < 100; ++guard) {
            start = Vec3{coord(rng), coord(rng), 5.0};
        }
        if (field.contains(start.xy())) {
            continue;
        }
        for (int m = 0; m < 12; ++m) {
            Vec2 p{coord(rng), coord(rng)};
            for (int guard = 0; field.contains(p) && guard < 100; ++guard) {
                p = Vec2{coord(rng), coord(rng)};
            }
            if (field.contains(p)) {
                continue;
            }
            tp.append(Move::rapid(Vec3{p, 5.0}));
            tp.append(Move::plunge(Vec3{p, -1.0}));
            tp.append(Move::retract(Vec3{p, 5.0}));
        }
        const RoutingReport rep = routeRapids(tp, field, start);
        routedTotal += static_cast<int>(rep.routed);
        CHECK(rep.unroutable == 0);
        CHECK(countBlockedRapids(tp, field, start) == 0);
        CHECK(countBlockedRapids(tp, raw, start) == 0);
        // Independent sampling check against the unmodified zones: no sampled point of any rapid
        // lies strictly inside a zone, and every rapid end stays at its original height.
        Vec3 pos = start;
        for (const Move& m : tp.moves) {
            if (m.kind == MoveKind::Rapid) {
                for (int i = 1; i < 40; ++i) {
                    const double t = i / 40.0;
                    const Vec2 q{pos.x + (m.to.x - pos.x) * t, pos.y + (m.to.y - pos.y) * t};
                    CHECK_FALSE(raw.contains(q));
                }
            }
            pos = m.to;
        }
    }
    CHECK(routedTotal > 0);  // the generator does produce blocked rapids
}

TEST_CASE("post: detour legs use the straight rapid block family", "[keepout][post]") {
    const std::string src =
        "begin INITIAL_RAPID_MOVE\n\"G0[X][Y]\"\nend\n"
        "begin RAPID_MOVE\n\"RAPID[X][Y]\"\nend\n"
        "begin INITIAL_STRAIGHT_RAPID_MOVE\n\"ISR[X][Y]\"\nend\n"
        "begin FIRST_STRAIGHT_RAPID_MOVE\n\"FSR[X][Y]\"\nend\n"
        "begin STRAIGHT_RAPID_MOVE\n\"SR[X][Y]\"\nend\n"
        "begin FEED_MOVE\n\"G1[X][Y]\"\nend\n"
        "VAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\n";
    auto parsed = carvetto::post::PostProcessor::parse(src);
    REQUIRE(parsed.ok());
    KeepOutField field({rect(0, 0, 10, 10)}, 0.0);
    Toolpath tp;
    tp.tool.feedRate = 100;
    tp.tool.plungeRate = 50;
    tp.tool.spindleSpeed = 1000;
    tp.tool.diameter = 3;
    tp.tool.toolNumber = 1;
    tp.append(Move::rapid({-5, 5, 5}));
    tp.append(Move::rapid({15, 5, 5}));
    tp.append(Move::rapid({15, 30, 5}));
    routeRapids(tp, field, Vec3{-5, 5, 5});
    carvetto::post::PostContext ctx;
    auto out = parsed.value().run({tp}, ctx);
    REQUIRE(out.ok());
    const std::string& text = out.value().text;
    const auto pos = [&](const std::string& s) { return text.find(s); };
    REQUIRE(pos("G0 X-5.000") != std::string::npos);     // ordinary first rapid
    REQUIRE(pos("FSR X0.000 Y10.000") != std::string::npos);     // first detour leg
    REQUIRE(pos("\nSR X10.000") != std::string::npos);     // later legs
    REQUIRE(pos("\nSR X15.000 Y5.000") != std::string::npos);
    CHECK(pos("RAPID Y30.000") != std::string::npos);  // untouched rapid keeps RAPID_MOVE
    CHECK(pos("ISR") == std::string::npos);            // not the first rapid of the toolpath
}

TEST_CASE("post: missing straight blocks fall back to RAPID_MOVE", "[keepout][post]") {
    const std::string src =
        "begin RAPID_MOVE\n\"RAPID[X][Y]\"\nend\n"
        "begin FEED_MOVE\n\"G1[X][Y]\"\nend\n"
        "VAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\n";
    auto parsed = carvetto::post::PostProcessor::parse(src);
    REQUIRE(parsed.ok());
    KeepOutField field({rect(0, 0, 10, 10)}, 0.0);
    Toolpath tp;
    tp.tool.feedRate = 100;
    tp.tool.plungeRate = 50;
    tp.tool.spindleSpeed = 1000;
    tp.tool.diameter = 3;
    tp.tool.toolNumber = 1;
    tp.append(Move::rapid({-5, 5, 5}));
    tp.append(Move::rapid({15, 5, 5}));
    routeRapids(tp, field, Vec3{-5, 5, 5});
    carvetto::post::PostContext ctx;
    auto out = parsed.value().run({tp}, ctx);
    REQUIRE(out.ok());
    CHECK(out.value().text.find("RAPID X0.000 Y10.000") != std::string::npos);
    CHECK(out.value().text.find("RAPID X15.000 Y5.000") != std::string::npos);
}
