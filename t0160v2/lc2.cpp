// t0160v2 lc2 (from t0160v1 lc + global floor scan + round-2 cases): corner sharpening checks per case (branch library; also links against main for "off" identity in hs.cpp).
// usage: lc [filter-substring] [--rand seed N]   env: LC_G=<fine grid step> LC_VERB=1 LC_DUMP=1
// Per case: run vCarvingToolpath with cornerSharpen on and off.
//  ins: on == off + inserted complete cuts (rapid safeZ, plunge to floor, feeds at floor, retract); clearance identical; diags identical
//  safe: every inserted feed: exact min dist(centre, original curves) - rho  (needs >= c/2 = chordal/6; cone needs >= 0)
//  left: per convex corner vertex (found by this driver from the curves), max leftover on the floor (dist >= rho + c) within
//        the corner's disc, off and on (grid + pattern-search refinement), vs ridge h(s/2); plus global max over the floor.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <vector>
#include <unordered_map>
#include "lo.hpp"
#include "shapes.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
static const double PI = 3.14159265358979323846;
static tooling::Tool mk(int type, double D, double angDeg, const std::string& name, int num, double so) {
    tooling::Tool t; t.id = name; t.name = name; t.diameter = D;
    t.type = type == 0 ? tooling::ToolType::VBit : type == 1 ? tooling::ToolType::BallNose : tooling::ToolType::EndMill;
    if (type == 0) t.includedAngle = angDeg * PI / 180;
    auto& c = t.defaultCuttingData; c.toolNumber = num; c.feedRate = 1000; c.plungeRate = 300; c.spindleSpeed = 18000; c.passDepth = 3; c.stepover = so; return t;
}
struct Case { std::string name; std::vector<Contour> cs; int ct = 0; double D = 12.7, ang = 90, so = 0.5; std::vector<double> mills{6}; VCarvingParams p; std::vector<Vec2> probes; double probeAng = PI / 2; };
static Contour arcC(Vec2 o, double r, double a0, double a1, bool cw) { Contour c; c.append(ArcSpan{o, r, a0, a1, cw ? ArcDirection::CW : ArcDirection::CCW}); return c; }
// path builder: sequence of spans
struct PB { Contour c; PB() { c.closed = true; } PB& L(Vec2 a, Vec2 b) { c.append(LineSpan{a, b}); return *this; }
    PB& A(Vec2 o, double r, double a0, double a1, bool cw) { c.append(ArcSpan{o, r, a0, a1, cw ? ArcDirection::CW : ArcDirection::CCW}); return *this; }
    PB& B(Vec2 a, Vec2 b, Vec2 d, Vec2 e) { c.append(BezierSpan{a, b, d, e}); return *this; } };
static Contour roundPoly(const std::vector<Vec2>& p, double rf, int segs) { // CCW polygon with every corner rounded by radius rf, as an arc (segs=0) or segs line pieces
    Contour c; c.closed = true; size_t n = p.size(); std::vector<Vec2> a(n), b(n); std::vector<Vec2> ctr(n); std::vector<double> a0(n), a1(n);
    for (size_t i = 0; i < n; ++i) { Vec2 P = p[i], Pp = p[(i + n - 1) % n], Pn = p[(i + 1) % n]; Vec2 u1 = (Pp - P) / distance(Pp, P), u2 = (Pn - P) / distance(Pn, P);
        double half = 0.5 * std::acos(std::clamp(u1.dot(u2), -1.0, 1.0)); double t = rf / std::tan(half); a[i] = P + u1 * t; b[i] = P + u2 * t; Vec2 bis = (u1 + u2) / (u1 + u2).length(); ctr[i] = P + bis * (rf / std::sin(half));
        a0[i] = std::atan2(a[i].y - ctr[i].y, a[i].x - ctr[i].x); a1[i] = std::atan2(b[i].y - ctr[i].y, b[i].x - ctr[i].x); }
    for (size_t i = 0; i < n; ++i) {
        if (rf > 0) { if (segs == 0) c.append(ArcSpan{ctr[i], rf, a0[i], a1[i], ArcDirection::CCW}); else { double s = a1[i] - a0[i]; while (s < 0) s += 2 * PI; Vec2 prev = a[i]; for (int k = 1; k <= segs; ++k) { double an = a0[i] + s * k / segs; Vec2 q = k == segs ? b[i] : Vec2{ctr[i].x + rf * std::cos(an), ctr[i].y + rf * std::sin(an)}; c.append(LineSpan{prev, q}); prev = q; } } }
        c.append(LineSpan{rf > 0 ? b[i] : p[i], rf > 0 ? a[(i + 1) % n] : p[(i + 1) % n]}); }
    return c; }
static Contour chamferPoly(const std::vector<Vec2>& p, double ch) { std::vector<Vec2> q; size_t n = p.size(); for (size_t i = 0; i < n; ++i) { Vec2 P = p[i], Pp = p[(i + n - 1) % n], Pn = p[(i + 1) % n]; q.push_back(P + (Pp - P) / distance(Pp, P) * (ch / std::sqrt(2.0))); q.push_back(P + (Pn - P) / distance(Pn, P) * (ch / std::sqrt(2.0))); } return vs::poly(q); }
static std::vector<Vec2> ngon(int n, double R, Vec2 o = {0, 0}, double rot = 0) { std::vector<Vec2> v; for (int i = 0; i < n; ++i) v.push_back({o.x + R * std::cos(rot + 2 * PI * i / n), o.y + R * std::sin(rot + 2 * PI * i / n)}); return v; }
static Contour xf(const Contour& c, double ang, Vec2 off) { return c.transformed(Transform2::translation(off) * Transform2::rotation(ang)); }

static std::vector<Case> cases() {
    std::vector<Case> v; auto add = [&](Case c) { v.push_back(c); };
    auto base = [](std::string n, std::vector<Contour> cs) { Case c; c.name = n; c.cs = cs; c.p.flatDepth = 1.0; c.p.tolerance.chordal = 0.05; c.p.safeZ = 5; return c; };
    std::vector<Vec2> sq{{0, 0}, {20, 0}, {20, 20}, {0, 20}};
    add(base("square", {vs::poly(sq)}));
    { Case c = base("square_raster", {vs::poly(sq)}); c.p.clearanceStrategy = PocketStrategy::Raster; c.p.clearanceRasterAngle = 0.4; add(c); }
    { Case c = base("square_mills6_1", {vs::poly(sq)}); c.mills = {6, 1}; add(c); }
    { Case c = base("square_mills1_6", {vs::poly(sq)}); c.mills = {1, 6}; add(c); }
    { Case c = base("square_mills6_3_1", {vs::poly(sq)}); c.mills = {6, 3, 1}; add(c); }
    { Case c = base("square_mills6_3_1_raster", {vs::poly(sq)}); c.mills = {6, 3, 1}; c.p.clearanceStrategy = PocketStrategy::Raster; add(c); }
    { Case c = base("square_V60", {vs::poly(sq)}); c.ang = 60; c.so = 0.4; add(c); }
    { Case c = base("square_V30", {vs::poly(sq)}); c.ang = 30; c.so = 0.2; c.p.flatDepth = 2.0; add(c); }
    { Case c = base("square_V150", {vs::poly(sq)}); c.ang = 150; c.D = 25; c.so = 2.0; c.p.flatDepth = 0.6; add(c); }
    { Case c = base("square_ball6", {vs::poly(sq)}); c.ct = 1; c.D = 6; c.so = 0.8; add(c); }
    { Case c = base("square_ch0.005", {vs::poly(sq)}); c.p.tolerance.chordal = 0.005; c.p.tolerance.distance = 0.001; add(c); }
    { Case c = base("square_so_big", {vs::poly(sq)}); c.so = 5; add(c); }          // stepover clamped to 2 rho
    { Case c = base("square_so0", {vs::poly(sq)}); c.so = 0; add(c); }             // no stepover: rho
    { Case c = base("square_start", {vs::poly(sq)}); c.p.startDepth = 1.3; add(c); }
    { Case c = base("square_far1e5", {xf(vs::poly(sq), 0.3, {1e5, -7e4})}); add(c); }
    { Case c = base("square_far1e6_ch001", {xf(vs::poly(sq), 0.3, {1e6, -7e5})}); c.p.tolerance.chordal = 0.001; c.p.tolerance.distance = 0.001; add(c); }
    for (double w : {3.0, 4.0, 5.0, 6.2, 7.0, 8.2, 9.0}) { Case c = base("rect_w" + std::to_string(w).substr(0, 4), {vs::poly({{0, 0}, {30, 0}, {30, w}, {0, w}})}); add(c); }
    for (double w : {6.2, 9.0}) { Case c = base("rect_w" + std::to_string(w).substr(0, 4) + "_m6_1", {vs::poly({{0, 0}, {30, 0}, {30, w}, {0, w}})}); c.mills = {6, 1}; add(c); }
    for (double a : {10.0, 20.0, 30.0, 45.0, 60.0, 90.0, 120.0, 150.0}) { double h = 15; double hw = h * std::tan(a * PI / 360);
        Case c = base("tri_apex" + std::to_string((int)a), {vs::poly({{0, 0}, {h, -hw}, {h, hw}})}); add(c);
        Case d = c; d.name += "_m3_1"; d.mills = {3, 1}; add(d); }
    { Case c = base("L", {vs::poly({{0, 0}, {30, 0}, {30, 12}, {12, 12}, {12, 30}, {0, 30}})}); add(c); }
    { Case c = base("Lhole", {Contour::rectangle(Box2{{0, 0}, {30, 30}}), vs::rev(vs::poly({{10, 10}, {20, 10}, {20, 14}, {14, 14}, {14, 20}, {10, 20}}))}); add(c); }
    { Case c = base("Lhole_m6_1", {Contour::rectangle(Box2{{0, 0}, {30, 30}}), vs::rev(vs::poly({{10, 10}, {20, 10}, {20, 14}, {14, 14}, {14, 20}, {10, 20}}))}); c.mills = {6, 1}; add(c); }
    { Case c = base("starhole", {Contour::rectangle(Box2{{-20, -20}, {20, 20}}), vs::rev(vs::star({0, 0}, 9, 4, 5))}); c.mills = {3}; add(c); }
    { Case c = base("star5", {vs::star({0, 0}, 15, 6, 5)}); c.mills = {3}; add(c); }
    { Case c = base("star5_m3_1", {vs::star({0, 0}, 15, 6, 5)}); c.mills = {3, 1}; add(c); }
    { Case c = base("star8_thin", {vs::star({0, 0}, 15, 9, 8)}); c.mills = {2}; add(c); }
    for (int n : {5, 6, 8, 12}) { Case c = base("ngon" + std::to_string(n), {vs::poly(ngon(n, 12))}); add(c); }
    // corners near other walls: island close to a corner; a hole whose corner points into the flat area
    for (double g : {1.6, 2.5, 3.5, 5.0}) { Case c = base("isle_near_corner_g" + std::to_string(g).substr(0, 3), {vs::poly(sq), vs::rev(vs::poly({{g, g}, {g + 3, g}, {g + 3, g + 3}, {g, g + 3}}))}); add(c); }
    for (double g : {2.0, 3.0, 4.0}) { Case c = base("diamondhole_near_corner_g" + std::to_string(g).substr(0, 3), {vs::poly(sq), vs::rev(vs::poly({{g + 2, g}, {g + 4, g + 2}, {g + 2, g + 4}, {g, g + 2}}))}); c.mills = {3}; add(c); }
    // narrow necks
    for (double w : {3.0, 4.2, 6.0, 7.2}) { Case c = base("neck_w" + std::to_string(w).substr(0, 3), {vs::poly({{0, 0}, {12, 0}, {12, 6 - w / 2}, {20, 6 - w / 2}, {20, 0}, {32, 0}, {32, 12}, {20, 12}, {20, 6 + w / 2}, {12, 6 + w / 2}, {12, 12}, {0, 12}})}); add(c); }
    // curved edges
    { Case c = base("Dshape", {PB().L({0, -10}, {0, 10}).A({0, 0}, 10, PI / 2, -PI / 2, true).c}); c.cs[0].reverse(); add(c); }  // CCW: line up the left?
    { Case c = base("lens", {PB().A({0, -8}, 14, PI / 2 - 0.9, PI / 2 + 0.9, false).A({0, 8}, 14, -PI / 2 - 0.9, -PI / 2 + 0.9, false).c}); add(c); }
    { // curved triangle with concave arcs (region edges bowing inwards): sharp cusp-like corners
        std::vector<Vec2> t = ngon(3, 14, {0, 0}, PI / 2); PB b; for (int i = 0; i < 3; ++i) { Vec2 a = t[i], e = t[(i + 1) % 3]; Vec2 m = (a + e) * 0.5; Vec2 out = m / m.length(); double sag = 2.5; double L = distance(a, e) / 2; double r = (L * L + sag * sag) / (2 * sag); Vec2 o = m + out * (r - sag);
            b.A(o, r, std::atan2(a.y - o.y, a.x - o.x), std::atan2(e.y - o.y, e.x - o.x), true); }
        Case c = base("concave_tri", {b.c}); c.mills = {3}; add(c); Case d = c; d.name = "concave_tri_m3_1"; d.mills = {3, 1}; add(d); }
    { // convex-arc triangle (Reuleaux-like): edges bulge outwards
        std::vector<Vec2> t = ngon(3, 12, {0, 0}, PI / 2); PB b; for (int i = 0; i < 3; ++i) { Vec2 a = t[i], e = t[(i + 1) % 3]; Vec2 m = (a + e) * 0.5; Vec2 out = m / m.length(); double sag = 2.0; double L = distance(a, e) / 2; double r = (L * L + sag * sag) / (2 * sag); Vec2 o = m - out * (r - sag);
            b.A(o, r, std::atan2(a.y - o.y, a.x - o.x), std::atan2(e.y - o.y, e.x - o.x), false); }
        Case c = base("convex_arc_tri", {b.c}); c.mills = {3}; add(c); }
    { // line + concave arc acute corner
        Case c = base("line_concave_arc", {PB().L({0, 0}, {20, 0}).A({20, 20}, 20, -PI / 2, PI, true).L({0, 20}, {0, 0}).c}); c.mills = {3}; add(c); }
    { // Bezier-edged blade with sharp ends
        Case c = base("bez_blade", {PB().B({0, 0}, {8, -6}, {16, -6}, {24, 0}).B({24, 0}, {16, 4}, {8, 4}, {0, 0}).c}); c.mills = {2}; add(c); }
    // rounded / chamfered / polygonised corners (near-degenerate spans, tiny roundings)
    for (double rf : {0.0005, 0.01, 0.1, 0.5, 1.0, 1.6}) { Case c = base("roundarc_r" + std::to_string(rf), {roundPoly(sq, rf, 0)}); c.probes = sq; add(c); }
    for (double rf : {0.05, 0.5, 1.5}) for (int s : {4, 12}) { Case c = base("roundpoly_r" + std::to_string(rf) + "_s" + std::to_string(s), {roundPoly(sq, rf, s)}); c.probes = sq; add(c); }
    for (double ch : {0.0005, 0.0015, 0.01, 0.2}) { Case c = base("chamfer" + std::to_string(ch), {chamferPoly(sq, ch)}); c.p.tolerance.distance = 0.001; c.probes = sq; add(c); }
    for (double ch : {0.0005, 0.0015, 0.01, 0.2}) { Case c = base("chamfer" + std::to_string(ch) + "_m1", {chamferPoly(sq, ch)}); c.mills = {1}; c.p.tolerance.distance = 0.001; c.probes = sq; add(c); }
    // threshold: hexagon, end mill radius around chordal / (1 - sin 60) = 0.37320508 (chordal 0.05, V90)
    for (double f : {0.98, 0.999, 0.99999, 1.00001, 1.001, 1.02}) { Case c = base("hexthr_" + std::to_string(f), {vs::poly(ngon(6, 10))}); c.mills = {2 * 0.05 / (1 - std::sin(PI / 3)) * f}; c.so = 0.3; add(c); }
    // no flat depth / no clearance tools (should be untouched)
    { Case c = base("square_noflat", {vs::poly(sq)}); c.p.flatDepth = 0; add(c); }
    { Case c = base("square_nomill", {vs::poly(sq)}); c.mills = {}; add(c); }
    // thin flat area: width just above 2 rho
    for (double w : {3.05, 3.2, 3.6}) { Case c = base("thinflat_w" + std::to_string(w).substr(0, 4), {vs::poly({{0, 0}, {30, 0}, {30, w}, {0, w}})}); c.mills = {0.5}; add(c); }
    // round 1 follow-up: small chordal (overlap min(chordal, R/2) shrinks) on acute corners
    for (double chd : {0.005, 0.001}) for (double a : {20.0, 30.0, 45.0, 60.0, 90.0}) for (double m : {6.0, 2.0}) {
        double h = 18; double hw = h * std::tan(a * PI / 360);
        Case c = base("smallch" + std::to_string(chd).substr(0, 5) + "_apex" + std::to_string((int)a) + "_m" + std::to_string((int)m), {vs::poly({{0, 0}, {h, -hw}, {h + 12, -hw}, {h + 12, hw}, {h, hw}})});
        c.p.tolerance.chordal = chd; c.p.tolerance.distance = std::min(0.001, chd); c.mills = {m}; add(c); }
    // acute corners the end mill reaches (big isosceles triangles), chordal 0.05 / 0.01 / 0.002
    for (double chd : {0.05, 0.01, 0.002}) for (double a : {12.0, 20.0, 30.0, 45.0}) {
        double h = 90; double hw = h * std::tan(a * PI / 360);
        Case c = base("bigtri_ch" + std::to_string(chd).substr(0, 5) + "_apex" + std::to_string((int)a), {vs::poly({{0, 0}, {h, -hw}, {h, hw}})});
        c.p.tolerance.chordal = chd; c.p.tolerance.distance = std::min(0.001, chd); c.mills = {6}; add(c);
        Case d = c; d.name += "_V60s03"; d.ang = 60; d.so = 0.3; add(d); }
    // small holes inside a corner's tent: passes are split by the hole's wall ring; links at depth must not cross it
    for (double hx : {1.9, 2.2, 2.6}) for (double hs : {0.05, 0.3}) {
        Case c = base("tenthole_x" + std::to_string(hx).substr(0, 3) + "_s" + std::to_string(hs).substr(0, 4), {vs::poly(sq), vs::rev(vs::poly({{hx, hx - 0.4}, {hx + hs, hx - 0.4}, {hx + hs, hx - 0.4 + hs}, {hx, hx - 0.4 + hs}}))});
        c.p.tolerance.chordal = 0.02; c.so = 0.3; add(c); }
    { Case c = base("tentslit", {vs::poly(sq), vs::rev(vs::poly({{2.3, 1.2}, {2.34, 1.2}, {2.34, 2.9}, {2.3, 2.9}}))}); c.p.tolerance.chordal = 0.02; c.so = 0.3; add(c); }
    // a 30-degree corner the 6 mm end mill reaches, with a small hole inside its tent (between the wall-ring corner and
    // the fillet, farther than R + rho from the end mill's corner centre): passes split around the hole's ring; links
    for (double hx : {6.0, 8.0, 10.0}) for (double hy : {0.0, 0.15, 0.6}) {
        double h = 60, hw = h * std::tan(15 * PI / 180);
        Case c = base("wedgehole_x" + std::to_string((int)hx) + "_y" + std::to_string(hy).substr(0, 4), {vs::poly({{0, 0}, {h, -hw}, {h, hw}}), vs::rev(vs::poly({{hx, hy}, {hx + 0.1, hy}, {hx + 0.1, hy + 0.1}, {hx, hy + 0.1}}))});
        c.mills = {6}; add(c); }

    // ===== round 2 additions =====
    // dead-end channels (part of the flat area, too narrow for the end mill): the wall ring's spike into it
    for (double w : {2.3, 2.6, 3.0, 4.0, 5.0, 7.0}) { Case c = base("channel_w" + std::to_string(w).substr(0, 4), {vs::poly({{0, 0}, {20, 0}, {20, 20}, {20 - 0, 20}, {10 + w / 2, 20}, {10 + w / 2, 30}, {10 - w / 2, 30}, {10 - w / 2, 20}, {0, 20}})}); add(c); }
    // two close reflex corners (a thin wall poking into the floor) and a narrow inlet between two bosses
    for (double g : {0.05, 0.4, 1.0}) { Case c = base("thinwall_g" + std::to_string(g).substr(0, 4), {vs::poly({{0, 0}, {20, 0}, {20, 20}, {10 + g / 2, 20}, {10 + g / 2, 8}, {10 - g / 2, 8}, {10 - g / 2, 20}, {0, 20}})}); add(c); }
    for (double g : {2.2, 2.6, 3.2}) { Case c = base("inlet2boss_g" + std::to_string(g).substr(0, 4), {Contour::rectangle(Box2{{0, 0}, {30, 20}}), vs::rev(vs::poly({{5, 5}, {15 - g / 2, 5}, {15 - g / 2, 15}, {5, 15}})), vs::rev(vs::poly({{15 + g / 2, 5}, {25, 5}, {25, 15}, {15 + g / 2, 15}}))}); add(c); }
    // necks between two swept areas
    for (double w : {2.4, 2.8, 3.5, 5.0, 8.5}) { Case c = base("dumbbell_w" + std::to_string(w).substr(0, 4), {vs::poly({{0, 0}, {12, 0}, {12, 6 - w / 2}, {16, 6 - w / 2}, {16, 0}, {28, 0}, {28, 12}, {16, 12}, {16, 6 + w / 2}, {12, 6 + w / 2}, {12, 12}, {0, 12}})}); add(c); }
    // swept area of several tools; rest tools that reach in part
    for (auto ms : std::vector<std::vector<double>>{{6, 3}, {6, 2, 0.8}, {3, 6}, {10, 6, 3, 1}, {0.8}}) { std::string nm = "sqmills"; for (double m : ms) nm += "_" + std::to_string(m).substr(0, 3);
        Case c = base(nm, {vs::poly(sq)}); c.mills = ms; add(c); Case d = base(nm + "_star", {vs::star({0, 0}, 15, 7, 6)}); d.mills = ms; add(d); Case e = base(nm + "_raster", {vs::star({0, 0}, 15, 7, 6)}); e.mills = ms; e.p.clearanceStrategy = PocketStrategy::Raster; e.p.clearanceRasterAngle = 0.7; add(e); }
    // ramps (excluded from the swept area)
    { Case c = base("sq_ramp", {vs::poly(sq)}); c.p.rampPlunge = true; c.p.rampDistance = 4; add(c); }
    { Case c = base("star_ramp_raster", {vs::star({0, 0}, 15, 7, 6)}); c.p.rampPlunge = true; c.p.rampDistance = 3; c.p.clearanceStrategy = PocketStrategy::Raster; add(c); }
    { Case c = base("sq_multipass", {vs::poly(sq)}); c.p.flatDepth = 2.5; add(c); }  // end mill pass depth 3: one level; V90 limit 2.5
    // stepover extremes
    for (double so : {0.01, 0.05, 1.9, 2.0, 50.0}) { Case c = base("sq_so" + std::to_string(so).substr(0, 4), {vs::poly(sq)}); c.so = so; add(c); Case d = base("tri30_so" + std::to_string(so).substr(0, 4), {vs::poly({{0, 0}, {40, -40 * std::tan(15 * PI / 180)}, {40, 40 * std::tan(15 * PI / 180)}})}); d.so = so; add(d); }
    // ball-nose carving tools
    for (double bd : {3.0, 6.0, 12.0}) for (double f : {0.3, 1.0, 1.5}) { Case c = base("ball" + std::to_string((int)bd) + "_f" + std::to_string(f).substr(0, 3), {vs::poly(sq)}); c.ct = 1; c.D = bd; c.so = 0.3 * bd; c.p.flatDepth = f; add(c);
        Case d = c; d.name += "_tri30"; d.cs = {vs::poly({{0, 0}, {40, -40 * std::tan(15 * PI / 180)}, {40, 40 * std::tan(15 * PI / 180)}})}; add(d); }
    // chordal extremes on a 90-degree corner with fillets near the threshold
    for (double chd : {0.2, 0.5}) { Case c = base("sq_ch" + std::to_string(chd).substr(0, 3), {vs::poly(sq)}); c.p.tolerance.chordal = chd; c.p.tolerance.distance = 0.01; add(c); }
    // letters-like random star polygons and glyph shapes with small end mills
    for (auto& sh : vs::all()) { Case c = base("glyph_" + sh.name, sh.cs); c.mills = {1.0}; c.p.flatDepth = 0.5; c.so = 0.25; add(c); Case d = c; d.name += "_m3"; d.mills = {3.0}; add(d); }
    { std::mt19937 rg(16002); for (int k = 0; k < 24; ++k) { int n = 3 + (int)(rg() % 9); std::vector<Vec2> p; for (int i = 0; i < 2 * n; ++i) { double a = 2 * PI * i / (2 * n) + 0.2 * ((rg() % 1000) / 1000.0 - 0.5); double r = (i % 2 ? 4 + 6 * ((rg() % 1000) / 1000.0) : 10 + 8 * ((rg() % 1000) / 1000.0)); p.push_back({r * std::cos(a), r * std::sin(a)}); }
        Case c = base("rstar" + std::to_string(k), {vs::poly(p)}); double mm[] = {1.0, 2.0, 3.0, 6.0}; c.mills = {mm[rg() % 4]}; double ang[] = {60, 90, 120}; c.ang = ang[rg() % 3]; c.so = 0.1 + 0.6 * ((rg() % 1000) / 1000.0); c.p.tolerance.chordal = (rg() % 2) ? 0.01 : 0.05; if (rg() % 3 == 0) c.p.clearanceStrategy = PocketStrategy::Raster; add(c); } }
    // threshold for fillets: 90-degree corner rounded by rf, 6 mm end mill, V90 F=1, chordal 0.05: predicted = 3(1-sin45)+..
    for (double rf : {1.2, 1.4, 1.5, 1.55, 1.6, 1.65, 1.7, 1.8, 2.0}) { Case c = base("filthr_r" + std::to_string(rf).substr(0, 4), {roundPoly(sq, rf, 0)}); c.probes = sq; add(c); }
    // rotated / mirrored copies
    { Case c = base("rot_tri30", {xf(vs::poly({{0, 0}, {40, -40 * std::tan(15 * PI / 180)}, {40, 40 * std::tan(15 * PI / 180)}}), 1.234, {3, -7})}); add(c); }
    { Case c = base("rot_Lhole", {xf(Contour::rectangle(Box2{{0, 0}, {30, 30}}), 0.77, {0, 0}), xf(vs::rev(vs::poly({{10, 10}, {20, 10}, {20, 14}, {14, 14}, {14, 20}, {10, 20}})), 0.77, {0, 0})}); c.mills = {6, 1}; add(c); }
    // two flat areas in one part: frame with a letter hole whose counter is an island (nested area the end mill enters / does not)
    { Case c = base("nested_areas", {Contour::rectangle(Box2{{-20, -20}, {20, 20}}), vs::rev(Contour::rectangle(Box2{{-12, -12}, {12, 12}})), Contour::rectangle(Box2{{-8, -8}, {8, 8}}), vs::rev(vs::poly({{-3, -3}, {3, -3}, {0, 3}}))}); c.mills = {3}; add(c); }
    return v;
}

struct CornerV { Vec2 v; double ang; };
static std::vector<CornerV> convexCorners(const std::vector<Contour>& cs, const va::Acc& AC) {
    std::vector<CornerV> out;
    for (auto& c : cs) { size_t n = c.spans.size(); for (size_t i = 0; i < n; ++i) { const Span& a = c.spans[i]; const Span& b = c.spans[(i + 1) % n];
        Vec2 p = vo::evalSpan(a, 1.0); Vec2 ta = p - vo::evalSpan(a, 1 - 1e-6), tb = vo::evalSpan(b, 1e-6) - p; ta = ta / ta.length(); tb = tb / tb.length();
        double turn = std::atan2(ta.cross(tb), ta.dot(tb)); if (std::fabs(turn) < 1e-3) continue;
        Vec2 bis = tb - ta; bis = bis / bis.length(); Vec2 q = p + bis * 1e-4; if (!AC.inside(q)) continue;  // bisector into the region: convex corner (interior angle < pi)
        out.push_back({p, PI - std::fabs(turn)}); } }
    return out;
}
static bool sameMove(const Move& a, const Move& b) { return a == b; }
int main(int argc, char** argv) {
    std::string filt = argc > 1 && argv[1][0] != '-' ? argv[1] : "";
    double gstep = getenv("LC_G") ? atof(getenv("LC_G")) : 0.02; setvbuf(stdout, nullptr, _IONBF, 0);
    auto all = cases(); int nfail = 0, nrun = 0; int shN = getenv("LC_SHN") ? atoi(getenv("LC_SHN")) : 1, shK = getenv("LC_SHK") ? atoi(getenv("LC_SHK")) : 0; int idx = -1;
    for (auto& C : all) { ++idx; if (idx % shN != shK) continue; if (getenv("LC_LIST")) { std::printf("%s\n", C.name.c_str()); continue; }
        if (!filt.empty()) { bool hit = false; size_t st = 0; while (st <= filt.size()) { size_t e = filt.find(',', st); if (e == std::string::npos) e = filt.size(); std::string f = filt.substr(st, e - st); if (!f.empty() && (getenv("LC_EXACT") ? C.name == f : C.name.rfind(f, 0) == 0)) hit = true; st = e + 1; } if (!hit) continue; } ++nrun;
        double ch = C.p.tolerance.chordal; double c3 = ch / 3.0;
        tooling::Tool tool = mk(C.ct, C.D, C.ang, "Carve", 1, C.so);
        std::vector<PocketTool> cl; for (size_t i = 0; i < C.mills.size(); ++i) cl.push_back(PocketTool{mk(2, C.mills[i], 0, "EM" + std::to_string(i), 2 + (int)i, 0.4 * C.mills[i]), std::nullopt});
        VCarvingParams on = C.p, off = C.p; on.cornerSharpen = true; off.cornerSharpen = false;
        VCarvingResult A = vCarvingToolpath(C.cs, tool, std::nullopt, cl, on), B = vCarvingToolpath(C.cs, tool, std::nullopt, cl, off);
        std::string verdict; bool fail = false;
        if (!A.ok() || !B.ok()) { std::printf("%-34s REFUSED on=%d off=%d", C.name.c_str(), (int)A.ok(), (int)B.ok()); for (auto& d : A.diagnostics) std::printf(" [%d %s]", (int)d.issue, d.message.substr(0, 90).c_str()); std::printf("\n"); continue; }
        // limit and rho
        double tanh = C.ct == 0 ? std::tan(tool.includedAngle / 2) : 0; lo::Shape cs{C.ct, C.D / 2, tanh};
        double full = C.ct == 0 ? (C.D / 2) / tanh : C.D / 2; double limit = std::min(C.p.flatDepth > 0 ? C.p.flatDepth : 1e300, full);
        double rho = C.ct == 0 ? limit * tanh : std::sqrt(limit * (C.D - limit)); if (C.ct == 1 && limit >= C.D / 2) rho = C.D / 2;
        double s = C.so > 0 ? std::min(C.so, 2 * rho) : rho; double ridge = C.ct == 0 ? (s / 2) / tanh : C.D / 2 - std::sqrt(C.D * C.D / 4 - s * s / 4);
        double floorZ = -(C.p.startDepth + limit);
        // insertion check
        bool clSame = A.clearance.size() == B.clearance.size(); for (size_t t = 0; clSame && t < A.clearance.size(); ++t) clSame = A.clearance[t].moves == B.clearance[t].moves;
        bool diagSame = A.diagnostics == B.diagnostics;
        const auto& ma = A.toolpath.moves; const auto& mb = B.toolpath.moves; size_t i = 0, j = 0; bool insOk = true; int insCuts = 0; std::vector<std::pair<Vec3, Vec3>> insFeeds;
        Vec3 prevA{C.p.startPosition.x, C.p.startPosition.y, C.p.safeZ};
        while (i < ma.size()) { if (j < mb.size() && sameMove(ma[i], mb[j])) { prevA = ma[i].to; ++i; ++j; continue; }
            if (ma[i].kind != MoveKind::Rapid || std::fabs(ma[i].to.z - C.p.safeZ) > 0) { insOk = false; break; } prevA = ma[i].to; ++i;
            if (i >= ma.size() || ma[i].kind != MoveKind::Plunge || ma[i].to.z != floorZ || ma[i].to.x != prevA.x || ma[i].to.y != prevA.y) { insOk = false; break; } prevA = ma[i].to; ++i;
            int nf = 0; while (i < ma.size() && ma[i].kind == MoveKind::Feed) { if (ma[i].to.z != floorZ) { insOk = false; break; } insFeeds.push_back({prevA, ma[i].to}); prevA = ma[i].to; ++i; ++nf; }
            if (!insOk || nf == 0 || i >= ma.size() || ma[i].kind != MoveKind::Retract || ma[i].to.z != C.p.safeZ) { insOk = false; break; } prevA = ma[i].to; ++i; ++insCuts; }
        if (j != mb.size()) insOk = false;
        bool expectNone = !(C.p.flatDepth > 0) || C.mills.empty();
        if (!insOk || !clSame || !diagSame || (expectNone && insCuts)) fail = true;
        // safety of inserted feeds
        va::Acc AC; AC.build(C.cs); double minClr = 1e300; Vec2 minAt{};
        for (auto& [a, b] : insFeeds) { Vec2 at; double d = lo::segMinClear(AC, {a.x, a.y}, {b.x, b.y}, &at) - rho; if (d < minClr) { minClr = d; minAt = at; }
            if (!AC.inside({0.5 * (a.x + b.x), 0.5 * (a.y + b.y)})) { minClr = -1e9; } }
        // ring moves of off (level at floorZ) for context
        double ringClr = 1e300; if (!getenv("LC_NORING")) { Vec3 at{0, 0, C.p.safeZ}; int cnt = 0; for (auto& m : mb) { if (m.kind == MoveKind::Feed && at.z == floorZ && m.to.z == floorZ && cnt < 20000) { ++cnt; ringClr = std::min(ringClr, lo::segMinClear(AC, {at.x, at.y}, {m.to.x, m.to.y}) - rho); } at = m.to; } }
        if (insCuts && minClr < c3 / 2 - 1e-9) fail = true;
        // leftover
        auto surf = [&](const VCarvingResult& r) { lo::Surface S; Vec3 st{C.p.startPosition.x, C.p.startPosition.y, C.p.safeZ};
            S.add(r.toolpath, cs, rho + 1e-9 + (C.ct == 0 ? 0 : 0), st); for (size_t t = 0; t < r.clearance.size(); ++t) S.add(r.clearance[t], lo::Shape{2, C.mills[t] / 2, 0}, C.mills[t] / 2, st); S.build(std::max(0.25, rho)); return S; };
        lo::Surface SA = surf(A), SB = surf(B);
        double floorFrom = rho + c3 + 1e-9;
        auto left = [&](const lo::Surface& S, Vec2 q) { return std::clamp(S.z(q) - floorZ, 0.0, limit); };
        auto onFloor = [&](Vec2 q) { return AC.inside(q) && AC.dist(q) >= floorFrom; };
        auto refine = [&](const lo::Surface& S, Vec2 q, double st) { double best = left(S, q); while (st > 1e-6) { bool imp = false; for (int k = 0; k < 8; ++k) { Vec2 d{std::cos(k * PI / 4) * st, std::sin(k * PI / 4) * st}; Vec2 qq = q + d; if (!onFloor(qq)) continue; double l = left(S, qq); if (l > best) { best = l; q = qq; imp = true; } } if (!imp) st *= 0.5; } return std::make_pair(best, q); };
        if (getenv("LC_PT")) { Vec2 q; std::sscanf(getenv("LC_PT"), "%lf %lf", &q.x, &q.y);
            std::printf("  PT (%.6f,%.6f) dist %.9f inside %d floorFrom %.9f | on left %.9f off left %.9f\n", q.x, q.y, AC.dist(q), (int)AC.inside(q), floorFrom, left(SA, q), left(SB, q));
            for (size_t t = 0; t < SA.shapes.size(); ++t) { double zz = 1e9; int bk = -1; for (int k = 0; k < (int)SA.segs.size(); ++k) if (SA.segs[k].tool == (int)t) { double v = lo::Surface::segZ(SA.segs[k], SA.shapes[t], q); if (v < zz) { zz = v; bk = k; } }
                if (bk >= 0) { auto& sg = SA.segs[bk]; std::printf("    tool %zu: z %.9f left %.9f via seg (%.6f,%.6f,%.4f)->(%.6f,%.6f,%.4f) d2d %.9f\n", t, zz, zz - floorZ, sg.a.x, sg.a.y, sg.a.z, sg.b.x, sg.b.y, sg.b.z, lo::segDist(q, {sg.a.x, sg.a.y}, {sg.b.x, sg.b.y})); } }
            std::vector<std::pair<double, int>> nf; for (int k = 0; k < (int)insFeeds.size(); ++k) nf.push_back({lo::segDist(q, {insFeeds[k].first.x, insFeeds[k].first.y}, {insFeeds[k].second.x, insFeeds[k].second.y}), k}); std::sort(nf.begin(), nf.end());
            for (size_t k = 0; k < std::min<size_t>(6, nf.size()); ++k) { auto& f = insFeeds[nf[k].second]; std::printf("    ins feed (%.6f,%.6f)->(%.6f,%.6f) dist %.6f\n", f.first.x, f.first.y, f.second.x, f.second.y, nf[k].first); } }
        auto corners = convexCorners(C.cs, AC); if (!C.probes.empty()) { corners.clear(); for (Vec2 q : C.probes) corners.push_back({q, C.probeAng}); } double Rmax = 0; for (double m : C.mills) Rmax = std::max(Rmax, m / 2);
        std::printf("%-34s ins=%d cuts=%d cl=%d dg=%d minClr-c/2=%+.2e ringClr-c/2=%+.2e ridge=%.4f lim=%.3f rho=%.4f s=%.4f\n", C.name.c_str(), (int)insOk, insCuts, (int)clSame, (int)diagSame, insCuts ? minClr - c3 / 2 : 0.0, ringClr - c3 / 2, ridge, limit, rho, s);
        double worstCornerOn = 0; 
        for (auto& cv : corners) {
            double rad = std::min(15.0, (rho + c3) / std::sin(cv.ang / 2) + Rmax / std::tan(cv.ang / 2) + 0.3);  // the tent: wall-ring corner to where the fillet meets the ring edges
            double offMax = 0, onMax = 0; Vec2 onAt = cv.v, offAt = cv.v; int pts = 0;
            int n = (int)std::ceil(rad / gstep);
            std::vector<std::pair<double, Vec2>> topOn, topOff;
            for (int a = -n; a <= n; ++a) for (int b = -n; b <= n; ++b) { Vec2 q{cv.v.x + a * gstep, cv.v.y + b * gstep}; if (distance(q, cv.v) > rad) continue;
                // nearest convex corner must be this one
                bool mine = true; for (auto& o : corners) if (&o != &cv && distance(q, o.v) < distance(q, cv.v)) { mine = false; break; } if (!mine) continue;
                double lb = left(SB, q); if (lb <= 1e-12) continue;  // on <= off (moves only added)
                if (!onFloor(q)) continue; ++pts; double la = left(SA, q); if (la > onMax) { onMax = la; onAt = q; } if (lb > offMax) { offMax = lb; offAt = q; } }
            if (pts == 0) { if (getenv("LC_VERB")) std::printf("    corner (%.4f,%.4f) ang %.2f: no floor points with leftover\n", cv.v.x, cv.v.y, cv.ang * 180 / PI); continue; }
            auto ro = refine(SA, onAt, gstep); auto rf = refine(SB, offAt, gstep); onMax = ro.first; onAt = ro.second; offMax = rf.first;
            worstCornerOn = std::max(worstCornerOn, onMax);
            bool bad = onMax > ridge + 1e-6 && onMax > ch;  // only above both ridge and chordal is a finding
            bool missed = offMax > ch && onMax >= offMax - 1e-9;
            if (bad) fail = true;
            if (bad || missed || getenv("LC_VERB")) std::printf("    corner (%.4f,%.4f) ang %.2f deg: off %.4f -> on %.6f at (%.6f,%.6f) pts %d %s%s\n", cv.v.x, cv.v.y, cv.ang * 180 / PI, offMax, onMax, onAt.x, onAt.y, pts, bad ? "ABOVE-RIDGE " : "", missed ? "UNSHARPENED>chordal" : "");
        }

        // ===== global floor scan (round 2): every floor point (dist >= rho + c) whose leftover without sharpening exceeds
        // max(ridge, chordal) must be at or below the ridge with it, unless it is in a flat area no end mill enters.
        // Also: oracle prediction for the threshold = max over unswept floor points of h(dist - rho - c).
        { double gs = getenv("LC_GS") ? atof(getenv("LC_GS")) : std::min(0.05, std::max(0.01, s / 4)); Box2 bb; for (auto& c0 : C.cs) for (auto& sp : c0.spans) for (int k = 0; k <= 16; ++k) bb.expand(vo::evalSpan(sp, k / 16.0));
            std::vector<std::pair<Vec2, Vec2>> lvl; std::vector<double> lvlR; { for (size_t t = 0; t < B.clearance.size(); ++t) { Vec3 at{0, 0, 1e9}; for (auto& m : B.clearance[t].moves) { if (m.kind != MoveKind::Rapid && at.z == floorZ && m.to.z == floorZ) { lvl.push_back({{at.x, at.y}, {m.to.x, m.to.y}}); lvlR.push_back(C.mills[t] / 2); } at = m.to; } } }
            // uniform grid of level segments for the swept test
            double cellE = 1.0; std::unordered_map<long long, std::vector<int>> eg; auto ek = [&](long long i, long long j) { return i * 4000037LL + j; };
            for (int k = 0; k < (int)lvl.size(); ++k) { double r = lvlR[k]; auto [a, b] = lvl[k]; for (long long i = (long long)std::floor((std::min(a.x, b.x) - r) / cellE); i <= (long long)std::floor((std::max(a.x, b.x) + r) / cellE); ++i) for (long long j = (long long)std::floor((std::min(a.y, b.y) - r) / cellE); j <= (long long)std::floor((std::max(a.y, b.y) + r) / cellE); ++j) eg[ek(i, j)].push_back(k); }
            auto sweptDist = [&](Vec2 q) { double best = 1e300; auto it = eg.find(ek((long long)std::floor(q.x / cellE), (long long)std::floor(q.y / cellE))); if (it == eg.end()) return best; for (int k : it->second) best = std::min(best, lo::segDist(q, lvl[k].first, lvl[k].second) - lvlR[k]); return best; };
            double thrPred = 0; Vec2 thrAt{}; int tent = 0, above = 0; double onWorst = 0; Vec2 onWorstAt{}; double offWorst = 0; double sameAbove = 0; Vec2 sameAt{};
            long long nx = (long long)std::ceil((bb.max.x - bb.min.x) / gs), ny = (long long)std::ceil((bb.max.y - bb.min.y) / gs);
            for (long long a = 0; a <= nx; ++a) for (long long b = 0; b <= ny; ++b) { Vec2 q{bb.min.x + a * gs, bb.min.y + b * gs};
                if (!AC.inside(q)) continue; double dq = AC.dist(q); if (dq < floorFrom) continue;
                if (sweptDist(q) > 1e-7) { double pr = dq - rho - c3; if (pr > thrPred) { thrPred = pr; thrAt = q; } } else continue;
                double lb = left(SB, q); offWorst = std::max(offWorst, lb); if (lb <= std::max(ridge, ch) + 1e-6) continue; ++tent;
                double la = left(SA, q); if (la > onWorst) { onWorst = la; onWorstAt = q; }
                if (la > ridge + 1e-6 && la > ch) { ++above; if (la == lb && la > sameAbove) { sameAbove = la; sameAt = q; } } }
            if (onWorst > 0) { auto ro = refine(SA, onWorstAt, gs); onWorst = std::max(onWorst, ro.first); onWorstAt = ro.second; }
            double predLeft = C.ct == 0 ? std::min(limit, thrPred / tanh) : std::min(limit, C.D / 2 - std::sqrt(std::max(0.0, C.D * C.D / 4 - std::min(thrPred * thrPred, C.D * C.D / 4))));
            bool gbad = onWorst > ridge + 1e-6 && onWorst > ch;
            if (gbad) fail = true;
            std::printf("    GSCAN gs=%.3f tentPts=%d above=%d onWorst=%.6f at (%.5f,%.5f) offWorst=%.4f ridge=%.4f | oraclePred=%.5f (at %.4f,%.4f) cuts=%d%s%s%s\n", gs, tent, above, onWorst, onWorstAt.x, onWorstAt.y, offWorst, ridge, predLeft, thrAt.x, thrAt.y, insCuts,
                gbad ? " GABOVE-RIDGE" : "", sameAbove > 0 ? " (some unchanged by sharpening)" : "", (predLeft > ch * 1.02 && insCuts == 0) ? " PRED>CH-NOCUTS" : (predLeft < ch * 0.98 && insCuts > 0 ? " PRED<CH-CUTS" : ""));
            if (sameAbove > 0) std::printf("    GSCAN unchanged above-ridge point %.5f at (%.5f,%.5f)\n", sameAbove, sameAt.x, sameAt.y); }
        std::printf("    => %s corners=%zu worstCornerOn=%.4f\n", fail ? "FAIL" : "ok", corners.size(), worstCornerOn);
        if (fail) ++nfail;
    }
    std::printf("SUMMARY run=%d fail=%d\n", nrun, nfail);
}
