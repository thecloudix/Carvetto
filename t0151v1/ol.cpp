// Outline oracle: random vectors (lines, arcs, Beziers; open/closed), random passes.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
using geometry::Contour; using geometry::Vec2; using geometry::Vec3;
static std::mt19937_64 rng; static const double PI = 3.14159265358979323846;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static Contour randContour(Vec2 o, bool& hasArc, bool& hasBez) {
    Contour c; int n = I(1, 6); Vec2 p = o; Vec2 first = o;
    bool closed = I(0, 1);
    if (closed && I(0, 4) == 0) { c.append(geometry::ArcSpan::fullCircle({o.x + U(-5,5), o.y + U(-5,5)}, U(0.05, 10), I(0,1)? geometry::ArcDirection::CW : geometry::ArcDirection::CCW)); c.closed = true; hasArc = true; return c; }
    for (int i = 0; i < n; ++i) {
        int k = I(0, 2); Vec2 q{p.x + U(-10, 10), p.y + U(-10, 10)};
        if (closed && i == n - 1) q = first;
        if (k == 0 || (closed && i == n-1 && n == 1)) c.append(geometry::LineSpan{p, q});
        else if (k == 1) { Vec2 m{(p.x+q.x)/2, (p.y+q.y)/2}; Vec2 d{q.x-p.x, q.y-p.y}; double s = U(-1.5, 1.5); Vec2 ctr{m.x - d.y*s, m.y + d.x*s};
            c.append(geometry::ArcSpan::fromCentre(ctr, p, q, I(0,1)? geometry::ArcDirection::CW : geometry::ArcDirection::CCW)); hasArc = true; }
        else { c.append(geometry::BezierSpan{p, {p.x+U(-8,8), p.y+U(-8,8)}, {q.x+U(-8,8), q.y+U(-8,8)}, q}); hasBez = true; }
        p = q;
    }
    c.closed = closed && n > 1;
    return c;
}
int main(int argc, char** argv) {
    rng.seed(std::strtoul(argv[1], nullptr, 10)); int N = std::atoi(argv[2]); long bad = 0, tot = 0;
    for (int sc = 0; sc < N; ++sc) {
        int nv = I(1, 8); std::vector<Contour> cs; bool ha = false, hb = false;
        for (int i = 0; i < nv; ++i) cs.push_back(randContour({U(-50, 50), U(-50, 50)}, ha, hb));
        EngraveParams E; E.depth = U(0.01, 3); E.passes = I(0,5)==0 ? I(1, 40) : I(1, 4); E.safeZ = U(0.1, 20);
        E.startPosition = {U(-60, 60), U(-60, 60)};
        if (I(0, 2) == 0) { E.tolerance.chordal = U(0.0001, 0.5); E.tolerance.distance = std::min(E.tolerance.distance, E.tolerance.chordal); }
        tooling::Tool t; t.id = "v"; t.name = "V"; t.type = I(0,1) ? tooling::ToolType::VBit : tooling::ToolType::BallNose; t.diameter = 6; t.includedAngle = t.type == tooling::ToolType::VBit ? PI / 3 : 0;
        auto& cd = t.defaultCuttingData; cd.toolNumber = 1; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 12000; cd.passDepth = 1; cd.stepover = 0.5;
        EngraveResult r = engraveToolpath(cs, t, std::nullopt, E); ++tot;
        std::vector<double> lv = engraveLevels(E.depth, E.passes);
        std::string err;
        if (!r.ok()) { std::printf("scene %d refused\n", sc); for (auto& d : r.diagnostics) std::printf("  %s\n", d.message.c_str()); continue; }
        const auto& mv = r.toolpath.moves; const double tol = E.tolerance.chordal;
        // split into vector groups: rapid ... retract
        std::vector<std::pair<size_t,size_t>> groups; size_t s = 0;
        for (size_t i = 0; i < mv.size(); ++i) if (mv[i].kind == MoveKind::Retract) { groups.push_back({s, i}); s = i + 1; }
        if (s != mv.size()) err += "trailing moves; ";
        std::set<size_t> used; Vec3 cur = mv.front().to; Vec2 pos = E.startPosition;
        size_t nValid = 0; for (size_t i = 0; i < cs.size(); ++i) { bool skipped = false; for (auto& d : r.diagnostics) if (d.contour && *d.contour == i) skipped = true; if (!skipped) ++nValid; }
        if (groups.size() != nValid) err += "groups " + std::to_string(groups.size()) + " vs valid " + std::to_string(nValid) + "; ";
        for (auto [a, b] : groups) {
            if (mv[a].kind != MoveKind::Rapid || mv[a].to.z != E.safeZ) err += "group not starting with rapid at safe Z; ";
            if (mv[b].to.z != E.safeZ) err += "retract not to safe Z; ";
            Vec2 st{mv[a].to.x, mv[a].to.y};
            // find vector
            int vi = -1; double best = 1e9;
            for (size_t i = 0; i < cs.size(); ++i) { if (used.count(i)) continue; Vec2 s0 = cs[i].startPoint(); double d = std::hypot(s0.x - st.x, s0.y - st.y); if (d < best) { best = d; vi = (int)i; } }
            if (vi < 0 || best > 1e-9) { err += "group start not at a vector start; "; continue; }
            used.insert(vi); const Contour& c = cs[vi];
            // levels and moves
            std::vector<double> plz; Vec3 from = mv[a].to; double len = 0; std::vector<double> lenPerLevel; double curLen = 0;
            for (size_t i = a + 1; i <= b; ++i) {
                const Move& m = mv[i];
                if (m.to.z < -E.depth - 1e-12) err += "below depth; ";
                if (m.kind == MoveKind::Rapid) err += "rapid inside group; ";
                if (m.kind == MoveKind::Plunge) { if (m.to.x != from.x || m.to.y != from.y) err += "non-vertical plunge; "; plz.push_back(m.to.z); if (!plz.empty() && plz.size() > 1) lenPerLevel.push_back(curLen); curLen = 0; }
                if (m.kind == MoveKind::Feed || isArc(m.kind)) {
                    if (m.to.z != from.z) err += "cut changes z; ";
                    if (plz.empty() || m.to.z != plz.back()) err += "cut not at plunge level; ";
                    // samples on contour
                    for (int k = 0; k <= 8; ++k) { double f = k / 8.0; Vec2 q;
                        if (isArc(m.kind)) { double rr = std::hypot(from.x - m.centre.x, from.y - m.centre.y); double a0 = std::atan2(from.y - m.centre.y, from.x - m.centre.x), a1 = std::atan2(m.to.y - m.centre.y, m.to.x - m.centre.x); double sw = a1 - a0; if (m.kind == MoveKind::ArcCCW) { while (sw <= 1e-12) sw += 2*PI; } else { while (sw >= -1e-12) sw -= 2*PI; } q = {m.centre.x + rr*std::cos(a0 + sw*f), m.centre.y + rr*std::sin(a0 + sw*f)}; if (k==0) { curLen += std::fabs(sw)*rr; } if (std::fabs(std::hypot(m.to.x-m.centre.x, m.to.y-m.centre.y)-rr) > 1e-9) err += "arc radius mismatch; "; }
                        else { q = {from.x + (m.to.x-from.x)*f, from.y + (m.to.y-from.y)*f}; if (k==0) curLen += std::hypot(m.to.x-from.x, m.to.y-from.y); }
                        double d = geometry::distanceTo(c, q); if (d > tol + 1e-9) { char bb[160]; snprintf(bb, sizeof bb, "off vector %.3e (tol %.3e) kind %d; ", d, tol, (int)m.kind); err += bb; break; }
                    }
                }
                if (m.kind == MoveKind::Retract && i != b) err += "retract mid-group; ";
                from = m.to;
            }
            lenPerLevel.push_back(curLen);
            if (plz != lv) err += "levels mismatch (" + std::to_string(plz.size()) + " vs " + std::to_string(lv.size()) + "); ";
            double L = c.length();
            for (double l : lenPerLevel) if (std::fabs(l - L) > 1e-6 * std::max(1.0, L) + 0.01 * L * 0 + 2 * tol * 0 + (hb ? (tol > 0.05 ? 0.06 : 0.01) * L : 1e-6)) { char bb[120]; snprintf(bb, sizeof bb, "level length %.6f vs vector %.6f; ", l, L); err += bb; break; }
            // closed: plunges at start; open: alternating ends
            for (size_t i = a + 1, k = 0; i <= b; ++i) if (mv[i].kind == MoveKind::Plunge) {
                Vec2 expect = (c.closed || k % 2 == 0) ? c.startPoint() : c.endPoint();
                if (std::hypot(mv[i].to.x - expect.x, mv[i].to.y - expect.y) > 1e-9) err += "plunge at wrong place; ";
                ++k;
            }
        }
        if (!err.empty()) { ++bad; std::printf("scene %d FAIL nv %d passes %d depth %.4f chordal %.4g: %s\n", sc, nv, E.passes, E.depth, E.tolerance.chordal, err.substr(0, 600).c_str()); }
        else std::printf("scene %d ok nv %d passes %d groups %zu arcs %zu diag %zu\n", sc, nv, E.passes, groups.size(), r.toolpath.count(MoveKind::ArcCW) + r.toolpath.count(MoveKind::ArcCCW), r.diagnostics.size());
    }
    std::printf("TOTAL %ld bad %ld\n", tot, bad);
}
