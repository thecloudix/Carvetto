// T-0153 round 3: extra families on top of gen2 (ids 16..23), aimed at the round-2 changes.
// 16 many tiny spans + one long span (D-hole hugging an arc wall; classification probes)
// 17 hole touching the wall at one point (vertex on edge, tangent circle, square corner on circle)
// 18 nested holes/islands within chordal of each other (alternating rings, offset centres)
// 19 chain of near-touching contours with one crossing pair at the end (iterative cluster growth)
// 20 big page where everything touches/overlaps (grid of circles/squares/glyphs)
// 21 dense sampling near a flat corner (near-180 deg corners, thin band with flat corners)
// 22 self-touching union (two pieces meeting at a vertex: squares corner to corner, letter tangent to frame)
// 23 nesting probes on boundaries (hole sharing edges/vertices with its outer, crossing elsewhere)
#pragma once
#include "gen2.hpp"
namespace g3 {
using namespace carvetto::geometry; using g2::U; using g2::I; using g2::circleAt;
inline Contour arcPoly(Vec2 o, double r, double a0, double a1, int n) { std::vector<Vec2> p; for (int i = 0; i <= n; ++i) { double a = a0 + (a1 - a0) * i / n; p.push_back({o.x + r * std::cos(a), o.y + r * std::sin(a)}); } return vs::poly(p); }
inline std::vector<Contour> gen(int fam, double ch, std::string& name) {
    if (fam < 16) return g2::gen(fam, ch, name);
    std::vector<Contour> cs; double gaps[] = {0.0, 1e-4, ch / 32, ch / 16, ch / 8, ch / 4, ch / 2, ch};
    switch (fam) {
    case 16: { double g = gaps[I(1, 7)]; double R = U(4, 15); Vec2 o{U(-5, 5), U(-5, 5)}; int m = I(0, 2); int n = I(20, 400); double span = U(0.3, 2.5); double a0 = U(0, 6);
        name = "tinyspans g=" + std::to_string(g / ch) + "ch n=" + std::to_string(n);
        if (m == 0) { cs.push_back(circleAt(o, R, U(0, 6))); Contour h = arcPoly(o, R - g, a0, a0 + span, n); // chord closes it; rotate so the long span is not span 0
            std::rotate(h.spans.begin(), h.spans.begin() + I(1, n - 1), h.spans.end()); if (I(0, 1)) h.reverse(); else h.reverse(), h.reverse(); cs.push_back(vs::rev(h)); name += "-Dhole"; }
        else if (m == 1) { // outer = polygonised arc wall (many tiny spans) + one long chord; hole circle near the tiny-span side
            Contour w = arcPoly(o, R, a0, a0 + U(2.5, 5.5), n); cs.push_back(w); double r = U(0.3, R * 0.3); double a = a0 + U(0.3, 2.0); double d = R * std::cos(vo::PI / n) - r - g - R * (1 - std::cos(1.0 / n)); cs.push_back(vs::rev(circleAt({o.x + d * std::cos(a), o.y + d * std::sin(a)}, r, U(0, 6)))); name += "-arcwall"; }
        else { // island (outer inside a hole) made of tiny spans near the hole wall, plus a long span
            cs.push_back(vs::poly({{o.x - R - 3, o.y - R - 3}, {o.x + R + 3, o.y - R - 3}, {o.x + R + 3, o.y + R + 3}, {o.x - R - 3, o.y + R + 3}}));
            cs.push_back(vs::rev(circleAt(o, R, 0))); Contour isl = arcPoly(o, R - g, a0, a0 + span, n); std::rotate(isl.spans.begin(), isl.spans.begin() + I(1, n - 1), isl.spans.end()); cs.push_back(isl); name += "-island"; }
        break; }
    case 17: { int m = I(0, 3); double w = U(3, 12); Vec2 o{U(-5, 5), U(-5, 5)}; name = "touch1";
        cs.push_back(vs::poly({{o.x, o.y}, {o.x + w, o.y}, {o.x + w, o.y + w}, {o.x, o.y + w}}));
        if (m == 0) { double s = U(0.2, 0.45) * w; double x = o.x + U(0.3, 0.7) * w; cs.push_back(vs::rev(vs::poly({{x, o.y}, {x + s, o.y + s}, {x, o.y + 2 * s}, {x - s, o.y + s}}))); name += "-diamondvertex"; }
        else if (m == 1) { double r = U(0.2, 0.45) * w; cs.push_back(vs::rev(circleAt({o.x + U(r, w - r), o.y + r}, r, U(0, 6)))); name += "-tangentcircle"; }
        else if (m == 2) { cs.clear(); double R = w / 2; Vec2 c{o.x + R, o.y + R}; cs.push_back(circleAt(c, R, U(0, 6))); double s = R * std::sqrt(0.5) * U(0.4, 0.95); // square corner exactly on the circle
            double a = U(0, 6); Vec2 v{c.x + R * std::cos(a), c.y + R * std::sin(a)}; Vec2 u{-std::cos(a), -std::sin(a)}; Vec2 nn{-u.y, u.x};
            cs.push_back(vs::rev(vs::poly({v, v + (u + nn) * s, v + u * (2 * s), v + (u - nn) * s}))); name += "-squarecorneroncircle"; }
        else { double s = U(0.15, 0.3) * w; cs.push_back(vs::rev(vs::poly({{o.x, o.y + w / 2}, {o.x + s, o.y + w / 2 - s}, {o.x + 2 * s, o.y + w / 2}, {o.x + s, o.y + w / 2 + s}}))); cs.push_back(vs::rev(vs::poly({{o.x + 2 * s, o.y + w / 2}, {o.x + 3 * s, o.y + w / 2 - s}, {o.x + 4 * s, o.y + w / 2}, {o.x + 3 * s, o.y + w / 2 + s}}))); name += "-twoholes-vertexwall-vertexvertex"; }
        break; }
    case 18: { int k = I(3, 7); double R = U(5, 15); Vec2 o{U(-5, 5), U(-5, 5)}; int sq = I(0, 1); double r = R; name = std::string("nestedtight") + (sq ? "-sq" : "-circ");
        for (int i = 0; i < k && r > 0.3; ++i) { Vec2 c{o.x + U(-0.02, 0.02), o.y + U(-0.02, 0.02)}; Contour cc = sq ? vs::poly({{c.x - r, c.y - r}, {c.x + r, c.y - r}, {c.x + r, c.y + r}, {c.x - r, c.y + r}}) : circleAt(c, r, U(0, 6)); if (i % 2) cc.reverse(); cs.push_back(cc);
            double gg = I(0, 2) == 0 ? U(0.5, 3) : gaps[I(1, 7)] + 0.03; r -= std::max(gg, 1e-4); }
        break; }
    case 19: { int k = I(3, 9); double r = U(0.8, 3); double g = std::max(1.2e-3, gaps[I(2, 5)]); double x = 0; Vec2 o{U(-5, 5), U(-5, 5)}; name = "chain g=" + std::to_string(g / ch) + "ch";
        int m = I(0, 1); // circles side by side (gap g), last pair crossing
        for (int i = 0; i < k; ++i) { double ri = r * U(0.7, 1.3); if (i == 0) x = o.x + ri; cs.push_back(m ? circleAt({x, o.y}, ri, U(0, 6)) : vs::poly({{x - ri, o.y - ri}, {x + ri, o.y - ri}, {x + ri, o.y + ri}, {x - ri, o.y + ri}}));
            double rn = r * U(0.7, 1.3); bool cross = i == k - 2 || (i == 0 && I(0, 1)); x += ri + rn + (cross ? -0.3 * rn : g); r = rn; }
        name += m ? "-circles" : "-squares"; break; }
    case 20: { int n = I(4, 9); double s = U(1, 4); name = "page-alltouch"; int m = I(0, 2);
        for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) { Vec2 c{i * s, j * s};
            if (m == 0) cs.push_back(circleAt(c, s * U(0.5, 0.75), U(0, 6)));
            else if (m == 1) { double h = s * 0.5; cs.push_back(vs::poly({{c.x - h, c.y - h}, {c.x + h, c.y - h}, {c.x + h, c.y + h}, {c.x - h, c.y + h}})); if ((i + j) % 3 == 0) cs.push_back(vs::rev(circleAt(c, s * 0.2, 0))); } // shared edges
            else { auto gl = g2::glyph(I(0, 3), s * 1.2, {c.x - 0.1 * s, c.y - 0.1 * s}); for (auto& q : gl) cs.push_back(q); } }
        name += m == 0 ? "-circles" : m == 1 ? "-squaressharededges" : "-glyphs"; break; }
    case 21: { int m = I(0, 1); double L = U(5, 30); double w = U(0.05, 2); double ang = U(0.01, 2) * vo::PI / 180; int n = I(5, 300); name = "flatcorner n=" + std::to_string(n);
        if (m == 0) { std::vector<Vec2> p{{0, 0}}; for (int i = 1; i <= n; ++i) { double t = double(i) / n; p.push_back({L * t, 0.5 * L * t * std::tan(ang) * (t < 0.5 ? t / 0.5 : (1 - t) / 0.5)}); } p.push_back({L, w}); p.push_back({0, w}); cs.push_back(g2::xform(vs::poly(p), 1, U(0, 6), {U(-5, 5), U(-5, 5)})); name += "-bentband"; }
        else { std::vector<Vec2> p; for (int i = 0; i <= n; ++i) { double t = double(i) / n; p.push_back({L * t, 0}); } p.push_back({L + w * std::tan(ang), w}); for (int i = n; i >= 0; --i) { double t = double(i) / n; p.push_back({L * t * 0.999 + (i == 0 ? -w * std::tan(ang) : 0), w}); } cs.push_back(g2::xform(vs::poly(p), 1, U(0, 6), {U(-5, 5), U(-5, 5)})); name += "-denseband"; }
        break; }
    case 22: { int m = I(0, 2); double s = U(2, 8); Vec2 o{U(-5, 5), U(-5, 5)}; name = "selftouch";
        if (m == 0) { cs.push_back(vs::poly({{o.x, o.y}, {o.x + s, o.y}, {o.x + s, o.y + s}, {o.x, o.y + s}})); cs.push_back(vs::poly({{o.x + s, o.y + s}, {o.x + 2 * s, o.y + s}, {o.x + 2 * s, o.y + 2 * s}, {o.x + s, o.y + 2 * s}})); // corner to corner
            cs.push_back(circleAt({o.x + 0.5 * s, o.y}, 0.3 * s, U(0, 6))); name += "-corners+cross"; }
        else if (m == 1) { double fw = U(0.5, 2); cs.push_back(vs::poly({{o.x - fw, o.y - fw}, {o.x + 4 * s + fw, o.y - fw}, {o.x + 4 * s + fw, o.y + s + fw}, {o.x - fw, o.y + s + fw}})); cs.push_back(vs::rev(vs::poly({{o.x, o.y}, {o.x + 4 * s, o.y}, {o.x + 4 * s, o.y + s}, {o.x, o.y + s}})));
            cs.push_back(vs::poly({{o.x + s, o.y}, {o.x + 1.5 * s, o.y + 0.5 * s}, {o.x + s, o.y + 0.9 * s}, {o.x + 0.5 * s, o.y + 0.5 * s}})); // diamond tangent to the frame's inner edge at a vertex
            cs.push_back(vs::poly({{o.x + 2.5 * s, o.y - 0.3 * s}, {o.x + 3 * s, o.y - 0.3 * s}, {o.x + 3 * s, o.y + 0.5 * s}, {o.x + 2.5 * s, o.y + 0.5 * s}})); name += "-frame+diamondtangent+crossing"; }
        else { cs.push_back(vs::poly({{o.x, o.y}, {o.x + 2 * s, o.y + s}, {o.x + 2 * s, o.y}, {o.x, o.y + s}})); cs.push_back(circleAt({o.x + 2 * s + 0.5 * s, o.y + 0.5 * s}, 0.5 * s, 3.14159265358979)); name += "-bowtie+tangentcircle"; }
        break; }
    default: { int m = I(0, 2); double w = U(3, 10); Vec2 o{U(-5, 5), U(-5, 5)}; name = "nestonboundary";
        cs.push_back(vs::poly({{o.x, o.y}, {o.x + w, o.y}, {o.x + w, o.y + w}, {o.x, o.y + w}}));
        if (m == 0) { double a = U(0.1, 0.4) * w, b = U(0.6, 0.9) * w; cs.push_back(vs::rev(vs::poly({{o.x + a, o.y}, {o.x + b, o.y}, {o.x + b, o.y + 0.5 * w}, {o.x + a, o.y + 0.5 * w}}))); name += "-holeedgeonwall"; }
        else if (m == 1) { cs.push_back(vs::rev(vs::poly({{o.x, o.y}, {o.x + 0.4 * w, o.y + 0.2 * w}, {o.x + 0.2 * w, o.y + 0.4 * w}}))); cs.push_back(circleAt({o.x + w, o.y + 0.5 * w}, 0.2 * w, U(0, 6))); name += "-holevertexatcorner+cross"; }
        else { cs.push_back(vs::rev(vs::poly({{o.x + 0.2 * w, o.y + 0.2 * w}, {o.x + 0.8 * w, o.y + 0.2 * w}, {o.x + 0.8 * w, o.y + 0.8 * w}, {o.x + 0.2 * w, o.y + 0.8 * w}}))); cs.push_back(vs::poly({{o.x + 0.2 * w, o.y + 0.2 * w}, {o.x + 0.5 * w, o.y + 0.2 * w}, {o.x + 0.5 * w, o.y + 0.5 * w}, {o.x + 0.2 * w, o.y + 0.5 * w}})); cs.push_back(circleAt({o.x, o.y}, 0.1 * w, 0)); name += "-islandsharingholecorner+cross"; }
        break; }
    }
    return cs;
}
}  // namespace g3
