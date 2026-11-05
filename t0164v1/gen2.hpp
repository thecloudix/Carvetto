// T-0153 round 2: broad generative families for the V-carve cone oracle (crossing allowed).
// family ids: 0 lib shape, 1 star, 2 holed star, 3 thin strip, 4 ring+neighbour (crossing/touching),
// 5 text row (overlapping/touching glyphs), 6 bezier blob, 7 overlapping polygons, 8 overlapping
// circles / tangent circles, 9 self-crossing (figure-8 bezier, bow tie, pentagram), 10 nearly tangent
// nested/side circles (gap in {0,1e-4,1e-3,ch/16,ch/8,ch/4,ch}), 11 frame with text inside (and
// glyphs crossing the frame), 12 holes in holes (alternating nesting, offset), 13 crossing slivers
// (rectangles at small angles, nearly coincident circles), 14 crossing cluster + non-crossing
// near-tangent neighbour (polygonisation vs exact curve), 15 coincident edges / duplicate contour.
#pragma once
#include <functional>
#include <random>
#include "shapes.hpp"
namespace g2 {
using namespace carvetto::geometry;
inline std::mt19937_64 rng;
inline double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
inline int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
inline Contour xform(const Contour& c, double s, double ang, Vec2 off) {
    Contour o; o.closed = c.closed; double ca = std::cos(ang), sa = std::sin(ang);
    auto P = [&](Vec2 p) { return Vec2{off.x + s * (ca * p.x - sa * p.y), off.y + s * (sa * p.x + ca * p.y)}; };
    for (auto& sp : c.spans) {
        if (auto* l = std::get_if<LineSpan>(&sp)) o.append(LineSpan{P(l->start), P(l->end)});
        else if (auto* a = std::get_if<ArcSpan>(&sp)) { ArcSpan b = *a; b.centre = P(a->centre); b.radius = a->radius * s; b.startAngle += ang; b.endAngle += ang; o.append(b); }
        else { auto b = std::get<BezierSpan>(sp); o.append(BezierSpan{P(b.p0), P(b.p1), P(b.p2), P(b.p3)}); }
    }
    return o;
}
inline Contour circleAt(Vec2 o, double r, double a0) { Contour c; c.closed = true; c.append(ArcSpan{o, r, a0, a0, ArcDirection::CCW}); return c; }
inline std::vector<Contour> glyph(int g, double h, Vec2 at) {
    static auto lib = vs::all(); int ids[4] = {17, 15, 16, 14}; auto& sh = lib[ids[g & 3]];
    Box2 bb; for (auto& c : sh.cs) bb.expand(c.bounds()); double s = h / std::max(bb.height(), bb.width());
    std::vector<Contour> out; for (auto& c : sh.cs) out.push_back(xform(c, s, 0, {at.x - bb.min.x * s, at.y - bb.min.y * s})); return out;
}
inline std::vector<Contour> gen(int fam, double chordal, std::string& name) {
    static auto lib = vs::all();
    std::vector<Contour> cs;
    switch (fam) {
    case 0: { auto& s = lib[I(0, (int)lib.size() - 1)]; name = "lib:" + s.name; double sc = U(0.2, 1.5), an = U(0, 2 * vo::PI); Vec2 o{U(-50, 50), U(-50, 50)}; for (auto& c : s.cs) cs.push_back(xform(c, sc, an, o)); break; }
    case 1: { name = "star"; int n = I(3, 12); double R = U(2, 20); cs.push_back(vs::star({U(-20, 20), U(-20, 20)}, R, R * U(0.2, 0.9), n, U(0, 6))); break; }
    case 2: { name = "holedstar"; int n = I(4, 9); double R = U(8, 20); Vec2 o{U(-20, 20), U(-20, 20)}; cs.push_back(vs::star(o, R, R * U(0.5, 0.9), n, U(0, 6))); cs.push_back(vs::rev(vs::star(o, R * 0.35, R * U(0.1, 0.3), I(3, 7), U(0, 6)))); break; }
    case 3: { name = "thinstrip"; double w = U(0.02, 1.0), L = U(5, 40); cs.push_back(xform(vs::poly({{0, 0}, {L, 0}, {L, w}, {0, w}}), 1, U(0, 6), {U(-9, 9), U(-9, 9)})); break; }
    case 4: { name = "ringneigh"; Vec2 o{U(-10, 10), U(-10, 10)}; double R = U(4, 12); cs.push_back(vs::circle(o, R)); cs.push_back(vs::rev(vs::circle({o.x + U(-1, 1), o.y + U(-1, 1)}, R * U(0.3, 0.8)))); double rn = U(1, 4); int m = I(0, 2); double dx = m == 0 ? U(-0.5, 0.5) * rn : m == 1 ? 0.0 : U(0.05, 3); cs.push_back(vs::circle({o.x + R + rn + (m == 0 ? dx - rn : dx), o.y}, rn)); name += m == 0 ? "-cross" : m == 1 ? "-touch" : "-apart"; break; }
    case 5: { name = "text"; double h = U(3, 12); double x = 0; int n = I(2, 6); int m = I(0, 2); for (int i = 0; i < n; ++i) { auto gl = glyph(I(0, 3), h, {x, U(-0.2, 0.2) * h * (m == 0)}); Box2 b; for (auto& c : gl) { b.expand(c.bounds()); cs.push_back(c); } x = b.max.x + (m == 0 ? U(-0.3, 0.1) * h : m == 1 ? 0.0 : U(0.02, 0.4) * h); } name += m == 0 ? "-overlap" : m == 1 ? "-touchbox" : "-apart"; break; }
    case 6: { name = "bezblob"; Contour c; c.closed = true; int n = I(3, 6); Vec2 o{U(-10, 10), U(-10, 10)}; std::vector<Vec2> p; for (int i = 0; i < n; ++i) { double a = 2 * vo::PI * i / n; double r = U(4, 12); p.push_back({o.x + r * std::cos(a), o.y + r * std::sin(a)}); }
        for (int i = 0; i < n; ++i) { Vec2 a = p[i], b = p[(i + 1) % n]; Vec2 d = b - a; Vec2 nn{-d.y, d.x}; c.append(BezierSpan{a, a + d * 0.3 + nn * U(-0.3, 0.6), b - d * 0.3 + nn * U(-0.3, 0.6), b}); } cs.push_back(c); break; }
    case 7: { name = "overpoly"; int k = I(2, 4); for (int i = 0; i < k; ++i) { int n = I(3, 7); Vec2 o{U(-6, 6), U(-6, 6)}; double R = U(3, 10); std::vector<Vec2> p; double a0 = U(0, 6); for (int j = 0; j < n; ++j) { double a = a0 + 2 * vo::PI * j / n; p.push_back({o.x + R * std::cos(a), o.y + R * std::sin(a)}); } Contour c = vs::poly(p); if (I(0, 1)) c.reverse(); cs.push_back(c); } break; }
    case 8: { name = "overcircles"; int k = I(2, 4); Vec2 o{U(-5, 5), U(-5, 5)}; double r0 = U(2, 8); cs.push_back(circleAt(o, r0, U(0, 6))); for (int i = 1; i < k; ++i) { double r = U(1, 8); double a = U(0, 6); int m = I(0, 3); double d = m == 0 ? r0 + r : m == 1 ? std::fabs(r0 - r) : U(std::fabs(r0 - r), r0 + r); if (m <= 1) name += m == 0 ? "-exttangent" : "-inttangent"; cs.push_back(circleAt({o.x + d * std::cos(a), o.y + d * std::sin(a)}, r, U(0, 6))); } break; }
    case 9: { int m = I(0, 2); Vec2 o{U(-20, 20), U(-20, 20)}; double s = U(3, 15);
        if (m == 0) { name = "fig8"; Contour c; c.closed = true; Vec2 a{o.x - s, o.y}, b{o.x + s, o.y}; double h = U(0.3, 1.2) * s; c.append(BezierSpan{o, {o.x - s * 0.5, o.y + h}, {a.x - s * 0.3, a.y + h}, a}); c.append(BezierSpan{a, {a.x - s * 0.3, a.y - h}, {o.x - s * 0.5, o.y - h}, o}); c.append(BezierSpan{o, {o.x + s * 0.5, o.y + h}, {b.x + s * 0.3, b.y + h}, b}); c.append(BezierSpan{b, {b.x + s * 0.3, b.y - h}, {o.x + s * 0.5, o.y - h}, o}); cs.push_back(c); }
        else if (m == 1) { name = "bowtie"; double h = U(0.2, 1.5) * s; cs.push_back(vs::poly({{o.x - s, o.y - h}, {o.x + s, o.y + h}, {o.x + s, o.y - h}, {o.x - s, o.y + h}})); }
        else { name = "pentagram"; int n = I(5, 9); int step = I(2, (n - 1) / 2); std::vector<Vec2> p; double a0 = U(0, 6); for (int j = 0; j < n; ++j) { double a = a0 + 2 * vo::PI * ((j * step) % n) / n; p.push_back({o.x + s * std::cos(a), o.y + s * std::sin(a)}); } cs.push_back(vs::poly(p)); }
        break; }
    case 10: { double gaps[] = {0, 1e-4, 1e-3, chordal / 16, chordal / 8, chordal / 4, chordal / 2, chordal}; double g = gaps[I(0, 7)]; double R = U(3, 12), r = U(0.5, R * 0.8); Vec2 o{U(-10, 10), U(-10, 10)}; double a = U(0, 6); int m = I(0, 2);
        cs.push_back(circleAt(o, R, U(0, 6)));
        if (m == 0) { name = "nearint g=" + std::to_string(g); double d = R - r - g; cs.push_back(vs::rev(circleAt({o.x + d * std::cos(a), o.y + d * std::sin(a)}, r, U(0, 6)))); }
        else if (m == 1) { name = "nearext g=" + std::to_string(g); double d = R + r + g; cs.push_back(circleAt({o.x + d * std::cos(a), o.y + d * std::sin(a)}, r, U(0, 6))); }
        else { name = "nearsq g=" + std::to_string(g); double w = U(2, 10); cs.clear(); cs.push_back(vs::poly({{0, 0}, {w, 0}, {w, w}, {0, w}})); double rr = U(0.3, w / 2.2); cs.push_back(circleAt({w - rr - g, U(rr + g, w - rr - g)}, rr, U(0, 6))); }
        break; }
    case 11: { name = "frametext"; double h = U(3, 8); int n = I(2, 5); double fw = U(0.5, 3); double pad = U(0.05, 2); int m = I(0, 3); double x = 0; std::vector<Contour> text; Box2 tb;
        for (int i = 0; i < n; ++i) { auto gl = glyph(I(0, 3), h, {x, 0}); Box2 b; for (auto& c : gl) { b.expand(c.bounds()); text.push_back(c); } tb.expand(b); x = b.max.x + U(0.05, 0.4) * h; }
        double p2 = m == 3 ? -U(0.05, 0.5) * h : pad; // m == 3: glyphs cross the inner frame edge
        Box2 in{{tb.min.x - p2, tb.min.y - p2}, {tb.max.x + p2, tb.max.y + p2}};
        if (m == 3) { in.min.x = tb.min.x + 0.2 * h; name += "-crossing"; }
        cs.push_back(vs::poly({{in.min.x - fw, in.min.y - fw}, {in.max.x + fw, in.min.y - fw}, {in.max.x + fw, in.max.y + fw}, {in.min.x - fw, in.max.y + fw}}));
        cs.push_back(vs::rev(vs::poly({{in.min.x, in.min.y}, {in.max.x, in.min.y}, {in.max.x, in.max.y}, {in.min.x, in.max.y}})));
        for (auto& c : text) cs.push_back(c); if (m == 2) { // rounded frame (arcs)
            Contour f; f.closed = true; double rr = fw * 2; Box2 o2{{in.min.x - fw - 3, in.min.y - fw - 3}, {in.max.x + fw + 3, in.max.y + fw + 3}};
            f.append(LineSpan{{o2.min.x + rr, o2.min.y}, {o2.max.x - rr, o2.min.y}}); f.append(ArcSpan{{o2.max.x - rr, o2.min.y + rr}, rr, -vo::PI / 2, 0, ArcDirection::CCW});
            f.append(LineSpan{{o2.max.x, o2.min.y + rr}, {o2.max.x, o2.max.y - rr}}); f.append(ArcSpan{{o2.max.x - rr, o2.max.y - rr}, rr, 0, vo::PI / 2, ArcDirection::CCW});
            f.append(LineSpan{{o2.max.x - rr, o2.max.y}, {o2.min.x + rr, o2.max.y}}); f.append(ArcSpan{{o2.min.x + rr, o2.max.y - rr}, rr, vo::PI / 2, vo::PI, ArcDirection::CCW});
            f.append(LineSpan{{o2.min.x, o2.max.y - rr}, {o2.min.x, o2.min.y + rr}}); f.append(ArcSpan{{o2.min.x + rr, o2.min.y + rr}, rr, vo::PI, 1.5 * vo::PI, ArcDirection::CCW});
            cs.push_back(f); name += "-outerring"; }
        break; }
    case 12: { name = "holesinholes"; int k = I(3, 6); Vec2 o{U(-5, 5), U(-5, 5)}; double R = U(8, 20); int sq = I(0, 1); for (int i = 0; i < k; ++i) { double r = R * (1 - double(i) / k) ; Vec2 c{o.x + U(-0.3, 0.3) * R / k, o.y + U(-0.3, 0.3) * R / k}; Contour cc = sq ? vs::poly({{c.x - r, c.y - r}, {c.x + r, c.y - r}, {c.x + r, c.y + r}, {c.x - r, c.y + r}}) : circleAt(c, r, U(0, 6)); if (I(0, 1)) cc.reverse(); cs.push_back(cc); } if (sq) name += "-sq"; break; }
    case 13: { int m = I(0, 1); if (m == 0) { name = "sliverrects"; double L = U(10, 40), w = U(0.5, 4); double ang = U(0.3, 3) * vo::PI / 180; Vec2 o{U(-5, 5), U(-5, 5)}; Contour a = vs::poly({{0, 0}, {L, 0}, {L, w}, {0, w}}); cs.push_back(xform(a, 1, 0, o)); cs.push_back(xform(a, 1, ang, {o.x + U(-1, 1), o.y + U(-0.3, 0.3) * w})); }
        else { name = "slivercircles"; double r = U(3, 15); Vec2 o{U(-5, 5), U(-5, 5)}; double d = U(0.005, 0.3); double a = U(0, 6); cs.push_back(circleAt(o, r, U(0, 6))); cs.push_back(circleAt({o.x + d * std::cos(a), o.y + d * std::sin(a)}, r * U(0.99, 1.01), U(0, 6))); }
        break; }
    case 14: { name = "cluster+near"; double R = U(4, 10); Vec2 o{U(-5, 5), U(-5, 5)}; cs.push_back(circleAt(o, R, U(0, 6))); cs.push_back(circleAt({o.x + R * U(0.8, 1.5), o.y}, R * U(0.6, 1.0), U(0, 6)));
        double gs[] = {1e-3, chordal / 32, chordal / 16, chordal / 10, chordal / 8, chordal / 4}; double g = std::max(gs[I(0, 5)], 1.2e-3); double r = U(0.5, R * 0.4); double a = U(0.6, 5.6); int m = I(0, 1);
        double d = m == 0 ? R - r - g : R + r + g; Contour c = circleAt({o.x + d * std::cos(a), o.y + d * std::sin(a)}, r, U(0, 6)); if (m == 0) c.reverse(); cs.push_back(c); name += m == 0 ? "-inside g=" : "-outside g="; name += std::to_string(g); break; }
    default: { int m = I(0, 2); double w = U(2, 10); Vec2 o{U(-10, 10), U(-10, 10)};
        if (m == 0) { name = "sharededge"; cs.push_back(vs::poly({{o.x, o.y}, {o.x + w, o.y}, {o.x + w, o.y + w}, {o.x, o.y + w}})); double h2 = U(0.3, 1.0) * w; cs.push_back(vs::poly({{o.x + w, o.y}, {o.x + 2 * w, o.y}, {o.x + 2 * w, o.y + h2}, {o.x + w, o.y + h2}})); }
        else if (m == 1) { name = "duplicate+other"; Contour c = vs::poly({{o.x, o.y}, {o.x + w, o.y}, {o.x + w, o.y + w}, {o.x, o.y + w}}); cs.push_back(c); cs.push_back(c); cs.push_back(circleAt({o.x + w * 2, o.y}, w / 3, 0)); }
        else { name = "partialoverlapedge"; cs.push_back(vs::poly({{o.x, o.y}, {o.x + w, o.y}, {o.x + w, o.y + w}, {o.x, o.y + w}})); cs.push_back(vs::poly({{o.x + w * 0.3, o.y}, {o.x + w * 0.7, o.y}, {o.x + w * 0.7, o.y - w}, {o.x + w * 0.3, o.y - w}})); }
        break; }
    }
    return cs;
}
}  // namespace g2
