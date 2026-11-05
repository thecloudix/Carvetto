#pragma once
#include <string>
#include <vector>
#include "oracle.hpp"
namespace vs {
using namespace carvetto::geometry;
using vo::PI;
inline Contour poly(std::vector<Vec2> p) { Contour c; c.closed = true; for (size_t i = 0; i < p.size(); ++i) c.append(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
inline Contour rev(Contour c) { c.reverse(); return c; }
inline Contour circle(Vec2 o, double r) { Contour c; c.closed = true; c.append(ArcSpan{o, r, 0.0, 0.0, ArcDirection::CCW}); return c; }
inline Contour bezEllipse(Vec2 c, double a, double b) {
    const double k = 0.55228474983; Contour e; e.closed = true;
    e.append(BezierSpan{{c.x + a, c.y}, {c.x + a, c.y + k * b}, {c.x + k * a, c.y + b}, {c.x, c.y + b}});
    e.append(BezierSpan{{c.x, c.y + b}, {c.x - k * a, c.y + b}, {c.x - a, c.y + k * b}, {c.x - a, c.y}});
    e.append(BezierSpan{{c.x - a, c.y}, {c.x - a, c.y - k * b}, {c.x - k * a, c.y - b}, {c.x, c.y - b}});
    e.append(BezierSpan{{c.x, c.y - b}, {c.x + k * a, c.y - b}, {c.x + a, c.y - k * b}, {c.x + a, c.y}});
    return e;
}
inline Contour star(Vec2 o, double R, double r, int n, double rot = 0) {
    std::vector<Vec2> p; for (int i = 0; i < 2 * n; ++i) { double a = rot + PI * i / n; double rr = i % 2 ? r : R; p.push_back({o.x + rr * std::cos(a), o.y + rr * std::sin(a)}); } return poly(p);
}
// stadium/slot: two semicircle arcs + two lines
inline Contour slot(Vec2 a, double len, double r) {
    Contour c; c.closed = true;
    c.append(LineSpan{{a.x, a.y - r}, {a.x + len, a.y - r}});
    c.append(ArcSpan{{a.x + len, a.y}, r, -PI / 2, PI / 2, ArcDirection::CCW});
    c.append(LineSpan{{a.x + len, a.y + r}, {a.x, a.y + r}});
    c.append(ArcSpan{{a.x, a.y}, r, PI / 2, 3 * PI / 2, ArcDirection::CCW});
    return c;
}
// "C" glyph: annular sector 270 deg with round caps (arcs only)
inline Contour cGlyph(Vec2 o, double R, double w) {
    double a0 = PI / 4, a1 = 2 * PI - PI / 4, rm = R - w / 2;
    Contour c; c.closed = true;
    c.append(ArcSpan{o, R, a0, a1, ArcDirection::CCW});
    Vec2 e1{o.x + rm * std::cos(a1), o.y + rm * std::sin(a1)};
    c.append(ArcSpan{e1, w / 2, a1, a1 + PI, ArcDirection::CCW});
    c.append(ArcSpan{o, R - w, a1, a0, ArcDirection::CW});
    Vec2 e0{o.x + rm * std::cos(a0), o.y + rm * std::sin(a0)};
    c.append(ArcSpan{e0, w / 2, a0 + PI, a0 + 2 * PI, ArcDirection::CCW});
    return c;
}
// "S"-like stroke built from Beziers (closed outline, sharp ends)
inline Contour sGlyph(Vec2 o, double s) {
    Contour c; c.closed = true;
    auto P = [&](double x, double y) { return Vec2{o.x + s * x, o.y + s * y}; };
    c.append(BezierSpan{P(0.9, 0.85), P(0.6, 1.1), P(0.0, 1.05), P(0.05, 0.72)});
    c.append(BezierSpan{P(0.05, 0.72), P(0.1, 0.45), P(0.9, 0.55), P(0.92, 0.25)});
    c.append(BezierSpan{P(0.92, 0.25), P(0.95, -0.1), P(0.2, -0.1), P(0.05, 0.12)});
    c.append(LineSpan{P(0.05, 0.12), P(0.17, 0.18)});
    c.append(BezierSpan{P(0.17, 0.18), P(0.3, 0.03), P(0.78, 0.03), P(0.76, 0.25)});
    c.append(BezierSpan{P(0.76, 0.25), P(0.74, 0.47), P(-0.08, 0.40), P(-0.08, 0.73)});
    c.append(BezierSpan{P(-0.08, 0.73), P(-0.08, 1.2), P(0.75, 1.2), P(0.98, 0.95)});
    c.append(LineSpan{P(0.98, 0.95), P(0.9, 0.85)});
    return c;
}
struct Shape { std::string name; std::vector<Contour> cs; };
inline std::vector<Shape> all() {
    std::vector<Shape> v;
    v.push_back({"rect40x20", {poly({{0, 0}, {40, 0}, {40, 20}, {0, 20}})}});
    v.push_back({"square_cw", {poly({{0, 0}, {0, 10}, {10, 10}, {10, 0}})}});
    v.push_back({"circle", {circle({1, 2}, 7)}});
    v.push_back({"ellipse_bez", {bezEllipse({0, 0}, 20, 8)}});
    v.push_back({"L", {poly({{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 15}, {0, 15}})}});
    v.push_back({"T", {poly({{0, 10}, {7, 10}, {7, 0}, {10, 0}, {10, 10}, {17, 10}, {17, 13}, {0, 13}})}});
    v.push_back({"star5", {star({0, 0}, 10, 4, 5, PI / 2)}});
    v.push_back({"star12_sharp", {star({0, 0}, 10, 7, 12)}});
    v.push_back({"sq_holes", {poly({{0, 0}, {30, 0}, {30, 30}, {0, 30}}), rev(poly({{5, 5}, {12, 5}, {12, 12}, {5, 12}})), circle({21, 20}, 4)}});
    v.push_back({"annulus", {circle({0, 0}, 10), rev(circle({0, 0}, 6))}});
    v.push_back({"dumbbell_neck05", {poly({{0, 0}, {5, 0}, {5, 2.475}, {7, 2.475}, {7, 0}, {12, 0}, {12, 5}, {7, 5}, {7, 2.525}, {5, 2.525}, {5, 5}, {0, 5}})}});
    v.push_back({"neck_0.03", {poly({{0, 0}, {5, 0}, {5, 2.485}, {7, 2.485}, {7, 0}, {12, 0}, {12, 5}, {7, 5}, {7, 2.515}, {5, 2.515}, {5, 5}, {0, 5}})}});
    v.push_back({"neck_0.021", {poly({{0, 0}, {5, 0}, {5, 2.4895}, {7, 2.4895}, {7, 0}, {12, 0}, {12, 5}, {7, 5}, {7, 2.5105}, {5, 2.5105}, {5, 5}, {0, 5}})}});
    v.push_back({"slot_arcs", {slot({0, 0}, 20, 3)}});
    v.push_back({"C_glyph_arcs", {cGlyph({0, 0}, 10, 3)}});
    v.push_back({"S_glyph_bez", {sGlyph({0, 0}, 20)}});
    v.push_back({"O_glyph_bez", {bezEllipse({0, 0}, 8, 11), rev(bezEllipse({0, 0}, 5, 8.5))}});
    v.push_back({"A_glyph", {poly({{0, 0}, {3, 0}, {4, 4}, {8, 4}, {9, 0}, {12, 0}, {7.5, 16}, {4.5, 16}}), rev(poly({{4.8, 6.5}, {7.2, 6.5}, {6, 11.5}}))}});
    v.push_back({"sliver5deg", {poly({{0, 0}, {30, 0}, {30 * std::cos(5 * PI / 180), 30 * std::sin(5 * PI / 180)}})}});
    v.push_back({"thin_rect_0.1", {poly({{0, 0}, {20, 0}, {20, 0.1}, {0, 0.1}})}});
    v.push_back({"concave_arc_notch", [] { Contour c; c.closed = true; c.append(LineSpan{{0, 0}, {20, 0}}); c.append(LineSpan{{20, 0}, {20, 10}}); c.append(LineSpan{{20, 10}, {14, 10}});
        c.append(ArcSpan{{10, 10}, 4, 0.0, PI, ArcDirection::CW}); c.append(LineSpan{{6, 10}, {0, 10}}); c.append(LineSpan{{0, 10}, {0, 0}}); return std::vector<Contour>{c}; }()});
    return v;
}
}  // namespace vs
