// t0160v1: verifier's own leftover + safety oracle for V-carve corner sharpening (independent of
// the branch's v_carve_leftover.hpp / v_carve_oracle.hpp). Distances and region membership against
// the ORIGINAL curves via acc.hpp (t0153v3). Carved surface: per move, min over the move of
// tip z + h(r) (level moves: closed form; inclined V/ball: ternary search of a convex function;
// inclined end mill: quadratic interval, lower end), indexed by a uniform grid of move boxes.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <vector>
#include "acc.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "carvetto/tooling/tool.hpp"
namespace lo {
using carvetto::geometry::Vec2; using carvetto::geometry::Vec3;
struct Shape { int type; double R; double tanh; };  // 0 V, 1 ball, 2 flat
inline double hgt(const Shape& s, double r) {  // tool surface height above the tip at radius r (convex extension past the rim)
    if (s.type == 2) return r <= s.R ? 0.0 : 1e9;
    if (s.type == 0) return r / s.tanh;
    double rr = 0.999 * s.R; if (r <= rr) return s.R - std::sqrt(s.R * s.R - r * r);
    double h0 = s.R - std::sqrt(s.R * s.R - rr * rr), k = rr / std::sqrt(s.R * s.R - rr * rr); return h0 + k * (r - rr);
}
inline double segDist(Vec2 p, Vec2 a, Vec2 b) { Vec2 d = b - a; double L2 = d.x * d.x + d.y * d.y; double t = L2 > 0 ? std::clamp(((p.x - a.x) * d.x + (p.y - a.y) * d.y) / L2, 0.0, 1.0) : 0.0; return std::hypot(p.x - a.x - d.x * t, p.y - a.y - d.y * t); }
struct Seg { Vec3 a, b; int tool; };
struct Surface {
    std::vector<Seg> segs; std::vector<Shape> shapes; std::vector<double> reach; double cell = 0.5;
    std::unordered_map<long long, std::vector<int>> grid;
    static long long key(long long i, long long j) { return i * 4000037LL + j; }
    // add every non-rapid move of tp (start from safe Z at the start position)
    void add(const carvetto::toolpath::Toolpath& tp, Shape sh, double reachR, Vec3 start) {
        int ti = (int)shapes.size(); shapes.push_back(sh); reach.push_back(reachR);
        Vec3 at = start;
        for (auto& m : tp.moves) { if (m.kind != carvetto::toolpath::MoveKind::Rapid) segs.push_back({at, m.to, ti}); at = m.to; }
    }
    void build(double c) { cell = c; grid.clear();
        for (int k = 0; k < (int)segs.size(); ++k) { auto& s = segs[k]; double r = reach[s.tool];
            long long i0 = (long long)std::floor((std::min(s.a.x, s.b.x) - r) / cell), i1 = (long long)std::floor((std::max(s.a.x, s.b.x) + r) / cell);
            long long j0 = (long long)std::floor((std::min(s.a.y, s.b.y) - r) / cell), j1 = (long long)std::floor((std::max(s.a.y, s.b.y) + r) / cell);
            for (long long i = i0; i <= i1; ++i) for (long long j = j0; j <= j1; ++j) grid[key(i, j)].push_back(k); } }
    static double segZ(const Seg& s, const Shape& sh, Vec2 q) {
        Vec2 a{s.a.x, s.a.y}, b{s.b.x, s.b.y};
        if (s.a.z == s.b.z || (a.x == b.x && a.y == b.y)) { double r = segDist(q, a, b); return std::min(s.a.z, s.b.z) + hgt(sh, r); }
        if (sh.type == 2) { // interval of t with |q - P(t)| <= R, lowest z there
            Vec2 d = b - a, f = a - q; double A = d.x * d.x + d.y * d.y, B = 2 * (f.x * d.x + f.y * d.y), C = f.x * f.x + f.y * f.y - sh.R * sh.R;
            double D = B * B - 4 * A * C; if (D < 0) return 1e9; double sq = std::sqrt(D); double t0 = std::max(0.0, (-B - sq) / (2 * A)), t1 = std::min(1.0, (-B + sq) / (2 * A)); if (t0 > t1) return 1e9;
            return std::min(s.a.z + (s.b.z - s.a.z) * t0, s.a.z + (s.b.z - s.a.z) * t1); }
        auto f = [&](double t) { Vec2 p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}; return s.a.z + (s.b.z - s.a.z) * t + hgt(sh, std::hypot(q.x - p.x, q.y - p.y)); };
        double lo = 0, hi = 1; for (int it = 0; it < 200 && hi - lo > 1e-15; ++it) { double m1 = lo + (hi - lo) * 0.381966011250105, m2 = hi - (hi - lo) * 0.381966011250105; if (f(m1) < f(m2)) hi = m2; else lo = m1; }
        return std::min({f(0), f(1), f(0.5 * (lo + hi))});
    }
    double z(Vec2 q) const { double best = 1e9; auto it = grid.find(key((long long)std::floor(q.x / cell), (long long)std::floor(q.y / cell))); if (it == grid.end()) return best;
        for (int k : it->second) { const Seg& s = segs[k]; double zz = segZ(s, shapes[s.tool], q); if (zz < best) best = zz; } return best; }
};
// exact min over a segment of (dist(P(t)) - off) via certified subdivision (dist is 1-Lipschitz)
inline double segMinClear(const va::Acc& AC, Vec2 a, Vec2 b, Vec2* at = nullptr) {
    double L = std::hypot(b.x - a.x, b.y - a.y); double best = 1e300; Vec2 bp = a;
    std::vector<std::pair<double, double>> st{{0, 1}}; int it = 0;
    while (!st.empty() && it < 400000) { ++it; auto [t0, t1] = st.back(); st.pop_back(); double tm = 0.5 * (t0 + t1); Vec2 p{a.x + (b.x - a.x) * tm, a.y + (b.y - a.y) * tm};
        double d = AC.dist(p); if (d < best) { best = d; bp = p; } double lb = d - 0.5 * L * (t1 - t0); if (lb >= best - 1e-10) continue; if (L * (t1 - t0) < 1e-9) continue; st.push_back({t0, tm}); st.push_back({tm, t1}); }
    for (Vec2 p : {a, b}) { double d = AC.dist(p); if (d < best) { best = d; bp = p; } }
    if (at) *at = bp; return best;
}
}  // namespace lo
