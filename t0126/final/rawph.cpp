// Do Clipper raw insets contain "partial phantoms": a raw outer ring where SOME (<= half) vertices are
// far too close to the walls (inverted pieces merged with a genuine region)? Then check what
// pocketCentreRegion does with it (exact edge distances to the input polygon).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto;
using geometry::Contour; using geometry::Vec2; using Poly = std::vector<Vec2>;
static const double TAU = 6.283185307179586;
static double segPt(Vec2 p, Vec2 a, Vec2 b) { double dx = b.x - a.x, dy = b.y - a.y, L = dx * dx + dy * dy; double t = L > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / L : 0; t = std::clamp(t, 0.0, 1.0); return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy); }
static double orient(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
static double segSeg(Vec2 a, Vec2 b, Vec2 c, Vec2 d) { double o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b); if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return 0; return std::min({segPt(a, c, d), segPt(b, c, d), segPt(c, a, b), segPt(d, a, b)}); }
static double distPolys(const std::vector<Poly>& W, Vec2 a, Vec2 b) { double best = 1e300; for (auto& p : W) for (size_t i = 0; i < p.size(); ++i) best = std::min(best, segSeg(a, b, p[i], p[(i + 1) % p.size()])); return best; }
static double area(const Poly& p) { double A = 0; for (size_t i = 0; i < p.size(); ++i) A += p[i].x * p[(i + 1) % p.size()].y - p[(i + 1) % p.size()].x * p[i].y; return A / 2; }

int main(int argc, char** argv) {
    int fam = atoi(argv[1]); uint64_t seed0 = strtoull(argv[2], 0, 10); int n = atoi(argv[3]); double ch = argc > 4 ? atof(argv[4]) : 0.01;
    geometry::Tolerance ft = geometry::Tolerance::defaults(); ft.chordal = 2e-4;
    long partial = 0, full = 0, gouges = 0, runs = 0, msgs = 0;
    for (int k = 0; k < n; ++k) {
        std::mt19937_64 R(seed0 + k);
        auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(R); };
        std::vector<Contour> parts;
        if (fam == 0) {  // big disc + protruding small discs
            double B = U(8, 20); parts.push_back(Contour::circle({0, 0}, B));
            int m = int(U(1, 6));
            for (int i = 0; i < m; ++i) { double t = U(0, TAU), rho = U(0.5, 0.45 * B), L = B + rho * U(-0.8, 0.9); parts.push_back(Contour::circle({L * std::cos(t), L * std::sin(t)}, rho)); }
        } else if (fam == 1) {  // peanut (two discs) + lobes at the waist
            double B1 = U(6, 15), B2 = U(6, 15), sep = (B1 + B2) * U(0.55, 0.95);
            parts.push_back(Contour::circle({0, 0}, B1)); parts.push_back(Contour::circle({sep, 0}, B2));
            int m = int(U(1, 4));
            for (int i = 0; i < m; ++i) { double x = sep * U(0.3, 0.7), rho = U(0.5, 4); double side = i % 2 ? 1 : -1; double y = side * (std::min(B1, B2) * U(0.3, 1.0) + rho * U(-0.5, 0.9)); parts.push_back(Contour::circle({x, y}, rho)); }
        } else if (fam == 2) {  // square with rounded bumps (rect + discs on edges) and a notch
            double S = U(15, 35); parts.push_back(Contour::rectangle({{0, 0}, {S, S}}));
            int m = int(U(2, 8));
            for (int i = 0; i < m; ++i) { double rho = U(0.5, 5); int e = int(U(0, 4)); double s = U(0.1, 0.9) * S, o = rho * U(-0.5, 0.9); Vec2 c = e == 0 ? Vec2{s, -o} : e == 1 ? Vec2{S + o, s} : e == 2 ? Vec2{s, S + o} : Vec2{-o, s}; parts.push_back(Contour::circle(c, rho)); }
        } else {  // star polygon with round knobs at the tips
            int K = int(U(3, 9)); double ro = U(10, 25), ri = ro * U(0.3, 0.7); Poly p;
            for (int i = 0; i < 2 * K; ++i) { double t = TAU / 2 * i / K, r = i % 2 ? ri : ro; p.push_back({r * std::cos(t), r * std::sin(t)}); }
            parts.push_back(Contour::fromPolyline(geometry::Polyline{p, true}));
            for (int i = 0; i < 2 * K; i += 2) { double rho = U(0.5, 4); parts.push_back(Contour::circle(p[i], rho)); }
        }
        auto u = geometry::unionContours(parts, ft);
        std::vector<Poly> W; std::vector<Contour> in;
        double maxR = 0;
        for (auto& c : u) { auto pts = c.polygonise(ft).points; W.push_back(pts); in.push_back(Contour::fromPolyline(geometry::Polyline{pts, true})); for (auto v : pts) maxR = std::max(maxR, std::hypot(v.x, v.y)); }
        geometry::Tolerance rt = geometry::Tolerance::defaults(); rt.chordal = ch / 2; rt.distance = std::min(rt.distance, ch);
        auto region = geometry::unionContours(in, rt, geometry::FillRule::EvenOdd);
        toolpath::PocketParams P; P.tolerance.chordal = ch; P.tolerance.distance = std::min(P.tolerance.distance, ch);
        for (int t = 0; t < 12; ++t) {
            double dia = U(0.5, maxR * 1.5), Dd = dia / 2 + ch;
            auto raw = geometry::offsetContours(region, -Dd, geometry::JoinType::Round, rt);
            bool isPartial = false, isFull = false; std::string info;
            for (auto& c : raw) {
                auto pts = c.polygonise(geometry::Tolerance::defaults()).points;
                if (area(pts) <= 0) continue;
                size_t bad = 0; double worst = 0;
                for (auto v : pts) { double dd = distPolys(W, v, v); if (dd < Dd - ch / 2 - 1e-6) { ++bad; worst = std::max(worst, Dd - ch / 2 - dd); } }
                if (bad && 2 * bad <= pts.size() && worst > 0.02) { isPartial = true; char b[200]; snprintf(b, sizeof b, " partial ring n=%zu bad=%zu worst=%.4f", pts.size(), bad, worst); info += b; }
                if (2 * bad > pts.size()) isFull = true;
            }
            ++runs; partial += isPartial; full += isFull;
            std::vector<std::string> ms;
            auto C = toolpath::pocketCentreRegion(in, dia, P, &ms);
            msgs += !ms.empty();
            double worstE = 1e9; Vec2 wa{};
            for (auto& c : C) { auto pts = c.polygonise(geometry::Tolerance::defaults()).points; for (size_t i = 0; i < pts.size(); ++i) { double dd = distPolys(W, pts[i], pts[(i + 1) % pts.size()]) - dia / 2; if (dd < worstE) { worstE = dd; wa = pts[i]; } } }
            bool g = worstE < -1e-7;
            gouges += g;
            if (isPartial || g || !ms.empty()) printf("fam %d seed %llu d=%.6f%s | C parts %zu worst edge-(r) %.3e at (%.4f,%.4f)%s msgs %zu\n", fam, (unsigned long long)(seed0 + k), dia, info.c_str(), C.size(), worstE, wa.x, wa.y, g ? " GOUGE" : "", ms.size());
        }
    }
    printf("fam %d: runs %ld, raw partial-phantom rings %ld, raw full phantoms %ld, gouges %ld, msgs %ld\n", fam, runs, partial, full, gouges, msgs);
}
