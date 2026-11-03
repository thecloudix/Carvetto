// T-0141 verifier throwaway: deterministic scene generator shared by main/branch drivers.
#pragma once
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
struct Scene { std::vector<Contour> cs; double d; JoinType join; double mitre; Tolerance tol; int kind; };
static Contour poly(const std::vector<Vec2>& p) { Contour c; c.closed = true;
  for (size_t i = 0; i < p.size(); ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static Contour rectC(Vec2 lo, Vec2 hi) { return poly({lo, {hi.x, lo.y}, hi, {lo.x, hi.y}}); }
struct Gen {
  std::mt19937_64 r;
  explicit Gen(uint64_t s) : r(s) {}
  double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(r); }
  int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(r); }
  Contour shape(Vec2 c, double s, int t) {
    switch (t) {
      case 0: return rectC(c - Vec2{s, 0.6 * s}, c + Vec2{s, 0.6 * s});
      case 1: return Contour::circle(c, s);
      case 2: return poly({c + Vec2{-s, -s}, c + Vec2{s, -0.9 * s}, c + Vec2{-0.7 * s, s}});
      case 3: return rectC(c - Vec2{s, 0.05 * s}, c + Vec2{s, 0.05 * s});  // sliver
      case 4: return poly({c + Vec2{-s, -0.08 * s}, c + Vec2{s, 0}, c + Vec2{-s, 0.08 * s}});  // acute spike -> +x
      case 5: return poly({c + Vec2{s, -0.08 * s}, c + Vec2{-s, 0}, c + Vec2{s, 0.08 * s}});   // spike -> -x
      case 6: {  // star (concave)
        std::vector<Vec2> p; for (int k = 0; k < 10; ++k) { double a = k * 0.6283185307179586, rr = (k % 2 ? 0.35 : 1.0) * s;
          p.push_back(c + Vec2{rr * std::cos(a), rr * std::sin(a)}); } return poly(p); }
      default: { // L shape
        return poly({c + Vec2{-s, -s}, c + Vec2{s, -s}, c + Vec2{s, -0.5 * s}, c + Vec2{-0.5 * s, -0.5 * s}, c + Vec2{-0.5 * s, s}, c + Vec2{-s, s}}); }
    }
  }
  Scene make(int idx) {
    Scene sc; sc.kind = idx % 8;
    const double size = U(0.2, 3.0);
    const int joinSel = I(0, 2);
    sc.join = joinSel == 0 ? JoinType::Round : joinSel == 1 ? JoinType::Square : JoinType::Sharp;
    const double mitres[] = {0.5, 1.0, 1.5, 2.0, 4.0, 10.0};
    sc.mitre = mitres[I(0, 5)];
    const double reach = sc.join == JoinType::Round ? 1.0 : sc.join == JoinType::Square ? std::sqrt(2.0) : std::max(std::sqrt(2.0), sc.mitre);
    // distance
    int q = I(0, 5);
    if (q <= 1) sc.d = -size * U(0.02, 0.4);
    else if (q <= 3) sc.d = size * U(0.01, 1.0);
    else if (q == 4) sc.d = 0.0;
    else sc.d = -size * 0.6 * U(0.95, 1.0);  // near collapse of rect/ sliver
    const double ad = std::fabs(sc.d);
    // gap between neighbouring shape boxes
    double gap;
    int g = I(0, 5);
    if (g == 0) gap = 2 * reach * ad;             // exactly the reach
    else if (g == 1) gap = 2 * reach * ad * (1 - 1e-7);
    else if (g == 2) gap = 2 * reach * ad + 3e-6;
    else if (g == 3) gap = 2 * ad;                 // offsets of edges touch exactly
    else gap = U(0.0, 3.0) * std::max(ad, 0.1 * size);
    const double big[] = {0.0, 0.0, 0.0, 1e3, 1e5, 1e7};
    const Vec2 origin{U(-1, 1) * big[I(0, 5)], U(-1, 1) * big[I(0, 5)]};
    const int nx = sc.kind == 7 ? I(2, 31) : I(30, 80);  // kind 7: below threshold
    const int ny = I(1, 3);
    const int orient = I(0, 4);  // 0 CCW, 1 mixed, 2 all CW, 3 holes + islands, 4 CW batch below everything
    const double pitch = 2 * size + gap;
    sc.tol = Tolerance{0.001, 1e-9, I(0, 1) ? 0.001 : 0.01};
    int t0 = I(0, 7);
    bool same = I(0, 1);
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < ny; ++j) {
        Vec2 c = origin + Vec2{pitch * i, pitch * j * 1.3};
        if (g >= 4) c = c + Vec2{U(-0.1, 0.1), U(-0.1, 0.1)} * size;
        int t = same ? t0 : I(0, 7);
        // spikes face each other in pairs
        if (t == 4 || t == 5) t = (i % 2 == 0) ? 4 : 5;
        Contour k = shape(c, size, t);
        bool flip = orient == 2 || (orient == 1 && I(0, 1));
        sc.cs.push_back(flip ? k.reversed() : k);
        if (orient == 3 && I(0, 1)) {
          sc.cs.push_back(Contour::circle(c, 0.3 * size, ArcDirection::CW));
          if (I(0, 1)) sc.cs.push_back(Contour::circle(c, 0.1 * size));
        }
      }
    if (orient == 4) {  // lowest contour CW, far below
      sc.cs.push_back(Contour::circle(origin + Vec2{0, -20 * size}, size, ArcDirection::CW));
      if (I(0, 1)) for (int i = 0; i < 40; ++i) sc.cs.push_back(Contour::circle(origin + Vec2{pitch * i, -10 * size}, size * 0.5, ArcDirection::CW));
    }
    if (I(0, 3) == 0) std::shuffle(sc.cs.begin(), sc.cs.end(), r);
    return sc;
  }
};
// FNV-1a over exact bits of the output.
static uint64_t hashOut(const std::vector<Contour>& cs) {
  uint64_t h = 1469598103934665603ull;
  auto mix = [&h](const void* p, size_t n) { auto b = static_cast<const unsigned char*>(p); for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; } };
  for (const Contour& c : cs) { size_t n = c.spans.size(); mix(&n, sizeof n);
    for (const Span& s : c.spans) { Vec2 a = startPoint(s), b = endPoint(s); mix(&a.x, 8); mix(&a.y, 8); mix(&b.x, 8); mix(&b.y, 8); } }
  return h;
}
