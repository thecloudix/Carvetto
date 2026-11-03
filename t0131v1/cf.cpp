// T-0131 verifier: ClearanceField::distance vs brute-force min of distanceTo (bit-exact), random
// lines/arcs/Beziers at many scales and offsets, NaN/inf obstacles, exclusions, query points near
// and far, equal-distance ties (symmetric obstacles).  usage: cf iters seed
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/clearance.hpp"
using namespace carvetto; using namespace carvetto::geometry;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
const double nan_ = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
int main(int argc, char** argv) {
  long iters = atol(argv[1]); rng.seed(atol(argv[2])); long bad = 0, q = 0;
  for (long it = 0; it < iters; ++it) {
    double sc = std::pow(10.0, U(-4, 4)); double off = I(0, 2) == 0 ? 0 : std::pow(10.0, U(0, 9)) * (I(0, 1) ? 1 : -1);
    Vec2 O{off, off * U(-1, 1)};
    auto P = [&]() { return O + Vec2{U(-1, 1), U(-1, 1)} * sc; };
    std::vector<Contour> obs; int n = I(1, I(0, 3) == 0 ? 150 : 12);
    bool sym = I(0, 4) == 0;
    for (int k = 0; k < n; ++k) {
      Contour c; int ns = I(1, 25); Vec2 cur = P(); double ss = sc * std::pow(10.0, U(-3, 0));
      for (int s = 0; s < ns; ++s) {
        int t = I(0, 2); Vec2 nx = cur + Vec2{U(-1, 1), U(-1, 1)} * ss;
        if (t == 0) c.append(LineSpan{cur, nx});
        else if (t == 1) { ArcSpan a; a.centre = cur + Vec2{U(-1, 1), U(-1, 1)} * ss; a.radius = ss * U(0.01, 2); a.startAngle = U(-7, 7); a.endAngle = a.startAngle + U(-6.3, 6.3); a.direction = I(0, 1) ? ArcDirection::CCW : ArcDirection::CW; if (I(0,9)==0) a.endAngle = a.startAngle + (a.direction==ArcDirection::CCW?1:-1)*6.283185307179586; c.spans.push_back(a); nx = a.endPoint(); }
        else c.append(BezierSpan{cur, cur + Vec2{U(-1, 1), U(-1, 1)} * ss, nx + Vec2{U(-1, 1), U(-1, 1)} * ss, nx});
        cur = nx;
      }
      if (I(0, 40) == 0 && !c.spans.empty()) { if (auto* l = std::get_if<LineSpan>(&c.spans[0])) l->end.x = I(0,1) ? nan_ : inf; }
      if (I(0, 60) == 0) c = Contour{};
      obs.push_back(c);
      if (sym) { Contour m = c; for (auto& sp : m.spans) { if (auto* l = std::get_if<LineSpan>(&sp)) { l->start.x = 2*O.x - l->start.x; l->end.x = 2*O.x - l->end.x; } } obs.push_back(m); }
    }
    toolpath::ClearanceField f(obs);
    std::vector<size_t> ex; if (I(0, 2) == 0) for (size_t i = 0; i < obs.size(); ++i) if (I(0, 3) == 0) ex.push_back(i);
    toolpath::ClearanceField g = ex.empty() ? f : f.excluding(ex);
    for (int k = 0; k < 40; ++k) {
      Vec2 p = I(0, 3) == 0 ? O + Vec2{U(-1, 1), U(-1, 1)} * sc * 20 : P();
      if (sym && I(0, 1)) p.x = O.x;  // on the mirror line: ties between mirrored obstacles
      if (I(0, 50) == 0) p.x = nan_;
      double best = inf;
      for (size_t i = 0; i < obs.size(); ++i) { bool skip = false; for (size_t e : ex) skip |= e == i; if (skip) continue; double d = distanceTo(obs[i], p); if (d < best) best = d; }
      double got = g.distance(p); ++q;
      if (std::memcmp(&got, &best, 8) != 0 && !(std::isnan(got) && std::isnan(best))) { if (bad++ < 10) std::printf("MISMATCH it %ld k %d sc %g off %g: got %.17g brute %.17g\n", it, k, sc, off, got, best); }
    }
  }
  std::printf("queries %ld mismatches %ld\n", q, bad); return bad ? 1 : 0;
}
