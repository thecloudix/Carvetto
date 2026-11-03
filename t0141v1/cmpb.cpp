// T-0141 verifier: one Clipper2 call (threshold = never) vs default batching vs threshold 0.
// prints per scene: idx kind nIn nOne nDef nZero defBatched H_def(units) H_zero xorDef/perim xorZero/perim orientOK
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <unordered_map>
#include "clipper2/clipper.h"
#include "offset_batch.hpp"
#include "scenes.hpp"
using namespace Clipper2Lib;
static Paths64 toP(const std::vector<Contour>& cs) { Paths64 o; for (auto& c : cs) { Path64 p; for (auto& s : c.spans) { Vec2 a = startPoint(s); p.emplace_back(std::llround(a.x * 1e6), std::llround(a.y * 1e6)); } o.push_back(p); } return o; }
static double segD(Point64 p, Point64 a, Point64 b) { double dx = double(b.x - a.x), dy = double(b.y - a.y), l = dx * dx + dy * dy;
  double t = l > 0 ? std::clamp((double(p.x - a.x) * dx + double(p.y - a.y) * dy) / l, 0.0, 1.0) : 0; return std::hypot(double(p.x - a.x) - t * dx, double(p.y - a.y) - t * dy); }
// one-sided: max over vertices of A of distance to B's segments (found only within ~R; else 1e300)
static double oneSided(const Paths64& A, const Paths64& B, double R) {
  double totlen = 0; for (auto& p : B) for (size_t i = 0; i < p.size(); ++i) totlen += std::hypot(double(p[(i+1)%p.size()].x - p[i].x), double(p[(i+1)%p.size()].y - p[i].y));
  const double cell = std::max(R, totlen / 3e5); std::unordered_map<uint64_t, std::vector<std::pair<Point64, Point64>>> grid;
  auto key = [](int64_t cx, int64_t cy) { return (uint64_t(cx) * 0x9E3779B97F4A7C15ull) ^ uint64_t(cy); };
  for (auto& p : B) for (size_t i = 0; i < p.size(); ++i) { Point64 a = p[i], b = p[(i + 1) % p.size()];
    double len = std::hypot(double(b.x - a.x), double(b.y - a.y)); int ns = 1 + int(len / (cell * 0.25));
    uint64_t lastk = 0; bool has = false;
    for (int k = 0; k <= ns; ++k) { double t = double(k) / ns; double x = a.x + t * (b.x - a.x), y = a.y + t * (b.y - a.y);
      uint64_t kk = key(int64_t(std::floor(x / cell)), int64_t(std::floor(y / cell))); if (has && kk == lastk) continue; has = true; lastk = kk;
      auto& v = grid[kk]; if (v.empty() || v.back().first != a || v.back().second != b) v.push_back({a, b}); } }
  double worst = 0;
  for (auto& p : A) for (auto v : p) { double best = 1e300; int64_t cx = int64_t(std::floor(v.x / cell)), cy = int64_t(std::floor(v.y / cell));
    for (int dx = -2; dx <= 2; ++dx) for (int dy = -2; dy <= 2; ++dy) { auto it = grid.find(key(cx + dx, cy + dy)); if (it == grid.end()) continue;
      for (auto& s : it->second) best = std::min(best, segD(v, s.first, s.second)); }
    worst = std::max(worst, best); }
  return worst;
}
static double perim(const Paths64& A) { double s = 0; for (auto& p : A) for (size_t i = 0; i < p.size(); ++i) { auto a = p[i], b = p[(i + 1) % p.size()]; s += std::hypot(double(b.x - a.x), double(b.y - a.y)); } return s; }
static bool orientMatch(const Paths64& A, const Paths64& B, double tolArea) {
  std::vector<double> a, b; for (auto& p : A) a.push_back(Area(p)); for (auto& p : B) b.push_back(Area(p));
  if (a.size() != b.size()) return false; std::sort(a.begin(), a.end()); std::sort(b.begin(), b.end());
  int na = 0, nb = 0; for (double x : a) na += x < 0; for (double x : b) nb += x < 0; if (na != nb) return false;
  for (size_t i = 0; i < a.size(); ++i) if (std::fabs(a[i] - b[i]) > tolArea) return false; return true; }
int main(int argc, char** argv) {
  int a = std::atoi(argv[1]), b = std::atoi(argv[2]);
  const size_t kNever = std::numeric_limits<size_t>::max();
  for (int i = a; i < b; ++i) { Gen g(0x141000ull + i); Scene s = g.make(i);
    auto one = carvetto::geometry::detail::offsetContoursBatched(s.cs, s.d, s.join, s.tol, s.mitre, kNever);
    auto def = offsetContours(s.cs, s.d, s.join, s.tol, s.mitre);
    auto zero = carvetto::geometry::detail::offsetContoursBatched(s.cs, s.d, s.join, s.tol, s.mitre, 0);
    Paths64 P1 = toP(one), PD = toP(def), PZ = toP(zero);
    bool batched = hashOut(one) != hashOut(def);
    double hd = batched ? std::max(oneSided(P1, PD, 500), oneSided(PD, P1, 500)) : 0.0;
    bool zsame = hashOut(one) == hashOut(zero);
    double hz = zsame ? 0.0 : std::max(oneSided(P1, PZ, 500), oneSided(PZ, P1, 500));
    double per = std::max(perim(P1), 1.0);
    double xd = 0, xz = 0;
    if (batched) for (auto& r : Xor(P1, PD, FillRule::NonZero)) xd += std::fabs(Area(r));
    if (!zsame) for (auto& r : Xor(P1, PZ, FillRule::NonZero)) xz += std::fabs(Area(r));
    bool od = orientMatch(P1, PD, 0) || orientMatch(P1, PD, per * 30), oz = orientMatch(P1, PZ, per * 30);
    std::printf("%d %d %zu %zu %zu %zu %d %.1f %.1f %.3f %.3f %d %d\n", i, s.kind, s.cs.size(), one.size(), def.size(), zero.size(), batched, hd, hz, xd / per, xz / per, od, oz);
    std::fflush(stdout); }
}
