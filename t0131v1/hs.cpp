// T-0131 verifier: bit-exact hash of Profile and Pocket output (moves, diagnostics, pocket plans)
// over random scenes and options, with scale/translation transforms. Same source for main/branch.
// usage: hs prof|pock seed N [scale] [offset]   (scale multiplies geometry AND tool sizes; offset translates)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour;
using geometry::Vec2;
using geometry::Box2;
using geometry::Polyline;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "scene_io.hpp"
#include "gen_helpers.inc"
struct H { unsigned long long h = 1469598103934665603ULL;
  void bytes(const void* p, size_t n) { auto* c = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; } }
  void d(double x) { bytes(&x, 8); } void i(long long x) { bytes(&x, 8); } void s(const std::string& x) { i((long long)x.size()); bytes(x.data(), x.size()); } };
static void hashTp(H& h, const Toolpath& tp) { h.i((long long)tp.moves.size());
  for (auto& m : tp.moves) { h.i((int)m.kind); h.d(m.to.x); h.d(m.to.y); h.d(m.to.z); h.d(m.centre.x); h.d(m.centre.y); h.i(m.feedOverride.has_value()); h.d(m.feedOverride.value_or(0)); h.d(m.dwellSeconds); } }
static void hashDiag(H& h, const std::vector<ToolpathDiagnostic>& ds) { for (auto& d : ds) { h.i((int)d.issue); h.i(d.contour ? (long long)*d.contour : -1); h.i(d.tool ? (long long)*d.tool : -1); h.s(d.message); } }
static Contour xf(Contour c, double S, Vec2 off) {
  auto P = [&](Vec2 p) { return Vec2{p.x * S + off.x, p.y * S + off.y}; };
  for (auto& sp : c.spans) {
    if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = P(l->start); l->end = P(l->end); }
    else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = P(a->centre); a->radius *= S; }
    else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = P(b->p0); b->p1 = P(b->p1); b->p2 = P(b->p2); b->p3 = P(b->p3); }
  }
  return c;
}
static std::vector<Contour> page(size_t n, double pitch) {
  std::vector<Contour> cs; size_t cols = size_t(std::ceil(std::sqrt(double(n))));
  for (size_t k = 0; cs.size() < n; ++k) {
    double s = U(2.0, 4.5), x = double(k % cols) * pitch + U(-0.5, 0.5), y = double(k / cols) * pitch + U(-0.5, 0.5);
    switch (I(0, 6)) {
      case 0: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); break;
      case 1: cs.push_back(letter({x, y}, s * 0.7)); break;
      case 2: cs.push_back(Contour::circle({x + s / 2, y + s / 2}, s / 2)); break;
      case 3: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); if (cs.size() < n) cs.push_back(Contour::circle({x + s / 2, y + s / 2}, 0.2 * s)); break;
      case 4: cs.push_back(spikes({x + s / 2, y + s / 2}, s / 2, I(3, 7))); break;
      case 5: cs.push_back(bezBlob({x + s / 2, y + s / 2}, s / 2, I(4, 7))); break;
      default: cs.push_back(anyOpen({x + s / 2, y + s / 2}, s / 2)); break;
    }
  }
  return cs;
}
static Contour combC(int teeth, double gap, double depth) {
  std::vector<Vec2> pts; for (int i = 0; i < teeth; ++i) { pts.push_back({i * 1.0, 0}); pts.push_back({i * 1.0, depth}); pts.push_back({i + gap, depth}); pts.push_back({i + gap, 0.0}); }
  pts.push_back({double(teeth), -5}); pts.push_back({0, -5}); return Contour::fromPolyline(Polyline{pts, true});
}
static PocketTool ptool(const char* nm, int num, double d, double so, double pd) {
  PocketTool t; t.tool.id = nm; t.tool.name = nm; t.tool.type = tooling::ToolType::EndMill; t.tool.diameter = d;
  auto& cd = t.tool.defaultCuttingData; cd.toolNumber = num; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 18000; cd.stepover = so; cd.passDepth = pd; return t;
}
int main(int argc, char** argv) {
  std::string what = argv[1]; unsigned long seed = std::strtoul(argv[2], nullptr, 10); int N = std::atoi(argv[3]);
  double S = argc > 4 ? std::atof(argv[4]) : 1.0; double O = argc > 5 ? std::atof(argv[5]) : 0.0;
  int target = getenv("TARGET") ? atoi(getenv("TARGET")) : -1;
  rng.seed(seed);
  for (int sc = 0; sc < N; ++sc) {
    if (what == "prof") {
      int fam = 0, forceSep = -1; int mode = sc % 4;  // 0 families, 1 cluster, 2 grid of many small, 3 text page
#include "gen_body.inc"
      p.tabs.enabled = I(0, 1) == 0;
      if (mode >= 1) {
        cs.clear();
        if (mode == 3) { cs = page((size_t)I(20, 300), U(5, 8)); D = std::vector<double>{0.5, 1.0, 2.0}[(size_t)I(0, 2)]; useV = false;
          p.passes.cutDepth = U(1, 4); p.tabs.thickness = U(0.2, p.passes.cutDepth); p.tabs.length = U(0.3, 2); p.leads.length = U(0.2, 2); p.leads.radius = U(0.2, 2); p.ramp.distance = U(0.5, 5);
          p.lastPassAllowance = U(0.02, 0.4); p.separateLastPass = I(0, 1); }
        else {
          int K = mode == 1 ? I(2, 25) : I(30, 300);
          double box = mode == 1 ? U(20, 150) : std::sqrt((double)K) * U(5, 14);
          for (int k = 0; k < K; ++k) {
            Vec2 c{U(0, box), U(0, box)}; double r = mode == 1 ? U(2, 25) : U(0.8, 5); int t = I(0, 9);
            if (t <= 3) cs.push_back(anyClosed(c, r));
            else if (t == 4) cs.push_back(Contour::circle(c, r, I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW));
            else if (t == 5) cs.push_back(Contour::rectangle(geometry::Box2{{c.x - r, c.y - r * 0.6}, {c.x + r, c.y + r * 0.6}}));
            else if (t == 6) cs.push_back(anyOpen(c, r));
            else if (t == 7) { cs.push_back(Contour::circle(c, r)); cs.push_back(Contour::circle(c + Vec2{U(-0.2, 0.2) * r, U(-0.2, 0.2) * r}, r * U(0.2, 0.6))); }
            else if (t == 8) cs.push_back(bezBlob(c, r, I(4, 8)));
            else cs.push_back(letter(c, r));
          }
          if (mode == 2) { D = std::vector<double>{0.5, 1.0, 2.0, 3.175}[(size_t)I(0, 3)]; useV = useV && I(0, 1); }
        }
        if (I(0, 4) == 0) p.side = ProfileSide::On;
      }
      if (target >= 0 && sc != target) continue;
      for (auto& c : cs) c = xf(c, S, {O, O * 0.7});
      p.allowance *= S; p.lastPassAllowance *= S; p.tabs.length *= S; p.tabs.distance *= S; p.leads.length *= S; p.leads.radius *= S; p.leads.overcut *= S; p.ramp.distance *= S;
      p.startPosition = {p.startPosition.x * S + O, p.startPosition.y * S + O * 0.7};
      tooling::Tool tool = useV ? vbit(D * 2 * S, vang) : endMill(D * S);
      auto res = profileToolpath(cs, tool, p);
      H h; hashTp(h, res.toolpath); hashDiag(h, res.diagnostics);
      std::printf("prof %d mode %d K %zu side %d sep %d tabs %d V %d moves %zu diags %zu hash %016llx\n", sc, mode, cs.size(), (int)p.side, (int)p.separateLastPass, (int)p.tabs.enabled, (int)useV, res.toolpath.moves.size(), res.diagnostics.size(), h.h);
    } else {
      std::vector<Contour> cs; int fam = I(0, 5);
      PocketParams p; p.cutDepth = U(1, 9); p.strategy = I(0, 1) ? PocketStrategy::Raster : PocketStrategy::Offset;
      double angs[] = {0, PI / 6, PI / 4, PI / 2, 2.0, -0.3}; p.rasterAngle = I(0, 2) ? angs[I(0, 5)] : U(-PI, PI);
      p.profilePass = (ProfilePass)I(0, 2); p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
      p.allowance = I(0, 2) == 0 ? U(0, 0.3) : 0.0; p.rampPlunge = I(0, 2) == 0; p.rampDistance = U(1, 15);
      if (I(0, 4) == 0) p.lastPassThickness = U(0.1, 1);
      std::vector<PocketTool> tools;
      double d0;
      switch (fam) {
        case 0: { int K = I(1, 6); for (int k = 0; k < K; ++k) cs.push_back(anyClosed({U(0, 80), U(0, 80)}, U(8, 30))); d0 = U(2, 8); break; }
        case 1: { double W = U(40, 120), Hh = U(30, 90); cs.push_back(Contour::rectangle(Box2{{0, 0}, {W, Hh}})); int n = I(0, 6);
          for (int i = 0; i < n; ++i) { double rr = U(2, 10); Vec2 c{U(rr + 1, W - rr - 1), U(rr + 1, Hh - rr - 1)}; cs.push_back(I(0, 1) ? Contour::circle(c, rr) : letter(c - Vec2{rr / 2, rr / 2}, rr)); }
          d0 = U(2, 8); break; }
        case 2: cs = page((size_t)I(20, 250), U(5, 8)); d0 = U(0.6, 1.5); break;
        case 3: cs.push_back(combC(I(5, 120), U(0.3, 0.7), U(5, 25))); d0 = U(0.2, 0.5); break;
        case 4: { int K = I(1, 4); for (int k = 0; k < K; ++k) { Vec2 c{U(0, 60), U(0, 60)}; int t = I(0, 3); cs.push_back(t == 0 ? bezBlob(c, U(8, 25), I(4, 8)) : t == 1 ? spikes(c, U(8, 25), I(3, 8)) : t == 2 ? gear(c, U(10, 25), I(6, 20), U(1, 5)) : flower(c, U(4, 10), 2 * I(2, 5))); } d0 = U(1, 6); break; }
        default: { int K = I(10, 120); double box = std::sqrt((double)K) * U(8, 15); for (int k = 0; k < K; ++k) { Vec2 c{U(0, box), U(0, box)}; double r = U(1, 6); int t = I(0, 4);
          cs.push_back(t == 0 ? Contour::circle(c, r) : t == 1 ? letter(c, r) : t == 2 ? anyClosed(c, r) : t == 3 ? bezBlob(c, r, I(4, 7)) : Contour::rectangle(Box2{{c.x - r, c.y - r / 2}, {c.x + r, c.y + r / 2}})); } d0 = U(0.5, 3); break; }
      }
      double so = U(0.25, 0.7);
      tools.push_back(ptool("T1", 1, d0 * S, d0 * so * S, U(1, 9)));
      int nr = I(0, 3) == 0 ? 0 : I(1, 2);
      double dr = d0;
      for (int k = 0; k < nr; ++k) { dr *= U(0.2, 0.6); tools.push_back(ptool(k ? "T3" : "T2", 2 + k, dr * S, dr * U(0.25, 0.7) * S, U(1, 9))); }
      if (target >= 0 && sc != target) continue;
      for (auto& c : cs) c = xf(c, S, {O, O * 0.7});
      p.allowance *= S; p.rampDistance *= S; p.lastPassThickness *= 1.0;
      auto res = pocketToolpath(cs, tools, p);
      H h; size_t mv = 0;
      for (auto& tp : res.toolpaths) { hashTp(h, tp); mv += tp.moves.size(); }
      for (auto& pl : res.plans) { h.i((long long)pl.paths.size()); for (auto& pp : pl.paths) { h.i((int)pp.role); h.i((long long)pp.cluster); h.i(pp.path.closed); for (auto& q : pp.path.points) { h.d(q.x); h.d(q.y); } } }
      hashDiag(h, res.diagnostics);
      std::printf("pock %d fam %d K %zu strat %d ang %.3f pp %d tools %zu moves %zu diags %zu hash %016llx\n", sc, fam, cs.size(), (int)p.strategy, p.rasterAngle, (int)p.profilePass, tools.size(), mv, res.diagnostics.size(), h.h);
    }
    std::fflush(stdout);
  }
}
