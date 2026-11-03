// T-0131 verifier: CPU-time cases. usage: pf case n [variant]
// cases: comb (angle via ANG env, deg; TOOL env diameter, default 0.3), sep (Profile separate last pass; TABS env),
//        rest (Pocket + rest tool; RASTER env; RD env rest diameter), tabs (Profile tabs), pockpage (plain pocket)
// variant 0 = the branch test's textPage (seed 77), 1 = t0123v1 perf.cpp page variant 1, 2 = hs.cpp page (other shapes/pitch)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2; using geometry::Box2; using geometry::Polyline;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng(77);
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "gen_helpers.inc"
static unsigned long long HH = 1469598103934665603ULL;
static void hb(const void* p, size_t n) { auto* c = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { HH ^= c[i]; HH *= 1099511628211ULL; } }
static void htp(const Toolpath& tp) { for (auto& m : tp.moves) { int k = (int)m.kind; hb(&k, 4); hb(&m.to.x, 8); hb(&m.to.y, 8); hb(&m.to.z, 8); hb(&m.centre.x, 8); hb(&m.centre.y, 8); double f = m.feedOverride.value_or(-1); hb(&f, 8); } }
static void hd(const std::vector<ToolpathDiagnostic>& ds) { for (auto& d : ds) hb(d.message.data(), d.message.size()); }
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
struct Lcg { std::uint64_t state; double next() { state = state * 6364136223846793005ULL + 1442695040888963407ULL; return static_cast<double>(state >> 11) * (1.0 / 9007199254740992.0); } double in(double a, double b) { return a + (b - a) * next(); } };
static std::vector<Contour> textPage(std::size_t n) {  // copy of the branch test's generator
  std::vector<Contour> cs; Lcg r{77}; const auto cols = (size_t)std::ceil(std::sqrt((double)n));
  for (std::size_t k = 0; cs.size() < n; ++k) {
    const double s = r.in(2.0, 4.5), x = (double)(k % cols) * 6.0 + r.in(-0.5, 0.5), y = (double)(k / cols) * 6.0 + r.in(-0.5, 0.5);
    const int kind = std::min(5, (int)r.in(0.0, 6.0));
    switch (kind) {
      case 0: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); break;
      case 1: { const double w = s * 0.375; cs.push_back(Contour::fromPolyline(Polyline{{{x, y}, {x + s, y}, {x + s, y + w}, {x + w, y + w}, {x + w, y + s}, {x, y + s}}, true})); break; }
      case 2: cs.push_back(Contour::circle({x + s / 2, y + s / 2}, s / 2)); break;
      case 3: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); if (cs.size() < n) { const double a = 0.3 * s, b = 0.7 * s; cs.push_back(Contour::rectangle(Box2{{x + a, y + a}, {x + b, y + b}})); } break;
      case 4: { std::vector<Vec2> pts; const int m = 2 * (3 + (int)r.in(0.0, 4.0)); for (int i = 0; i < m; ++i) { const double t = 2 * PI * i / m, rr = (i % 2 ? 0.25 : 0.5) * s; pts.push_back({x + s / 2 + rr * std::cos(t), y + s / 2 + rr * std::sin(t)}); } cs.push_back(Contour::fromPolyline(Polyline{pts, true})); break; }
      default: cs.push_back(Contour::fromPolyline(Polyline{{{x, y}, {x + s, y + s}, {x + s, y}}, false})); break;
    }
  }
  return cs;
}
static std::vector<Contour> page1(size_t n) {  // t0123v1 perf.cpp variant 1
  std::vector<Contour> cs; size_t cols = size_t(std::ceil(std::sqrt(double(n)))); double pitch = 6.0;
  for (size_t k = 0; cs.size() < n; ++k) {
    double x = double(k % cols) * pitch + U(-0.5, 0.5), y = double(k / cols) * pitch + U(-0.5, 0.5), s = U(2.0, 4.5);
    switch (int(U(0, 6))) {
      case 0: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); break;
      case 1: cs.push_back(Contour::fromPolyline(Polyline{{{x, y}, {x + s, y}, {x + s, y + s * 0.375}, {x + s * 0.375, y + s * 0.375}, {x + s * 0.375, y + s}, {x, y + s}}, true})); break;
      case 2: cs.push_back(Contour::circle({x + s / 2, y + s / 2}, s / 2)); break;
      case 3: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); if (cs.size() < n) cs.push_back(Contour::rectangle(Box2{{x + 0.3 * s, y + 0.3 * s}, {x + 0.7 * s, y + 0.7 * s}})); break;
      case 4: { std::vector<Vec2> p; int m = 5 + int(U(0, 8)); for (int i = 0; i < m; ++i) { double t = 6.283185307 * i / m, r = (i % 2 ? 0.25 : 0.5) * s; p.push_back({x + s / 2 + r * std::cos(t), y + s / 2 + r * std::sin(t)}); } cs.push_back(Contour::fromPolyline(Polyline{p, true})); break; }
      default: cs.push_back(Contour::fromPolyline(Polyline{{{x, y}, {x + s, y + s}, {x + s, y}}, false})); break;
    }
  }
  return cs;
}
static std::vector<Contour> page2(size_t n) {  // letters, beziers, spikes, arcs-with-holes, open strokes, pitch 7
  std::vector<Contour> cs; size_t cols = size_t(std::ceil(std::sqrt(double(n))));
  for (size_t k = 0; cs.size() < n; ++k) {
    double s = U(2.0, 4.5), x = double(k % cols) * 7 + U(-0.5, 0.5), y = double(k / cols) * 7 + U(-0.5, 0.5);
    switch (I(0, 6)) {
      case 0: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); break;
      case 1: cs.push_back(letter({x, y}, s * 0.7)); break;
      case 2: cs.push_back(Contour::circle({x + s / 2, y + s / 2}, s / 2)); break;
      case 3: cs.push_back(Contour::circle({x + s / 2, y + s / 2}, s / 2)); if (cs.size() < n) cs.push_back(Contour::circle({x + s / 2, y + s / 2}, 0.2 * s)); break;
      case 4: cs.push_back(spikes({x + s / 2, y + s / 2}, s / 2, I(3, 7))); break;
      case 5: cs.push_back(bezBlob({x + s / 2, y + s / 2}, s / 2, I(4, 7))); break;
      default: cs.push_back(anyOpen({x + s / 2, y + s / 2}, s / 2)); break;
    }
  }
  return cs;
}
static Contour combC(int teeth) { std::vector<Vec2> pts; for (int i = 0; i < teeth; ++i) { pts.push_back({i * 1.0, 0}); pts.push_back({i * 1.0, 20}); pts.push_back({i + 0.5, 20}); pts.push_back({i + 0.5, 0.0}); } pts.push_back({double(teeth), -5}); pts.push_back({0, -5}); return Contour::fromPolyline(Polyline{pts, true}); }
static PocketTool ptool(const char* nm, int num, double d, double so, double pd) { PocketTool t; t.tool.id = nm; t.tool.name = nm; t.tool.type = tooling::ToolType::EndMill; t.tool.diameter = d; auto& cd = t.tool.defaultCuttingData; cd.toolNumber = num; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 18000; cd.stepover = so; cd.passDepth = pd; return t; }
static tooling::Tool ptoolP() { tooling::Tool t; t.name = "EM 1"; t.diameter = 1.0; t.defaultCuttingData.passDepth = 3.0; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; return t; }
int main(int argc, char** argv) {
  std::string c = argv[1]; size_t n = strtoul(argv[2], 0, 10); int var = argc > 3 ? atoi(argv[3]) : 0;
  auto pg = [&](size_t m) { return var == 0 ? textPage(m) : var == 1 ? page1(m) : page2(m); };
  double t0 = 0, t1 = 0; size_t moves = 0, diags = 0, retr = 0;
  if (c == "comb") {
    Contour cb = combC((int)n); PocketParams p; p.cutDepth = 9; p.strategy = getenv("OFFSET") ? PocketStrategy::Offset : PocketStrategy::Raster;
    p.rasterAngle = getenv("ANG") ? atof(getenv("ANG")) * PI / 180 : 0.0; if (getenv("PP")) p.profilePass = (ProfilePass)atoi(getenv("PP"));
    double d = getenv("TOOL") ? atof(getenv("TOOL")) : 0.3; std::vector<PocketTool> tools{ptool("B", 1, d, d * 0.4, 9)};
    if (getenv("RD")) { double rd = atof(getenv("RD")); tools.push_back(ptool("S", 2, rd, rd * 0.4, 9)); }
    t0 = cpu(); auto r = pocketToolpath({cb}, tools, p); t1 = cpu(); for (auto& tp : r.toolpaths) { moves += tp.moves.size(); retr += tp.count(MoveKind::Retract); htp(tp); } diags = r.diagnostics.size(); hd(r.diagnostics);
  } else if (c == "sep" || c == "tabs") {
    auto cs = pg(n); ProfileParams p; p.passes.cutDepth = 3; if (c == "sep") { p.separateLastPass = true; p.lastPassAllowance = getenv("LPA") ? atof(getenv("LPA")) : 0.2; }
    if (getenv("NOSEP")) p.separateLastPass = false; if (c == "tabs" || getenv("TABS")) { p.tabs.enabled = true; p.tabs.length = 1; p.tabs.count = 1; p.tabs.thickness = 1; }
    if (getenv("SIDE")) p.side = (ProfileSide)atoi(getenv("SIDE")); if (getenv("REV")) p.lastPassReverse = true; if (getenv("LEADS")) p.leads.type = LeadType::Straight; if (getenv("ORDER")) p.order.leftToRight = true;
    t0 = cpu(); auto r = profileToolpath(cs, ptoolP(), p); t1 = cpu(); moves = r.toolpath.moves.size(); diags = r.diagnostics.size(); retr = r.toolpath.count(MoveKind::Retract); htp(r.toolpath); hd(r.diagnostics);
  } else {  // rest / pockpage
    auto cs = pg(n); PocketParams p; p.cutDepth = 3; if (getenv("RASTER")) { p.strategy = PocketStrategy::Raster; p.rasterAngle = getenv("ANG") ? atof(getenv("ANG")) * PI / 180 : 0.0; }
    double rd = getenv("RD") ? atof(getenv("RD")) : 0.3; std::vector<PocketTool> tools{ptool("EM 1", 1, 1.0, 0.4, 3.0)}; if (c == "rest") tools.push_back(ptool("EM .3", 2, rd, rd * 0.4, 3.0));
    t0 = cpu(); auto r = pocketToolpath(cs, tools, p); t1 = cpu(); for (auto& tp : r.toolpaths) { moves += tp.moves.size(); retr += tp.count(MoveKind::Retract); htp(tp); } diags = r.diagnostics.size(); hd(r.diagnostics);
  }
  std::printf("%s n=%zu var=%d cpu %.3f moves %zu diags %zu retr %zu hash %016llx\n", c.c_str(), n, var, t1 - t0, moves, diags, retr, HH);
}
