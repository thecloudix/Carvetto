// T-0150 verifier: generates drilling programs through every shipped post (+ sidecar params) and
// checks move-level invariants directly.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/drill.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static int fails = 0;
#define CHK(c, ...) do { if (!(c)) { ++fails; if (fails < 40) { std::printf("FAIL %d: ", __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } } while (0)
static const char* kPosts[] = {"grbl_mm", "grbl_inch", "mach3_mm", "mach3_inch", "mach4_mm", "linuxcnc_mm", "linuxcnc_inch", "marlin_mm"};
static std::string slurp(const std::string& p) { std::ifstream in(p, std::ios::binary); std::ostringstream s; s << in.rdbuf(); return s.str(); }
static tooling::Tool tool(bool drill, double d, double pass, double plunge) { tooling::Tool t; t.id="t"; t.name = drill ? "Drill" : "End mill"; t.type = drill ? tooling::ToolType::Drill : tooling::ToolType::EndMill; t.diameter=d; t.includedAngle = drill ? 118*kPi/180 : 0;
  t.defaultCuttingData.toolNumber=3; t.defaultCuttingData.feedRate=700; t.defaultCuttingData.plungeRate=plunge; t.defaultCuttingData.spindleSpeed=12000; t.defaultCuttingData.passDepth=pass; return t; }
struct Case { std::string name; DrillParams p; tooling::Tool t; std::vector<Contour> holes; };
// move-level invariants
static void checkMoves(const Case& c, const DrillResult& r) {
  const auto& m = r.toolpath.moves; const DrillParams& p = c.p; double startZ = -p.startDepth, bottom = -(p.startDepth + p.cutDepth);
  std::size_t i = 0; std::size_t holes = 0; const double eps = 1e-9;
  while (i < m.size()) {
    CHK(m[i].kind == MoveKind::Rapid && std::fabs(m[i].to.z - p.safeZ) < eps, "%s: hole must start with rapid at safe Z (move %zu)", c.name.c_str(), i);
    double x = m[i].to.x, y = m[i].to.y; ++i; ++holes; double z = p.safeZ, deepest = 1e300, lastPlunge = 0; int passes = 0;
    while (i < m.size() && m[i].kind != MoveKind::Rapid) {
      const Move& v = m[i];
      CHK(v.to.x == x && v.to.y == y, "%s: XY changes inside a hole (move %zu)", c.name.c_str(), i);
      if (v.kind == MoveKind::Plunge) { CHK(v.to.z < z - 1e-12, "%s: plunge not downward %g -> %g", c.name.c_str(), z, v.to.z); CHK(!v.feedOverride, "%s: plunge feed override", c.name.c_str());
        if (passes > 0) CHK(v.to.z < lastPlunge, "%s: pass not deeper", c.name.c_str());
        lastPlunge = v.to.z; deepest = std::min(deepest, v.to.z); ++passes; if (p.peck && v.to.z > bottom + eps) CHK(std::fabs((startZ - v.to.z) - passes * c.t.defaultCuttingData.passDepth) < 1e-6, "%s: pass level %g not at pass depth multiple", c.name.c_str(), v.to.z); }
      else if (v.kind == MoveKind::Dwell) { CHK(p.dwellTime > 0 && v.dwellSeconds == p.dwellTime && v.to.z == z && i > 0 && m[i-1].kind == MoveKind::Plunge, "%s: bad dwell", c.name.c_str()); }
      else if (v.kind == MoveKind::Retract) { CHK(v.to.z > z, "%s: retract not upward", c.name.c_str()); CHK(v.to.z <= p.safeZ + eps, "%s: retract above safe Z", c.name.c_str());
        bool last = (i + 1 == m.size() || m[i+1].kind == MoveKind::Rapid);
        if (last) CHK(std::fabs(v.to.z - p.safeZ) < eps, "%s: final retract not at safe Z", c.name.c_str());
        else if (p.peckRetract == PeckRetract::AboveStart) CHK(std::fabs(v.to.z - std::min(startZ + p.retractGap, p.safeZ)) < eps, "%s: AboveStart retract to %g", c.name.c_str(), v.to.z);
        else CHK(std::fabs(v.to.z - std::min(lastPlunge + p.retractGap, p.safeZ)) < eps && v.to.z >= deepest, "%s: AbovePrev retract to %g (last plunge %g)", c.name.c_str(), v.to.z, lastPlunge);
        if (!last) CHK(p.peck, "%s: in-hole retract without peck", c.name.c_str()); }
      else CHK(false, "%s: unexpected move kind %d", c.name.c_str(), int(v.kind));
      z = v.to.z; ++i;
    }
    CHK(std::fabs(deepest - bottom) < 1e-12, "%s: hole bottom %g != %g", c.name.c_str(), deepest, bottom);
    CHK(std::fabs(z - p.safeZ) < eps, "%s: hole does not end at safe Z", c.name.c_str());
    if (!p.peck) CHK(passes == 1, "%s: unpecked hole has %d passes", c.name.c_str(), passes);
    else { int want = int(std::ceil(p.cutDepth / c.t.defaultCuttingData.passDepth - 1e-9)); CHK(passes == want, "%s: %d passes, want %d", c.name.c_str(), passes, want); }
  }
  CHK(holes == r.holes.size(), "%s: hole count", c.name.c_str());
}
int main(int argc, char** argv) {
  std::string out = argv[1]; std::string posts = argv[2]; int nrand = argc > 3 ? std::atoi(argv[3]) : 60;
  std::mt19937 rng(777); std::uniform_real_distribution<double> U(0,1);
  std::vector<Case> cases;
  auto grid = [](int n, double pitch) { std::vector<Contour> v; int s = int(std::ceil(std::sqrt(n))); for (int i=0;i<n;++i) v.push_back(Contour::circle({(i%s)*pitch + 1, (i/s)*pitch + 1}, 1.0)); return v; };
  auto base = [&](std::string name) { Case c; c.name = name; c.t = tool(true, 3, 2.0, 200); c.holes = grid(6, 10); c.p.cutDepth = 7; c.p.safeZ = 5; return c; };
  for (int mode = 0; mode < 2; ++mode) for (int dw = 0; dw < 2; ++dw) {
    std::string sfx = std::string(mode ? "_prev" : "_start") + (dw ? "_dwell" : "");
    auto set = [&](Case c) { c.p.peck = true; c.p.peckRetract = mode ? PeckRetract::AbovePreviousPass : PeckRetract::AboveStart; c.p.dwellTime = dw ? 0.75 : 0; c.name += sfx; cases.push_back(c); };
    set(base("basic"));
    { Case c = base("equal_pass"); c.p.cutDepth = 2.0; set(c); }
    { Case c = base("tiny_rem"); c.p.cutDepth = 6.000001; set(c); }
    { Case c = base("startdepth"); c.p.startDepth = 3; c.p.retractGap = 0.5; set(c); }
    { Case c = base("safez_close"); c.p.safeZ = 0.2; c.p.retractGap = 1.0; set(c); }
    { Case c = base("gap_gt_pass"); c.p.retractGap = 3.5; set(c); }
    { Case c = base("endmill"); c.t = tool(false, 6, 1.5, 300); set(c); }
    { Case c = base("nopeck"); c.p.peck = false; set(c); cases.back().p.peck = false; }
  }
  { Case c = base("many20k_prev_dwell"); c.holes = grid(20000, 4); c.p.peck = true; c.p.peckRetract = PeckRetract::AbovePreviousPass; c.p.dwellTime = 0.5; c.p.cutDepth = 5; c.p.order.grid = true; cases.push_back(c); }
  { Case c = base("many20k_nopeck"); c.holes = grid(20000, 4); c.p.cutDepth = 5; cases.push_back(c); }
  for (int k = 0; k < nrand; ++k) { Case c = base("rand" + std::to_string(k)); c.t = tool(U(rng) < 0.5, 0.5 + U(rng)*10, 0.2 + U(rng)*5, 20 + U(rng)*1000);
    c.p.startDepth = U(rng) < 0.3 ? U(rng)*5 : 0; c.p.cutDepth = 0.1 + U(rng)*25; c.p.safeZ = -c.p.startDepth + 0.05 + U(rng)*20; if (c.p.safeZ < 0.05) c.p.safeZ = 0.05 + U(rng);
    c.p.peck = U(rng) < 0.8; c.p.peckRetract = U(rng) < 0.5 ? PeckRetract::AboveStart : PeckRetract::AbovePreviousPass; c.p.retractGap = 0.01 + U(rng)*5; c.p.dwellTime = U(rng) < 0.5 ? 0 : std::round((0.01 + U(rng)*3)*100)/100;
    int n = 1 + int(U(rng)*30); c.holes.clear(); for (int i=0;i<n;++i) c.holes.push_back(Contour::circle({(U(rng)-0.5)*600, (U(rng)-0.5)*400}, 0.5 + U(rng)*5));
    int o = int(U(rng)*5); c.p.order.selectionOrder = o==1; c.p.order.leftToRight = o==2; c.p.order.bottomToTop = o==3; c.p.order.grid = o==4; cases.push_back(c); }
  std::ofstream idx(out + "/index.txt");
  for (auto& c : cases) {
    timespec a, b; clock_gettime(CLOCK_MONOTONIC, &a);
    auto r = drillToolpath(c.holes, c.t, std::nullopt, c.p);
    clock_gettime(CLOCK_MONOTONIC, &b); double secs = double(b.tv_sec - a.tv_sec) + 1e-9 * double(b.tv_nsec - a.tv_nsec);
    if (!r.ok()) { std::printf("case %s: no toolpath: %s\n", c.name.c_str(), r.diagnostics.empty() ? "?" : r.diagnostics[0].message.c_str()); ++fails; continue; }
    if (c.holes.size() >= 20000) std::printf("case %s: %zu holes, %zu moves, %.3f s\n", c.name.c_str(), r.holes.size(), r.toolpath.moves.size(), secs);
    checkMoves(c, r);
    for (const char* post : kPosts) {
      auto pp = post::PostProcessor::parse(slurp(posts + "/" + post + ".pp"));
      post::PostContext ctx; ctx.fileName = c.name; ctx.home = {0, 0, 20}; ctx.safeZ = c.p.safeZ;
      auto res = pp.value().run({r.toolpath}, ctx);
      if (!res.ok()) { std::printf("case %s post %s: %s\n", c.name.c_str(), post, res.error().describe().c_str()); ++fails; continue; }
      std::string fn = out + "/" + c.name + "." + post + ".nc"; std::ofstream(fn, std::ios::binary) << res.value().text;
      idx << fn << " " << post << " safe=" << c.p.safeZ << " start=" << -c.p.startDepth << " bottom=" << -(c.p.startDepth + c.p.cutDepth) << " gap=" << c.p.retractGap
          << " mode=" << (c.p.peck ? (c.p.peckRetract == PeckRetract::AboveStart ? "start" : "prev") : "none") << " dwell=" << c.p.dwellTime << " plunge=" << c.t.defaultCuttingData.plungeRate
          << " pass=" << c.t.defaultCuttingData.passDepth << " holes=" << r.holes.size() << "\n";
    }
  }
  // dwell too small for a post's precision; must be refused, not printed as P0
  { Case c = base("tinydwell"); c.p.dwellTime = 0.001; auto r = drillToolpath(c.holes, c.t, std::nullopt, c.p);
    for (const char* post : kPosts) { auto pp = post::PostProcessor::parse(slurp(posts + "/" + post + ".pp")); post::PostContext ctx; ctx.safeZ = 5; auto res = pp.value().run({r.toolpath}, ctx);
      std::printf("tiny dwell 0.001 s via %s: %s\n", post, res.ok() ? (res.value().text.find("P0\n") != std::string::npos || res.value().text.find("P0.00") != std::string::npos ? "PRINTED ZERO" : "ok") : res.error().describe().c_str()); } }
  std::printf("cases=%zu fails=%d\n", cases.size(), fails); return fails ? 1 : 0;
}
