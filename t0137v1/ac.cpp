// T-0137 verifier: AC1 boundary probes for the On +r limit (end mill, V-bit at depth, start depth,
// tiny/denormal radii, roughing sums, -r interplay, Left/Right/Inside/Outside unaffected).
// usage: ac   (prints one line per probe; "FAIL" lines are unexpected)
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static const double INF = std::numeric_limits<double>::infinity();
static int fails = 0;
static tooling::Tool em(double d) { tooling::Tool t; t.id = "e"; t.name = "e"; t.type = tooling::ToolType::EndMill; t.diameter = d;
  auto& cd = t.defaultCuttingData; cd.toolNumber = 1; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 18000; cd.stepover = d * 0.4; cd.passDepth = 1; return t; }
static tooling::Tool vb(double d, double deg) { auto t = em(d); t.type = tooling::ToolType::VBit; t.includedAngle = deg * M_PI / 180; return t; }
static std::vector<Contour> scene(bool open, double s = 1) {
  if (open) { Contour c; c.append(geometry::LineSpan{{0, 0}, {30 * s, 0}}); c.append(geometry::LineSpan{{30 * s, 0}, {30 * s, 20 * s}}); return {c}; }
  return {Contour::rectangle(geometry::Box2{{0, 0}, {40 * s, 30 * s}}), Contour::circle({80 * s, 15 * s}, 12 * s)};
}
// expect: 0 valid (moves > 0, no InvalidParameters), 1 refused above, 2 refused below (-r message)
static void probe(const char* name, const std::vector<Contour>& cs, const tooling::Tool& t, ProfileParams p, int expect) {
  auto res = profileToolpath(cs, t, p); double r = profileToolRadius(t, p.passes);
  bool inv = res.has(ProfileIssue::InvalidParameters); std::string m = res.diagnostics.empty() ? "" : res.diagnostics[0].message;
  int got = !inv ? 0 : m.find("above the tool radius") != std::string::npos ? 1 : m.find("below minus") != std::string::npos ? 2 : 3;
  bool ok = got == expect && (expect != 0 || !res.toolpath.moves.empty()) && (expect == 0 || (res.toolpath.moves.empty() && res.diagnostics.size() == 1));
  if (!ok) ++fails;
  std::printf("%s %-44s r=%a a=%a lpa=%a sep=%d side=%d exp=%d got=%d moves=%zu ndiag=%zu | %s\n", ok ? "ok  " : "FAIL", name, r, p.allowance, p.lastPassAllowance,
              (int)p.separateLastPass, (int)p.side, expect, got, res.toolpath.moves.size(), res.diagnostics.size(), m.c_str());
}
int main() {
  ProfileParams b; b.passes.cutDepth = 3; b.passes.passDepth = 1.5; b.safeZ = 5; b.lastPassAllowance = 0.2;
  struct T { const char* n; tooling::Tool t; double start; double cut; } tools[] = {
    {"em6", em(6), 0, 3}, {"em3.175", em(3.175), 0, 3}, {"em0.1", em(0.1), 0, 0.5}, {"em1e-3", em(1e-3), 0, 0.5},
    {"vb60 d3", vb(12, 60), 0, 3}, {"vb90 start1.3 d0.7", vb(12, 90), 1.3, 0.7}, {"vb120 d0.01", vb(12, 120), 0, 0.01}, {"vb30 d1e-3", vb(6, 30), 0, 1e-3},
    {"vb90 full width (d>height)", vb(6, 90), 0, 10}};
  for (auto& T : tools) for (int open = 0; open < 2; ++open) {
    ProfileParams p = b; p.passes.startDepth = T.start; p.passes.cutDepth = T.cut; p.passes.passDepth = T.cut; p.side = ProfileSide::On;
    double r = profileToolRadius(T.t, p.passes); double s = r < 0.1 ? 0.05 : 1;  // small scene for tiny tools
    auto cs = scene(open, s);
    std::string n = std::string(T.n) + (open ? " open" : " closed");
    auto P = [&](const char* tag, double a, bool sep, double lpa, int e, ProfileSide side = ProfileSide::On) { ProfileParams q = p; q.allowance = a; q.separateLastPass = sep; q.lastPassAllowance = lpa; q.side = side;
      probe((n + " " + tag).c_str(), cs, T.t, q, e); };
    P("+r", r, false, 0.2, 0);
    P("+r-ulp", std::nextafter(r, -INF), false, 0.2, 0);
    P("+r+ulp", std::nextafter(r, INF), false, 0.2, 1);
    P("+r+1e-9", r + 1e-9, false, 0.2, 1);
    P("+r+1", r + 1, false, 0.2, 1);
    P("-r", -r, false, 0.2, 0);
    P("-r-ulp", std::nextafter(-r, -INF), false, 0.2, 2);
    P("nosep +r big lpa ignored", r, false, 100, 0);
    P("sep +r lpa0", r, true, 0.0, 0);
    P("sep +r lpa denorm", r, true, 4.9e-324, 0);
    P("sep +r lpa ulp(r)", r, true, std::nextafter(r, INF) - r, 1);
    P("sep r-lpa lpa", r - 0.2 * r, true, 0.2 * r, (r - 0.2 * r) + 0.2 * r > r ? 1 : 0);
    P("sep -r lpa 2r", -r, true, 2 * r, 0);
    P("sep -r lpa next(2r)", -r, true, std::nextafter(2 * r, INF), (-r + std::nextafter(2 * r, INF)) > r ? 1 : 0);
    P("sep +r+ulp lpa0", std::nextafter(r, INF), true, 0, 1);
    P("sep below -r and rough above", -r - 1, true, 2 * r + 3, 2);
    P("Outside a=r+1", r + 1, false, 0.2, 0, ProfileSide::Outside);
    P("Outside a=100r sep", 100 * r, true, 0.5, 0, ProfileSide::Outside);
    if (open) { P("Left(Inside) a=r+1", r + 1, false, 0.2, 0, ProfileSide::Inside); P("Left(Inside) a=5r sep", 5 * r, true, 0.3, 0, ProfileSide::Inside); }
  }
  // exactly +r On a straight line: every cutting move sits at |y| == r (tool edge on the vector)
  { Contour c; c.append(geometry::LineSpan{{0, 0}, {30, 0}}); ProfileParams p = b; p.side = ProfileSide::On; auto t = em(6); double r = 3; p.allowance = r;
    auto res = profileToolpath({c}, t, p); double mx = 0, mn = 1e9; for (auto& m : res.toolpath.moves) if (m.to.z < 0) { mx = std::max(mx, std::fabs(m.to.y)); mn = std::min(mn, std::fabs(m.to.y)); }
    bool ok = !res.toolpath.moves.empty() && mx == r && mn == r; if (!ok) ++fails; std::printf("%s line On a=+r: |y| of cutting moves in [%a, %a]\n", ok ? "ok  " : "FAIL", mn, mx); }
  // open Left/Right with large allowance stays on its side: Outside(Right) y<0 for a line +x, Inside(Left) y>0
  for (int sd = 0; sd < 2; ++sd) { Contour c; c.append(geometry::LineSpan{{0, 0}, {30, 0}}); ProfileParams p = b; p.side = sd ? ProfileSide::Inside : ProfileSide::Outside; p.allowance = 10;
    auto res = profileToolpath({c}, em(6), p); double mn = 1e9, mx = -1e9; for (auto& m : res.toolpath.moves) if (m.to.z < 0) { mn = std::min(mn, m.to.y); mx = std::max(mx, m.to.y); }
    std::printf("info open line side %d a=10: cutting y in [%g, %g] (expect |y| = 13 on one side)\n", (int)p.side, mn, mx); }
  // radius 0 / denormal
  { ProfileParams p = b; p.side = ProfileSide::On; p.allowance = 0; auto res = profileToolpath(scene(false), em(0), p);
    std::printf("info em(0): ndiag %zu first '%s'\n", res.diagnostics.size(), res.diagnostics.empty() ? "" : res.diagnostics[0].message.c_str()); }
  { auto t = em(1e-310); ProfileParams p = b; p.side = ProfileSide::On; double r = profileToolRadius(t, p.passes);
    p.allowance = std::nextafter(r, INF); auto res = profileToolpath(scene(false, 1e-3), t, p);
    std::printf("info denormal r=%a a=r+ulp: moves %zu first '%s'\n", r, res.toolpath.moves.size(), res.diagnostics.empty() ? "" : res.diagnostics[0].message.c_str());
    p.allowance = r; res = profileToolpath(scene(false, 1e-3), t, p);
    std::printf("info denormal r=%a a=r: moves %zu first '%s'\n", r, res.toolpath.moves.size(), res.diagnostics.empty() ? "" : res.diagnostics[0].message.c_str()); }
  std::printf("FAILS %d\n", fails);
}
