#include <cstdio>
#include <cmath>
#include <string>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static tooling::Tool em(std::string name, double d) { tooling::Tool t; t.id = "e"; t.name = name; t.type = tooling::ToolType::EndMill; t.diameter = d;
  auto& c = t.defaultCuttingData; c.toolNumber = 1; c.feedRate = 1000; c.plungeRate = 300; c.spindleSpeed = 10000; c.passDepth = 1; c.stepover = 0.4 * d; return t; }
static long bad = 0, n = 0;
static void pr(const char* what, const std::string& name, const EngraveResult& r) {
  std::printf("%s [%s]: ok %d moves %zu\n", what, name.c_str(), (int)r.ok(), r.toolpath.moves.size());
  for (auto& d : r.diagnostics) {
    ++n; std::printf("   [%d] %s\n", (int)d.issue, d.message.c_str());
    std::string m = d.message, pre = "tool '" + name + "'";
    std::string rest = m;
    if (m.rfind("tool ", 0) == 0) { if (m.rfind(pre, 0) != 0) { ++bad; std::printf("   !! name not intact\n"); continue; } rest = m.substr(pre.size()); }
    if (rest.find("pocket") != std::string::npos || rest.find("Pocket") != std::string::npos) { ++bad; std::printf("   !! stray pocket\n"); }
    if (rest.find("tool 1") != std::string::npos) { ++bad; std::printf("   !! stray tool 1\n"); }
  }
}
int main() {
  std::vector<Contour> small{Contour::rectangle(geometry::Box2{Vec2{0,0}, Vec2{2,2}})};
  std::vector<Contour> big{Contour::rectangle(geometry::Box2{Vec2{0,0}, Vec2{400,400}})};
  std::vector<Contour> mixed = small; { Contour o; o.append(geometry::LineSpan{{5,5},{9,9}}); mixed.push_back(o); mixed.push_back(o); }
  std::vector<Contour> detailed; for (int i = 0; i < 300; ++i) { Contour c; int k = 1000; for (int j = 0; j < k; ++j) { double a0 = 2*M_PI*j/k, a1 = 2*M_PI*(j+1)/k; c.append(geometry::LineSpan{{i*10+3*std::cos(a0), 3*std::sin(a0)}, {i*10+3*std::cos(a1), 3*std::sin(a1)}}); } c.closed = true; detailed.push_back(c); }
  EngraveParams f; f.depth = 0.2; f.strategy = EngraveStrategy::Fill; f.safeZ = 5;
  const char* names[] = {"my pocket cutter", "tool 1 (x)", "tool 1 (", "pocket", "the pocket", "\"pocket\" 'q' (a)(b)", "Fräser Tasche – pocket ü 日本", "pocket boundary too large for the pocket", "", "at the pocket depth", "tool 1 (pocket)"};
  for (std::string nm : names) {
    pr("too large", nm, engraveToolpath(small, em(nm, 3), std::nullopt, f));
    EngraveParams fs = f; fs.stepover = 0.002; fs.tolerance.distance = 1e-4; fs.tolerance.chordal = 1e-4; pr("tiny stepover (work limit, pattern)", nm, engraveToolpath(big, em(nm, 0.5), std::nullopt, fs));
    EngraveParams fr = fs; fr.fillPattern = PocketStrategy::Raster; pr("tiny stepover raster", nm, engraveToolpath(big, em(nm, 0.5), std::nullopt, fr));
    pr("open vectors + too large", nm, engraveToolpath(mixed, em(nm, 3), std::nullopt, f));
    pr("open vectors ok", nm, engraveToolpath(mixed, em(nm, 0.5), std::nullopt, f));
    EngraveParams ft = f; ft.stepover = 0.009; ft.tolerance.distance = 0.001; ft.tolerance.chordal = 0.01; pr("stepover < 10x tol", nm, engraveToolpath(small, em(nm, 0.5), std::nullopt, ft));
    pr("detailed boundary", nm, engraveToolpath(detailed, em(nm, 1), std::nullopt, f));
    EngraveParams fw = f; fw.stepover = 1.5; pr("stepover > width", nm, engraveToolpath(small, em(nm, 1), std::nullopt, fw));
    EngraveParams fz = f; fz.passes = 1000; fz.depth = 1e-4; pr("floor", nm, engraveToolpath(small, em(nm, 1), std::nullopt, fz));
    { EngraveParams fm = f; fm.passes = 1000; fm.depth = 1.0; fm.stepover = 0.05; fm.tolerance.distance=1e-3; fm.tolerance.chordal=1e-3; std::vector<Contour> mid{Contour::rectangle(geometry::Box2{Vec2{0,0}, Vec2{60,60}})}; pr("moves cap", nm, engraveToolpath(mid, em(nm, 0.5), std::nullopt, fm)); }
    pr("only open", nm, engraveToolpath({mixed[1]}, em(nm, 1), std::nullopt, f));
  }
  std::printf("diags %ld bad %ld\n", n, bad);
}
