#include <cstdio>
#include <cmath>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static tooling::Tool em(const char* name, double d) { tooling::Tool t; t.id = "e"; t.name = name; t.type = tooling::ToolType::EndMill; t.diameter = d;
  auto& c = t.defaultCuttingData; c.toolNumber = 1; c.feedRate = 1000; c.plungeRate = 300; c.spindleSpeed = 10000; c.passDepth = 1; c.stepover = 0.4 * d; return t; }
static void pr(const char* what, const EngraveResult& r) { std::printf("%s: ok %d moves %zu\n", what, (int)r.ok(), r.toolpath.moves.size()); for (auto& d : r.diagnostics) std::printf("   [%d] c=%d %s\n", (int)d.issue, d.contour ? (int)*d.contour : -1, d.message.c_str()); }
int main() {
  std::vector<Contour> small{Contour::rectangle(geometry::Box2{Vec2{0,0}, Vec2{2,2}})};
  EngraveParams f; f.depth = 0.2; f.strategy = EngraveStrategy::Fill; f.safeZ = 5;
  pr("pocket-named tool too large", engraveToolpath(small, em("my pocket cutter", 3), std::nullopt, f));
  pr("tool named (1)", engraveToolpath(small, em("tool 1 (x)", 3), std::nullopt, f));
  pr("fill only empty contours", engraveToolpath({Contour{}}, em("a", 1), std::nullopt, f));
  pr("fill no contours", engraveToolpath({}, em("a", 1), std::nullopt, f));
  EngraveParams fs = f; fs.stepover = 0.005; 
  pr("fill tiny stepover", engraveToolpath(small, em("a", 0.5), std::nullopt, fs));
  EngraveParams o; o.depth = 1e-6; o.passes = 1000; o.safeZ = 5;
  auto r = engraveToolpath({Contour::circle({0,0}, 1)}, em("a", 1), std::nullopt, o); pr("tiny depth 1000 passes", r);
  double mn = 0; for (auto& m : r.toolpath.moves) mn = std::min(mn, m.to.z); std::printf("   min z %.17g count plunge %zu\n", mn, r.toolpath.count(MoveKind::Plunge));
  EngraveParams big; big.depth = 0.3; big.passes = 3; big.safeZ = 5;
  auto rb = engraveToolpath({Contour::circle({1e6, -1e6}, 0.5)}, em("a", 1), std::nullopt, big); pr("huge coords circle", rb);
  for (auto& m : rb.toolpath.moves) std::printf("   k%d (%.6f %.6f %.4f) c(%.3f %.3f)\n", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y);
  EngraveParams bad = big; bad.passes = 1000; 
  std::vector<Contour> many; for (int i = 0; i < 2000; ++i) { Contour c; for (int k = 0; k < 20; ++k) c.append(geometry::LineSpan{{double(k), double(i)}, {double(k+1), double(i)}}); many.push_back(c); }
  pr("work limit", engraveToolpath(many, em("a", 1), std::nullopt, bad));
  EngraveParams sz = big; sz.safeZ = 1e-9; pr("tiny safe z", engraveToolpath({Contour::circle({0,0}, 1)}, em("a", 1), std::nullopt, sz));
  tooling::Tool zf = em("zf", 1); zf.defaultCuttingData.feedRate = 0; pr("zero feed", engraveToolpath({Contour::circle({0,0}, 1)}, zf, std::nullopt, big));
  tooling::Tool v; v.id="v"; v.name="V"; v.type = tooling::ToolType::VBit; v.diameter = 6; v.includedAngle = M_PI/2; v.defaultCuttingData = zf.defaultCuttingData; v.defaultCuttingData.feedRate = 1000;
  EngraveParams deep = f; deep.depth = 10; pr("vbit fill deeper than cone", engraveToolpath({Contour::rectangle(geometry::Box2{Vec2{0,0}, Vec2{30,30}})}, v, std::nullopt, deep));
  EngraveParams deepo = big; deepo.depth = 10; pr("vbit outline deeper than cone", engraveToolpath({Contour::circle({0,0}, 5)}, v, std::nullopt, deepo));
}
