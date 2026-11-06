// t0169v1 vcx: V-Carve flat-area moves vs the original walls with oc2 (own exact oracle).
// usage: vcx <file> <chordal> <distance> <flat> <rasterAngle|-1 offset> [endmill D] ; V-bit D6 60 deg, stepover 0.1732
// env VCX_FAR translation. Prints min(clearance) of carving-tool moves at the flat depth (radius = footprint)
// and of every clearance-tool move below the surface (radius D/2).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "oc2.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static std::vector<Contour> loadFile(const char* path) { std::vector<Contour> cs; std::ifstream in(path); std::string line;
  while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t;
    if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
    else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(geometry::LineSpan{{a, b}, {c, d}}); }
    else if (t == "A") { double x, y, r, s0, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s0 >> e >> dd >> dir; cs.back().append(geometry::ArcSpan{{x, y}, r, s0, e, dir ? geometry::ArcDirection::CW : geometry::ArcDirection::CCW}); }
    else if (t == "B") { double v[8]; for (double& q : v) is >> q; cs.back().append(geometry::BezierSpan{{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}}); } }
  return cs; }
int main(int argc, char** argv) {
  auto cs = loadFile(argv[1]); double far = getenv("VCX_FAR") ? atof(getenv("VCX_FAR")) : 0;
  if (far != 0) for (auto& c : cs) c = c.transformed(geometry::Transform2::translation({far, -0.7 * far}));
  tooling::Tool v; v.id = "V"; v.name = "V"; v.type = tooling::ToolType::VBit; v.diameter = 6; v.includedAngle = M_PI / 3; auto& cd = v.defaultCuttingData; cd.toolNumber = 1; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 18000; cd.passDepth = 1.1244448085417738; cd.stepover = 0.1732050807568877;
  VCarvingParams p; p.safeZ = 5.2367691924120914; p.tolerance.chordal = atof(argv[2]); p.tolerance.distance = atof(argv[3]); p.flatDepth = atof(argv[4]); double ra = atof(argv[5]);
  if (ra >= 0) { p.clearanceStrategy = PocketStrategy::Raster; p.clearanceRasterAngle = ra; }
  std::vector<PocketTool> cl; if (argc > 6) { PocketTool t; t.tool.id = "E"; t.tool.name = "E"; t.tool.type = tooling::ToolType::EndMill; t.tool.diameter = atof(argv[6]); auto& c2 = t.tool.defaultCuttingData; c2.toolNumber = 2; c2.feedRate = 1000; c2.plungeRate = 300; c2.spindleSpeed = 18000; c2.stepover = 0.4 * t.tool.diameter; c2.passDepth = 3; t.tool.defaultCuttingData = c2; cl.push_back(t); }
  auto r = vCarvingToolpath(cs, v, std::nullopt, cl, p);
  double rad = p.flatDepth * std::tan(M_PI / 6); Toolpath flat; size_t nf = 0;
  for (size_t i = 0; i < r.toolpath.moves.size(); ++i) { auto& m = r.toolpath.moves[i]; auto s = r.toolpath.startOf(i);
    if (m.kind == MoveKind::Rapid || std::fabs(s.z + p.flatDepth) > 1e-9 || std::fabs(m.to.z + p.flatDepth) > 1e-9) continue;
    flat.moves.push_back(Move::rapid({s.x, s.y, 5})); flat.moves.push_back(Move::feed({s.x, s.y, s.z})); flat.moves.push_back(m); ++nf; }
  auto rep = oc2::check(flat, cs, rad, -1e-5, 0.0, 0.05);
  std::printf("flat moves %zu min clearance %.4e cert(-1e-5) %d worst (%.9f,%.9f)->(%.9f,%.9f)\n", nf, rep.minc, (int)rep.cert, rep.wa.x, rep.wa.y, rep.wb.x, rep.wb.y);
  for (auto& tp : r.clearance) { if (tp.isEmpty()) continue; auto e = oc2::check(tp, cs, 0.5 * cl[0].tool.diameter, -1e-5, 0.0, 0.05); std::printf("clearance tool moves %zu min clearance %.4e cert %d\n", e.checked, e.minc, (int)e.cert); }
}
