// one4.cpp: run one scene once (env S = scale, OX/OY = translation, VSOUT, FLIP) with stderr passed through (TFS_LOG/TFS_DETAIL
// from the logd library); prints the first rapid XY mapped back.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
#include "scene_io.hpp"
int main(int argc, char** argv) {
  std::vector<Contour> cs; ProfileParams p; double D = 6; int vd = 0; if (!readScene(argv[1], cs, p, D, vd)) return 1;
  double S = getenv("S") ? strtod(getenv("S"), nullptr) : 1.0; Vec2 o{getenv("OX") ? atof(getenv("OX")) : 0.0, getenv("OY") ? atof(getenv("OY")) : 0.0};
  if (getenv("UP")) { S = 1; for (int k = 0; k < atoi(getenv("UP")); ++k) S = std::nextafter(S, 2.0); }
  if (getenv("DN")) { S = 1; for (int k = 0; k < atoi(getenv("DN")); ++k) S = std::nextafter(S, 0.0); }
  for (auto& c : cs) for (auto& sp : c.spans) {
    if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start * S + o; l->end = l->end * S + o; }
    else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = a->centre * S + o; a->radius *= S; }
    else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = b->p0 * S + o; b->p1 = b->p1 * S + o; b->p2 = b->p2 * S + o; b->p3 = b->p3 * S + o; } }
  p.startPosition = p.startPosition * S + o;
  if (getenv("FLIP")) p.direction = p.direction == CutDirection::Climb ? CutDirection::Conventional : CutDirection::Climb;
  if (getenv("VSOUT")) { p.startAt = StartAt::VectorStart; p.side = ProfileSide::Outside; }
  tooling::Tool t; if (vd) { t.type = tooling::ToolType::VBit; t.includedAngle = vd * PI / 180; } else t.type = tooling::ToolType::EndMill; t.diameter = D;
  auto r = profileToolpath(cs, t, p); fflush(stderr);
  if (!r.toolpath.moves.empty()) { Vec2 q = (r.toolpath.moves.front().to.xy() - o) * (1.0 / S); std::printf("start %.12f %.12f moves %zu\n", q.x, q.y, r.toolpath.moves.size()); }
}
