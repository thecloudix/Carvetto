// mvnear.cpp: moves of the cut whose rapid lands nearest the first contour's start vertex T (S/UP/DN/OX env as one4): prints
// the start, its distance to T, and the cut's first K and last K feed points (mapped back) with distance to T.
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
  Vec2 T = geometry::startPoint(cs[0].spans.front());
  for (auto& c : cs) for (auto& sp : c.spans) {
    if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start * S + o; l->end = l->end * S + o; }
    else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = a->centre * S + o; a->radius *= S; }
    else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = b->p0 * S + o; b->p1 = b->p1 * S + o; b->p2 = b->p2 * S + o; b->p3 = b->p3 * S + o; } }
  p.startPosition = p.startPosition * S + o; p.tabs.enabled = false; p.ramp.type = RampType::None; p.leads.type = LeadType::None; p.separateLastPass = false; p.passes.passDepth = p.passes.cutDepth;
  tooling::Tool t; if (vd) { t.type = tooling::ToolType::VBit; t.includedAngle = vd * PI / 180; } else t.type = tooling::ToolType::EndMill; t.diameter = D;
  auto r = profileToolpath(cs, t, p); const auto& mv = r.toolpath.moves; size_t best = 0; double bd = 1e300;
  auto back = [&](Vec2 q) { return (q - o) * (1.0 / S); };
  for (size_t i = 0; i < mv.size(); ++i) if (mv[i].kind == MoveKind::Rapid) { double dd = geometry::distance(back(mv[i].to.xy()), T); if (dd < bd) { bd = dd; best = i; } }
  std::vector<Vec2> pts; for (size_t i = best + 1; i < mv.size() && mv[i].kind != MoveKind::Rapid; ++i) if (mv[i].kind == MoveKind::Feed || isArc(mv[i].kind)) pts.push_back(back(mv[i].to.xy()));
  Vec2 s0 = back(mv[best].to.xy()); int K = getenv("K") ? atoi(getenv("K")) : 4;
  std::printf("start %.9f %.9f |S-T| %.9f npts %zu\n", s0.x, s0.y, bd, pts.size());
  for (int k = 0; k < K && k < (int)pts.size(); ++k) std::printf("  +%d %.9f %.9f |q-T| %.9f\n", k, pts[k].x, pts[k].y, geometry::distance(pts[k], T));
  for (int k = K; k >= 1; --k) if ((int)pts.size() - k - 1 >= 0) { Vec2 q = pts[pts.size() - k - 1]; std::printf("  -%d %.9f %.9f |q-T| %.9f\n", k, q.x, q.y, geometry::distance(q, T)); }
}
