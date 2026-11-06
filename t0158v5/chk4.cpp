// chk4.cpp (T-0158 round 4): classify where a cut starts relative to its vector start T (first contour), for scene files.
// usage: chk4 file... [env VSOUT=1: force VectorStart/Outside]. Per file and cut direction (as stored, and flipped with FLIP=1):
//   S = first rapid XY, N = first XY-moving move after S, P = point before the last feed that closes at S.
//   cls: foot (|S-T| = d), chamferFirst (line S->N at distance d from T, S and N farther than d), chamferLast (line P->S at
//   distance d, S farther), mitre (|S-M| < 1e-7 with M the mitre of T's two tangents), other.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
#include "scene_io.hpp"
static tooling::Tool vbit(double D, int deg) { tooling::Tool t; t.type = tooling::ToolType::VBit; t.diameter = D; t.includedAngle = deg * PI / 180; return t; }
static tooling::Tool endMill(double D) { tooling::Tool t; t.type = tooling::ToolType::EndMill; t.diameter = D; return t; }
static double lineDist(Vec2 a, Vec2 b, Vec2 t) { Vec2 d = b - a; double L = d.length(); return L > 0 ? std::fabs((t - a).cross(d)) / L : geometry::distance(a, t); }
int main(int argc, char** argv) {
  for (int k = 1; k < argc; ++k) {
    std::vector<Contour> cs; ProfileParams p; double D = 6; int vd = 0;
    if (!readScene(argv[k], cs, p, D, vd)) continue;
    if (getenv("VSOUT")) { p.startAt = StartAt::VectorStart; p.side = ProfileSide::Outside; }
    tooling::Tool tool = vd ? vbit(D, vd) : endMill(D);
    // SIMPLE (default on): one pass, no tabs, ramps, leads or separate last pass, so the moves are rapid, plunge, the loop.
    if (!getenv("NOSIMPLE")) { p.tabs.enabled = false; p.ramp.type = RampType::None; p.leads.type = LeadType::None; p.separateLastPass = false; p.passes.passDepth = p.passes.cutDepth; }
    for (int flip = 0; flip < (getenv("FLIP") ? 2 : 1); ++flip) {
      ProfileParams q = p; if (flip) q.direction = q.direction == CutDirection::Climb ? CutDirection::Conventional : CutDirection::Climb;
      auto r = profileToolpath(cs, tool, q);
      if (r.toolpath.moves.empty() || cs.empty()) { std::printf("%s flip %d EMPTY\n", argv[k], flip); continue; }
      const double d = profileToolRadius(tool, q.passes) + q.allowance;
      const Vec2 T = geometry::startPoint(cs[0].spans.front());
      const auto& mv = r.toolpath.moves; Vec2 S = mv.front().to.xy();
      // the closed cut's XY moves from S around back to S (first cut only): carriers' distances from T
      std::vector<double> car; std::vector<Vec2> pts{S}; size_t i = 1;
      while (i < mv.size() && !(mv[i].kind == MoveKind::Feed || isArc(mv[i].kind))) ++i;
      Vec2 prev = S;
      for (; i < mv.size() && mv[i].kind != MoveKind::Rapid; ++i) {
        Vec2 q = mv[i].to.xy(); if (geometry::distance(q, prev) < 1e-9) continue;
        double cd = isArc(mv[i].kind) ? std::fabs(geometry::distance(T, mv[i].centre) - geometry::distance(q, mv[i].centre)) : lineDist(prev, q, T);
        car.push_back(cd); pts.push_back(q); prev = q;
        if (geometry::distance(q, S) < 1e-9) break;
      }
      const double dS = geometry::distance(S, T);
      std::string cls = "other";
      auto at = [&](double c) { return std::fabs(c - d) < 1e-6; };
      size_t m = car.size();
      if (std::fabs(dS - d) < 1e-6) cls = "foot";
      else if (m >= 4 && geometry::distance(pts.back(), S) < 1e-9) {
        bool in = at(car[m - 1]), in2 = at(car[m - 2]), out = at(car[0]), out2 = at(car[1]);
        if (in && out && out2 && !in2) cls = "chamferFirst";
        else if (in && out && in2 && !out2) cls = "chamferLast";
        else if (in && out && !in2 && !out2) cls = "mitre";
        else if (in && out && in2 && out2) cls = "run4";
        else cls = "beyond";
      }
      // mitre of T when both edges at T are lines (orientation from the signed area)
      double dM = -1;
      { const auto& sp = cs[0].spans; auto* l0 = std::get_if<geometry::LineSpan>(&sp.front()); auto* l1 = std::get_if<geometry::LineSpan>(&sp.back());
        if (l0 && l1) { double ar = 0; for (auto& z : sp) { Vec2 a = geometry::startPoint(z), b = geometry::endPoint(z); ar += a.cross(b); }
          auto unit = [](Vec2 w) { return w * (1.0 / w.length()); }; Vec2 ui = unit(l1->end - l1->start), uo = unit(l0->end - l0->start);
          double sg = ar > 0 ? 1 : -1; Vec2 ni{ui.y * sg, -ui.x * sg}, no{uo.y * sg, -uo.x * sg}; Vec2 p0 = T + ni * d, q0 = T + no * d;
          double den = ui.cross(uo); if (std::fabs(den) > 1e-15) { Vec2 M = p0 + ui * ((q0 - p0).cross(uo) / den); dM = geometry::distance(S, M); } } }
      std::printf("%s flip %d dir %d cls %s |S-T|-d %.3g S %.17g %.17g |S-mitre| %.4g\n", argv[k], flip, (int)q.direction, cls.c_str(), dS - d, S.x, S.y, dM);
    }
  }
}
