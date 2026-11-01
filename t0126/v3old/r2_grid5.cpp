#include "../t0121v2/common.hpp"
int main(int argc, char** argv) {
  int which = atoi(argv[1]); double so = atof(argv[2]); int pp = atoi(argv[3]);
  Scene s;
  if (which == 0) { s = rect(0, 0, 70, 50); for (int a = 0; a < 3; ++a) for (int b = 0; b < 2; ++b) s.addPoly({{8.0 + a * 20, 8.0 + b * 20}, {20.0 + a * 20, 8.0 + b * 20}, {20.0 + a * 20, 20.0 + b * 20}, {8.0 + a * 20, 20.0 + b * 20}}); }
  else { s = rect(0, 0, 70, 40); s.addCircle({15, 20}, 6); s.addPoly({{28, 8}, {40, 8}, {40, 32}, {28, 32}}); s.addCircle({55, 12}, 4); s.addCircle({55, 29}, 5); }
  PocketParams p; p.cutDepth = 9; p.safeZ = 5; p.profilePass = ProfilePass(pp);
  auto r = pocketToolpath(s.contours, {tool(6, so, 3)}, p);
  int outers = 0; for (auto& c : r.plans[0].centreRegion) { double A=0; for (auto& sp : c.spans) { auto& l = std::get<LineSpan>(sp); A += l.start.x*l.end.y - l.end.x*l.start.y; } if (A>0) ++outers; }
  printf("centre outers=%d paths=%zu levels=%zu retracts=%zu\n", outers, r.plans[0].paths.size(), r.plans[0].levels.size(), r.toolpaths[0].count(MoveKind::Retract));
  const auto& mv = r.toolpaths[0].moves; Vec3 pos{};
  for (size_t i = 0; i < mv.size(); ++i) { if (mv[i].kind == MoveKind::Retract) { printf(" retract #%zu at (%.3f,%.3f,%.3f) -> next rapid to (%.3f,%.3f)\n", i, pos.x, pos.y, pos.z, mv[i+1].to.x, mv[i+1].to.y); } pos = mv[i].to; }
}
