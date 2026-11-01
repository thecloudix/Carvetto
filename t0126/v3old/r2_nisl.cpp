#include "../t0121v2/common.hpp"
int main() {
  for (int n = 1; n <= 6; ++n) for (double so : {1.2, 2.4}) {
    Scene s = rect(0, 0, 8 + 20.0 * n, 30);
    for (int a = 0; a < n; ++a) s.addPoly({{8.0 + a * 20, 8}, {20.0 + a * 20, 8}, {20.0 + a * 20, 22}, {8.0 + a * 20, 22}});
    PocketParams p; p.cutDepth = 9; p.safeZ = 5;
    auto r = pocketToolpath(s.contours, {tool(6, so, 3)}, p);
    printf("row of %d islands (gaps 8 mm) so=%.1f: retracts %zu (limit 2)\n", n, so, r.toolpaths[0].count(MoveKind::Retract));
  }
  for (double so : {1.2, 2.4}) {
    Scene s = rect(0, 0, 50, 50); s.addPoly({{8,8},{20,8},{20,20},{8,20}}); s.addPoly({{30,8},{42,8},{42,20},{30,20}}); s.addPoly({{8,30},{20,30},{20,42},{8,42}});
    PocketParams p; p.cutDepth = 9; p.safeZ = 5;
    auto r = pocketToolpath(s.contours, {tool(6, so, 3)}, p);
    printf("L of 3 islands so=%.1f: retracts %zu (limit 2)\n", so, r.toolpaths[0].count(MoveKind::Retract));
  }
}
