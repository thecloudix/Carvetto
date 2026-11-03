// T-0135 r3 verifier: adversarial scenes for the exact loose-piece construction. Writes adv/NAME.scn.
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "scene_io.hpp"
#include "gen_helpers.inc"
static Contour sq(double x0, double y0, double x1, double y1) { return poly({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}, true); }
static Contour circ(Vec2 c, double r) { return Contour::circle(c, r); }
static Contour star(Vec2 c, double ro, double ri, int n) { std::vector<Vec2> p; for (int i = 0; i < 2 * n; ++i) { double a = PI * i / n, r = i % 2 ? ri : ro; p.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)}); } return poly(p, true); }
static ProfileParams base(double L, int count = 1) {
    ProfileParams p; p.side = ProfileSide::On; p.allowance = 0; p.passes.cutDepth = 3; p.passes.passDepth = 3;
    p.tabs.enabled = true; p.tabs.count = count; p.tabs.length = L; p.tabs.thickness = 1; p.startAt = StartAt::VectorStart;
    p.leads.type = LeadType::None; p.separateLastPass = false; return p;
}
static void out(const std::string& n, const std::vector<Contour>& cs, const ProfileParams& p, double D, int vdeg = 0) {
    writeScene(("adv/" + n + ".scn").c_str(), cs, p, D, vdeg);
}
int main() {
    // c1: open line from outside cuts a corner triangle off square A (A's tabs elsewhere)
    { auto p = base(2, 4); std::vector<Contour> cs{sq(0, 0, 40, 40), poly({{-20, 10}, {30, 60}}, false)}; out("c1_intrude", cs, p, 2);
      p.separateLastPass = true; p.lastPassAllowance = 0.3; out("c1_intrude_sep", cs, p, 2);
      p = base(2, 4); p.side = ProfileSide::Outside; p.allowance = -0.8; out("c1_intrude_outneg", cs, p, 2); }
    // c1b: same, many tabs on the line so one lands on the triangle's edge
    { auto p = base(1, 12); std::vector<Contour> cs{sq(0, 0, 40, 40), poly({{-20, 10}, {30, 60}}, false)}; out("c1b_intrude_tabs", cs, p, 2); }
    // c2: nested tab-less loops inside a held square (tab length trick: small loops get no tab)
    { auto p = base(40); std::vector<Contour> cs{sq(0, 0, 40, 40), circ({20, 20}, 6), circ({20, 20}, 3)}; out("c2_nested", cs, p, 2);
      p.tabs.threeD = true; out("c2_nested_3d", cs, p, 2); p.tabs.threeD = false;
      out("c2_nested_v90", cs, p, 6, 90); }
    // c3: tab on a star C inside tab-less circle B: C's tab joins C's slug to B's slug ring; both one loose piece
    { auto p = base(55); std::vector<Contour> cs{sq(-20, -20, 20, 20), circ({0, 0}, 8), star({0, 0}, 6.5, 2.0, 12)}; out("c3_samehole", cs, p, 0.5);
      p.tabs.threeD = true; out("c3_samehole_3d", cs, p, 0.5); p.tabs.threeD = false;
      p.side = ProfileSide::Inside; p.allowance = -0.2; out("c3_samehole_inneg", cs, p, 0.5); }
    // c3b: spur zigzag (open, carries a tab) inside a tab-less circle
    { auto p = base(20); std::vector<Vec2> z; for (int i = 0; i < 5; ++i) { double x = -1.6 + 0.8 * i; if (i % 2) { z.push_back({x, 1.8}); z.push_back({x, -1.8}); } else { z.push_back({x, -1.8}); z.push_back({x, 1.8}); } }
      std::vector<Contour> cs{sq(-20, -20, 20, 20), circ({0, 0}, 3), poly(z, false)}; out("c3b_spur", cs, p, 0.5); }
    // c5: open cluster: four open lines forming a frame; tabs too long to fit -> interior loose
    { auto p = base(50); std::vector<Contour> cs{poly({{-1, 0}, {31, 0}}, false), poly({{30, -1}, {30, 31}}, false), poly({{31, 30}, {-1, 30}}, false), poly({{0, 31}, {0, -1}}, false)}; out("c5_openframe", cs, p, 2);
      // one line extends far so its (single) tab lands outside the frame
      cs[0] = poly({{-1, 0}, {131, 0}}, false); out("c5_openframe_tabout", cs, p, 2);
      auto q = base(2, 1); cs[0] = poly({{-1, 0}, {131, 0}}, false); out("c5_openframe_tabout2", cs, q, 2); }
    // c8: # frame of four long thin rectangles; the middle is enclosed but outside every box (documented gap?)
    { auto p = base(2, 4); std::vector<Contour> cs{sq(0, 0, 40, 3), sq(0, 37, 40, 40), sq(0, 0, 3, 40), sq(37, 0, 40, 40)}; out("c8_hash", cs, p, 2);
      std::vector<Contour> cs2{sq(-5, 0, 45, 2), sq(-5, 38, 45, 40), sq(0, -5, 2, 45), sq(38, -5, 40, 45)}; out("c8_hash2", cs2, p, 2); }
    // c9: three circles whose kerfs overlap pairwise; middle curvilinear triangle enclosed
    { auto p = base(1, 3); double s = 20.5; std::vector<Contour> cs{circ({0, 0}, 10), circ({s, 0}, 10), circ({s / 2, s * std::sqrt(3) / 2}, 10)}; out("c9_tri", cs, p, 2); }
    // c10: tab on the island band: square part held, hole B inside with tab -> slug held; nested C in slug no tab
    { auto p = base(8); std::vector<Contour> cs{sq(0, 0, 40, 40), sq(10, 10, 30, 30), circ({20, 20}, 1.5)}; out("c10_holdslug", cs, p, 2); }
    // c11: overlapping squares, On: lens region + crescent pieces
    { auto p = base(2, 2); std::vector<Contour> cs{sq(0, 0, 30, 30), sq(15, 15, 45, 45)}; out("c11_overlap", cs, p, 2);
      p.tabs.count = 1; out("c11_overlap1", cs, p, 2); p.tabs.count = 8; out("c11_overlap8", cs, p, 2); }
    // c12: many identical tab-less circles (message cap 50 + dedupe across region boxes)
    { auto p = base(40); std::vector<Contour> cs{sq(0, 0, 100, 100)}; for (int i = 0; i < 8; ++i) for (int j = 0; j < 8; ++j) cs.push_back(circ({8 + 12.0 * i, 8 + 12.0 * j}, 3)); out("c12_many", cs, p, 2); }
    // c13: open line crossing two closed loops' shared region (piece in two boxes, attributed once)
    { auto p = base(2, 4); std::vector<Contour> cs{sq(0, 0, 30, 30), sq(25, 0, 55, 30), poly({{20, -10}, {35, 40}}, false)}; out("c13_twobox", cs, p, 2); }
    std::printf("ok\n");
}
