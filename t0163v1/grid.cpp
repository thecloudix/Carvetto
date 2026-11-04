// T-0163 verifier: option grid over the collapsing blobs (the branch test's tinyBlob near the origin,
// and repro vector 21 at its original place, -500/-350): sides, starts, anchors, leads, lead-out,
// overcut, separate last pass, tabs (2D/3D), ramps, end mill / V-bit. Meant for the ASan build: the
// config is printed before every run, so a sanitizer abort names it. Prints one line per run.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::BezierSpan; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "scene_io.hpp"
#include "gen_helpers.inc"
int main(int argc, char** argv) {
  Contour blob; blob.closed = true;
  blob.spans = {
      BezierSpan{{-0.187086822, -0.120915918}, {-0.187087152, -0.12069675}, {-0.187207274, -0.120374426}, {-0.187405755, -0.120228396}},
      BezierSpan{{-0.187405755, -0.120228396}, {-0.187604236, -0.120082366}, {-0.188050888, -0.119920716}, {-0.188277708, -0.120039738}},
      BezierSpan{{-0.188277708, -0.120039738}, {-0.188504528, -0.12015876}, {-0.188793055, -0.120695069}, {-0.188766163, -0.120946042}},
      BezierSpan{{-0.188766163, -0.120946042}, {-0.18873927, -0.121197014}, {-0.188495223, -0.121448916}, {-0.188269986, -0.121547799}},
      BezierSpan{{-0.188269986, -0.121547799}, {-0.188044749, -0.121646683}, {-0.187730812, -0.121651905}, {-0.187564689, -0.121547244}},
      BezierSpan{{-0.187564689, -0.121547244}, {-0.187398566, -0.121442583}, {-0.187088059, -0.121145133}, {-0.187086822, -0.120915918}}};
  std::vector<std::vector<Contour>> scenes{{blob}}; size_t argScene = 99;
  ProfileParams base; double D = 0.004; int vd = 30;
  { std::vector<Contour> v; ProfileParams p0; if (argc > 1 && readScene(argv[1], v, p0, D, vd)) { argScene = scenes.size(); scenes.push_back(v); base = p0; } }
  // the blob next to a normal neighbour (so ordering, finish matching and tabs see other cuts)
  { std::vector<Contour> v{blob, Contour::rectangle(geometry::Box2{{-0.18, -0.13}, {-0.16, -0.11}}), Contour::circle({-0.2, -0.12}, 0.005)}; scenes.push_back(v); }
  long runs = 0, empty = 0;
  for (size_t si = 0; si < scenes.size(); ++si)
  for (int side = 0; side < 3; ++side) for (int st = 0; st < 3; ++st) for (int an : {0, 4, 8}) for (int L = 0; L < 3; ++L) for (int lo = 0; lo < 2; ++lo)
  for (int oc = 0; oc < 2; ++oc) for (int sep = 0; sep < 2; ++sep) for (int tabs = 0; tabs < 3; ++tabs) for (int ramp = 0; ramp < 4; ramp += (st == 1 ? 1 : 3)) for (int vb = 0; vb < 2; ++vb) {
    if (st != 2 && an != 0) continue;
    static long idx = -1; ++idx;  // SHARD=k/N: run only configs with idx % N == k
    if (getenv("SHARD")) { int k = 0, n = 1; std::sscanf(getenv("SHARD"), "%d/%d", &k, &n); if (idx % n != k) continue; }
    ProfileParams p = base; p.side = (ProfileSide)side; p.startAt = (StartAt)st; p.startAnchor = (BoxAnchor)an;
    if (getenv("ONLY") && (long)si != atol(getenv("ONLY"))) continue;
    if (si != argScene) { p.passes.cutDepth = 0.0004; p.passes.passDepth = 0.005; p.lastPassAllowance = 0.0003777814985338173; p.leads.length = 0.0006; p.leads.radius = 0.0003; p.allowance = 0; }
    p.leads.type = (LeadType)L; p.leads.leadOut = lo; p.leads.overcut = oc ? 0.0005 : 0.0; p.separateLastPass = sep;
    p.tabs.enabled = tabs > 0; p.tabs.threeD = tabs == 2; p.tabs.length = 0.0003; p.tabs.thickness = 0.5 * p.passes.cutDepth; p.tabs.count = 3; p.tabs.spacing = TabSpacing::ConstantNumber;
    p.ramp.type = (RampType)ramp; p.ramp.distance = 0.001;
    tooling::Tool tool = vb ? vbit(0.004, 30) : endMill(0.004);
    std::fprintf(stderr, "cfg s%zu side%d st%d an%d L%d lo%d oc%d sep%d tabs%d ramp%d vb%d\n", si, side, st, an, L, lo, oc, sep, tabs, ramp, vb);
    auto r = profileToolpath(scenes[si], tool, p);
    bool fin = true; for (auto& m : r.toolpath.moves) fin = fin && std::isfinite(m.to.x) && std::isfinite(m.to.y) && std::isfinite(m.to.z);
    ++runs; if (r.toolpath.moves.empty()) ++empty;
    std::printf("s%zu side%d st%d an%d L%d lo%d oc%d sep%d tabs%d ramp%d vb%d moves %zu diags %zu finite %d | %s\n", si, side, st, an, L, lo, oc, sep, tabs, ramp, vb, r.toolpath.moves.size(), r.diagnostics.size(), (int)fin, r.diagnostics.empty() ? "" : r.diagnostics[0].message.c_str());
    std::fflush(stdout);
  }
  std::printf("RUNS %ld emptyToolpaths %ld\n", runs, empty);
}
