// T-0137 verifier: bit-exact hash of Profile output (moves + diagnostics) main vs branch with the
// allowance drawn around the On limits (+-r, +-1 ulp, r - lpa, ...) over all sides, tabs, separate
// last pass, ramps, leads, V-bits, clusters/grids/text pages, scale and translation transforms.
// usage: hs seed N [scale] [offset]      random scenes
//        hs scn file...                  stored .scn scenes: original + allowance/side variants (ORIGONLY=1: original only)
// Each line: tag, expect (same | onabove), moves, diags, hash. "onabove" = side On and allowance or
// allowance + lpa (separate last pass) > r: the branch is expected to refuse (0 moves).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour;
using geometry::Vec2;
using geometry::Box2;
using geometry::Polyline;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "scene_io.hpp"
#include "gen_helpers.inc"
struct H { unsigned long long h = 1469598103934665603ULL;
  void bytes(const void* p, size_t n) { auto* c = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; } }
  void d(double x) { bytes(&x, 8); } void i(long long x) { bytes(&x, 8); } void s(const std::string& x) { i((long long)x.size()); bytes(x.data(), x.size()); } };
static void hashTp(H& h, const Toolpath& tp) { h.i((long long)tp.moves.size());
  for (auto& m : tp.moves) { h.i((int)m.kind); h.d(m.to.x); h.d(m.to.y); h.d(m.to.z); h.d(m.centre.x); h.d(m.centre.y); h.i(m.feedOverride.has_value()); h.d(m.feedOverride.value_or(0)); h.d(m.dwellSeconds); } }
static void hashDiag(H& h, const std::vector<ToolpathDiagnostic>& ds) { for (auto& d : ds) { h.i((int)d.issue); h.i(d.contour ? (long long)*d.contour : -1); h.i(d.tool ? (long long)*d.tool : -1); h.s(d.message); } }
static Contour xf(Contour c, double S, Vec2 off) {
  auto P = [&](Vec2 p) { return Vec2{p.x * S + off.x, p.y * S + off.y}; };
  for (auto& sp : c.spans) {
    if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = P(l->start); l->end = P(l->end); }
    else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = P(a->centre); a->radius *= S; }
    else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = P(b->p0); b->p1 = P(b->p1); b->p2 = P(b->p2); b->p3 = P(b->p3); }
  }
  return c;
}
static std::vector<Contour> page(size_t n, double pitch) {
  std::vector<Contour> cs; size_t cols = size_t(std::ceil(std::sqrt(double(n))));
  for (size_t k = 0; cs.size() < n; ++k) {
    double s = U(2.0, 4.5), x = double(k % cols) * pitch + U(-0.5, 0.5), y = double(k / cols) * pitch + U(-0.5, 0.5);
    switch (I(0, 6)) {
      case 0: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); break;
      case 1: cs.push_back(letter({x, y}, s * 0.7)); break;
      case 2: cs.push_back(Contour::circle({x + s / 2, y + s / 2}, s / 2)); break;
      case 3: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); if (cs.size() < n) cs.push_back(Contour::circle({x + s / 2, y + s / 2}, 0.2 * s)); break;
      case 4: cs.push_back(spikes({x + s / 2, y + s / 2}, s / 2, I(3, 7))); break;
      case 5: cs.push_back(bezBlob({x + s / 2, y + s / 2}, s / 2, I(4, 7))); break;
      default: cs.push_back(anyOpen({x + s / 2, y + s / 2}, s / 2)); break;
    }
  }
  return cs;
}
static const double INF = 1.0 / 0.0;
// allowance drawn around the limits of r (lpa already set)
static double drawAllowance(double r, double lpa) {
  switch (I(0, 15)) {
    case 0: return r;
    case 1: return std::nextafter(r, -INF);
    case 2: return std::nextafter(r, INF);           // above (On refuses)
    case 3: return -r;
    case 4: return U(-r, r);
    case 5: return 0.0;
    case 6: return r * (1 - U(0, 1e-6));
    case 7: return r - 1e-9;
    case 8: return r + 1e-9;                         // above
    case 9: return U(r, 3 * r);                      // above
    case 10: return r - lpa;                         // roughing at (about) r
    case 11: return std::nextafter(r - lpa, -INF);
    case 12: return std::nextafter(r - lpa, INF);
    case 13: return -r + U(0, 0.5 * r);
    case 14: return U(0, 1);
    default: return -0.0;
  }
}
static bool onAbove(const ProfileParams& p, double r) {
  if (p.side != ProfileSide::On) return false;
  double rough = p.separateLastPass ? p.allowance + p.lastPassAllowance : p.allowance;
  return p.allowance > r || rough > r;
}
static bool belowMinus(const ProfileParams& p, double r) {
  double rough = p.separateLastPass ? p.allowance + p.lastPassAllowance : p.allowance;
  return p.allowance < -r || rough < -r;
}
static void run(const char* tag, const std::vector<Contour>& cs, const tooling::Tool& tool, const ProfileParams& p) {
  double r = profileToolRadius(tool, p.passes);
  auto res = profileToolpath(cs, tool, p);
  H h; hashTp(h, res.toolpath); hashDiag(h, res.diagnostics);
  bool ab = onAbove(p, r) && !belowMinus(p, r);
  std::string d0 = res.diagnostics.empty() ? "" : res.diagnostics[0].message;
  for (auto& ch : d0) if (ch == '\n') ch = ' ';
  std::printf("%s expect %s side %d sep %d tabs %d a %a lpa %a r %a moves %zu diags %zu hash %016llx | %s\n", tag, ab ? "onabove" : "same", (int)p.side,
              (int)p.separateLastPass, (int)p.tabs.enabled, p.allowance, p.lastPassAllowance, r, res.toolpath.moves.size(), res.diagnostics.size(), h.h,
              ab ? d0.c_str() : "");
  std::fflush(stdout);
}
int main(int argc, char** argv) {
  if (std::string(argv[1]) == "scn") {
    for (int k = 2; k < argc; ++k) {
      std::vector<Contour> cs0; ProfileParams p0; double D = 6; int vd = 0;
      if (!readScene(argv[k], cs0, p0, D, vd)) continue;
      tooling::Tool tool = vd ? vbit(D, vd) : endMill(D);
      std::string b = argv[k]; b = b.substr(b.find_last_of('/') + 1);
      double r = profileToolRadius(tool, p0.passes);
      run((b + ":orig").c_str(), cs0, tool, p0);
      if (std::getenv("ORIGONLY")) continue;
      const ProfileSide sides[3] = {ProfileSide::Outside, ProfileSide::Inside, ProfileSide::On};
      const double as[6] = {r, std::nextafter(r, -INF), std::nextafter(r, INF), -r, 0.5 * r, r - p0.lastPassAllowance};
      for (int s = 0; s < 3; ++s) for (int ai = 0; ai < 6; ++ai) for (int sep = 0; sep < 2; ++sep) {
        ProfileParams p = p0; p.side = sides[s]; p.allowance = as[ai]; p.separateLastPass = sep;
        char t[512]; std::snprintf(t, sizeof t, "%s:s%d:a%d:sep%d", b.c_str(), s, ai, sep); run(t, cs0, tool, p);
      }
    }
    return 0;
  }
  unsigned long seed = std::strtoul(argv[1], nullptr, 10); int N = std::atoi(argv[2]);
  double S = argc > 3 ? std::atof(argv[3]) : 1.0; double O = argc > 4 ? std::atof(argv[4]) : 0.0;
  int target = getenv("TARGET") ? atoi(getenv("TARGET")) : -1;  // TARGET=k: only scene k; FROM=k: scenes >= k
  rng.seed(seed);
  for (int sc = 0; sc < N; ++sc) {
    int fam = 0, forceSep = -1; int mode = sc % 5 == 4 ? 0 : sc % 4;  // 0 families, 1 cluster, 2 grid of many small, 3 text page
#include "gen_body.inc"
    p.tabs.enabled = I(0, 1) == 0;
    if (mode >= 1) {
      cs.clear();
      if (mode == 3) { cs = page((size_t)I(20, 150), U(5, 8)); D = std::vector<double>{0.5, 1.0, 2.0}[(size_t)I(0, 2)]; useV = I(0, 3) == 0;
        p.passes.cutDepth = U(1, 4); p.tabs.thickness = U(0.2, p.passes.cutDepth); p.tabs.length = U(0.3, 2); p.leads.length = U(0.2, 2); p.leads.radius = U(0.2, 2); p.ramp.distance = U(0.5, 5);
        p.lastPassAllowance = U(0.02, 0.4); p.separateLastPass = I(0, 1); }
      else {
        int K = mode == 1 ? I(2, 25) : I(30, 150);
        double box = mode == 1 ? U(20, 150) : std::sqrt((double)K) * U(5, 14);
        for (int k = 0; k < K; ++k) {
          Vec2 c{U(0, box), U(0, box)}; double rr = mode == 1 ? U(2, 25) : U(0.8, 5); int t = I(0, 9);
          if (t <= 3) cs.push_back(anyClosed(c, rr));
          else if (t == 4) cs.push_back(Contour::circle(c, rr, I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW));
          else if (t == 5) cs.push_back(Contour::rectangle(geometry::Box2{{c.x - rr, c.y - rr * 0.6}, {c.x + rr, c.y + rr * 0.6}}));
          else if (t <= 7) cs.push_back(anyOpen(c, rr));
          else if (t == 8) cs.push_back(bezBlob(c, rr, I(4, 8)));
          else cs.push_back(letter(c, rr));
        }
        if (mode == 2) { D = std::vector<double>{0.5, 1.0, 2.0, 3.175}[(size_t)I(0, 3)]; useV = useV && I(0, 1); }
      }
    }
    if (mode == 0 && I(0, 2) == 0) { cs.clear(); cs.push_back(anyOpen({0, 0}, U(8, 35))); if (I(0, 1)) cs.push_back(translate(anyOpen({0, 0}, U(6, 20)), {U(30, 60), U(-10, 10)})); }
    if (I(0, 6) == 0 && !useV) { useV = true; vang = std::vector<double>{30, 60, 90, 120, 150}[(size_t)I(0, 4)]; p.passes.cutDepth = U(0.05, 6); p.tabs.thickness = U(0.02, p.passes.cutDepth); }
    int sk = I(0, 4); p.side = sk == 0 ? ProfileSide::Outside : sk == 1 ? ProfileSide::Inside : ProfileSide::On;
    if (I(0, 1)) p.separateLastPass = true;
    if (I(0, 7) == 0) p.lastPassAllowance = 0.0;
    for (auto& c : cs) c = xf(c, S, {O, O * 0.7});
    p.lastPassAllowance *= S; p.tabs.length *= S; p.tabs.distance *= S; p.leads.length *= S; p.leads.radius *= S; p.leads.overcut *= S; p.ramp.distance *= S;
    p.passes.startDepth *= S; p.passes.cutDepth *= S; p.passes.passDepth *= S; p.tabs.thickness *= S; p.passes.lastPassThickness *= S;
    p.startPosition = {p.startPosition.x * S + O, p.startPosition.y * S + O * 0.7};
    tooling::Tool tool = useV ? vbit(D * 2 * S, vang) : endMill(D * S);
    double r = profileToolRadius(tool, p.passes);
    p.allowance = drawAllowance(r, p.separateLastPass ? p.lastPassAllowance : 0.0);
    // skip only after every draw, so TARGET/FROM reproduce the sequential scene exactly
    if (target >= 0 && sc != target) continue;
    if (getenv("FROM") && sc < atoi(getenv("FROM"))) continue;
    if (getenv("DUMP")) writeScene(getenv("DUMP"), cs, p, tool.diameter, useV ? (int)std::lround(vang) : 0);  // DUMP=file.scn (scaled scene, readable by 'hs scn')
    char t[64]; std::snprintf(t, sizeof t, "r%lu:%d:m%d:V%d", seed, sc, mode, (int)useV);
    run(t, cs, tool, p);
  }
}
