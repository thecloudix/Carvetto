// T-0158 verifier (round 3: T line + mTurn pick; round 2: TFS2 log format of tfs_instr.py; T line = tag call# rule nearCount runs du dpt dRule x y mArcC mArcR mArcS mCut mNear mGap zda cutd): Profile cut starts. Links against the INSTRUMENTED branch lib (logsrc: tieFreeStart
// logs every call to stderr when TFS_LOG is set; TFS_OFF=1 disables the rule = main behaviour), or any lib.
// Output per run (stdout):
//   T <tag> <call#> fired nq wrap du dpt dRule chosenX chosenY nearMissC nearMissR     (one per tieFreeStart call)
//   R <tag> moves N diags D hash H left L free F
// usage: st seed N [scale] [offset]           random scenes (t0137/t0163 hs generator)
//        st scn file...                        stored scenes (VARS=1: + VectorStart/Outside, BBox Centre, Optimise variants)
//        st pert file...                       perturbation sweep of stored scenes (also PERTSEED=s PERTN=n: random scenes)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include <unistd.h>
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

struct Call { int rule = 0, nc = -1, runs = -1, zda = 0; double du = 0, dpt = 0, dRule = 0, x = 0, y = 0, tx = 0, ty = 0, mArcC = 1e300, mArcR = 1e300, mArcS = 1e300, mCut = 1e300, mNear = 1e300, mGap = 1e300, cutd = 0, nearest = 0, mTurn = 1e300, jpm = 0; int pick = -1, jmin = -1, runLen = -1; };
struct Run { size_t moves = 0, diags = 0, left = 0, freeN = 0; unsigned long long hash = 0; std::vector<Call> calls; };

static size_t tabsLeftOut(const ProfileResult& r) { size_t n = 0;
  for (auto& d : r.diagnostics) if (d.issue == ProfileIssue::TabsNotPlaced && d.message.find(" tab(s) left out") != std::string::npos) n += std::stoul(d.message.substr(d.message.find(": ") + 2));
  return n; }
static size_t comesFree(const ProfileResult& r) { size_t n = 0; for (auto& d : r.diagnostics) if (d.message.find("comes free") != std::string::npos) ++n; return n; }

static Run runOnce(const std::vector<Contour>& cs, const tooling::Tool& tool, const ProfileParams& p) {
  fflush(stderr);
  FILE* tmp = tmpfile(); int saved = dup(2); dup2(fileno(tmp), 2);
  setenv("TFS_LOG", "1", 1);
  auto res = profileToolpath(cs, tool, p);
  fflush(stderr); dup2(saved, 2); close(saved);
  Run R; H h; hashTp(h, res.toolpath); hashDiag(h, res.diagnostics);
  R.hash = h.h; R.moves = res.toolpath.moves.size(); R.diags = res.diagnostics.size(); R.left = tabsLeftOut(res); R.freeN = comesFree(res);
  rewind(tmp); char line[4096];
  while (fgets(line, sizeof line, tmp)) {
    if (strncmp(line, "TFS2 ", 5)) continue;
    Call c; size_t n; int closed; double rx, ry, px, py, eps;
    if (sscanf(line, "TFS2 rule %d n %zu closed %d nearCount %d runs %d du %lg dpt %lg nearest %lg cutd %lg tgt %lg %lg ruleXY %lg %lg projXY %lg %lg mArcC %lg mArcR %lg mArcS %lg mCut %lg mNear %lg mGap %lg zda %d eps %lg",
               &c.rule, &n, &closed, &c.nc, &c.runs, &c.du, &c.dpt, &c.nearest, &c.cutd, &c.tx, &c.ty, &rx, &ry, &px, &py, &c.mArcC, &c.mArcR, &c.mArcS, &c.mCut, &c.mNear, &c.mGap, &c.zda, &eps) == 23) {
      bool off = getenv("TFS_OFF") != nullptr;
      c.x = (c.rule && !off) ? rx : px; c.y = (c.rule && !off) ? ry : py;
      c.dRule = std::hypot(rx - c.tx, ry - c.ty) - c.nearest;
      if (const char* mt = strstr(line, "mTurn ")) sscanf(mt, "mTurn %lg pick %d runLen %d slack %*g jmin %d jPickMinusMin %lg", &c.mTurn, &c.pick, &c.runLen, &c.jmin, &c.jpm);
      R.calls.push_back(c);
    }
  }
  fclose(tmp);
  return R;
}
static void print(const char* tag, const Run& R) {
  for (size_t i = 0; i < R.calls.size(); ++i) { auto& c = R.calls[i];
    std::printf("T %s %zu %d %d %d %.6g %.6g %.3g %.17g %.17g %.4g %.4g %.4g %.4g %.4g %.4g %d %.6g %.4g %d %d %d %.4g\n", tag, i, c.rule, c.nc, c.runs, c.du, c.dpt, c.dRule, c.x, c.y,
                c.mArcC, c.mArcR, c.mArcS, c.mCut, c.mNear, c.mGap, c.zda, c.cutd, c.mTurn, c.pick, c.runLen, c.jmin, c.jpm); }
  std::printf("R %s moves %zu diags %zu hash %016llx left %zu free %zu\n", tag, R.moves, R.diags, R.hash, R.left, R.freeN);
  std::fflush(stdout);
}
static Contour xf(Contour c, double S, Vec2 off) {
  auto P = [&](Vec2 p) { return Vec2{p.x * S + off.x, p.y * S + off.y}; };
  for (auto& sp : c.spans) {
    if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = P(l->start); l->end = P(l->end); }
    else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = P(a->centre); a->radius *= S; }
    else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = P(b->p0); b->p1 = P(b->p1); b->p2 = P(b->p2); b->p3 = P(b->p3); }
  }
  return c;
}
// exact rotation by q quarter turns about the origin (arc angles get q*pi/2 added: rounded)
static Vec2 rotq(Vec2 p, int q) { for (int i = 0; i < q; ++i) p = Vec2{-p.y, p.x}; return p; }
static Contour rot(Contour c, int q) {
  for (auto& sp : c.spans) {
    if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = rotq(l->start, q); l->end = rotq(l->end, q); }
    else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = rotq(a->centre, q); a->startAngle += q * PI / 2; a->endAngle += q * PI / 2; }
    else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = rotq(b->p0, q); b->p1 = rotq(b->p1, q); b->p2 = rotq(b->p2, q); b->p3 = rotq(b->p3, q); }
  }
  return c;
}
static BoxAnchor rotAnchor(BoxAnchor a, int q) { int k = (int)a; for (int i = 0; i < q; ++i) { int col = k % 3, row = k / 3; k = col * 3 + (2 - row); } return (BoxAnchor)k; }

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
static double drawAllowance(double r, double lpa) {
  switch (I(0, 15)) {
    case 0: return r; case 1: return std::nextafter(r, -INF); case 2: return std::nextafter(r, INF); case 3: return -r; case 4: return U(-r, r); case 5: return 0.0;
    case 6: return r * (1 - U(0, 1e-6)); case 7: return r - 1e-9; case 8: return r + 1e-9; case 9: return U(r, 3 * r); case 10: return r - lpa;
    case 11: return std::nextafter(r - lpa, -INF); case 12: return std::nextafter(r - lpa, INF); case 13: return -r + U(0, 0.5 * r); case 14: return U(0, 1); default: return -0.0;
  }
}
static void doScene(const std::string& tag, const std::vector<Contour>& cs0, const tooling::Tool& tool, const ProfileParams& p, bool pert) {
  if (!pert) { print(tag.c_str(), runOnce(cs0, tool, p)); return; }
  struct X { std::string n; double s; Vec2 off; int q; };
  std::vector<X> xs{{"id", 1, {0, 0}, 0}};
  double up = 1, dn = 1;
  for (int i = 1; i <= 4; ++i) { up = std::nextafter(up, 2.0); dn = std::nextafter(dn, 0.0); xs.push_back({"u+" + std::to_string(i), up, {0, 0}, 0}); xs.push_back({"u-" + std::to_string(i), dn, {0, 0}, 0}); }
  for (double r : {1e-12, 1e-10, 1e-9}) { char t[32]; std::snprintf(t, sizeof t, "r+%g", r); xs.push_back({t, 1 + r, {0, 0}, 0}); std::snprintf(t, sizeof t, "r-%g", r); xs.push_back({t, 1 - r, {0, 0}, 0}); }
  xs.push_back({"t3", 1, {3.0, -7.0}, 0}); xs.push_back({"t0.1", 1, {0.1, 0.3}, 0}); xs.push_back({"t1000", 1, {1000.25, -999.75}, 0});
  xs.push_back({"q1", 1, {0, 0}, 1}); xs.push_back({"q2", 1, {0, 0}, 2}); xs.push_back({"q3", 1, {0, 0}, 3});
  for (auto& x : xs) {
    std::vector<Contour> cs; for (auto& c : cs0) cs.push_back(x.q ? rot(c, x.q) : xf(c, x.s, x.off));
    ProfileParams pp = p;
    if (x.q) { pp.startPosition = rotq(pp.startPosition, x.q); pp.startAnchor = rotAnchor(pp.startAnchor, x.q); }
    else { pp.startPosition = {pp.startPosition.x * x.s + x.off.x, pp.startPosition.y * x.s + x.off.y}; }
    // tool and lengths kept (s within 1e-9 of 1): only geometry perturbed
    Run R = runOnce(cs, tool, pp);
    for (auto& c : R.calls) { Vec2 q{c.x, c.y};
      if (x.q) q = rotq(q, 4 - x.q); else q = Vec2{(q.x - x.off.x) / x.s, (q.y - x.off.y) / x.s};
      c.x = q.x; c.y = q.y; }
    print((tag + ":" + x.n).c_str(), R);
  }
}
int main(int argc, char** argv) {
  if (argc < 2) return 1;
  std::string mode0 = argv[1];
  const bool pert = mode0 == "pert" || getenv("PERT");
  if (mode0 == "scn" || mode0 == "pert") {
    for (int k = 2; k < argc; ++k) {
      std::vector<Contour> cs0; ProfileParams p0; double D = 6; int vd = 0;
      if (!readScene(argv[k], cs0, p0, D, vd)) continue;
      tooling::Tool tool = vd ? vbit(D, vd) : endMill(D);
      std::string b = argv[k]; b = b.substr(b.find_last_of('/') + 1);
      std::vector<std::pair<std::string, ProfileParams>> vars{{":orig", p0}};
      if (getenv("VARS")) {
        ProfileParams a = p0; a.startAt = StartAt::VectorStart; a.side = ProfileSide::Outside; vars.push_back({":vsout", a});
        a = p0; a.startAt = StartAt::ClosestOnBoundingBox; a.startAnchor = BoxAnchor::Centre; vars.push_back({":bbc", a});
        a = p0; a.startAt = StartAt::Optimise; vars.push_back({":opt", a});
      }
      for (auto& [vn, p] : vars) doScene(b + vn, cs0, tool, p, pert);
    }
    return 0;
  }
  unsigned long seed = std::strtoul(argv[1], nullptr, 10); int N = std::atoi(argv[2]);
  double S = argc > 3 ? std::atof(argv[3]) : 1.0; double O = argc > 4 ? std::atof(argv[4]) : 0.0;
  int target = getenv("TARGET") ? atoi(getenv("TARGET")) : -1;
  rng.seed(seed);
  for (int sc = 0; sc < N; ++sc) {
    int fam = 0, forceSep = -1; int mode = sc % 5 == 4 ? 0 : sc % 4;
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
    if (target >= 0 && sc != target) continue;
    if (getenv("FROM") && sc < atoi(getenv("FROM"))) continue;
    if (getenv("DUMP")) writeScene(getenv("DUMP"), cs, p, tool.diameter, useV ? (int)std::lround(vang) : 0);
    char t[64]; std::snprintf(t, sizeof t, "r%lu:%d:m%d:V%d", seed, sc, mode, (int)useV);
    doScene(t, cs, tool, p, pert);
  }
}
