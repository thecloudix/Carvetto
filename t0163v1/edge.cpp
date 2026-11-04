// T-0163 verifier: degenerate / collapsing closed cuts (tiny Bezier loops and blobs, tiny circles,
// rectangles, back-and-forth zero-area loops, single-span loops) mixed with normal neighbours, at
// scales 1 .. 1e-4, over random Profile params (sides incl. On, sep last pass, tabs 2D/3D, leads
// None/Straight/Circular with lead-out and overcut, start choice, ramps, ordering, end mills and
// V-bits). Each scene runs in a forked child, so a crash costs one scene and is reported.
// usage: edge seed N   -> per line: tag sig moves diags hash emptyCut? (hash of moves + diags)
//        DUMP=dir: writes dir/<tag>.scn for every scene (readable by hs scn)
#include <sys/wait.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour;
using geometry::Vec2;
using geometry::Box2;
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
static Contour bezLoop(Vec2 p, double s) {  // one closed Bezier (p0 == p3), a teardrop
  Contour c; c.closed = true; double a = U(0, 2 * PI);
  c.append(geometry::BezierSpan{p, p + Vec2::fromAngle(a, s), p + Vec2::fromAngle(a + U(0.3, 2.5), s), p}); return c; }
static Contour backForth(Vec2 p, double s) {  // zero-area closed A -> B -> A
  Vec2 q = p + Vec2::fromAngle(U(0, 2 * PI), s); return poly({p, q}); }
static Contour backForth3(Vec2 p, double s) { Vec2 q = p + Vec2{s, 0}, r = p + Vec2{2 * s, 0}; return poly({p, q, r, q}); }
static Contour pointLoop(Vec2 p) { Contour c; c.closed = true; c.append(geometry::LineSpan{p, p}); return c; }
static Contour tiny(Vec2 c, double s) {
  switch (I(0, 9)) {
    case 0: return bezLoop(c, s);
    case 1: return bezBlob(c, s, I(3, 7));
    case 2: return Contour::circle(c, s * U(0.1, 1), I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW);
    case 3: return Contour::rectangle(Box2{c, c + Vec2{s, s * U(0.01, 1)}});
    case 4: return backForth(c, s);
    case 5: return backForth3(c, s);
    case 6: return I(0, 3) ? spikes(c, s, I(3, 6)) : pointLoop(c);
    case 7: return ellipse(c, s, s * U(0.001, 1), U(0, PI));
    case 8: return dshape(c, s);
    default: return slot(c, s, s * U(0.01, 0.5), U(0, PI));
  }
}
int main(int argc, char** argv) {
  unsigned long seed = std::strtoul(argv[1], nullptr, 10); int N = std::atoi(argv[2]);
  rng.seed(seed);
  for (int sc = 0; sc < N; ++sc) {
    const double scales[] = {1, 0.1, 0.01, 0.001, 1e-4};
    double S = scales[I(0, 4)];
    std::vector<Contour> cs;
    int nt = I(1, 6);
    for (int k = 0; k < nt; ++k) cs.push_back(tiny({U(0, 6) , U(0, 6)}, std::pow(10.0, U(-4, 0.3))));
    int nn = I(0, 3);
    for (int k = 0; k < nn; ++k) cs.push_back(I(0, 2) ? anyClosed({U(0, 30), U(0, 30)}, U(2, 10)) : anyOpen({U(0, 30), U(0, 30)}, U(8, 15)));
    if (I(0, 3) == 0) cs.push_back(Contour::rectangle(Box2{{-2, -2}, {40, 40}}));  // nesting: tiny ones become holes
    if (I(0, 4) == 0) std::swap(cs.front(), cs.back());
    for (auto& c : cs) for (auto& sp : c.spans) {
      auto P = [&](Vec2 p) { return Vec2{p.x * S, p.y * S}; };
      if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = P(l->start); l->end = P(l->end); }
      else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = P(a->centre); a->radius *= S; }
      else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = P(b->p0); b->p1 = P(b->p1); b->p2 = P(b->p2); b->p3 = P(b->p3); }
    }
    ProfileParams p;
    double D = std::vector<double>{0.5, 1, 3.175, 6}[(size_t)I(0, 3)] * S * (I(0, 2) ? 1 : U(0.001, 0.1));
    bool useV = I(0, 1); double vang = std::vector<double>{30, 60, 90, 150}[(size_t)I(0, 3)];
    int sk = I(0, 2); p.side = sk == 0 ? ProfileSide::Outside : sk == 1 ? ProfileSide::Inside : ProfileSide::On;
    p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
    p.passes.cutDepth = U(0.05, 3) * S; p.passes.passDepth = U(0.2, 2) * S; p.passes.startDepth = I(0, 3) == 0 ? U(0, 0.5) * S : 0.0;
    if (I(0, 4) == 0) p.passes.lastPassThickness = U(0.01, 0.5) * S;
    double r0 = 0.5 * D;
    switch (I(0, 5)) { case 0: p.allowance = 0; break; case 1: p.allowance = U(-r0, r0); break; case 2: p.allowance = r0; break; case 3: p.allowance = -r0; break; default: p.allowance = U(0, 0.3) * S; }
    p.separateLastPass = I(0, 1); p.lastPassAllowance = I(0, 5) == 0 ? 0.0 : U(0.0, 0.5) * S; p.lastPassReverse = I(0, 1);
    p.tabs.enabled = I(0, 1); p.tabs.length = U(0.05, 3) * S; p.tabs.thickness = U(0.01, 1) * p.passes.cutDepth;
    p.tabs.spacing = I(0, 1) ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance; p.tabs.count = I(1, 6); p.tabs.distance = U(0.1, 20) * S;
    p.tabs.avoidCorners = I(0, 1); p.tabs.threeD = I(0, 2) == 0;
    p.ramp.type = (RampType)I(0, 3); p.ramp.distance = U(0.05, 5) * S; p.ramp.useAngle = I(0, 2) == 0; p.ramp.angle = U(1, 30) * PI / 180; p.ramp.onLeadIn = I(0, 1);
    p.leads.type = (LeadType)I(0, 2); p.leads.length = U(0.01, 2) * S; p.leads.angle = U(0.05, PI / 2); p.leads.radius = U(0.01, 2) * S;
    p.leads.sweep = U(5, 90) * PI / 180; p.leads.leadOut = I(0, 2) != 0; p.leads.overcut = I(0, 1) ? U(0, 1) * S : 0.0;
    p.startAt = (StartAt)I(0, 2); p.startAnchor = (BoxAnchor)I(0, 8); p.startPosition = {U(-5, 40) * S, U(-5, 40) * S};
    p.order.leftToRight = I(0, 3) == 0; p.order.grid = I(0, 3) == 0; p.order.selectionOrder = I(0, 4) == 0;
    p.safeZ = 5;
    if (I(0, 3) == 0) { p.tolerance.distance *= std::pow(10.0, U(-3, 1)); p.tolerance.chordal *= std::pow(10.0, U(-3, 1)); }
    if (getenv("BLOB")) {  // BLOB=1: repro-like scenes: µm Bezier blobs (about the distance tolerance) among small shapes, mostly Optimise
      cs.clear(); double off = I(0, 1) ? 0.0 : U(-600, 600);
      int nb = I(1, 8); for (int k = 0; k < nb; ++k) cs.push_back(bezBlob({off + U(0, 0.05), off * 0.7 + U(0, 0.05)}, U(0.0003, 0.003), I(4, 8)));
      int no = I(0, 4); for (int k = 0; k < no; ++k) cs.push_back(tiny({off + U(0, 0.05), off * 0.7 + U(0, 0.05)}, U(0.001, 0.01)));
      if (I(0, 1)) std::swap(cs.front(), cs.back());
      D = U(0.002, 0.02); p.passes.cutDepth = U(0.0002, 0.003); p.passes.passDepth = U(0.0005, 0.005); p.passes.startDepth = 0;
      p.allowance = I(0, 2) ? U(-0.00005, 0.00005) : 0.0; p.lastPassAllowance = U(0.0001, 0.001);
      p.leads.length = U(0.0002, 0.002); p.leads.radius = U(0.0002, 0.002); p.leads.overcut = I(0, 1) ? U(0, 0.001) : 0.0;
      p.tabs.length = U(0.0002, 0.002); p.tabs.thickness = U(0.1, 0.9) * p.passes.cutDepth; p.tabs.distance = U(0.001, 0.01); p.ramp.distance = U(0.0005, 0.005);
      if (I(0, 3)) p.startAt = StartAt::Optimise; if (I(0, 1)) p.side = ProfileSide::On; p.tolerance = geometry::Tolerance::defaults();
      p.startPosition = {off, off * 0.7};
    }
    tooling::Tool tool = useV ? vbit(D, vang) : endMill(D);
    char tag[64]; std::snprintf(tag, sizeof tag, "e%lu:%d", seed, sc);
    if (getenv("TARGET") && atoi(getenv("TARGET")) != sc) continue;
    if (getenv("DUMP")) { std::string f = std::string(getenv("DUMP")) + "/" + tag + ".scn"; for (auto& ch : f) if (ch == ':') ch = '_';
      if (p.tolerance == geometry::Tolerance::defaults()) writeScene(f.c_str(), cs, p, tool.diameter, useV ? (int)vang : 0); }
    std::fflush(stdout);
    int fd[2]; if (pipe(fd)) return 2;
    pid_t pid = fork();
    if (pid == 0) {
      close(fd[0]);
      if (getenv("CHILD_TIMEOUT")) alarm((unsigned)atoi(getenv("CHILD_TIMEOUT")));  // a slow scene ends as TIMEOUT (signal 14)
      auto res = profileToolpath(cs, tool, p);
      H h; h.i((long long)res.toolpath.moves.size());
      for (auto& m : res.toolpath.moves) { h.i((int)m.kind); h.d(m.to.x); h.d(m.to.y); h.d(m.to.z); h.d(m.centre.x); h.d(m.centre.y); h.i(m.feedOverride.has_value()); h.d(m.feedOverride.value_or(0)); h.d(m.dwellSeconds); }
      for (auto& d : res.diagnostics) { h.i((int)d.issue); h.i(d.contour ? (long long)*d.contour : -1); h.s(d.message); }
      bool fin = true; for (auto& m : res.toolpath.moves) fin = fin && std::isfinite(m.to.x) && std::isfinite(m.to.y) && std::isfinite(m.to.z);
      char buf[256]; int n = std::snprintf(buf, sizeof buf, "moves %zu diags %zu finite %d hash %016llx", res.toolpath.moves.size(), res.diagnostics.size(), (int)fin, h.h);
      (void)!write(fd[1], buf, (size_t)n); close(fd[1]); _exit(0);
    }
    close(fd[1]); char buf[256] = {0}; ssize_t got = 0, k; while ((k = read(fd[0], buf + got, sizeof buf - 1 - (size_t)got)) > 0) got += k; close(fd[0]);
    int st = 0; waitpid(pid, &st, 0);
    int sig = WIFSIGNALED(st) ? WTERMSIG(st) : (WIFEXITED(st) ? WEXITSTATUS(st) : -1);
    std::printf("%s S %g side %d sep %d tabs %d lead %d lo %d st %d V %d %s %d %s\n", tag, S, (int)p.side, (int)p.separateLastPass, (int)p.tabs.enabled, (int)p.leads.type, (int)p.leads.leadOut, (int)p.startAt, (int)useV, sig == 14 ? "to" : "rc", sig, got > 0 ? buf : (sig == 14 ? "TIMEOUT" : "CRASH"));
  }
}
