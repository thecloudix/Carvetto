// T-0142 verifier (e): OpenPathClipper (strip-cropped clip) vs booleanOp on the whole clip.
// usage: strip seed N   (env VERB=1 prints every mismatch)
// Clip families (all >= 1024 edges so the crop is active): 0 comb along v (every line crosses every
// tooth), 1 rectangular spiral, 2 random stars both orientations (crossing), 3 grid of squares aligned to the raster
// pitch (edges collinear with raster lines / strip borders), 4 serpentine ring crossing each strip many times,
// 5 edges ending just outside the strip (+-1..3 grid units of the 2-unit margin).
// Subjects: Pocket-like batches of parallel lines (angle 0/30/45/90/random), random open zigzags.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
using namespace carvetto::geometry;
using Ring = std::vector<Vec2>;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static Contour poly(const Ring& p, bool closed = true) { Contour c; c.closed = closed; size_t n = closed ? p.size() : p.size() - 1;
  for (size_t i = 0; i < n; ++i) c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]}); return c; }
static double segDist(Vec2 p, Vec2 a, Vec2 b) { Vec2 d = b - a; double l = d.dot(d); double t = l > 0 ? std::clamp((p - a).dot(d) / l, 0.0, 1.0) : 0; return distance(p, a + d * t); }
static double setDist(const std::vector<Contour>& cs, Vec2 p) { double b = 1e300; for (auto& c : cs) for (auto& s : c.spans) b = std::min(b, segDist(p, startPoint(s), endPoint(s))); return b; }
static double len(const Contour& c) { double L = 0; for (auto& s : c.spans) L += distance(startPoint(s), endPoint(s)); return L; }

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IOLBF, 0); unsigned long seed = strtoul(argv[1], 0, 10); int N = atoi(argv[2]); bool verb = getenv("VERB");
  long realEnds = 0, illEnds = 0; long calls = 0, mism = 0, cntMism = 0, maxEnd = 0; double worstEnd = 0, worstLen = 0; long fam[6] = {0}, famBad[6] = {0};
  for (int sc = 0; sc < N; ++sc) {
    if (getenv("SC") && sc != atoi(getenv("SC"))) continue;
    rng.seed(seed * 7919 + sc);
    int family = I(0, 5);
    int angSel = I(0, 4); double deg = angSel == 0 ? 0 : angSel == 1 ? 30 : angSel == 2 ? 45 : angSel == 3 ? 90 : U(0, 180);
    double a = deg * M_PI / 180; Vec2 u{std::cos(a), std::sin(a)}; Vec2 v = u.perp();
    // local frame (s along u, t along v) -> world
    auto W = [&](double s, double t) { Vec2 p = u * s + v * t; if (angSel == 0 || angSel == 3) { p.x = std::round(p.x * 1e6) / 1e6; p.y = std::round(p.y * 1e6) / 1e6; } return p; };
    std::vector<Ring> rings; double pitch = U(0.05, 0.4);
    double tmin = 0, tmax = 0;
    if (family == 0) { int teeth = I(260, 700); Ring r; for (int i = 0; i < teeth; ++i) { r.push_back(W(0, i)); r.push_back(W(20, i)); r.push_back(W(20, i + 0.5)); r.push_back(W(0, i + 0.5)); }
      r.push_back(W(-5, teeth)); r.push_back(W(-5, 0)); std::reverse(r.begin(), r.end()); if (I(0, 1)) std::reverse(r.begin(), r.end()); rings.push_back(r); tmin = 0; tmax = teeth; }
    else if (family == 1) { // rectangular spiral band (thick polyline closed back)
      Ring out, in; double w = 0.3; int turns = I(520, 800); double x0 = 0, y0 = 0, step = 0.8; int dir = 0; double L = step;
      Vec2 p{0, 0}; std::vector<Vec2> pts{p};
      for (int k = 0; k < turns; ++k) { Vec2 d = dir == 0 ? Vec2{1, 0} : dir == 1 ? Vec2{0, 1} : dir == 2 ? Vec2{-1, 0} : Vec2{0, -1}; p = p + d * L; pts.push_back(p); dir = (dir + 1) % 4; if (dir % 2 == 0) L += step; }
      for (auto& q : pts) out.push_back(W(q.x, q.y)); for (int k = (int)pts.size() - 1; k >= 0; --k) in.push_back(W(pts[k].x + w, pts[k].y + w));
      Ring r = out; r.insert(r.end(), in.begin(), in.end()); rings.push_back(r); tmin = -L; tmax = L; (void)x0; (void)y0; }
    else if (family == 2) { int ns = I(6, 30); int tot = 0; while (tot < 1200) { for (int k = 0; k < ns; ++k) { Vec2 c{U(0, 60), U(0, 60)}; double R = U(2, 20); int nv = I(20, 200); Ring r; double ph = U(0, 6.3);
          for (int j = 0; j < nv; ++j) { double an = ph + 6.283185307179586 * j / nv, rr = R * U(0.5, 1.0); r.push_back(W(c.x + rr * std::cos(an), c.y + rr * std::sin(an))); }
          if (I(0, 2) == 0) std::reverse(r.begin(), r.end()); rings.push_back(r); tot += nv; } } tmin = 0; tmax = 60; }
    else if (family == 3) { int nxs = I(18, 30), nys = I(18, 30); for (int i = 0; i < nxs; ++i) for (int j = 0; j < nys; ++j) { double s0 = i * 2.0, t0 = j * pitch * I(1, 4), h = pitch * I(1, 3);
          Ring r{W(s0, t0), W(s0 + 1.5, t0), W(s0 + 1.5, t0 + h), W(s0, t0 + h)}; if (I(0, 3) == 0) std::reverse(r.begin(), r.end()); rings.push_back(r); tmax = std::max(tmax, t0 + h); } tmin = 0; }
    else if (family == 4) { // serpentine closed ring: zigzag in t, going back and forth in s
      int m = I(520, 900); Ring r; double amp = 30; for (int i = 0; i < m; ++i) { double t = i * 0.11; r.push_back(W(i % 2 ? amp : 0, t)); r.push_back(W(i % 2 ? amp : 0, t + 0.05)); }
      std::vector<Vec2> back; for (int i = m - 1; i >= 0; --i) { double t = i * 0.11 + 0.025; back.push_back(W(i % 2 ? amp - 1 : 1, t)); }
      // make a simple ring: outer along zigzag, return along a straight line far left
      r.push_back(W(-3, m * 0.11 + 1)); r.push_back(W(-3, -1)); rings.push_back(r); tmin = -1; tmax = m * 0.11 + 1; }
    else { // squares whose edges end just outside the strip border (only exact in axis frames)
      angSel = I(0, 1) ? 0 : 3; deg = angSel == 0 ? 0 : 90; a = deg * M_PI / 180; u = {std::cos(a), std::sin(a)}; v = u.perp();
      for (int k = 0; k < 400; ++k) { double t0 = std::round(U(0, 40) * 1e6) / 1e6, h = std::round(U(0.01, 0.2) * 1e6) / 1e6, s0 = U(0, 30);
        Ring r{W(s0, t0), W(s0 + 1, t0), W(s0 + 1, t0 + h), W(s0, t0 + h)}; if (I(0, 3) == 0) std::reverse(r.begin(), r.end()); rings.push_back(r); }
      tmin = 0; tmax = 40; pitch = 0.1; }
    fam[family]++;
    std::vector<Contour> clip; size_t edges = 0; for (auto& r : rings) { clip.push_back(poly(r)); edges += r.size(); }
    FillRule fill = I(0, 1) ? FillRule::EvenOdd : FillRule::NonZero;
    Tolerance tol = Tolerance::defaults();
    OpenPathClipper clipper(clip, v, tol, fill);
    // smin/smax of clip in frame
    double smin = 1e300, smax = -1e300; for (auto& r : rings) for (auto& p : r) { smin = std::min(smin, p.dot(u)); smax = std::max(smax, p.dot(u)); tmin = std::min(tmin, p.dot(v)); tmax = std::max(tmax, p.dot(v)); }
    int batches = I(3, 8); bool bad = false;
    for (int b = 0; b < batches; ++b) {
      std::vector<Contour> subj; int kind = I(0, 3);
      double t0 = U(tmin - 1, tmax);
      if (family == 3 && I(0, 1)) t0 = std::floor(t0 / pitch) * pitch;  // lines exactly on square edges
      if (family == 5) { // strip border exactly at an edge end +- k grid units from margin
        int k = I(0, (int)rings.size() - 1); const Ring& r = rings[k]; double te = r[0].dot(v); int g = I(-4, 4); t0 = te + 2e-6 + g * 1e-6; if (I(0, 1)) t0 = r[2].dot(v) - 2e-6 - g * 1e-6 - pitch * 255; }
      if (kind <= 1) { int K = kind == 0 ? 256 : I(1, 40); for (int k = 0; k < K; ++k) { double t = t0 + k * pitch; Contour c; c.append(LineSpan{W(smin - 1, t), W(smax + 1, t)}); subj.push_back(c); }
        if (family == 5 && kind == 1) { // reverse direction: strip upper border near an edge's low end
          subj.clear(); int k2 = I(0, (int)rings.size() - 1); double te = rings[k2][0].dot(v) - 2e-6 - I(-3, 3) * 1e-6; for (int k = 0; k < K; ++k) { Contour c; double t = te - k * pitch; c.append(LineSpan{W(smin - 1, t), W(smax + 1, t)}); subj.push_back(c); } } }
      else if (kind == 2) { int K = I(1, 20); for (int k = 0; k < K; ++k) { Ring z; int m = I(2, 12); for (int j = 0; j < m; ++j) z.push_back(W(U(smin - 1, smax + 1), t0 + U(0, 3))); subj.push_back(poly(z, false)); } }
      else { // a single line along the exact edge of the strip border (collinear with a clip edge) in axis frames
        int k = I(0, (int)rings.size() - 1); const Ring& r = rings[k]; int e = I(0, (int)r.size() - 1); Vec2 p = r[e], q = r[(e + 1) % r.size()];
        Contour c; c.append(LineSpan{p + (q - p) * -0.5, p + (q - p) * 1.5}); subj.push_back(c); Contour c2; c2.append(LineSpan{W(smin - 1, p.dot(v) + 0.01), W(smax + 1, p.dot(v) + 0.01)}); subj.push_back(c2); }
      for (int op = 0; op < 4; ++op) {
        BooleanOp bop = (BooleanOp)op;
        std::vector<Contour> got = clipper.clip(bop, subj);
        std::vector<Contour> want = booleanOp(bop, subj, clip, tol, fill).open;
        ++calls;
        double Lg = 0, Lw = 0; for (auto& c : got) Lg += len(c); for (auto& c : want) Lw += len(c);
        // match pieces by end points
        std::vector<char> used(want.size(), 0); size_t matched = 0; double wend = 0;
        for (auto& g : got) { double bestd = 1e300; size_t bi = 0;
          for (size_t i = 0; i < want.size(); ++i) if (!used[i]) { double d = std::max(distance(g.startPoint(), want[i].startPoint()), distance(g.endPoint(), want[i].endPoint()));
              double d2 = std::max(distance(g.startPoint(), want[i].endPoint()), distance(g.endPoint(), want[i].startPoint())); d = std::min(d, d2); if (d < bestd) { bestd = d; bi = i; } }
          if (bestd <= 2.01e-6) { used[bi] = 1; ++matched; wend = std::max(wend, bestd); } }
        worstEnd = std::max(worstEnd, wend); worstLen = std::max(worstLen, std::fabs(Lg - Lw));
        bool cm = got.size() != want.size(); bool m = cm || matched != got.size() || std::fabs(Lg - Lw) > 1e-5;
        if (m) { ++mism; cntMism += cm; bad = true;
          if (getenv("DIAG")) { // sample only the unmatched pieces against the other result
            std::vector<char> ug(got.size(), 0); { std::vector<char> uw(want.size(), 0);
              for (size_t gi = 0; gi < got.size(); ++gi) for (size_t i = 0; i < want.size(); ++i) if (!uw[i]) { auto& g = got[gi]; double d = std::max(distance(g.startPoint(), want[i].startPoint()), distance(g.endPoint(), want[i].endPoint()));
                  double d2 = std::max(distance(g.startPoint(), want[i].endPoint()), distance(g.endPoint(), want[i].startPoint())); if (std::min(d, d2) <= 2.01e-6) { uw[i] = 1; ug[gi] = 1; break; } }
              std::vector<Contour> G2, W2; for (size_t i = 0; i < got.size(); ++i) if (!ug[i]) G2.push_back(got[i]); for (size_t i = 0; i < want.size(); ++i) if (!uw[i]) W2.push_back(want[i]);
              double maxEdge = 0; double dis = 0; double maxOff = 0; Vec2 first{1e9, 1e9};
              auto scan = [&](const std::vector<Contour>& A, const std::vector<Contour>& Bset) { for (auto& c : A) for (auto& sp : c.spans) { Vec2 p0 = startPoint(sp), p1 = endPoint(sp); int m = std::max(1, (int)std::ceil(distance(p0, p1) / 0.0005));
                  for (int k = 0; k <= m; ++k) { Vec2 p = p0 + (p1 - p0) * (double(k) / m); double off = setDist(Bset, p); if (off < 1e-7) continue; maxOff = std::max(maxOff, off);
                    double de = setDist(clip, p); if (first.x == 1e9) first = p; dis += distance(p0, p1) / m; maxEdge = std::max(maxEdge, de); } } };
              scan(G2, want); scan(W2, got);
              std::printf("  DIAG unmatched got %zu want %zu; disagreeing length ~%.5f mm, max dist of such points to a clip edge %.3g mm, first %.7f %.7f\n", G2.size(), W2.size(), dis, maxEdge, first.x, first.y);
              // exact reference for each unmatched end: intersection of its subject line with the nearest clip edge (grid-rounded inputs, long double)
              auto snap = [](Vec2 p) { return Vec2{std::round(p.x * 1e6), std::round(p.y * 1e6)}; };
              auto exactErr = [&](Vec2 e) { // clip edges within 1e-5 mm of the end: smallest sine to the subject; error vs the best-conditioned edge
                double bestS = 1e300; Vec2 sa, sb; for (auto& sj : subj) for (auto& sp : sj.spans) { double ds = segDist(e, startPoint(sp), endPoint(sp)); if (ds < bestS) { bestS = ds; sa = startPoint(sp); sb = endPoint(sp); } }
                Vec2 C = snap(sa), D = snap(sb); double minSin = 1e9, maxSin = -1, err = -1; int near = 0;
                for (size_t ri = 0; ri < rings.size(); ++ri) for (size_t j = 0; j < rings[ri].size(); ++j) { Vec2 p = rings[ri][j], q = rings[ri][(j + 1) % rings[ri].size()]; if (segDist(e, p, q) > 1e-5) continue; ++near;
                  Vec2 A = snap(p), B = snap(q); long double d1x = B.x - A.x, d1y = B.y - A.y, d2x = D.x - C.x, d2y = D.y - C.y; long double den = d1x * d2y - d1y * d2x;
                  double sn = (double)(std::fabs((double)den) / (std::hypot((double)d1x, (double)d1y) * std::hypot((double)d2x, (double)d2y))); minSin = std::min(minSin, sn);
                  if (sn > maxSin && den != 0) { maxSin = sn; long double t = ((C.x - A.x) * d2y - (C.y - A.y) * d2x) / den; long double ex = A.x + t * d1x, ey = A.y + t * d1y; err = std::hypot((double)(e.x * 1e6L - ex), (double)(e.y * 1e6L - ey)); } }
                bool ill = near == 0 || minSin < 1e-3; if (!ill && err > 2.0) ++realEnds; else if (ill) ++illEnds;
                std::printf("      end %.7f %.7f: near edges %d minSin %.3g maxSin %.3g err vs best-conditioned edge %.2f units %s\n", e.x, e.y, near, minSin, maxSin, err, ill ? "ILL" : (err > 2.0 ? "REAL" : "ok")); };
              for (auto& c : G2) { std::printf("    got  %.7f %.7f -> %.7f %.7f\n", c.startPoint().x, c.startPoint().y, c.endPoint().x, c.endPoint().y); exactErr(c.startPoint()); exactErr(c.endPoint()); }
              for (auto& c : W2) { exactErr(c.startPoint()); exactErr(c.endPoint()); }
              for (auto& c : W2) std::printf("    want %.7f %.7f -> %.7f %.7f\n", c.startPoint().x, c.startPoint().y, c.endPoint().x, c.endPoint().y); } }
          if (verb || mism <= 20) std::printf("MISMATCH seed %lu sc %d fam %d deg %.3f fill %d op %d kind %d edges %zu got %zu want %zu matched %zu len %.9f vs %.9f\n", seed, sc, family, deg, (int)fill, op, kind, edges, got.size(), want.size(), matched, Lg, Lw); }
      }
    }
    if (bad) famBad[family]++;
    std::printf("scene %d fam %d deg %.2f edges %zu bad %d\n", sc, family, deg, edges, (int)bad);
  }
  std::printf("seed %lu calls %ld mismatches %ld (count %ld) worstEnd %.3g mm worstLenDiff %.3g; fam scenes/bad:", seed, calls, mism, cntMism, worstEnd, worstLen);
  for (int f = 0; f < 6; ++f) std::printf(" %ld/%ld", fam[f], famBad[f]); std::printf(" ; unmatched ends: ill-conditioned %ld REAL %ld\n", illEnds, realEnds);
  return mism ? 1 : 0;
}
