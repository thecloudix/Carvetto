#include <cstring>
// Brute-force confirmation of an xk coverage miss (no grids): re-run the generator seed, take the miss
// point q reported by xk, take the input vertex V nearest to q at distance ~d, and walk a fan of rays from
// V: for each direction find the first point exactly d from the input (bisection), and report how much of
// that true boundary is farther than tol from the output (brute-force distances).
#include "v9lib.cpp"
#include "xchk.hpp"
int main(int argc, char** argv) {
    std::string gen = argv[1]; std::uint64_t seed = std::strtoull(argv[2], nullptr, 0);
    if (gen != "v9") { std::printf("v9 only\n"); return 1; }
    C9 cc = make9(seed); Case& c = cc.c;
    auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    Contour ck = polyOf(c.in, c.tol);
    xk::Res r; xk::Checker X(ck, out, c.d, c.side, c.join, c.tol, c.mitre); X.run(r);
    double qx, qy; const char* at = std::strstr(r.covFirst.c_str(), "at (");
    if (!at || std::sscanf(at, "at (%lf,%lf)", &qx, &qy) != 2) { std::printf("no xk miss: %s\n", r.first.c_str()); return 0; }
    Vec2 q{qx, qy};
    std::vector<xk::S> I, O; for (auto& s : ck.spans) if (length(s) > 0) I.push_back(xk::mk(s)); for (auto& k : out) for (auto& s : k.spans) O.push_back(xk::mk(s));
    auto fI = [&](Vec2 p) { double b = 1e300; for (auto& g : I) b = std::min(b, xk::dPt(g, p)); return b; };
    auto fO = [&](Vec2 p) { double b = 1e300; for (auto& g : O) b = std::min(b, xk::dPt(g, p)); return b; };
    auto out2 = offsetOpenContour(c.in, c.d, c.side == OffsetSide::Left ? OffsetSide::Right : OffsetSide::Left, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    std::vector<xk::S> O2; for (auto& k : out2) for (auto& s : k.spans) O2.push_back(xk::mk(s));
    auto fO2 = [&](Vec2 p) { double b = 1e300; for (auto& g : O2) b = std::min(b, xk::dPt(g, p)); return b; };
    int both = 0;
    Vec2 V = I[0].a; double bd = 1e300; for (auto& g : I) for (Vec2 v : {g.a, g.b}) { double e = std::fabs(distance(v, q) - c.d); if (e < bd) { bd = e; V = v; } }
    double an0 = (q - V).angle(); int miss = 0, pts = 0; double worst = 0, arcMiss = 0, prevMissAng = 1e9;
    const int n = 400; const double span = 1.0;
    for (int k = 0; k <= n; ++k) {
        double an = an0 - span / 2 + span * k / n; Vec2 u{std::cos(an), std::sin(an)};
        double lo = 0, hi = 3 * c.d; if (fI(V + u * hi) < c.d) continue;
        for (int it = 0; it < 100; ++it) { double m = (lo + hi) / 2; if (fI(V + u * m) < c.d) lo = m; else hi = m; }
        Vec2 p = V + u * hi; if (std::fabs(hi - c.d) > 1e-9 * std::max(1.0, c.d)) continue;  // only boundary points on V's own circle
        pts++; double o = fO(p); if (o > c.tol) { both += fO2(p) > c.tol; miss++; worst = std::max(worst, o); arcMiss += c.d * span / n; }
    }
    std::printf("%s seed 0x%llx %s side %d d=%.6g closed %d: q dist-in %.12g (d %.12g) dist-out %.4g | fan about V: %d boundary pts on V's circle, %d farther than tol (arc %.4g mm, worst %.4g), %d of them also not covered by the other side's offset\n",
        gen.c_str(), (unsigned long long)seed, jn(c.join), (int)c.side, c.d, ck.closed, fI(q), c.d, fO(q), pts, miss, arcMiss, worst, both);
}
