// T-0016 verifier AC2: branch vs main (namespaced) on the T-0006 fuzz families and later adversarial families,
// every join x side; bit identity; otherwise exact checker (xk) + vchk metrics for both and a two-way
// sampled distance between the outputs (brute force, exact span distance).
// usage: cmp6 <fam> <N> <seed0>    fam: 0 makeCase (T-0006), 2 makeCase2, 3 makeCase3, 4 makeCase4, 5 makeCase5,
//        20 dense zig-zag / hatch family (this verifier), 21 same with jitter / rotation / translation
#include "v9lib.cpp"
#include "xchk.hpp"
static void dumpIn(const Contour& k, FILE* f) {
    std::fprintf(f, "Contour in; in.closed = %s;\n", k.closed ? "true" : "false");
    for (auto& sp : k.spans) if (auto* l = std::get_if<LineSpan>(&sp)) std::fprintf(f, "in.spans.emplace_back(LineSpan{{%.17g, %.17g}, {%.17g, %.17g}});\n", l->start.x, l->start.y, l->end.x, l->end.y);
}
using carvetto::geometry::testing::distanceToSpanExact;
static double sideDist(const std::vector<Contour>& A, const std::vector<Contour>& B, Vec2& at) {
    double worst = 0;
    for (auto& c : A) for (auto& s : c.spans) for (int k = 0; k <= 16; ++k) {
        Vec2 q = pointAt(s, k / 16.0); double best = 1e300;
        for (auto& c2 : B) for (auto& s2 : c2.spans) { best = std::min(best, distanceToSpanExact(q, s2)); if (best == 0) break; }
        if (best > worst) { worst = best; at = q; }
    }
    return worst;
}
Case makeZ(std::uint64_t seed, bool noisy) {
    Rng g(seed * 0x9E3779B97F4A7C15ULL + 0x16);
    Case c; c.tol = 0.001; c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 6.0);
    c.join = (JoinType)g.irange(0, 2); c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    int kind = g.irange(0, 4); int n = g.irange(6, 120);
    double p = std::exp(g.range(std::log(0.005), std::log(0.5))), a = p * std::exp(g.range(std::log(0.5), std::log(100.0)));
    double jy = noisy ? std::pow(10.0, g.range(-14, -6)) : 0.0;
    std::vector<Vec2> pts{{0, 0}};
    if (kind == 0) { for (int i = 1; i <= n; ++i) pts.push_back({i * p + (noisy ? g.range(-jy, jy) : 0), (i % 2) * a + (noisy ? g.range(-jy, jy) : 0)}); }
    else if (kind == 1) { for (int i = 1; i <= n; ++i) pts.push_back({(i / 2) * p, (i % 2) * a + (noisy ? g.range(-jy, jy) : 0)}); }  // sawtooth (vertical drops)
    else if (kind == 2) { for (int i = 0; i < n / 2; ++i) { pts.push_back({(i % 2) ? 0.0 : a, i * p}); pts.push_back({(i % 2) ? 0.0 : a, (i + 1) * p}); } }  // boustrophedon hatch
    else if (kind == 3) { double amp2 = a * g.range(0.2, 1.0); for (int i = 1; i <= n; ++i) pts.push_back({i * p, (i % 2) * ((i % 4 == 1) ? a : amp2)}); }  // two amplitudes
    else { for (int i = 1; i <= n; ++i) pts.push_back({i * p * g.range(0.7, 1.3), (i % 2) * a * g.range(0.9, 1.1)}); }  // irregular
    bool closed = g.uni() < 0.25;
    if (closed && kind != 2) { Vec2 l = pts.back(); double B = -g.range(0.1, 3.0) * a; pts.push_back({l.x, B}); pts.push_back({0, B}); }
    c.in = fromPts(pts, closed);
    if (closed && kind == 2) c.in.closed = false;
    double dm = std::exp(g.range(std::log(0.3), std::log(40.0)));
    c.d = std::max(p, std::min(a, 50.0)) * dm * 0.5;
    if (c.d > 60) c.d = 60;
    double u = g.uni();
    if (noisy || u < 0.3) transformC(c.in, g.uni() < 0.5 ? 0.0 : g.range(-kPi, kPi), g.uni() < 0.5 ? Vec2{0, 0} : Vec2{g.range(-1000, 1000), g.range(-1000, 1000)});
    return c;
}
// 22: zig-zag with a random subset of vertices jittered by 1e-9..1e-5 (copies on nearly, not exactly, one carrier:
//     mergeOnCarriers groups them by `lateral`, pruneLineCopies only by `overlapping`).
// 23: S-hatch: zig-zag forwards, then back above it so its valley chamfers lie on the forward peak chamfer line
//     (copies running opposite ways), the gap perturbed by 0 / ulps / 1e-12 / 1e-9 / 1e-6.
Case makeZ2(std::uint64_t seed, int fam) {
    Rng g(seed * 0xD1B54A32D192ED03ULL + 0x22);
    Case c; c.tol = 0.001; c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 6.0);
    c.join = (JoinType)g.irange(0, 2); c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    int n = g.irange(6, 80);
    double p = std::exp(g.range(std::log(0.005), std::log(0.5))), a = p * std::exp(g.range(std::log(0.5), std::log(60.0)));
    c.d = std::min(60.0, std::max(p, a) * std::exp(g.range(std::log(0.3), std::log(20.0))) * 0.5);
    std::vector<Vec2> pts{{0, 0}};
    if (fam == 26) {  // every vertex jittered by 1e-12..1e-9 (copies tilted within sameCarrier: carriers cross inside stretches), d >> pitch
        c.d = std::min(60.0, std::max(p, a) * std::exp(g.range(std::log(2.0), std::log(30.0))));
        double jy = std::pow(10.0, g.range(-12, -9)) * std::max(1.0, a);
        for (int i = 1; i <= n; ++i) pts.push_back({i * p + g.range(-jy, jy), (i % 2) * a + g.range(-jy, jy)});
    } else if (fam == 25) {  // zig-zag with some taller or deeper teeth (their neighbourhoods and corner regions cut the shared chamfer line)
        double frac = g.range(0.02, 0.3);
        for (int i = 1; i <= n; ++i) { Vec2 q{i * p, (i % 2) * a}; if (g.uni() < frac) q.y += (i % 2 ? 1 : -1) * g.range(0.0, 2.0 * c.d) * (g.uni() < 0.5 ? 1 : -1); pts.push_back(q); }
    } else if (fam == 24) {  // irregular pitch, one amplitude (chamfers of different lengths on one line), subset tilted by jitter
        double frac = g.range(0.05, 0.5), jy = std::pow(10.0, g.range(-8, -5)); double x = 0;
        for (int i = 1; i <= n; ++i) { x += p * g.range(0.3, 3.0); Vec2 q{x, (i % 2) * a}; if (g.uni() < frac) { q.y += g.range(-jy, jy); q.x += g.range(-jy, jy) * p; } pts.push_back(q); }
    } else if (fam == 22) {
        double frac = g.range(0.05, 0.6), jy = std::pow(10.0, g.range(-9, -5));
        for (int i = 1; i <= n; ++i) { Vec2 q{i * p, (i % 2) * a}; if (g.uni() < frac) { q.y += g.range(-jy, jy); if (g.uni() < 0.5) q.x += g.range(-jy, jy); } pts.push_back(q); }
    } else {
        for (int i = 1; i <= n; ++i) pts.push_back({i * p, (i % 2) * a});
        double k = g.uni() < 0.5 ? 1.0 : c.mitre;
        double pert[] = {0, 2e-16, -2e-16, 1e-15, -1e-15, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6};
        double Y0 = a + 2 * c.d * k + pert[g.irange(0, 10)] * std::max(1.0, a + 2 * c.d);
        c.side = OffsetSide::Left;  // forward peaks' chamfers on the left at a + d; the return runs -x, its left is -y
        double xr = pts.back().x + p * g.range(0.0, 2.0); int m = g.irange(4, n + 4);
        double sh = g.uni() < 0.5 ? 0.0 : p * g.range(0, 1);
        for (int i = 0; i <= m; ++i) pts.push_back({xr - i * p - sh * (i > 0), Y0 + (i % 2) * a});
        if (g.uni() < 0.3) c.side = OffsetSide::Right;
    }
    c.in = fromPts(pts, false);
    if (g.uni() < 0.3) transformC(c.in, g.range(-kPi, kPi), Vec2{g.range(-1000, 1000), g.range(-1000, 1000)});
    return c;
}
// r2 verifier white-box: branch-only run on the W lib (offset_W.cpp prints WVIOL / WADD for every stray put back that is
// neither within d + 5e-7 max(1,d) of the input nor in a corner region). Extra fam 90: adversarial loop strands
// (needles / thin rectangles / near-tangent arcs, Sharp/Square/mixed, far coordinates).
static Case makeAdv(std::uint64_t seed) {
    Rng g(seed * 0x9E3779B97F4A7C15ull + 7); Case c; c.tol = 1e-3; c.mitre = g.range(1.0, 4.0);
    double d = std::pow(10.0, g.range(-1.2, 0.7)); c.d = d;
    double far = std::array<double, 4>{0.0, 1.2e4, 1e5, 3.3e5}[g.next() % 4];
    double ang = g.range(0, 2 * kPi); Vec2 u{std::cos(ang), std::sin(ang)}, v{-u.y, u.x};
    auto P = [&](double x, double y) { return Vec2{far, -far * 0.7} + u * x + v * y; };
    int kind = int(g.next() % 5);
    double w = std::array<double, 6>{0.0, 1e-12, 1e-9, 1e-7, 1e-6, 5e-5}[g.next() % 6] * (g.uni() < 0.5 ? 1 : g.range(0.5, 2));
    double L = std::pow(10.0, g.range(-2, 1));
    Contour in;
    if (kind == 0) {  // closed thin rectangle (2d + L) x (2d + w), possibly with a tilted / jagged long wall
        std::vector<Vec2> pts{P(0, 0), P(2 * d + L, g.range(-w, w)), P(2 * d + L, 2 * d + w), P(0, 2 * d + w + g.range(-w, w))};
        in = fromPts(pts, true);
    } else if (kind == 1) {  // U channel (open) with a slanted end wall and a jog
        double s = g.range(-0.5, 0.5) * d;
        std::vector<Vec2> pts{P(-1, 0), P(L + d, 0), P(L + d + s, 2 * d + w), P(-1, 2 * d + w)};
        in = fromPts(pts, false);
    } else if (kind == 2) {  // arc channel: two concentric arcs (radius R, R + 2d + w) joined by a line/arc end
        double R = std::pow(10.0, g.range(-0.5, 2)), sw = g.range(0.05, 3.0);
        Vec2 C = P(0, 0); double a0 = ang;
        ArcSpan inner{C, R, a0, a0 + sw, ArcDirection::CCW};
        ArcSpan outer{C, R + 2 * d + w, a0 + sw, a0, ArcDirection::CW};
        in.spans.emplace_back(LineSpan{C + Vec2{std::cos(a0), std::sin(a0)} * (R + 2 * d + w), inner.startPoint()});
        in.spans.emplace_back(inner);
        in.spans.emplace_back(LineSpan{inner.endPoint(), outer.startPoint()});
        in.spans.emplace_back(outer);
        in.closed = g.uni() < 0.5;
        if (!in.closed) in.spans.erase(in.spans.begin());
    } else if (kind == 3) {  // near-tangent arcs: an arc wall of huge radius facing a straight wall 2d + w away
        double R = std::pow(10.0, g.range(2, 6));
        double half = std::min(L, 0.5 * R);
        double th = half / R;
        Vec2 C = P(0, 2 * d + w + R);
        ArcSpan top{C, R, -kPi / 2 - th + ang, -kPi / 2 + th + ang, ArcDirection::CCW};
        // straight wall below the arc's lowest point
        Vec2 a = P(-half, 0), b = P(half, 0);
        in.spans.emplace_back(LineSpan{a, b});
        in.spans.emplace_back(LineSpan{b, top.endPoint()});
        ArcSpan back{C, R, top.endAngle, top.startAngle, ArcDirection::CW};
        in.spans.emplace_back(back);
        in.spans.emplace_back(LineSpan{back.endPoint(), a});
        in.closed = false;
        in.spans.pop_back();
    } else {  // needle zig-zag: teeth exactly 2d (+w) wide, depth L, random Sharp corners, mixed with arcs at the tips
        std::vector<Vec2> pts{P(0, 5)}; double x = 0; int nt = 1 + int(g.next() % 6);
        for (int i = 0; i < nt; ++i) {
            double W = 2 * d + w * g.range(-1, 1);
            pts.push_back(P(x + 1, 5)); pts.push_back(P(x + 1 + g.range(-1e-4, 1e-4), 5 - L - d)); pts.push_back(P(x + 1 + W, 5 - L - d + g.range(-1e-3, 1e-3) * d)); pts.push_back(P(x + 1 + W, 5)); x += 1 + W;
        }
        pts.push_back(P(x + 1, 5));
        in = fromPts(pts, false);
    }
    c.in = in; return c;
}
int main(int argc, char** argv) {
    int fam = atoi(argv[1]); long N = atol(argv[2]); std::uint64_t s0 = strtoull(argv[3], nullptr, 0);
    long n = 0;
    for (long i = 0; i < N; ++i) {
        std::uint64_t seed = s0 + i; Case c;
        if (fam == 0) c = makeCase(seed); else if (fam == 2) c = makeCase2(seed).c; else if (fam == 3) c = makeCase3(seed).c;
        else if (fam == 4) c = makeCase4(seed).c; else if (fam == 5) c = makeCase5(seed).c; else if (fam == 90) c = makeAdv(seed);
        else if (fam >= 22) c = makeZ2(seed, fam); else c = makeZ(seed, fam == 21);
        if (c.in.spans.empty() || hasBez(c.in)) continue;
        for (int j = 0; j < 3; ++j) for (int sd = 0; sd < 2; ++sd) {
            n++;
            std::fprintf(stderr, "CASE fam %d seed 0x%llx join %d side %d\n", fam, (unsigned long long)seed, j, sd);
            auto ob = offsetOpenContour(c.in, c.d, (OffsetSide)sd, (JoinType)j, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        }
    }
    std::printf("DONE fam %d N %ld offsets %ld\n", fam, N, n);
}
