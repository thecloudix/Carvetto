#define V3_NO_MAIN
#include "v3.cpp"
// Phantom-ring scan: in the Offset strategy, every ring other than the boundary rings is an inset of
// the centre region by >= min(stepover, r); a ring with a vertex much closer to the walls than that
// (while not being a boundary ring) is an inside-out phantom kept by the guard. Also checks every
// ring vertex is inside the pocket at >= r + margin (exact oracle).
int main(int argc, char** argv) {
    uint64_t seed0 = strtoull(argv[1], 0, 10); int n = atoi(argv[2]);
    long flagged = 0, runs = 0;
    for (int k = 0; k < n; ++k) {
        Rng R(seed0 + k);
        Scene sc; int kind = R.i(0, 3);
        if (kind == 0) { int kk = R.i(2, 9); double R0 = R.u(8, 30), a = R.u(0.3, 0.9); int np = kk * R.i(12, 40); std::vector<Vec2> p; double ph = R.u(0, 6.3); for (int i = 0; i < np; ++i) { double t = 2 * PI * i / np; double rr = R0 * (1 + a * std::cos(kk * t + ph)); p.push_back({rr * std::cos(t), rr * std::sin(t)}); } sc.addPoly(p); sc.desc = "flower"; }
        else if (kind == 1) { int kk = R.i(3, 16); double ro = R.u(12, 35), ri = ro * R.u(0.1, 0.6); std::vector<Vec2> p; for (int i = 0; i < 2 * kk; ++i) { double t = PI * i / kk; p.push_back(Vec2{std::cos(t), std::sin(t)} * ((i % 2 ? ri : ro) * R.u(0.85, 1.0))); } sc.addPoly(p); sc.desc = "star"; }
        else if (kind == 2) { sc.addPoly(starPoly(R, {0, 0}, R.u(3, 10), R.u(15, 35), R.i(5, 20), false)); sc.desc = "randstar"; }
        else { // discs union polygon
            int m = R.i(2, 5); std::vector<Vec2> c; std::vector<double> rad; Vec2 cur{0, 0};
            for (int i = 0; i < m; ++i) { double ri = R.u(4, 16); if (i) { double t = R.u(-PI, PI); cur = c[R.i(0, i - 1)] + Vec2{std::cos(t), std::sin(t)} * ((rad.back() + ri) * R.u(0.3, 0.9)); } c.push_back(cur); rad.push_back(ri); }
            Vec2 g{0, 0}; for (auto q : c) g = g + q; g = g * (1.0 / m);
            std::vector<Vec2> p; bool ok = true; for (int i = 0; i < 1440; ++i) { double t = 2 * PI * i / 1440; Vec2 u{std::cos(t), std::sin(t)}; double best = -1; for (int j = 0; j < m; ++j) { Vec2 f = g - c[j]; double B = f.dot(u), C = f.dot(f) - rad[j] * rad[j]; double disc = B * B - C; if (disc >= 0) best = std::max(best, -B + std::sqrt(disc)); } if (best <= 0) ok = false; p.push_back(g + u * best); }
            if (!ok) continue; sc.addPoly(p); sc.desc = "discs"; }
        if (!simpleScene(sc)) continue;
        for (int t = 0; t < 4; ++t) {
            double d = R.u(0.5, 20), so = d * R.u(0.1, 1.0);
            PocketParams p; p.cutDepth = 1; p.safeZ = 5; p.profilePass = ProfilePass(R.i(0, 2));
            if (R.i(0, 2) == 0) p.tolerance.chordal = R.u(0.002, 0.08);
            auto res = pocketToolpath(sc.contours, {tool(d, so, 1)}, p);
            ++runs;
            if (res.toolpaths[0].moves.empty()) continue;
            double r = d / 2, need = r + p.tolerance.chordal;
            double lim = std::min(so, r);
            for (auto& pa : res.plans[0].paths) {
                double mn = 1e9, mx = -1e9;
                for (auto q : pa.path.points) { double e = sc.sd(q) - need; mn = std::min(mn, e); mx = std::max(mx, e); }
                bool outside = mn < -p.tolerance.chordal - 1e-6; bool phantom = outside || (mx > 0.05 + p.tolerance.chordal && mn < lim - 0.05 - 2 * p.tolerance.chordal && mn > 0.02 + p.tolerance.chordal);
                if (phantom) {
                    // second opinion: boundary ring touching a cusp has mn ~ 0: only flag if no vertex near 0
                    ++flagged;
                    printf("seed %llu [%s] d=%.3f so=%.3f ch=%.4f pp=%d: ring n=%zu sd-need min %.4f max %.4f (limit %.3f)%s\n", (unsigned long long)(seed0 + k), sc.desc.c_str(), d, so, p.tolerance.chordal, int(p.profilePass), pa.path.points.size(), mn, mx, lim, outside ? " CLOSER THAN RADIUS" : " PHANTOM?");
                    break;
                }
            }
        }
    }
    printf("phantom scan: %ld runs, %ld flagged\n", runs, flagged);
}
