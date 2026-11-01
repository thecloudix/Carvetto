// Round-3 verifier driver for T-0121. Throwaway, not committed.
// Reuses the round-2 oracle/simulator; adds: effective-boundary oracle (even-odd: only edge
// pieces with different parity on their two sides are walls), simplicity check for generated
// scenes, new generators (dense/touching island grids, nested, serpentine corridors, lobed
// flowers, dumbbell chains, deep stars, disc unions with cusps, skylines), a rest-air check
// (rest tools must not cut where no material is left), strict travel check for every strategy.
#define main v2_unused_main
#include "/tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0121v2/v2.cpp"
#undef main

// ----------------------------------------------------------------- effective boundary
static double crs(Vec2 u, Vec2 v) { return u.x * v.y - u.y * v.x; }
static Scene effective(const Scene& sc, int* removedOut = nullptr) {
    Scene e = sc;
    e.prims.clear();
    int removed = 0;
    for (size_t i = 0; i < sc.prims.size(); ++i) {
        const Prim& p = sc.prims[i];
        if (p.kind != 0) { e.prims.push_back(p); continue; }
        Vec2 a = p.a, b = p.b, d = b - a;
        double L2 = d.dot(d);
        if (L2 <= 0) continue;
        std::vector<double> ts{0.0, 1.0};
        for (size_t j = 0; j < sc.prims.size(); ++j) {
            if (j == i) continue;
            const Prim& q = sc.prims[j];
            if (q.kind == 0) {
                Vec2 d2 = q.b - q.a;
                double den = crs(d, d2);
                double off = std::fabs(crs(d, q.a - a)) / std::sqrt(L2);
                if (std::fabs(den) < 1e-12 * std::sqrt(L2 * d2.dot(d2))) {
                    if (off < 1e-9) {
                        for (Vec2 w : {q.a, q.b}) { double t = (w - a).dot(d) / L2; if (t > 0 && t < 1) ts.push_back(t); }
                    }
                } else {
                    double t = crs(q.a - a, d2) / den, u = crs(q.a - a, d) / den;
                    if (t > 0 && t < 1 && u >= -1e-12 && u <= 1 + 1e-12) ts.push_back(t);
                }
            } else {
                // line-circle
                Vec2 f = a - q.c;
                double A = L2, B = 2 * f.dot(d), C = f.dot(f) - q.R * q.R;
                double disc = B * B - 4 * A * C;
                if (disc > 0) for (double s : {-1.0, 1.0}) { double t = (-B + s * std::sqrt(disc)) / (2 * A); if (t > 0 && t < 1) ts.push_back(t); }
            }
        }
        std::sort(ts.begin(), ts.end());
        Vec2 n = Vec2{-d.y, d.x} * (1.0 / std::sqrt(L2));
        for (size_t k = 0; k + 1 < ts.size(); ++k) {
            if (ts[k + 1] - ts[k] < 1e-14) continue;
            Vec2 m = a + d * (0.5 * (ts[k] + ts[k + 1]));
            bool l = sc.inside(m + n * 1e-6), r = sc.inside(m - n * 1e-6);
            if (l != r) e.prims.push_back(Prim{0, a + d * ts[k], a + d * ts[k + 1], {}, 0, 0, 0});
            else ++removed;
        }
    }
    if (removedOut) *removedOut = removed;
    return e;
}

// ----------------------------------------------------------------- simplicity
static double primPrimDist(const Prim& p, const Prim& q) {
    auto pts = [](const Prim& x) {
        std::vector<Vec2> v;
        if (x.kind == 0) { v = {x.a, x.b}; }
        else { int N = 720; for (int k = 0; k <= N; ++k) v.push_back(arcPt(x, x.a0 + x.sw * k / N)); }
        return v;
    };
    if (p.kind == 0 && q.kind == 0) {
        Vec2 d1 = p.b - p.a, d2 = q.b - q.a;
        double den = crs(d1, d2);
        if (std::fabs(den) > 1e-18) {
            double t = crs(q.a - p.a, d2) / den, u = crs(q.a - p.a, d1) / den;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return 0.0;
        }
        return std::min({segDist(p.a, q.a, q.b), segDist(p.b, q.a, q.b), segDist(q.a, p.a, p.b), segDist(q.b, p.a, p.b)});
    }
    double best = 1e300;
    auto vp = pts(p), vq = pts(q);
    if (p.kind == 0) { for (auto w : vq) best = std::min(best, primDist(p, w)); for (auto w : vp) best = std::min(best, primDist(q, w)); return best; }
    if (q.kind == 0) { for (auto w : vp) best = std::min(best, primDist(q, w)); for (auto w : vq) best = std::min(best, primDist(p, w)); return best; }
    for (auto w : vp) best = std::min(best, primDist(q, w));
    return best;
}
// true when no two non-adjacent prims come within eps (adjacent = share an end point exactly).
static bool simpleScene(const Scene& sc, double eps = 1e-6) {
    const auto& P = sc.prims;
    for (size_t i = 0; i < P.size(); ++i)
        for (size_t j = i + 1; j < P.size(); ++j) {
            const Prim &p = P[i], &q = P[j];
            bool adj = false;
            if (p.kind == 0 && q.kind == 0) adj = (p.b == q.a) || (p.a == q.b) || (p.a == q.a) || (p.b == q.b);
            else if (p.kind == 1 && q.kind == 1 && p.c.x == q.c.x && p.c.y == q.c.y && p.R == q.R) adj = true;
            else {
                // line-arc adjacency (round rect / bezier sampled are lines): share end point within 1e-9
                const Prim& l = p.kind == 0 ? p : q;
                const Prim& c = p.kind == 0 ? q : p;
                Vec2 s = arcPt(c, c.a0), e = arcPt(c, c.a0 + c.sw);
                for (Vec2 x : {l.a, l.b}) for (Vec2 y : {s, e}) if (std::hypot(x.x - y.x, x.y - y.y) < 1e-9) adj = true;
            }
            if (adj) {
                // adjacent lines must not be collinear-overlapping (fold at a vertex)
                if (p.kind == 0 && q.kind == 0) {
                    Vec2 shared = (p.b == q.a || p.b == q.b) ? p.b : p.a;
                    Vec2 po = shared == p.a ? p.b : p.a, qo = shared == q.a ? q.b : q.a;
                    Vec2 u = po - shared, v = qo - shared;
                    double c = u.dot(v) / (std::sqrt(u.dot(u) * v.dot(v)) + 1e-300);
                    if (c > 1 - 1e-12) return false;
                }
                continue;
            }
            if (primPrimDist(p, q) < eps) return false;
        }
    return true;
}

// ----------------------------------------------------------------- new generators
static std::vector<Vec2> rectPts(Vec2 lo, Vec2 hi) { return {lo, {hi.x, lo.y}, hi, {lo.x, hi.y}}; }

struct Scene3 {
    Scene sc;
    double rotAngle{0};
    int regions{-1};  // -1: count from centre region
};

static Scene3 genScene3(Rng& R, double D) {
    Scene3 s3;
    Scene& sc = s3.sc;
    double r = D / 2;
    int kind = R.i(0, 10);
    switch (kind) {
        case 0: {  // dense island grid, gaps around 2r
            sc.desc = "dense-grid";
            int nx = R.i(2, 7), ny = R.i(2, 5);
            double gf = R.u(0.6, 1.6);
            double gx = 2 * r * gf * R.u(0.9, 1.1) + R.u(-0.05, 0.05), gy = 2 * r * gf * R.u(0.8, 1.2);
            gx = std::max(gx, 0.05); gy = std::max(gy, 0.05);
            double iw = R.u(0.5, 6), ih = R.u(0.5, 6);
            double W = nx * iw + (nx + 1) * gx, H = ny * ih + (ny + 1) * gy;
            double ang = R.i(0, 1) ? R.u(-PI, PI) : 0.0;
            s3.rotAngle = ang;
            sc.addPoly(rotPts(rectPts({0, 0}, {W, H}), {0, 0}, ang));
            for (int a = 0; a < nx; ++a)
                for (int b = 0; b < ny; ++b) {
                    double x = gx + a * (iw + gx), y = gy + b * (ih + gy);
                    if (R.i(0, 3) == 0) {
                        Vec2 c = rotPts({{x + iw / 2, y + ih / 2}}, {0, 0}, ang)[0];
                        sc.addCircle(c, 0.5 * std::min(iw, ih), R.i(0, 1));
                    } else sc.addPoly(rotPts(rectPts({x, y}, {x + iw, y + ih}), {0, 0}, ang));
                }
            break;
        }
        case 1: {  // islands touching each other within 2r (pairs / chains), random sizes
            sc.desc = "touching";
            double W = R.u(25, 60), H = R.u(20, 45);
            sc.addPoly(rectPts({0, 0}, {W, H}));
            int n = R.i(2, 6);
            double x = R.u(0.3, 3) * r + R.u(0.01, 1);
            double y0 = R.u(0.5 * r, H * 0.3);
            for (int k = 0; k < n; ++k) {
                double w = R.u(1, 8), h = R.u(1, H - y0 - 2 * r - 0.5);
                if (h < 0.5 || x + w > W - 0.3) break;
                double yy = y0 + R.u(-0.4, 0.4) * y0 * 0.5;
                if (yy + h > H - 0.2) break;
                if (R.i(0, 2) == 0) sc.addCircle({x + w / 2, yy + w / 2}, w / 2);
                else sc.addPoly(rectPts({x, yy}, {x + w, yy + h}));
                double gap = R.i(0, 2) == 0 ? R.u(0.005, 0.3) : 2 * r * R.u(0.9, 1.1);
                x += w + gap;
            }
            break;
        }
        case 2: {  // nested pocket / island / pocket / island with band islands
            sc.desc = "nested3";
            double W = R.u(35, 70), H = R.u(30, 60);
            double g1 = R.u(1.1, 3) * D, t = R.u(0.5, 4), g2 = R.u(1.1, 3) * D;
            sc.addPoly(rectPts({0, 0}, {W, H}));
            Vec2 lo1{g1, g1}, hi1{W - g1, H - g1};
            sc.addPoly(rectPts(lo1, hi1));
            Vec2 lo2{lo1.x + t, lo1.y + t}, hi2{hi1.x - t, hi1.y - t};
            if (hi2.x - lo2.x > 2 * g2 + 2 && hi2.y - lo2.y > 2 * g2 + 2) {
                sc.addPoly(rectPts(lo2, hi2));
                Vec2 c = (lo2 + hi2) * 0.5;
                double rr = std::min(hi2.x - lo2.x, hi2.y - lo2.y) / 2 - g2;
                if (rr > 0.5) sc.addCircle(c, rr * R.u(0.3, 1.0));
                s3.regions = 2;
            } else s3.regions = 1;
            // band islands (in the outer band) if it is wide enough
            if (g1 > 2.5 && R.i(0, 1)) {
                double ir = R.u(0.2, g1 / 2 - 0.1 - (R.i(0, 1) ? 0 : 0));
                sc.addCircle({W / 2, g1 / 2}, std::min(ir, g1 / 2 - 0.05));
            }
            break;
        }
        case 3: {  // serpentine corridor
            sc.desc = "serpentine";
            int nf = R.i(2, 8);
            double w = 2 * r * R.u(0.95, 2.2), ft = R.u(0.2, 3), H = R.u(15, 40);
            std::vector<Vec2> p{{0, 0}};
            double x = 0;
            // bottom fins (from y=0 to H-w) at odd positions; top fins (from H down to w) at even
            std::vector<std::pair<double, int>> fins;
            for (int f = 0; f < nf; ++f) { x += w; fins.push_back({x, f % 2}); x += ft; }
            double W = x + w;
            for (auto& [fx, top] : fins) if (!top) { p.push_back({fx, 0}); p.push_back({fx, H - w}); p.push_back({fx + ft, H - w}); p.push_back({fx + ft, 0}); }
            p.push_back({W, 0}); p.push_back({W, H});
            for (auto it = fins.rbegin(); it != fins.rend(); ++it) if (it->second) { double fx = it->first; p.push_back({fx + ft, H}); p.push_back({fx + ft, w}); p.push_back({fx, w}); p.push_back({fx, H}); }
            p.push_back({0, H});
            double ang = R.i(0, 1) ? R.u(-PI, PI) : 0.0;
            s3.rotAngle = ang;
            sc.addPoly(rotPts(p, {0, 0}, ang));
            s3.regions = 1;
            break;
        }
        case 4: {  // lobed flower
            sc.desc = "flower";
            int k = R.i(2, 9);
            double R0 = R.u(8, 30), a = R.u(0.2, 0.85);
            int n = k * R.i(12, 40);
            std::vector<Vec2> p;
            double ph = R.u(0, 2 * PI);
            for (int i = 0; i < n; ++i) { double t = 2 * PI * i / n; double rr = R0 * (1 + a * std::cos(k * t + ph)); p.push_back({rr * std::cos(t), rr * std::sin(t)}); }
            sc.addPoly(p);
            if (R.i(0, 2) == 0) sc.addCircle({0, 0}, R0 * (1 - a) * R.u(0.2, 0.8));
            break;
        }
        case 5: {  // dumbbell chain: discs on a line joined by necks
            sc.desc = "chain";
            int m = R.i(2, 6);
            std::vector<double> cx, rr, neck;
            double x = 0;
            for (int i = 0; i < m; ++i) {
                double ri = R.u(0.4, 2.5) * D;
                if (i) x += rr.back() + ri + R.u(0.5, 3) * D;
                cx.push_back(x); rr.push_back(ri);
                if (i) neck.push_back(R.u(0.3, 1.4) * D * 0.5);  // half-width of neck
            }
            auto f = [&](double xx) {
                double h = 0;
                for (int i = 0; i < m; ++i) { double dx = xx - cx[i]; if (std::fabs(dx) < rr[i]) h = std::max(h, std::sqrt(rr[i] * rr[i] - dx * dx)); }
                for (int i = 0; i + 1 < m; ++i) if (xx >= cx[i] && xx <= cx[i + 1]) h = std::max(h, neck[i]);
                return h;
            };
            double x0 = cx.front() - rr.front(), x1 = cx.back() + rr.back();
            int N = 600;
            std::vector<Vec2> top, bot;
            for (int i = 1; i < N; ++i) { double xx = x0 + (x1 - x0) * i / N; double h = f(xx); top.push_back({xx, h}); }
            std::vector<Vec2> p{{x0, 0}};
            for (auto it = top.begin(); it != top.end(); ++it) p.push_back({it->x, -it->y});
            p.push_back({x1, 0});
            for (auto it = top.rbegin(); it != top.rend(); ++it) p.push_back(*it);
            double ang = R.u(-PI, PI);
            s3.rotAngle = ang;
            sc.addPoly(rotPts(p, {0, 0}, ang));
            break;
        }
        case 6: {  // deep star with island
            sc.desc = "deepstar";
            int k = R.i(4, 16);
            double ro = R.u(12, 35), ri = ro * R.u(0.1, 0.5);
            std::vector<Vec2> p;
            for (int i = 0; i < 2 * k; ++i) { double t = PI * i / k; double rr = (i % 2 ? ri : ro) * R.u(0.9, 1.0); p.push_back({rr * std::cos(t), rr * std::sin(t)}); }
            sc.addPoly(p);
            if (R.i(0, 1) && ri > 2) sc.addCircle({0, 0}, ri * R.u(0.2, 0.7));
            break;
        }
        case 7: {  // union of 2-4 discs (concave cusps), sampled polygon
            sc.desc = "discs";
            int m = R.i(2, 4);
            std::vector<Vec2> c; std::vector<double> rad;
            Vec2 cur{0, 0};
            for (int i = 0; i < m; ++i) {
                double ri = R.u(5, 18);
                if (i) { double d = (rad.back() + ri) * R.u(0.3, 0.95); double t = R.u(-0.6, 0.6); cur = cur + Vec2{std::cos(t), std::sin(t)} * d; }
                c.push_back(cur); rad.push_back(ri);
            }
            // trace union boundary: march angles on each circle, keep points outside the others, then
            // build polygon by sorting? Not generally simple for >2; use 2 discs only when m>2 fails.
            // Build the outline as a star-shaped polygon about the centroid (union is star-shaped
            // for these chains only approximately) -> use ray casting from centroid of centres.
            Vec2 g{0, 0}; for (auto q : c) g = g + q; g = g * (1.0 / m);
            int N = 1440;
            std::vector<Vec2> p;
            bool ok = true;
            for (int i = 0; i < N && ok; ++i) {
                double t = 2 * PI * i / N; Vec2 u{std::cos(t), std::sin(t)};
                // farthest exit of the ray from g through the union (assume ray leaves once)
                double best = -1;
                for (int j = 0; j < m; ++j) { Vec2 f = g - c[j]; double B = f.dot(u), C = f.dot(f) - rad[j] * rad[j]; double disc = B * B - C; if (disc >= 0) best = std::max(best, -B + std::sqrt(disc)); }
                if (best <= 0) ok = false;
                p.push_back(g + u * best);
            }
            if (!ok) { sc.addCircle({0, 0}, 15); break; }
            sc.addPoly(p);
            break;
        }
        case 8: {  // skyline (orthogonal) with islands
            sc.desc = "skyline";
            int n = R.i(3, 12);
            double x = 0, base = R.u(1, 2.5) * D;
            std::vector<Vec2> p{{0, 0}};
            double W = 0;
            std::vector<double> xs, hs;
            for (int i = 0; i < n; ++i) { double w = R.u(0.4, 3) * D; xs.push_back(x); hs.push_back(base + R.u(0, 30)); x += w; }
            W = x;
            p.push_back({W, 0});
            for (int i = n - 1; i >= 0; --i) { double xe = i + 1 < n ? xs[i + 1] : W; p.push_back({xe, hs[i]}); p.push_back({xs[i], hs[i]}); }
            // remove consecutive duplicates / collinear
            std::vector<Vec2> q;
            for (auto v : p) if (q.empty() || v != q.back()) q.push_back(v);
            sc.addPoly(q);
            if (R.i(0, 1)) sc.addCircle({W * R.u(0.2, 0.8), base * 0.5}, std::max(0.2, base * 0.5 - R.u(0.6, 1.4) * r));
            break;
        }
        case 9: {  // rect with random circle islands (any gaps, may be < 2r)
            sc.desc = "rand-circles";
            double W = R.u(25, 60), H = R.u(20, 45);
            sc.addPoly(rectPts({0, 0}, {W, H}));
            std::vector<std::pair<Vec2, double>> placed;
            int want = R.i(2, 10);
            for (int t = 0; t < 400 && (int)placed.size() < want; ++t) {
                double ir = R.u(0.3, 6);
                Vec2 ic{R.u(0, W), R.u(0, H)};
                if (ic.x - ir < 0.02 || ic.y - ir < 0.02 || ic.x + ir > W - 0.02 || ic.y + ir > H - 0.02) continue;
                bool ok = true;
                for (auto& [pc, pr] : placed) if (std::hypot(ic.x - pc.x, ic.y - pc.y) < ir + pr + 0.02) ok = false;
                if (ok) placed.push_back({ic, ir});
            }
            for (auto& [c, rr] : placed) sc.addCircle(c, rr, R.i(0, 1));
            break;
        }
        default: {  // bezier blob with concave cusps (sampled prims used for the oracle)
            sc.desc = "bez-cusp";
            int k = R.i(3, 7);
            std::vector<Vec2> pts;
            double R0 = R.u(12, 28);
            for (int i = 0; i < 2 * k; ++i) { double t = PI * i / k; double rr = i % 2 ? R0 * R.u(0.35, 0.6) : R0; pts.push_back({rr * std::cos(t), rr * std::sin(t)}); }
            sc.addBezierBlob(pts);
            break;
        }
    }
    return s3;
}

// ----------------------------------------------------------------- full check v3
static bool gVerbose = false;
static int fullCheck3(const std::string& tag, const Scene& sc, const Config& cf, double hgrid, int regionsHint,
                      std::string* summary = nullptr) {
    PocketResult res = pocketToolpath(sc.contours, cf.tools, cf.p);
    PocketResult res2 = pocketToolpath(sc.contours, cf.tools, cf.p);
    bool bad = false;
    std::string why;
    if (serialize(res) != serialize(res2)) { bad = true; why += " NONDETERMINISTIC"; }
    double bottom = -(cf.p.startDepth + cf.p.cutDepth);
    double tolG = sc.hasBezier ? 2e-4 : 1e-6;
    for (size_t t = 0; t < res.toolpaths.size(); ++t) {
        Check ck;
        checkToolpath(sc, res.toolpaths[t], 0.5 * cf.tools[t].tool.diameter, cf.p.allowance, cf.p.safeZ, bottom, ck, tolG);
        if (!ck.ok) { bad = true; why += " tool" + std::to_string(t) + " SAFETY:" + ck.why; }
    }
    double rmax = 0;
    for (auto& t : cf.tools) rmax = std::max(rmax, t.tool.diameter / 2);
    HMap g = makeHMap(sc, hgrid, rmax + 1, cf.p.startDepth);
    SimStats st;
    long restAir = 0;
    std::string restAirFirst;
    bool anyEarlier = false;
    for (size_t t = 0; t < res.toolpaths.size(); ++t) {
        const Toolpath& tp = res.toolpaths[t];
        double r = cf.tools[t].tool.diameter / 2;
        if (!tp.moves.empty() && anyEarlier) {
            // rest-air: every cutting move of a rest tool must come within r (+ grid slack) of
            // material still left above the final depth.
            Grid gg; gg.x0 = g.x0; gg.y0 = g.y0; gg.h = g.h; gg.nx = g.nx; gg.ny = g.ny;
            std::vector<uint8_t> M(g.H.size());
            for (size_t k = 0; k < g.H.size(); ++k) M[k] = g.sd[k] >= cf.p.allowance + 1e-3 && g.H[k] > bottom + 1e-6;
            auto dm = edt(gg, M);
            std::vector<int> role;
            std::string err;
            if (classify(tp, res.plans[t], role, err)) {
                Vec3 pos{};
                for (size_t mi = 0; mi < tp.moves.size(); ++mi) {
                    const Move& m = tp.moves[mi];
                    Vec3 from = mi ? pos : m.to;
                    pos = m.to;
                    if (role[mi] != 1) continue;
                    Vec2 a = from.xy(), b = m.to.xy();
                    int n = std::max(1, int(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.05)));
                    for (int k = 0; k <= n; ++k) {
                        Vec2 q = a + (b - a) * (double(k) / n);
                        int i = int((q.x - g.x0) / g.h), j = int((q.y - g.y0) / g.h);
                        if (i < 0 || j < 0 || i >= g.nx || j >= g.ny) continue;
                        double dd = dm[size_t(j) * g.nx + i];
                        if (dd > r + 0.3) {
                            if (!restAir) { char b2[200]; snprintf(b2, sizeof b2, "tool %zu move %zu at (%.3f,%.3f) dist to rest material %.3f > r %.3f", t, mi, q.x, q.y, dd, r); restAirFirst = b2; }
                            ++restAir;
                            break;
                        }
                    }
                }
            }
        }
        simulate(g, tp, res.plans[t], r, cf.p.allowance, int(t), st, 0.02);
        if (!tp.moves.empty()) anyEarlier = true;
    }
    if (st.travelPen > 0.02) { bad = true; why += " TRAVEL-ENGAGES: " + st.travelFirst + " engagedMoves=" + std::to_string(st.travelEngaged); }
    if (!st.parseFail.empty()) { bad = true; why += " PARSE: " + st.parseFail; }
    if (st.gougeCells) { bad = true; why += " HMAP-GOUGE cells=" + std::to_string(st.gougeCells) + " " + st.gougeFirst; }
    if (restAir) { bad = true; why += " REST-AIR moves=" + std::to_string(restAir) + " " + restAirFirst; }
    bool rasterNone = cf.p.strategy == PocketStrategy::Raster && cf.p.profilePass == ProfilePass::None;
    for (size_t t = 0; t < res.toolpaths.size(); ++t) {
        if (res.toolpaths[t].moves.empty()) continue;
        // raster without boundary pass leaves wall scallops by design: check coverage only when a
        // later tool exists (the scallops are rest material for it) or the strategy has a boundary pass
        bool lastUsed = true;
        for (size_t u = t + 1; u < res.toolpaths.size(); ++u) if (!res.toolpaths[u].moves.empty()) lastUsed = false;
        if (rasterNone && lastUsed && t + 1 == res.toolpaths.size() && res.toolpaths.size() == 1) continue;
        if (rasterNone && lastUsed) continue;  // the final tool's own scallops remain by design
        Grid gg; gg.x0 = g.x0; gg.y0 = g.y0; gg.h = g.h; gg.nx = g.nx; gg.ny = g.ny;
        std::vector<uint8_t> zset(g.sd.size());
        double rr = cf.tools[t].tool.diameter / 2;
        for (size_t k = 0; k < g.sd.size(); ++k) zset[k] = g.sd[k] >= rr + cf.p.allowance + 2.5 * cf.p.tolerance.chordal + 0.001;
        auto dz = edt(gg, zset);
        long miss = 0, real = 0, reach = 0;
        Vec2 realAt{};
        double worstEx = -1e9;
        long checked = 0;
        for (int j = 0; j < g.ny; ++j)
            for (int i = 0; i < g.nx; ++i) {
                size_t k = size_t(j) * g.nx + i;
                if (!(dz[k] <= rr - 1e-9)) continue;
                ++reach;
                if (g.H[k] <= bottom + 1e-9) continue;
                ++miss;
                if (++checked > 3000) continue;
                Vec2 q = g.at(i, j);
                double ex = 1e9;
                for (size_t u = 0; u < res.toolpaths.size(); ++u) {
                    double ru = cf.tools[u].tool.diameter / 2; Vec3 pos{};
                    const auto& mv = res.toolpaths[u].moves;
                    for (size_t mi = 0; mi < mv.size(); ++mi) { Vec3 f = mi ? pos : mv[mi].to; if (mv[mi].kind != MoveKind::Rapid && f.z <= bottom + 1e-9 && mv[mi].to.z <= bottom + 1e-9) ex = std::min(ex, segDist(q, f.xy(), mv[mi].to.xy()) - ru); pos = mv[mi].to; }
                }
                worstEx = std::max(worstEx, ex);
                if (ex > std::min(cf.p.tolerance.chordal, 0.01) + 1e-6) { if (!real) realAt = q; ++real; }
            }
        if (real) {
            bad = true;
            char b[240];
            snprintf(b, sizeof b, " COVERAGE(reach tool %zu): reach=%ld miss=%ld real=%ld first (%.3f,%.3f) worst excess %.4f", t, reach, miss, real, realAt.x, realAt.y, worstEx);
            why += b;
        }
    }
    int regions = 0;
    if (cf.p.strategy == PocketStrategy::Offset && !res.toolpaths.empty() && !res.toolpaths[0].moves.empty()) {
        for (auto& c : res.plans[0].centreRegion) { double A = 0; for (auto& sp : c.spans) { auto& l = std::get<LineSpan>(sp); A += l.start.x * l.end.y - l.end.x * l.start.y; } if (A > 0) ++regions; }
        size_t n = res.toolpaths[0].count(MoveKind::Retract);
        if (n > size_t(regions + 1)) { bad = true; why += " AC5 retracts=" + std::to_string(n) + " > " + std::to_string(regions + 1); }
    }
    (void)regionsHint;
    if (bad || gVerbose) {
        printf("%s [%s] %s:%s | travel %ld engaged %ld pen %.4f; links %ld engaged %ld; retracts", tag.c_str(), sc.desc.c_str(),
               cfgStr(cf).c_str(), why.empty() ? " ok" : why.c_str(), st.travelMoves, st.travelEngaged, st.travelPen, st.linkMoves,
               st.linkEngaged);
        for (auto& tp : res.toolpaths) printf(" %zu", tp.count(MoveKind::Retract));
        printf(" regions %d", regions);
        for (auto& m : res.messages) printf(" msg[%s]", m.c_str());
        printf("\n");
        fflush(stdout);
    }
    if (summary) {
        char b[200];
        size_t tr = 0; for (auto& tp : res.toolpaths) tr += tp.count(MoveKind::Retract);
        snprintf(b, sizeof b, "travel=%ld retracts=%zu", st.travelMoves, tr);
        *summary = b;
    }
    return bad ? 1 : 0;
}

static bool makeScene3(uint64_t seed, Scene3& s3, Config& cf, int& attempts) {
    Rng R(seed);
    for (attempts = 1; attempts <= 20; ++attempts) {
        double D = R.u(0.8, 10);
        s3 = genScene3(R, D);
        if (s3.sc.contours.empty()) continue;
        if (!simpleScene(s3.sc)) continue;
        cf = genConfig2(R, Scene2{s3.sc, s3.rotAngle, 1, true}, D);
        return true;
    }
    return false;
}

static int sweep3(uint64_t seed0, int n, double hgrid) {
    int bad = 0, hang = 0, skipped = 0;
    auto t0 = std::chrono::steady_clock::now();
    std::map<std::string, int> kinds;
    for (int k = 0; k < n; ++k) {
        uint64_t seed = seed0 + k;
        int rc = forked([&] {
            Scene3 s3; Config cf; int att;
            if (!makeScene3(seed, s3, cf, att)) return 7;
            return fullCheck3("seed " + std::to_string(seed), s3.sc, cf, hgrid, s3.regions);
        }, 600);
        if (rc == 7) ++skipped;
        else if (rc == 1) ++bad;
        else if (rc != 0) { ++hang; printf("seed %llu: CRASH/TIMEOUT rc=%d\n", (unsigned long long)seed, rc); fflush(stdout); }
    }
    double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("sweep3: %d scenes from seed %llu: %d failed, %d crash/timeout, %d unbuildable, %.1fs\n", n, (unsigned long long)seed0, bad, hang, skipped, el);
    return bad + hang;
}

// round-2 generator, simplicity-filtered, new seeds, v3 checks
static int sweep2b(uint64_t seed0, int n, double hgrid) {
    int bad = 0, hang = 0, nonsimple = 0;
    for (int k = 0; k < n; ++k) {
        uint64_t seed = seed0 + k;
        int rc = forked([&] {
            Rng R(seed);
            double D = R.u(1, 10);
            Scene2 s2 = genScene2(R, D);
            if (s2.sc.contours.empty()) return 0;
            if (!simpleScene(s2.sc)) return 7;
            Config cf = genConfig2(R, s2, D);
            return fullCheck3("r2gen " + std::to_string(seed), s2.sc, cf, hgrid, s2.regions);
        }, 600);
        if (rc == 7) ++nonsimple;
        else if (rc == 1) ++bad;
        else if (rc != 0) { ++hang; printf("r2gen %llu: CRASH/TIMEOUT rc=%d\n", (unsigned long long)seed, rc); fflush(stdout); }
    }
    printf("sweep2b: %d scenes from seed %llu: %d failed, %d crash/timeout, %d non-simple skipped\n", n, (unsigned long long)seed0, bad, hang, nonsimple);
    return bad + hang;
}

// ----------------------------------------------------------------- fold analysis
static int foldMode(const std::vector<uint64_t>& seeds) {
    for (uint64_t seed : seeds) {
        Rng R(seed);
        double D = R.u(1, 10);
        Scene2 s2 = genScene2(R, D);
        Config cf = genConfig2(R, s2, D);
        int removed = 0;
        Scene eff = effective(s2.sc, &removed);
        bool simple = simpleScene(s2.sc);
        PocketResult res = pocketToolpath(s2.sc.contours, cf.tools, cf.p);
        double bottom = -(cf.p.startDepth + cf.p.cutDepth);
        printf("seed %llu [%s] simple=%d fold-pieces-removed=%d %s\n", (unsigned long long)seed, s2.sc.desc.c_str(), int(simple), removed, cfgStr(cf).c_str());
        for (auto& pr : s2.sc.prims) if (pr.kind == 0) printf("   edge (%.4f,%.4f)-(%.4f,%.4f)\n", pr.a.x, pr.a.y, pr.b.x, pr.b.y);
        for (size_t t = 0; t < res.toolpaths.size(); ++t) {
            Check c1, c2;
            checkToolpath(s2.sc, res.toolpaths[t], cf.tools[t].tool.diameter / 2, cf.p.allowance, cf.p.safeZ, bottom, c1, 1e-6);
            checkToolpath(eff, res.toolpaths[t], cf.tools[t].tool.diameter / 2, cf.p.allowance, cf.p.safeZ, bottom, c2, 1e-6);
            printf("   tool %zu: raw-oracle %s (min margin %.6f) | effective-boundary %s (min margin %.6f)\n", t, c1.ok ? "ok" : c1.why.c_str(), c1.minMargin,
                   c2.ok ? "ok" : c2.why.c_str(), c2.minMargin);
        }
        // height-map on the effective scene: does any cut remove material outside the even-odd region?
        HMap g = makeHMap(eff, 0.05, 6, cf.p.startDepth);
        SimStats st;
        for (size_t t = 0; t < res.toolpaths.size(); ++t) simulate(g, res.toolpaths[t], res.plans[t], cf.tools[t].tool.diameter / 2, cf.p.allowance, int(t), st, 0.02);
        printf("   hmap(effective): gougeCells=%ld %s travelPen=%.4f\n", st.gougeCells, st.gougeFirst.c_str(), st.travelPen);
    }
    return 0;
}

static int oneMode(uint64_t seed, double h) {
    gVerbose = true;
    Scene3 s3; Config cf; int att;
    if (!makeScene3(seed, s3, cf, att)) { printf("unbuildable\n"); return 0; }
    printf("attempts %d\n", att);
    return fullCheck3("seed " + std::to_string(seed), s3.sc, cf, h, s3.regions);
}

#ifndef V3_NO_MAIN
int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "sweep3") return sweep3(strtoull(argv[2], 0, 10), atoi(argv[3]), argc > 4 ? atof(argv[4]) : 0.1);
    if (mode == "sweep2b") return sweep2b(strtoull(argv[2], 0, 10), atoi(argv[3]), argc > 4 ? atof(argv[4]) : 0.1);
    if (mode == "one3") return oneMode(strtoull(argv[2], 0, 10), argc > 3 ? atof(argv[3]) : 0.05);
    if (mode == "fold") { std::vector<uint64_t> s; for (int i = 2; i < argc; ++i) s.push_back(strtoull(argv[i], 0, 10)); return foldMode(s); }
    if (mode == "v2") { argv[1] = argv[0]; return v2_unused_main(argc - 1, argv + 1); }
    printf("modes: sweep3 seed n [h] | sweep2b seed n [h] | one3 seed [h] | fold seeds... | v2 <v2 args>\n");
    return 0;
}
#endif
