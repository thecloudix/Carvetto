// T-0009 round-4 verifier: adversarial generator aimed at the round-4 cluster logic (collapsed clusters, net-turn
// runs, size thresholds 2 tol / 4 tol, chain ends, neighbouring clusters, arc/Bezier clusters, d vs cluster extent).
static Vec2 startPointOr(const Contour& k, Vec2 p) { return k.spans.empty() ? p : startPoint(k.spans.front()); }
struct AdvCase { Contour in; double d, mitre, tol = 0.001; int fam, sub; double factor = 0; };
static AdvCase makeAdv(std::uint64_t seed) {
    Rng g(seed * 0xD6E8FEB86659FD93ULL + 0x1234567);
    AdvCase c; c.fam = g.irange(0, 5); c.sub = 0; const double tol = c.tol;
    c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 4.0);
    Contour& k = c.in; Vec2 p{0, 0}; double h = g.range(-3.14159, 3.14159);
    auto line = [&](double L) { Vec2 q = p + Vec2::fromAngle(h, L); if (q != p) k.spans.push_back(LineSpan{p, q}); p = q; };
    auto lineTo = [&](Vec2 q) { if (q != p) { k.spans.push_back(LineSpan{p, q}); h = (q - p).angle(); } p = q; };
    // threshold factors for the cluster's box diagonal (in tol): around 2 and 4, plus random
    auto pickFactor = [&]() { double u = g.uni(); const double fs[] = {1.9, 1.99, 2.0, 2.01, 2.1, 3.9, 3.99, 4.0, 4.01, 4.1};
        if (u < 0.6) return fs[g.irange(0, 9)] * (1 + g.range(-1e-3, 1e-3)); if (u < 0.8) return g.range(0.05, 2.0); return g.range(2.0, 7.0); };
    // micro-cluster: random walk of m steps (each <= tol) confined to a w x h box (diagonal factor*tol) anchored so
    // that the first step reverses the incoming heading (a near-reversal at a sub-tol span)
    auto cluster = [&](double factor, int m, bool reverseFirst) {
        double ang = g.range(0, 1.5707963); double D = factor * tol; double W = D * std::cos(ang), H = D * std::sin(ang);
        Vec2 ex = Vec2::fromAngle(h), ey = ex.perp(); Vec2 o = p - ex * g.range(0, W) - ey * g.range(0, H);  // box origin in heading frame
        auto clampBox = [&](Vec2 q) { Vec2 r = q - o; double a = std::clamp(r.dot(ex), 0.0, W), b = std::clamp(r.dot(ey), 0.0, H); return o + ex * a + ey * b; };
        for (int i = 0; i < m; ++i) {
            double dir = (i == 0 && reverseFirst) ? h + 3.14159 + g.range(-0.6, 0.6) : g.range(-3.14159, 3.14159);
            double L = tol * (g.uni() < 0.3 ? std::pow(10.0, -g.range(0, 6)) : g.range(0.05, 1.0));
            Vec2 q = clampBox(p + Vec2::fromAngle(dir, L)); if (distance(p, q) > tol) q = p + (q - p).normalized() * tol;
            lineTo(q);
        }
    };
    // net-turn run: m tiny steps turning by total/m each, scaled to fit a box of diagonal factor*tol
    auto netTurn = [&](double total, int m, double factor) {
        // build unit-length polyline, then scale so its bounding box diagonal = factor*tol
        std::vector<Vec2> pts{{0, 0}}; double hh = h; Vec2 q{0, 0};
        for (int i = 0; i < m; ++i) { hh += total / m * g.range(0.7, 1.3); q = q + Vec2::fromAngle(hh, g.range(0.3, 1.0)); pts.push_back(q); }
        double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300; for (auto& v : pts) { x0 = std::min(x0, v.x); x1 = std::max(x1, v.x); y0 = std::min(y0, v.y); y1 = std::max(y1, v.y); }
        double diag = std::hypot(x1 - x0, y1 - y0); if (diag <= 0) return; double s = factor * tol / diag;
        Vec2 base = p; for (size_t i = 1; i < pts.size(); ++i) { Vec2 t = base + pts[i] * s; if (distance(p, t) <= tol) lineTo(t); else lineTo(p + (t - p).normalized() * tol); }
        h = hh;
    };
    auto tinyArc = [&](double sw, double r) { Vec2 left = Vec2::fromAngle(h).perp(); Vec2 ctr = sw > 0 ? p + left * r : p - left * r;
        ArcSpan a{ctr, r, (p - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW}; a.endAngle = a.startAngle + sw; if (a.endPoint() != p) { k.spans.push_back(a); p = a.endPoint(); h += sw; } };
    auto bigArc = [&](double R, double sw) { Vec2 left = Vec2::fromAngle(h).perp(); Vec2 ctr = sw > 0 ? p + left * R : p - left * R;
        ArcSpan a{ctr, R, (p - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW}; a.endAngle = a.startAngle + sw; k.spans.push_back(a); p = a.endPoint(); h += sw; };
    auto bez = [&](double L) { Vec2 q = p + Vec2::fromAngle(h + g.range(-1.5, 1.5), L); Vec2 c1 = p + Vec2::fromAngle(h, L * g.range(0.1, 0.6)); Vec2 c2 = q + Vec2::fromAngle(g.range(-3.14, 3.14), L * g.range(0, 0.5)); k.spans.push_back(BezierSpan{p, c1, c2, q}); h = (q - c2).angle(); p = q; };
    auto turnB = [&]() { double u = g.uni(); if (u < 0.45) h += (g.uni() < 0.5 ? 1 : -1) * g.range(2.36, 3.14159); else if (u < 0.6) h += g.range(-0.3, 0.3); else h += g.range(-3.14159, 3.14159); };
    c.factor = pickFactor();
    switch (c.fam) {
        case 0: {  // threshold clusters between long lines (A -> cluster -> B), several along the path
            int nv = g.irange(1, 4); line(g.range(0.3, 5));
            for (int i = 0; i < nv; ++i) { double f = i == 0 ? c.factor : pickFactor(); cluster(f, g.irange(2, 30), g.uni() < 0.8); turnB(); line(g.range(0.3, 5)); }
            break; }
        case 1: {  // net-turn runs of many small turns (none > 135), total 150..420 deg, box around the thresholds
            int nv = g.irange(1, 3); line(g.range(0.3, 5));
            for (int i = 0; i < nv; ++i) { double tot = (g.uni() < 0.5 ? 1 : -1) * g.range(2.6, 7.3); int m = g.irange(3, 80); netTurn(tot, m, i == 0 ? c.factor : pickFactor()); if (g.uni() < 0.5) turnB(); else h += g.range(-0.2, 0.2); line(g.range(0.3, 5)); }
            break; }
        case 2: {  // clusters at open-chain ends and close to each other / to other parts of the path
            c.sub = g.irange(0, 3);
            if (c.sub == 0 || c.sub == 2) cluster(c.factor, g.irange(2, 20), false);
            line(g.range(0.3, 4));
            int pairs = g.irange(1, 3);
            for (int i = 0; i < pairs; ++i) { cluster(pickFactor(), g.irange(2, 15), true); double gap = tol * (g.uni() < 0.5 ? g.range(0.3, 1.0) : g.range(1.0, 10.0)); turnB(); line(gap); cluster(pickFactor(), g.irange(2, 15), true); turnB(); line(g.range(0.2, 4)); }
            if (c.sub == 3) {  // hairpin coming back past the last cluster within ~d
                Vec2 cl = p; cluster(pickFactor(), g.irange(2, 10), true); h += 3.14159 + g.range(-0.05, 0.05); line(g.range(0.5, 3)); h += 3.14159; Vec2 q = cl + (p - cl).perp().normalized() * g.range(0, 0.02); lineTo(q); line(g.range(0.3, 2)); }
            if (c.sub == 1 || c.sub == 2) cluster(pickFactor(), g.irange(2, 20), true);
            break; }
        case 3: {  // arc and Bezier clusters; A/B arcs or Beziers
            int nv = g.irange(1, 4);
            auto longSpan = [&]() { double u = g.uni(); if (u < 0.35) line(g.range(0.3, 4)); else if (u < 0.7) bigArc(g.range(0.05, 10), (g.uni() < 0.5 ? 1 : -1) * g.range(0.2, 4)); else bez(g.range(0.3, 4)); };
            longSpan();
            for (int i = 0; i < nv; ++i) {
                int m = g.irange(1, 8);
                for (int j = 0; j < m; ++j) { double u = g.uni(); if (u < 0.45) { double sw = (g.uni() < 0.5 ? 1 : -1) * g.range(2.0, 6.2); double L = tol * g.range(0.01, 1.0); tinyArc(sw, L / std::fabs(sw)); }
                    else if (u < 0.6) bez(tol * g.range(0.1, 1.0)); else { h += 3.14159 + g.range(-0.7, 0.7); line(tol * g.range(0.01, 1.0)); } }
                turnB(); longSpan();
            }
            break; }
        default: {  // closed polygons with clusters at vertices, spikes, and a cluster at the closing vertex
            int nv = g.irange(3, 7); double R = g.range(0.2, 5);
            std::vector<Vec2> V; for (int i = 0; i < nv; ++i) V.push_back(Vec2::fromAngle(6.283185307179586 * i / nv + g.range(-0.3, 0.3), R * g.range(0.5, 1.5)));
            p = V[0]; bool atStart = g.uni() < 0.5;
            if (atStart) { h = (V[0] - V[nv - 1]).angle(); cluster(c.factor, g.irange(2, 12), g.uni() < 0.7); }
            Vec2 start = startPointOr(k, p);
            for (int i = 1; i <= nv; ++i) { Vec2 q = i < nv ? V[i] : start; lineTo(q); if (i < nv && g.uni() < 0.6) { if (g.uni() < 0.5) cluster(pickFactor(), g.irange(2, 12), true); else netTurn((g.uni() < 0.5 ? 1 : -1) * g.range(2.6, 6.5), g.irange(3, 30), pickFactor()); } }
            if (p != start) lineTo(start);
            k.closed = true; break; }
    }
    // d: tol-scale, around the cluster extent, ordinary, or large
    double u = g.uni();
    if (u < 0.2) c.d = tol * g.range(1.5, 6.0); else if (u < 0.3) c.d = std::max(1.5 * tol, c.factor * tol * g.range(0.4, 1.6)); else if (u < 0.75) c.d = std::exp(g.range(std::log(0.01), std::log(5.0))); else c.d = std::exp(g.range(std::log(5.0), std::log(100.0)));
    if (g.uni() < 0.2) { Vec2 t{g.range(-1e3, 1e3), g.range(-1e3, 1e3)}; translate(k, t); }
    return c;
}
