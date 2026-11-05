// extracted from x2.cpp (g2 generator) for the small probes
// ------------------------------------------------------------- g2 generator
struct G2 { Contour in; double d, mitre, tol = 0.001; OffsetSide side; JoinType join; int kind; };
static double tinyLen(Rng& g, double tol) { double u = g.uni(); if (u < 0.4) return tol * std::pow(10.0, -g.range(0, 10)); if (u < 0.55) return tol * g.range(0.95, 1.0); return tol * g.range(1e-3, 1.0); }
static G2 makeG2(std::uint64_t seed) {
    Rng g(seed * 0x9E3779B97F4A7C15ULL + 0xA5A5);
    G2 c; c.kind = g.irange(0, 6);
    c.d = std::exp(g.range(std::log(0.01), std::log(5.0)));
    c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    c.join = (JoinType)g.irange(0, 2);
    c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 4.0);
    const double tol = c.tol;
    Contour& k = c.in;
    Vec2 p{0, 0}; double h = g.range(-3.14, 3.14);
    auto line = [&](double L) { Vec2 q = p + Vec2::fromAngle(h, L); k.spans.push_back(LineSpan{p, q}); p = q; };
    auto tinyRun = [&](int m, int style) {  // run of m sub-tol spans starting at p, continuous
        double hh = h;
        for (int i = 0; i < m; ++i) {
            double L = tinyLen(g, tol);
            if (style == 0) hh = g.range(-3.14159, 3.14159);                  // micro cluster
            else if (style == 1) hh = h + (i % 2 ? 1 : -1) * g.range(0.3, 1.4); // zig-zag
            else hh = h + g.range(-0.3, 0.3);                                   // noisy sampled line
            Vec2 q = p + Vec2::fromAngle(hh, L); if (q == p) continue; k.spans.push_back(LineSpan{p, q}); p = q;
        }
    };
    switch (c.kind) {
        case 0: {  // sharp corners with sub-tol runs at them
            int nv = g.irange(2, 6);
            for (int i = 0; i < nv; ++i) { line(g.range(0.2, 3)); if (g.uni() < 0.7) tinyRun(g.irange(1, 20), g.irange(0, 2)); h += g.range(-2.8, 2.8); }
            line(g.range(0.2, 3)); break; }
        case 1: {  // open chain made only of tiny pieces
            int m = g.irange(3, 400); int st = g.irange(0, 2);
            if (g.uni() < 0.3) {  // densely sampled arc of radius R
                double R = g.range(0.01, 2), a0 = g.range(0, 6.28), sw = g.range(-6, 6); int nn = std::max(3, (int)(R * std::fabs(sw) / (tol * g.range(0.3, 1.0))));
                nn = std::min(nn, 3000);
                Vec2 prev{R * std::cos(a0), R * std::sin(a0)};
                for (int i = 1; i <= nn; ++i) { double a = a0 + sw * i / nn; Vec2 q{R * std::cos(a), R * std::sin(a)}; if (q != prev) k.spans.push_back(LineSpan{prev, q}); prev = q; }
                p = prev;  // (round-2 verifier fix: closing line starts at the last sample)
            } else tinyRun(m, st);
            if (g.uni() < 0.2 && k.spans.size() > 3) { k.spans.push_back(LineSpan{p, startPoint(k.spans.front())}); if (length(k.spans.back()) > 0) k.closed = true; else k.spans.pop_back(); }
            break; }
        case 2: {  // B1: tiny large-sweep arc between nearly collinear lines
            line(g.range(0.1, 3));
            double sw = g.range(0.5, 6.2831) * (g.uni() < 0.5 ? 1 : -1);
            double r = std::pow(10.0, -g.range(4, 16));
            Vec2 left = Vec2::fromAngle(h).perp(); Vec2 ctr = sw > 0 ? p + left * r : p - left * r;
            ArcSpan a{ctr, r, (p - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW}; a.endAngle = a.startAngle + sw;
            k.spans.push_back(a); Vec2 e = a.endPoint();
            h += g.range(-1e-3, 1e-3) + (g.uni() < 0.3 ? sw : 0.0);
            p = e; line(g.range(0.1, 3)); break; }
        case 3: {  // collapsing arcs: radius equal to d (to rounding) or just around it
            line(g.range(0.2, 2));
            double r = c.d * (g.uni() < 0.5 ? 1.0 : 1.0 + g.range(-1e-9, 1e-9)); double sw = g.range(0.2, 3.0) * (g.uni() < 0.5 ? 1 : -1);
            Vec2 left = Vec2::fromAngle(h).perp(); Vec2 ctr = sw > 0 ? p + left * r : p - left * r;
            ArcSpan a{ctr, r, (p - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW}; a.endAngle = a.startAngle + sw;
            k.spans.push_back(a); p = a.endPoint(); h += sw + g.range(-0.5, 0.5);
            if (g.uni() < 0.5) tinyRun(g.irange(1, 5), 0);
            line(g.range(0.2, 2)); break; }
        case 4: {  // long noisy sampled line through corners (Sharp/Square heavy), tiny ends
            if (g.uni() < 0.5) tinyRun(g.irange(1, 6), 0);
            int nv = g.irange(1, 4);
            for (int i = 0; i < nv; ++i) { tinyRun(g.irange(20, 300), 2); h += g.range(-2.5, 2.5); line(g.range(0.05, 1)); }
            if (g.uni() < 0.5) tinyRun(g.irange(1, 6), 0);
            break; }
        case 5: {  // closed polygon with micro-clusters and tiny closing spans
            int nv = g.irange(3, 7); double R = g.range(0.3, 3); Vec2 c0{0, 0};
            std::vector<Vec2> V; for (int i = 0; i < nv; ++i) V.push_back(c0 + Vec2::fromAngle(6.283185307179586 * i / nv + g.range(-0.3, 0.3), R * g.range(0.5, 1.5)));
            p = V[0];
            for (int i = 0; i < nv; ++i) {
                Vec2 q = V[(i + 1) % nv]; Vec2 tgt = q;
                if (g.uni() < 0.6) { int m = g.irange(1, 15); Vec2 back = q - (q - p).normalized() * (m * tol * 0.5); k.spans.push_back(LineSpan{p, back}); p = back; h = (q - p).angle(); for (int j = 0; j < m - 1; ++j) { Vec2 r2 = p + Vec2::fromAngle(g.range(-3.14, 3.14), tinyLen(g, tol) * 0.4); if (r2 != p) { k.spans.push_back(LineSpan{p, r2}); p = r2; } } if (p != tgt) k.spans.push_back(LineSpan{p, tgt}); }
                else k.spans.push_back(LineSpan{p, tgt});
                p = tgt;
            }
            k.closed = true; break; }
        default: {  // tiny arcs of any radius/sweep sprinkled along a path with Beziers
            int nv = g.irange(2, 6);
            for (int i = 0; i < nv; ++i) {
                line(g.range(0.1, 2));
                double u = g.uni();
                if (u < 0.4) { double sw = g.range(-6.2, 6.2); double L = tinyLen(g, tol); double r = L / std::max(1e-6, std::fabs(sw));
                    Vec2 left = Vec2::fromAngle(h).perp(); Vec2 ctr = sw > 0 ? p + left * r : p - left * r;
                    ArcSpan a{ctr, r, (p - ctr).angle(), 0, sw > 0 ? ArcDirection::CCW : ArcDirection::CW}; a.endAngle = a.startAngle + sw; k.spans.push_back(a); p = a.endPoint(); h += sw; }
                else if (u < 0.7) { Vec2 q = p + Vec2::fromAngle(h + g.range(-2, 2), g.range(0.05, 1)); Vec2 c1 = p + Vec2::fromAngle(h, tinyLen(g, tol)); Vec2 c2 = q + Vec2::fromAngle(g.range(-3, 3), g.range(0, 0.5)); k.spans.push_back(BezierSpan{p, c1, c2, q}); h = (q - c2).angle(); p = q; }
                h += g.range(-2, 2);
            }
            line(g.range(0.1, 2));
        }
    }
    if (g.uni() < 0.2) { Vec2 t{g.range(-1e3, 1e3), g.range(-1e3, 1e3)}; translate(k, t); }
    return c;
}

