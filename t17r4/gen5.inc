// ================================================================ T-0017 round-2 verifier: generators aimed at the round-2 code
// 40 tieGap      collinear runs on y = 0 and y = 2d + g (Round arcs about run ends 2d + g apart: centre shift)
// 41 coCircGap   co-circular arcs r0 and r0 + 2d + g (opposite), cusp / centre shift on arcs
// 42 gapCorner   walks / mixed with explicit gaps at vertices (Round corner = arc about each gap end)
// 43 farZig      steep zig-zags / collinear runs at 1e3..1e5 mm, rotated (checked line-end moves)
// 44 tinyLoops   dense walks with steps 1.05..6 tol, sharp turns (global stitching, tiny loops)
// 45 needleTrans hairpins with rho ~ d at translations 0..1e5 (needle removal translation-independent)
// 46 axisCusp    two spikes 2d + g apart with centres exactly on an axis (exactJunction centre shift)
double g5gap(Rng& g, double scale) {
    double lo = 1e-16 * scale, hi = 3e-5;
    double v = logU(g, lo, hi);
    return g.uni() < 0.15 ? 0.0 : v * (g.uni() < 0.5 ? -1 : 1);
}
Contour collinearRunsGap(Rng& g, double d, double gap) {
    std::vector<Vec2> pts;
    int k = g.irange(1, 4);
    struct Run { double y, x0, x1; };
    std::vector<Run> runs;
    for (int i = 0; i < k; ++i) {
        double a = g.range(0, 6), b = a + g.range(0.5, 6);
        runs.push_back({0.0, a, b});
        runs.push_back({2 * d + gap, b, a});
        if (g.uni() < 0.3) runs.push_back({0.0, b, a});
        if (g.uni() < 0.3) runs.push_back({2 * d + gap, a, b});
    }
    for (std::size_t i = runs.size(); i > 1; --i) std::swap(runs[i - 1], runs[static_cast<std::size_t>(g.irange(0, static_cast<int>(i) - 1))]);
    double up = 30 + 4 * d;
    for (std::size_t i = 0; i < runs.size(); ++i) {
        const Run& r = runs[i];
        if (i > 0) {
            Vec2 pe = pts.back();
            double yy = (g.uni() < 0.5 ? up : -up) + g.range(-3, 3);
            pts.push_back({pe.x + g.range(-2, 2), yy});
            pts.push_back({r.x0 + g.range(-2, 2), yy + g.range(-1, 1)});
        }
        pts.push_back({r.x0, r.y});
        pts.push_back({r.x1, r.y});
    }
    std::vector<Vec2> q;
    for (Vec2 p : pts) if (q.empty() || distance(q.back(), p) > 1e-6) q.push_back(p);
    return fromPts(q, false);
}
Contour coCircularGap(Rng& g, double d, double gap) {
    Contour c;
    Vec2 C{0, 0};
    double r0 = g.range(0.2, 3.0);
    int k = g.irange(1, 3);
    struct A { double r, a0, sw; };
    std::vector<A> arcs;
    for (int i = 0; i < k; ++i) {
        double a0 = g.range(-kPi, kPi), sw = g.range(0.3, 2.5);
        arcs.push_back({r0, a0, sw});
        arcs.push_back({r0 + 2 * d + gap, a0 + sw * g.range(0.6, 1.4), -sw * g.range(0.6, 1.4)});
    }
    for (std::size_t i = arcs.size(); i > 1; --i) std::swap(arcs[i - 1], arcs[static_cast<std::size_t>(g.irange(0, static_cast<int>(i) - 1))]);
    double far = r0 + 2 * d + 20 + 3 * d;
    for (std::size_t i = 0; i < arcs.size(); ++i) {
        const A& a = arcs[i];
        ArcSpan s{C, a.r, a.a0, a.a0 + a.sw, a.sw > 0 ? ArcDirection::CCW : ArcDirection::CW};
        if (i > 0) {
            Vec2 pe = endPoint(c.spans.back());
            Vec2 st = s.startPoint();
            Vec2 m1 = Vec2::fromAngle(pe.angle(), far + g.range(0, 3));
            Vec2 m2 = Vec2::fromAngle(st.angle() + g.range(-0.2, 0.2), far + g.range(0, 3));
            c.spans.push_back(LineSpan{pe, m1});
            c.spans.push_back(LineSpan{m1, m2});
            c.spans.push_back(LineSpan{m2, st});
        }
        c.spans.push_back(s);
    }
    return c;
}
// open input gaps: move the start of some line spans (end of previous unchanged)
void injectGaps(Contour& c, Rng& g, double lo, double hi, double frac) {
    for (std::size_t i = 1; i < c.spans.size(); ++i) {
        if (g.uni() > frac) continue;
        if (auto* l = std::get_if<LineSpan>(&c.spans[i])) {
            Vec2 v = Vec2::fromAngle(g.range(-kPi, kPi), logU(g, lo, hi));
            if (distance(l->start + v, l->end) > 0.01) l->start += v;
        }
    }
}
// two spikes (reversals) whose Round arcs nearly touch at the origin; centres on the x axis
Contour axisCusp(Rng& g, double d, double gap) {
    double r = d + gap / 2;
    double a1 = g.range(0.3, 1.3), a2 = g.range(0.3, 1.3), L = g.range(3, 8) + d;
    Vec2 P1{-r, 0}, P2{r, 0};
    Vec2 A1 = P1 + Vec2::fromAngle(kPi + a1, L), A2 = P2 + Vec2::fromAngle(-a2, L);
    double yb = std::min(A1.y, A2.y) - 2 * d - 1;
    std::vector<Vec2> pts{A1 + Vec2{0.0, -0.3}, P1, A1, {A1.x, yb}, {A2.x, yb}, A2, P2, A2 + Vec2{0.0, -0.3}};
    if (g.uni() < 0.5) {  // vertical walls instead of spikes on one side: arc meets line
        pts = {A1 + Vec2{0.0, -0.3}, P1, A1, {A1.x, yb}, {A2.x, yb}, A2, P2, A2 + Vec2{0.3, 0.0}};
    }
    return fromPts(pts, false);
}
Case2 makeCase5(std::uint64_t seed) {
    Rng g(seed ^ 0x5851F42D4C957F2DULL);
    Case c;
    c.tol = 0.001;
    c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 4.2);
    c.join = static_cast<JoinType>(g.irange(0, 2));
    c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    c.d = logU(g, 0.05, 5.0);
    c.gen = g.irange(40, 46);
    const char* nm = "?";
    bool noisy = false;
    // translation magnitude chosen first: gaps are generated relative to it
    double tsel = g.uni();
    double T = tsel < 0.2 ? 0.0 : tsel < 0.4 ? 1e3 : tsel < 0.7 ? 1e4 : tsel < 0.85 ? 1e5 : g.range(10, 1e4);
    switch (c.gen) {
        case 40: {
            nm = "tieGap";
            if (g.uni() < 0.6) c.join = JoinType::Round;
            double gp = g5gap(g, std::max(1.0, T));
            c.in = collinearRunsGap(g, c.d, gp);
            c.side = OffsetSide::Left;
            noisy = gp != 0.0;
            break;
        }
        case 41: {
            nm = "coCircGap";
            if (g.uni() < 0.6) c.join = JoinType::Round;
            double gp = g5gap(g, std::max(1.0, T));
            c.in = coCircularGap(g, c.d, gp);
            c.side = OffsetSide::Right;
            noisy = gp != 0.0;
            break;
        }
        case 42: {
            nm = "gapCorner";
            if (g.uni() < 0.7) c.join = JoinType::Round;
            if (g.uni() < 0.5) c.in = walk(g, g.irange(4, 60), g.range(0.05, 1.5), g.range(0.2, 3.0));
            else { Case o = makeCase(seed ^ 0x77AA55ULL); c.in = o.in; c.in.closed = false; if (!c.in.spans.empty() && o.in.closed) c.in.spans.pop_back(); }
            noisy = true;
            break;
        }
        case 43: {
            nm = "farZig";
            double p = g.range(0.02, 0.5), a = p * g.range(0.5, 25.0);
            if (g.uni() < 0.5) { c.in = zigzagPts(g.irange(3, 30), p, a, nullptr, 0, 0); c.d = a * g.range(0.3, 8); }
            else { c.in = collinearRunsGap(g, c.d, 0.0); }
            if (c.join == JoinType::Round && g.uni() < 0.6) c.join = g.uni() < 0.5 ? JoinType::Sharp : JoinType::Square;
            if (T < 1e3) T = 1e3 * std::pow(100.0, g.uni());
            break;
        }
        case 44: {
            nm = "tinyLoops";
            if (g.uni() < 0.5) c.in = walk(g, g.irange(20, 300), logU(g, 1.05 * c.tol, 6 * c.tol), g.range(0.5, 3.1));
            else { double p = logU(g, 2.2 * c.tol, 0.02); c.in = zigzagPts(g.irange(4, 60), p, p * g.range(0.3, 5), nullptr, 0, 0); }
            c.d = logU(g, 0.005, 1.0);
            break;
        }
        case 45: {
            nm = "needleTrans";
            c.in = hairpin(g, c.d, c.tol);
            if (g.uni() < 0.6) c.join = JoinType::Round;
            break;
        }
        default: {
            nm = "axisCusp";
            if (g.uni() < 0.7) c.join = JoinType::Round;
            double gp = g5gap(g, std::max(1.0, T));
            c.in = axisCusp(g, c.d, gp);
            noisy = gp != 0.0;
            break;
        }
    }
    double u = g.uni();
    double th = u < 0.5 ? 0.0 : (u < 0.7 ? kPi / 2 * g.irange(1, 3) : g.range(-kPi, kPi));
    Vec2 tr{0, 0};
    if (T > 0) {
        double v = g.uni();
        if (v < 0.3) tr = Vec2{T * (g.uni() < 0.5 ? 1 : -1), 0.0};          // centres stay on an axis
        else if (v < 0.6) tr = Vec2{0.0, T * (g.uni() < 0.5 ? 1 : -1)};
        else tr = Vec2{T * (g.uni() < 0.5 ? 1 : -1), g.range(-T, T)};
    }
    if (th != 0.0 || T > 0) transformC(c.in, th, tr);
    if (c.gen == 42) injectGaps(c.in, g, 1e-12, 2e-4, g.range(0.2, 1.0));
    return {c, nm, noisy};
}
