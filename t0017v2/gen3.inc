// ================================================================ round-3 adversarial generators
int g_gen3 = 0;  // 0: makeCase2 families, 1: round-3 families
void restitch(Contour& c) {
    const std::size_t n = c.spans.size();
    for (std::size_t i = 0; i < n; ++i) {
        std::size_t j = i + 1;
        if (j == n) { if (!c.closed) break; j = 0; }
        if (auto* l = std::get_if<LineSpan>(&c.spans[j])) l->start = endPoint(c.spans[i]);
        else if (auto* l2 = std::get_if<LineSpan>(&c.spans[i])) l2->end = startPoint(c.spans[j]);
    }
}
// perturb line vertices (shared points) and arc centres/radii by `amp`
void perturb(Contour& c, Rng& g, double amp) {
    const std::size_t n = c.spans.size();
    // perturb line vertex positions consistently: compute moved points per junction
    std::vector<Vec2> jp(n + 1);
    for (std::size_t i = 0; i < n; ++i) jp[i] = startPoint(c.spans[i]);
    jp[n] = endPoint(c.spans[n - 1]);
    for (auto& p : jp) p += Vec2{g.range(-amp, amp), g.range(-amp, amp)};
    if (c.closed) jp[n] = jp[0];
    for (std::size_t i = 0; i < n; ++i) {
        if (auto* l = std::get_if<LineSpan>(&c.spans[i])) { l->start = jp[i]; l->end = jp[i + 1]; }
        else if (auto* a = std::get_if<ArcSpan>(&c.spans[i])) {
            a->centre += Vec2{g.range(-amp, amp), g.range(-amp, amp)};
            a->radius = std::max(a->radius + g.range(-amp, amp), 1e-6);
        }
    }
    restitch(c);
}
// Two (or three) strands whose offsets lie at lateral separation ~ tol/2 and tilt ~ the
// parallel threshold (|u x v| * L ~ tol/2), opposite or same directions, joined by detours.
Contour groupBoundary(Rng& g, double d, double tol, OffsetSide& side) {
    std::vector<Vec2> pts;
    double L = g.range(0.3, 8.0);
    double fs[] = {0.5, 0.9, 0.98, 0.999, 1.0, 1.001, 1.02, 1.1, 2.0};
    double f1 = g.uni() < 0.7 ? fs[g.irange(0, 8)] : g.range(0.0, 2.5);
    double f2 = g.uni() < 0.7 ? fs[g.irange(0, 8)] : g.range(0.0, 2.5);
    double lat = f1 * tol / 2 * (g.uni() < 0.5 ? -1 : 1);
    double tilt = f2 * tol / 2 * (g.uni() < 0.5 ? -1 : 1);  // y change across length L
    double x0 = g.range(-1, 1), xs = g.range(-0.5 * L, 0.5 * L);
    bool opposite = g.uni() < 0.65;
    side = OffsetSide::Left;
    double up = 20 + 4 * d;
    pts.push_back({x0, 0});
    pts.push_back({x0 + L, 0});
    pts.push_back({x0 + L + g.range(0.5, 2), up});
    if (opposite) {
        // westward at y = 2d + lat; left offset south at y = d + lat
        pts.push_back({x0 + xs + L + g.range(0, 1), up + 1});
        pts.push_back({x0 + xs + L, 2 * d + lat + tilt});
        pts.push_back({x0 + xs, 2 * d + lat});
        if (g.uni() < 0.4) {  // third strand eastward again on y ~ 0 (+lat2)
            pts.push_back({x0 + xs - g.range(0.5, 2), -up});
            double xs2 = g.range(-0.5 * L, 0.5 * L);
            pts.push_back({x0 + xs2, g.range(-1, 1) * tol / 2});
            pts.push_back({x0 + xs2 + L, g.range(-1, 1) * tol / 2});
        }
    } else {
        // eastward again at y = lat (offset at d + lat), overlapping first strand's offset
        pts.push_back({x0 + xs - g.range(0.5, 2), up + 1});
        pts.push_back({x0 + xs, lat});
        pts.push_back({x0 + xs + L, lat + tilt});
    }
    return fromPts(pts, false);
}
// Opposite strands 2d apart; the return strand is a wiggle of short pieces at moderate angles,
// so its offset pieces cross the other strand's offset at 1-30 degrees.
Contour crossShort(Rng& g, double d, double tol) {
    std::vector<Vec2> pts;
    double L = g.range(0.5, 5.0);
    pts.push_back({0, 0});
    pts.push_back({L, 0});
    double up = 15 + 4 * d;
    pts.push_back({L + 1, up});
    pts.push_back({L + 0.5, 2 * d + g.range(-1, 1) * tol});
    double x = L + 0.5 - g.range(0, 0.2), y = pts.back().y;
    double angMax = g.range(0.017, 0.52);
    double stepMin = tol * g.range(1.05, 3), stepMax = std::max(stepMin * 1.5, g.range(0.005, 0.1));
    int sgn = 1;
    while (x > -0.5) {
        double a = sgn * g.range(0.3, 1.0) * angMax;
        double st = g.range(stepMin, stepMax);
        // stay within +-amp of 2d
        double ny = y + st * std::sin(a);
        if (std::fabs(ny - 2 * d) > g.range(0.2, 3) * tol * 10) { a = -a; ny = y + st * std::sin(a); }
        x -= st * std::cos(a);
        y = ny;
        pts.push_back({x, y});
        if (g.uni() < 0.6) sgn = -sgn;
        if (pts.size() > 3000) break;
    }
    return fromPts(pts, false);
}
// zig-zag whose peaks' chamfers just touch end to end (gen 13), plus vertex noise
Contour touchNoisy(Rng& g, double& d, JoinType& join, double& mitre, bool& noisy) {
    double p = g.range(0.02, 0.5), ratio = g.range(0.5, 25.0), a = p * ratio;
    double al = std::atan2(a, p);
    double deltas[] = {0, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6, 1e-4, -1e-4, 1e-3, -1e-3};
    d = p / std::tan(al / 2) * (1 + deltas[g.irange(0, 10)]);
    if (d > 40) d = 40;
    join = g.uni() < 0.6 ? JoinType::Square : JoinType::Sharp;
    if (join == JoinType::Sharp) mitre = g.uni() < 0.5 ? g.range(1.0, 1.5) : g.range(1.5, 4.0);
    double jy = g.uni() < 0.2 ? 0.0 : std::pow(10.0, g.range(-10, -3.3));
    noisy = jy > 0;
    Rng j2(g.next());
    return zigzagPts(g.irange(3, 40), p, a, jy > 0 ? &j2 : nullptr, jy, g.uni() < 0.5 ? 0.0 : jy);
}
struct Case3 { Case c; const char* name; bool noisy = false; };
Case2 makeCase3(std::uint64_t seed) {
    Rng g(seed ^ 0x3C6EF372FE94F82BULL);
    Case c;
    c.tol = 0.001;
    c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 4.2);
    c.join = static_cast<JoinType>(g.irange(0, 2));
    if (g.uni() < 0.6 && c.join == JoinType::Round) c.join = g.uni() < 0.5 ? JoinType::Square : JoinType::Sharp;
    c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    c.d = std::exp(g.range(std::log(0.05), std::log(7.0)));
    c.gen = g.irange(20, 27);
    const char* nm = "?";
    bool noisy = false;
    switch (c.gen) {
        case 20: {
            if (g.uni() < 0.5) { nm = "grpBoundary"; c.in = groupBoundary(g, c.d, c.tol, c.side); }
            else {  // zig-zag with vertex noise ~ tol/2 (beyond the coder's 1e-4 sweep)
                nm = "zigzag-noise-tolhalf"; noisy = true;
                double p = g.range(0.02, 0.5), a = p * g.range(0.5, 25.0);
                double jy = c.tol * g.range(0.05, 0.6);
                Rng j2(g.next());
                c.in = zigzagPts(g.irange(3, 30), p, a, &j2, jy, g.uni() < 0.5 ? 0.0 : jy);
                c.d = a * g.range(0.5, 12);
                if (c.join == JoinType::Round) c.join = JoinType::Square;
            }
            break;
        }
        case 21: {
            if (g.uni() < 0.6) { nm = "crossShort"; c.in = crossShort(g, c.d, c.tol); c.side = g.uni() < 0.75 ? OffsetSide::Left : OffsetSide::Right; }
            else { nm = "shortWalk"; c.in = walk(g, g.irange(20, 300), g.range(0.0012, 0.03), g.range(0.02, 0.6)); c.d = std::exp(g.range(std::log(0.02), std::log(3.0))); }
            break;
        }
        case 22: {  // outermost-copy ties: collinear runs, exact or with lateral noise 1e-13..1e-8
            nm = "outerTie";
            c.in = collinearRuns(g, c.d, g.uni() < 0.7);
            c.side = OffsetSide::Left;
            if (g.uni() < 0.5) { noisy = true; perturb(c.in, g, std::pow(10.0, g.range(-13, -8))); }
            break;
        }
        case 23: { nm = "touchNoisy"; c.in = touchNoisy(g, c.d, c.join, c.mitre, noisy); break; }
        case 24: {  // noisy co-circular arcs
            nm = "coCircularNoisy";
            c.in = coCircular(g, c.d, g.uni() < 0.7);
            c.side = g.uni() < 0.7 ? OffsetSide::Right : OffsetSide::Left;
            if (g.uni() < 0.8) { noisy = true; perturb(c.in, g, std::pow(10.0, g.range(-13, -4))); }
            break;
        }
        case 25: {  // closed zig-zag, seam inside merged stretches, noise up to 1e-3
            nm = "closedSeamNoisy";
            double p = g.range(0.02, 0.5), a = p * g.range(0.5, 25.0);
            int n = 2 * g.irange(2, 16);
            double jyz = g.uni() < 0.25 ? 0.0 : std::pow(10.0, g.range(-13, -3.2));
            noisy = jyz > 0;
            c.in = closedZigzag(g, n, p, a, jyz);
            rotateSpans(c.in, g.irange(0, n));
            c.d = a * g.range(0.3, 8);
            break;
        }
        case 26: {  // steep zig-zag at near-parallel-test angles (very tall, tiny pitch), noise
            nm = "zigzag-tall";
            double p = g.range(0.002, 0.05), a = g.range(0.5, 5.0);
            double jy = g.uni() < 0.4 ? 0.0 : std::pow(10.0, g.range(-12, -4));
            noisy = jy > 0;
            Rng j2(g.next());
            c.in = zigzagPts(g.irange(3, 40), p, a, jy > 0 ? &j2 : nullptr, jy, jy);
            c.d = a * g.range(0.2, 4);
            if (c.join == JoinType::Round && g.uni() < 0.7) c.join = JoinType::Sharp;
            break;
        }
        default: {  // general mixed lines/arcs with noise
            nm = "mixedNoisy";
            Case o = makeCase(seed ^ 0xA5A5F00DULL);
            c.in = o.in; c.d = o.d; c.join = o.join; c.side = o.side; c.mitre = o.mitre;
            if (g.uni() < 0.7 && !c.in.spans.empty()) { noisy = true; perturb(c.in, g, std::pow(10.0, g.range(-12, -4))); }
            break;
        }
    }
    double u = g.uni();
    if (u < 0.2) transformC(c.in, g.range(-kPi, kPi), Vec2{0, 0});
    else if (u < 0.45) transformC(c.in, g.uni() < 0.5 ? 0.0 : g.range(-kPi, kPi), Vec2{g.range(-1e4, 1e4), g.range(-1e4, 1e4)});
    else if (u < 0.6) transformC(c.in, 0.0, Vec2{g.uni() < 0.5 ? 1e4 : -1e4, g.uni() < 0.5 ? 1e4 : -1e4});
    else if (u < 0.7) transformC(c.in, 0.0, Vec2{g.range(-1000, 1000), g.range(-1000, 1000)});
    return {c, nm, noisy};
}
Case2 makeCaseX(std::uint64_t seed) { return g_gen3 ? makeCase3(seed) : makeCase2(seed); }

// ---------------------------------------------------------------- needle-tip classifier
struct TipStats { long tips = 0, genuine = 0, suspicious = 0, inputRev = 0, printed = 0; double minRatio = 1e9; };
double tipCheck(const Contour& in, const std::vector<Contour>& out, double d, double tol, TipStats& ts, const char* tag, std::uint64_t seed) {
    std::vector<Seg> inS;
    for (auto& s : in.spans) inS.push_back(toSeg(s));
    double worstRatio = 1e9;
    for (const Contour& c : out) {
        const std::size_t n = c.spans.size();
        for (std::size_t j = 0; j < n; ++j) {
            if (j + 1 == n && !c.closed) break;
            const Span& a = c.spans[j];
            const Span& b = c.spans[(j + 1) % n];
            double dt = tangentAt(a, 1.0).dot(tangentAt(b, 0.0));
            if (dt >= -0.9999) continue;
            ts.tips++;
            double phi = std::acos(std::clamp(-dt, -1.0, 1.0));  // opening angle of the needle
            double la = length(a), lb = length(b), sh = std::min(la, lb);
            double ratio = 1e9;
            for (double f : {0.001, 0.01, 0.1, 0.3, 0.5, 1.0}) {
                double delta = f * sh;
                Vec2 pa = pointAt(a, std::max(0.0, 1.0 - delta / la));
                Vec2 pb = pointAt(b, std::min(1.0, delta / lb));
                double apart = std::min(carvetto::geometry::testing::distanceToSpanExact(pb, a), carvetto::geometry::testing::distanceToSpanExact(pa, b));
                ratio = std::min(ratio, apart / (delta * std::sin(phi)));
            }
            // nearest input vertex turning
            Vec2 J = endPoint(a);
            double best = 1e300; double vdot = 1;
            for (std::size_t k = 0; k + 1 < inS.size() || (in.closed && k < inS.size()); ++k) {
                std::size_t k2 = (k + 1) % inS.size();
                Vec2 V = inS[k].b;
                double dd = distance(V, J);
                if (dd < best) { best = dd; vdot = segTan(inS[k], 1).dot(segTan(inS[k2], 0)); }
            }
            double dj = 1e300;
            for (auto& s : inS) dj = std::min(dj, distPt(s, J));
            bool inRev = vdot < -0.99;
            if (inRev) ts.inputRev++;
            bool ok = ratio > 0.25 && sh > tol && dj >= d - allowance(d, tol) - 1e-12 * std::max(1.0, J.length());
            if (ok) ts.genuine++;
            else {
                ts.suspicious++;
                if (ts.printed < 15) {
                    ts.printed++;
                    std::printf("  TIP? %s seed=0x%llx dot %.9f phi %.3g ratio %.3g la %.3g lb %.3g tipDist-d %.3g nearestInputVertexDot %.4f dist %.3g\n", tag, (unsigned long long)seed, dt, phi, ratio, la, lb, dj - d, vdot, best);
                }
            }
            worstRatio = std::min(worstRatio, ratio);
            ts.minRatio = std::min(ts.minRatio, ratio);
        }
    }
    return worstRatio;
}
