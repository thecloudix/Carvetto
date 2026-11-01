// ================================================================ T-0017 verifier: new adversarial generators
int g_gen4 = 0;
double logU(Rng& g, double a, double b) { return std::exp(g.range(std::log(a), std::log(b))); }

// Tapered slot: wall 1 eastward with a shallow dip in the middle (V of two lines, a polyline
// parabola, or a concave arc), far detour, wall 2 westward at y = 2d. Left offsets (inside the U)
// form a lens of max width ~w and length ~L between the wall offsets (valid tool positions).
Contour lensSlot(Rng& g, double d, double w, double L, int kind) {
    std::vector<Vec2> pts;
    const double ext = std::max(1.0, 0.3 * L);  // wall continues beyond the lens with gap < 2d
    const double s = 2 * w / L;                 // slope of the V
    double up = 10 + 4 * d;
    Contour c;
    if (kind == 0) {  // V
        pts.push_back({-L / 2 - ext, -w + s * (L / 2 + ext)});
        pts.push_back({0, -w});
        pts.push_back({L / 2 + ext, -w + s * (L / 2 + ext)});
    } else if (kind == 1) {  // polyline parabola y = -w + w (2x/L)^2, k pieces
        int k = g.irange(3, 40);
        double X = L / 2 + ext;
        for (int i = 0; i <= k; ++i) {
            double x = -X + 2 * X * i / k;
            pts.push_back({x, -w + w * (2 * x / L) * (2 * x / L)});
        }
    }
    if (kind == 2) {  // concave arc: circle below, radius R = L^2 / (8 w), lowest point (0,-w)
        double R = L * L / (8 * w);
        if (R > 5e5) R = 5e5;
        double X = L / 2 + ext;
        double half = std::asin(std::min(1.0, X / R));
        Vec2 C{0, -w + R};  // centre above: arc is a smile, y = C.y - sqrt(R^2 - x^2)
        ArcSpan a{C, R, -kPi / 2 - half, -kPi / 2 + half, ArcDirection::CCW};
        c.spans.push_back(a);
        Vec2 e = a.endPoint();
        std::vector<Vec2> q{e, {e.x + 1, up}, {L / 2 + 0.5, up + 1}, {L / 2 + ext * 0.9, 2 * d}, {-L / 2 - ext * 0.9, 2 * d}};
        for (std::size_t i = 1; i < q.size(); ++i) c.spans.push_back(LineSpan{q[i - 1], q[i]});
        return c;
    }
    Vec2 e = pts.back();
    pts.push_back({e.x + 1, up});
    pts.push_back({L / 2 + 0.5, up + 1});
    pts.push_back({L / 2 + ext * 0.9, 2 * d});
    pts.push_back({-L / 2 - ext * 0.9, 2 * d});
    return fromPts(pts, false);
}

// Two strands whose offsets nearly coincide: angle th between them (around the 1e-3 rad limit),
// lateral offset lat, short or long elements, same or opposite direction; strand 2 may be split
// into short pieces with alternating tiny angles (chaining / drift).
Contour nearAngle(Rng& g, double d, double tol, bool& opposite) {
    std::vector<Vec2> pts;
    double th = logU(g, 1e-5, 0.3) * (g.uni() < 0.5 ? -1 : 1);
    double L = g.uni() < 0.5 ? logU(g, 1.2 * tol, 0.02) : logU(g, 0.02, 5.0);
    double lat = g.range(-1.0, 1.0) * tol * (g.uni() < 0.5 ? 0.1 : 1.0);
    opposite = g.uni() < 0.5;
    double up = 10 + 4 * d;
    double x0 = g.range(-0.5, 0.5) * L;
    pts.push_back({-L, 0});
    pts.push_back({0, 0});
    pts.push_back({L, 0});
    pts.push_back({L + 0.5, up});
    int k = g.irange(1, 12);
    double piece = 2 * L / k;
    if (opposite) {
        pts.push_back({x0 + L + 0.5, up + 1});
        double y = 2 * d + lat, x = x0 + L;
        pts.push_back({x, y});
        for (int i = 0; i < k; ++i) {
            double a = th * (g.uni() < 0.3 ? -1 : 1);
            x -= piece * std::cos(a);
            y += piece * std::sin(a);
            pts.push_back({x, y});
        }
    } else {
        pts.push_back({x0 - L - 0.5, up + 1});
        double y = lat, x = x0 - L;
        pts.push_back({x, y});
        for (int i = 0; i < k; ++i) {
            double a = th * (g.uni() < 0.3 ? -1 : 1);
            x += piece * std::cos(a);
            y += piece * std::sin(a);
            pts.push_back({x, y});
        }
    }
    return fromPts(pts, false);
}

// Hairpin: out along +x, U-turn of radius rho (arc) or two short lines, back along -x, at
// width ~ 2 rho; offset on the outside (Round) of the turn gives an arc about the tip, on the
// inside a needle whose width is 2(rho - d). d chosen near rho.
Contour hairpin(Rng& g, double& d, double tol) {
    Builder b;
    b.h = 0;
    double L = g.range(0.5, 5);
    double rho = logU(g, 0.01, 2.0);
    b.line(L);
    double k = g.uni();
    if (k < 0.5) b.arc(rho, kPi * (1 + g.range(-1e-3, 1e-3)));
    else {
        double turn = kPi * (1 + g.range(-0.02, 0.02));
        int m = g.irange(1, 4);
        for (int i = 0; i < m; ++i) { b.h += turn / (m + 1); b.line(2 * rho / m); }
        b.h += turn / (m + 1);
    }
    b.line(L * g.range(0.6, 1.4));
    double rel[] = {1e-12, 1e-9, 1e-7, 1e-6, 1e-5, 1e-4, 5e-4, 1e-3, 2e-3, 0.01};
    double r = rel[g.irange(0, 9)] * (g.uni() < 0.5 ? -1 : 1);
    d = rho * (1 + r) + (g.uni() < 0.3 ? g.range(-1, 1) * tol : 0.0);
    if (d < 0.005) d = 0.005;
    return b.c;
}

// Two arcs whose offsets nearly coincide: centres delta apart, same or opposite direction,
// meeting end to end or overlapping; joined by far detours.
Contour coCircNear(Rng& g, double d, double tol) {
    Contour c;
    double r0 = g.range(0.2, 3.0);
    double delta = logU(g, 1e-10, 2 * tol);
    Vec2 C1{0, 0}, C2 = Vec2::fromAngle(g.range(-kPi, kPi), delta);
    double dr = g.range(-1, 1) * logU(g, 1e-10, 2 * tol);
    bool opp = g.uni() < 0.5;
    double a0 = g.range(-kPi, kPi), sw = g.range(0.2, 2.5);
    ArcSpan A{C1, r0, a0, a0 + sw, ArcDirection::CCW};  // Right offset at r0 + d
    ArcSpan B;
    double ov = g.uni() < 0.5 ? 0.0 : g.range(-0.5, 0.5) * sw;
    if (opp) {
        double rb = r0 + 2 * d + dr;  // CW arc of radius r0+2d: Right offset at r0+d
        double s0 = a0 + sw + ov;
        B = ArcSpan{C2, rb, s0, s0 - sw * g.range(0.5, 1.5), ArcDirection::CW};
    } else {
        double s0 = a0 + sw - ov;  // continues (or overlaps) A
        B = ArcSpan{C2, r0 + dr, s0, s0 + sw * g.range(0.5, 1.5), ArcDirection::CCW};
    }
    c.spans.push_back(A);
    double far = r0 + 2 * d + 20 + 3 * d;
    Vec2 pe = A.endPoint(), st = B.startPoint();
    if (!opp && ov == 0.0 && g.uni() < 0.6) {
        // end to end: connect with a tiny line (or exact if ends coincide)
        if (distance(pe, st) > 1e-15) c.spans.push_back(LineSpan{pe, st});
    } else {
        Vec2 m1 = Vec2::fromAngle(pe.angle(), far), m2 = Vec2::fromAngle(st.angle() + 0.1, far);
        c.spans.push_back(LineSpan{pe, m1});
        c.spans.push_back(LineSpan{m1, m2});
        c.spans.push_back(LineSpan{m2, st});
    }
    c.spans.push_back(B);
    return c;
}

// Chained near-collinear strands (transitive drift): k eastward strands on y_i = i*eps with
// small angles, joined by detours, all left offsets near y = d.
Contour chainDrift(Rng& g, double d, double tol) {
    std::vector<Vec2> pts;
    int k = g.irange(3, 6);
    double eps = g.range(0.1, 0.6) * tol;
    double up = 10 + 4 * d;
    double L = logU(g, 0.003, 3.0);
    for (int i = 0; i < k; ++i) {
        double y0 = i * eps * (g.uni() < 0.8 ? 1 : -1), th = g.range(-1, 1) * 1.5e-3;
        double xs = g.range(-0.3, 0.3) * L;
        if (i > 0) {
            Vec2 pe = pts.back();
            pts.push_back({pe.x + 0.5, (i % 2 ? up : -up)});
            pts.push_back({xs - 0.5, (i % 2 ? up : -up) + 1});
        }
        pts.push_back({xs, y0});
        pts.push_back({xs + L * std::cos(th), y0 + L * std::sin(th)});
    }
    return fromPts(pts, false);
}

// Rounded steep zig-zag: peaks are small arcs (fillets), with noise.
Contour roundedZig(Rng& g, double& d) {
    Builder b;
    double p = g.range(0.02, 0.4), a = p * g.range(1, 20);
    double ang = std::atan2(a, p / 2);
    double rf = std::max(logU(g, 1e-3, 0.05) * p, 1.6e-3 / (2 * ang));  // fillet arcs longer than tol
    int n = g.irange(3, 20);
    b.h = ang;
    for (int i = 0; i < n; ++i) {
        b.line(std::hypot(a, p / 2) * 0.9);
        double turn = (i % 2 == 0 ? -1 : 1) * 2 * ang;
        b.arc(rf, turn);
    }
    b.line(std::hypot(a, p / 2) * 0.9);
    d = a * g.range(0.3, 6);
    return b.c;
}

Case2 makeCase4(std::uint64_t seed) {
    Rng g(seed ^ 0x7F4A7C15D1B54A32ULL);
    Case c;
    c.tol = 0.001;
    c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 4.2);
    c.join = static_cast<JoinType>(g.irange(0, 2));
    c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    c.d = logU(g, 0.05, 5.0);
    c.gen = g.irange(30, 36);
    const char* nm = "?";
    bool noisy = false;
    switch (c.gen) {
        case 30: {
            nm = "lensSlot";
            double w = logU(g, c.tol / 64, 3 * c.tol), L = logU(g, 0.005, 10.0);
            c.in = lensSlot(g, c.d, w, L, g.irange(0, 2));
            c.side = OffsetSide::Left;
            if (g.uni() < 0.3) { noisy = true; perturb(c.in, g, logU(g, 1e-12, 1e-6)); }
            break;
        }
        case 31: { nm = "nearAngle"; bool o; c.in = nearAngle(g, c.d, c.tol, o); c.side = OffsetSide::Left;
                   if (g.uni() < 0.3) { noisy = true; perturb(c.in, g, logU(g, 1e-12, 1e-5)); } break; }
        case 32: { nm = "hairpin"; c.in = hairpin(g, c.d, c.tol); if (g.uni() < 0.6) c.join = JoinType::Round;
                   if (g.uni() < 0.3) { noisy = true; perturb(c.in, g, logU(g, 1e-12, 1e-6)); } break; }
        case 33: { nm = "coCircNear"; c.in = coCircNear(g, c.d, c.tol); c.side = g.uni() < 0.8 ? OffsetSide::Right : OffsetSide::Left; noisy = true; break; }
        case 34: { nm = "chainDrift"; c.in = chainDrift(g, c.d, c.tol); c.side = OffsetSide::Left; noisy = true; break; }
        case 35: { nm = "roundedZig"; c.in = roundedZig(g, c.d);
                   if (g.uni() < 0.5) { noisy = true; perturb(c.in, g, logU(g, 1e-12, 1e-4)); } break; }
        default: { nm = "shortWalkFar"; c.in = walk(g, g.irange(20, 200), logU(g, 1.1 * c.tol, 0.01), g.range(0.01, 1.5)); c.d = logU(g, 0.02, 1.0); break; }
    }
    double u = g.uni();
    if (u < 0.2) transformC(c.in, g.range(-kPi, kPi), Vec2{0, 0});
    else if (u < 0.45) transformC(c.in, g.uni() < 0.5 ? 0.0 : g.range(-kPi, kPi), Vec2{g.range(-1e4, 1e4), g.range(-1e4, 1e4)});
    else if (u < 0.6) transformC(c.in, 0.0, Vec2{g.uni() < 0.5 ? 1e4 : -1e4, g.uni() < 0.5 ? 1e4 : -1e4});
    return {c, nm, noisy};
}
