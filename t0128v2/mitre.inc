// ---------------------------------------------------------------- AC2 oracle
struct Carrier { bool arc; Vec2 P; Vec2 t; Vec2 c; double r; bool ccw; double sweepAbs; };
// candidates of carrier intersections
static void carrierX(const Carrier& A, const Carrier& B, std::vector<Vec2>& out) {
    auto lineCircle = [&](Vec2 P, Vec2 t, Vec2 c, double r) {
        Vec2 f = P - c;
        double b = f.dot(t), cc = f.dot(f) - r * r, disc = b * b - cc;
        if (disc < 0) return;
        double s = std::sqrt(disc);
        out.push_back(P + t * (-b - s));
        out.push_back(P + t * (-b + s));
    };
    if (!A.arc && !B.arc) {
        double den = A.t.cross(B.t);
        if (std::fabs(den) < 1e-14) return;
        double s = (B.P - A.P).cross(B.t) / den;
        out.push_back(A.P + A.t * s);
    } else if (!A.arc) lineCircle(A.P, A.t, B.c, B.r);
    else if (!B.arc) lineCircle(B.P, B.t, A.c, A.r);
    else {
        Vec2 dc = B.c - A.c; double D = dc.length();
        if (D == 0 || D > A.r + B.r || D < std::fabs(A.r - B.r)) return;
        double a = (A.r * A.r - B.r * B.r + D * D) / (2 * D), h = std::sqrt(std::max(0.0, A.r * A.r - a * a));
        Vec2 u = dc / D, base = A.c + u * a;
        out.push_back(base + u.perp() * h); out.push_back(base - u.perp() * h);
    }
}
// forward extension (angle for arcs, length for lines) from carrier end P to X; backward=true for B
static bool extension(const Carrier& C, Vec2 X, bool backward, double& len, double& ang) {
    if (!C.arc) {
        double e = (X - C.P).dot(C.t) * (backward ? -1 : 1);
        if (e < -1e-12) return false;
        len = std::max(0.0, e); ang = 0; return true;
    }
    double aP = (C.P - C.c).angle(), aX = (X - C.c).angle();
    bool fwdCCW = C.ccw != backward;
    double phi = fwdCCW ? wrap2pi(aX - aP) : wrap2pi(aP - aX);
    if (phi > kTwoPi - 1e-12) phi = 0;
    ang = phi; len = phi * C.r; return true;
}
struct Expect { int kind; Vec2 X, CA, CB; };  // 0 Round,1 Sharp mitre at X,2 Square chamfer CA-CB
static Expect oracle(const Carrier& A, const Carrier& B, Vec2 V, double d, double mitre, JoinType join, Vec2 nA, Vec2 nB, double tol) {
    if (join == JoinType::Sharp) {
        std::vector<Vec2> xs; carrierX(A, B, xs);
        double best = 1e300; Vec2 bx; bool ok = false; double bA = 0, bB = 0;
        for (Vec2 X : xs) {
            double la, aa, lb, ab;
            if (!extension(A, X, false, la, aa) || !extension(B, X, true, lb, ab)) continue;
            if (A.arc && aa > kTwoPi - A.sweepAbs) continue;
            if (B.arc && ab > kTwoPi - B.sweepAbs) continue;
            if (la + lb < best) { best = la + lb; bx = X; ok = true; bA = aa; bB = ab; }
        }
        if (ok && bA <= kPi / 2 + 1e-12 && bB <= kPi / 2 + 1e-12 && distance(bx, V) <= mitre * d + 1e-12) return {1, bx, {}, {}};
        join = JoinType::Square;
    }
    if (join == JoinType::Square) {
        Vec2 m = (nA + nB);
        m = m.length() < 1e-9 ? A.t : m.normalized();
        Carrier L{false, V + m * d, m.perp(), {}, 0, false, 0};
        auto hit = [&](const Carrier& C, bool backward, Vec2& out) {
            std::vector<Vec2> xs; carrierX(C, L, xs);
            double best = 1e300; bool ok = false; double bang = 0;
            for (Vec2 X : xs) {
                double l, a;
                if (!extension(C, X, backward, l, a)) continue;
                if (C.arc && a > kTwoPi - C.sweepAbs) continue;
                if (l < best) { best = l; out = X; ok = true; bang = a; }
            }
            if (!ok) return false;
            if (C.arc && bang > kPi / 2 + 1e-12) return false;
            if (distance(out, V) > 1.5 * d + 1e-12 || (C.arc && distance(out, V) > mitre * d + 1e-12)) return false;
            return true;
        };
        Vec2 CA, CB;
        if (hit(A, false, CA) && hit(B, true, CB) && (CB - CA).dot(B.P - A.P) > 0 && distance(CA, CB) > tol) return {2, {}, CA, CB};
        if (hit(A, false, CA) && hit(B, true, CB) && distance(CA, CB) <= tol) return {-1, {}, CA, CB};  // near-tangent: skip
    }
    return {0, {}, {}, {}};
}

int mitreMode(long N, std::uint64_t seed0, bool verbose, bool tinyCase) {
    long tested = 0, ok = 0, bad = 0, skipped = 0;
    long byKind[3] = {0, 0, 0};
    for (long k = 0; k < N; ++k) {
        Rng g(seed0 + static_cast<std::uint64_t>(k));
        double d = std::exp(g.range(std::log(0.1), std::log(3.0)));
        double mitre = g.uni() < 0.5 ? 2.0 : g.range(1.05, 4.0);
        JoinType join = static_cast<JoinType>(g.irange(0, 2));
        Vec2 V0{g.range(-50, 50), g.range(-50, 50)};
        double h0 = g.range(-kPi, kPi);
        double theta = g.range(0.25, 2.9) * (g.uni() < 0.5 ? -1 : 1);
        OffsetSide side = theta < 0 ? OffsetSide::Left : OffsetSide::Right;
        double sgn = side == OffsetSide::Left ? 1 : -1;
        bool arcA = g.uni() < 0.6, arcB = g.uni() < 0.6;
        if (!arcA && !arcB) arcA = true;
        Vec2 tA = Vec2::fromAngle(h0);
        Contour in;
        Vec2 V = V0;
        // span A ending at V0
        double rA = 0, rB = 0; bool ccwA = true, ccwB = true;
        auto inner = [&](bool ccw) { return (ccw && side == OffsetSide::Left) || (!ccw && side == OffsetSide::Right); };
        auto pickR = [&](bool ccw) {
            if (tinyCase && inner(ccw)) return d * g.range(1.01, 1.05);  // offset radius ~1-5 % of d
            double r = d * std::exp(g.range(std::log(0.1), std::log(6.0)));
            if (inner(ccw) && r < d * 1.01) r = d * g.range(1.01, 6.0);
            return r;
        };
        if (arcA) {
            ccwA = g.uni() < 0.5;
            if (tinyCase) ccwA = side == OffsetSide::Left;
            rA = pickR(ccwA);
            double sw = g.range(0.4, 2.2);
            Vec2 left = tA.perp();
            Vec2 c = ccwA ? V0 + left * rA : V0 - left * rA;
            double ea = (V0 - c).angle();
            ArcSpan a{c, rA, ccwA ? ea - sw : ea + sw, ea, ccwA ? ArcDirection::CCW : ArcDirection::CW};
            in.spans.push_back(a);
            V = a.endPoint();
            tA = a.tangentAt(1.0);
        } else {
            double L = d * g.range(1.5, 5.0);
            in.spans.push_back(LineSpan{V0 - tA * L, V0});
            V = V0;
        }
        Vec2 tB0 = Vec2::fromAngle(tA.angle() + theta);
        if (arcB) {
            ccwB = g.uni() < 0.5;
            rB = pickR(ccwB);
            double sw = g.range(0.4, 2.2);
            Vec2 left = tB0.perp();
            Vec2 c = ccwB ? V + left * rB : V - left * rB;
            double sa = (V - c).angle();
            ArcSpan b{c, rB, sa, ccwB ? sa + sw : sa - sw, ccwB ? ArcDirection::CCW : ArcDirection::CW};
            // ensure start is exactly V for continuity: if A is a line, set its end to b.startPoint
            if (auto* l = std::get_if<LineSpan>(&in.spans[0])) { l->end = b.startPoint(); V = l->end; }
            in.spans.push_back(b);
        } else {
            double L = d * g.range(1.5, 5.0);
            in.spans.push_back(LineSpan{V, V + tB0 * L});
        }
        Vec2 tAe = tangentAt(in.spans[0], 1.0), tBs = tangentAt(in.spans[1], 0.0);
        Vec2 nA = tAe.perp() * sgn, nB = tBs.perp() * sgn;
        auto carrier = [&](const Span& s, bool atEnd) {
            Carrier C{};
            Vec2 t = tangentAt(s, atEnd ? 1.0 : 0.0);
            C.P = V + t.perp() * sgn * d;
            C.t = t;
            if (auto* a = std::get_if<ArcSpan>(&s)) {
                C.arc = true; C.c = a->centre; C.ccw = a->direction == ArcDirection::CCW;
                C.r = inner(C.ccw) ? a->radius - d : a->radius + d;
                C.sweepAbs = std::fabs(a->sweepAngle());
                C.P = C.c + (V - C.c).normalized() * C.r;
            }
            return C;
        };
        Carrier A = carrier(in.spans[0], true), B = carrier(in.spans[1], false);
        Expect e = oracle(A, B, V, d, mitre, join, nA, nB, 0.001);
        if (e.kind < 0) { skipped++; continue; }
        auto out = offsetOpenContour(in, d, side, join, Tolerance::defaults(), mitre);
        double eps = 1e-9 * std::max(1.0, V.length());
        bool pass = false;
        std::string what;
        if (e.kind == 1) {
            for (auto& c : out)
                for (std::size_t i = 0; i + 1 < c.spans.size(); ++i)
                    if (distance(endPoint(c.spans[i]), e.X) <= eps) pass = true;
            // and no round arc about V of radius d
            for (auto& c : out) for (auto& s : c.spans) if (auto* a = std::get_if<ArcSpan>(&s)) if (distance(a->centre, V) < eps && std::fabs(a->radius - d) < eps) pass = false;
            what = "mitre";
        } else if (e.kind == 2) {
            for (auto& c : out) for (auto& s : c.spans) if (auto* l = std::get_if<LineSpan>(&s))
                if (distance(l->start, e.CA) <= eps && distance(l->end, e.CB) <= eps) pass = true;
            what = "chamfer";
        } else {
            for (auto& c : out) for (auto& s : c.spans) if (auto* a = std::get_if<ArcSpan>(&s))
                if (distance(a->centre, V) <= eps && std::fabs(a->radius - d) <= eps) pass = true;
            what = "round";
        }
        // also the general checker
        Metrics m;
        check(in, out, d, side, join, 0.001, mitre, m, true, true);
        int issues = m.lineJunctionNotBitwise + m.arcGapBeyond + m.reversals + m.zeroLen + m.subTolAtEnd + m.badSpan + m.distViol + m.roundViol + m.selfX + m.covMiss;
        tested++;
        byKind[e.kind]++;
        if (pass && !issues) ok++;
        else {
            bad++;
            if (verbose || bad <= 15) {
                std::printf("MISMATCH seed=0x%llx %s d=%.6g mitre=%.4g side=%d arcA=%d(r%.4g %s) arcB=%d(r%.4g %s) theta=%.4f expected %s",
                            static_cast<unsigned long long>(seed0 + k), jn(join), d, mitre, static_cast<int>(side), arcA, rA, ccwA ? "CCW" : "CW",
                            arcB, rB, ccwB ? "CCW" : "CW", theta, what.c_str());
                if (e.kind == 1) std::printf(" X=(%.12g,%.12g) |X-V|/d=%.4f", e.X.x, e.X.y, distance(e.X, V) / d);
                if (e.kind == 2) std::printf(" CA=(%.12g,%.12g) CB=(%.12g,%.12g)", e.CA.x, e.CA.y, e.CB.x, e.CB.y);
                std::printf(" | checker: %s\n", m.firstIssue.c_str());
                for (auto& c : out) {
                    std::printf("   chain closed=%d:", c.closed);
                    for (auto& s : c.spans) {
                        if (auto* l = std::get_if<LineSpan>(&s)) std::printf(" L(%.9g,%.9g)->(%.9g,%.9g)", l->start.x, l->start.y, l->end.x, l->end.y);
                        else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf(" A[c(%.9g,%.9g) r%.6g](%.9g,%.9g)->(%.9g,%.9g)", a->centre.x, a->centre.y, a->radius, a->startPoint().x, a->startPoint().y, a->endPoint().x, a->endPoint().y);
                    }
                    std::printf("\n");
                }
            }
        }
    }
    std::printf("mitre oracle: tested %ld (expect round %ld, mitre %ld, chamfer %ld), agree %ld, mismatch %ld, skipped(near-tangent chamfer) %ld\n",
                tested, byKind[0], byKind[1], byKind[2], ok, bad, skipped);
    return 0;
}

int keyholeMode() {
    const double w = 0.267, R = 0.5, d = 0.25;
    const double h = std::sqrt(R * R - w * w);
    Contour in;
    in.spans.push_back(LineSpan{{w, -h - 3.0}, {w, -h}});
    in.spans.push_back(ArcSpan::fromCentre({0, 0}, {w, -h}, {-w, -h}, ArcDirection::CCW));
    in.spans.push_back(LineSpan{{-w, -h}, {-w, -h - 3.0}});
    for (JoinType j : {JoinType::Sharp, JoinType::Square, JoinType::Round}) for (double mt : {2.0, 1.2, 1.25}) {
        auto out = offsetOpenContour(in, d, OffsetSide::Left, j, Tolerance::defaults(), mt);
        std::printf("keyhole %s mitre %.2f: %zu chain(s)", jn(j), mt, out.size());
        for (auto& c : out) {
            std::printf(" [");
            for (auto& s : c.spans) {
                if (auto* l = std::get_if<LineSpan>(&s)) std::printf(" L(%.6f,%.6f)->(%.6f,%.6f)", l->start.x, l->start.y, l->end.x, l->end.y);
                else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf(" A[c(%.4f,%.4f) r%.4f]", a->centre.x, a->centre.y, a->radius);
            }
            std::printf(" ]");
        }
        // independent mitre point: x = w - d on circle r = R - d
        Vec2 m1{w - d, -std::sqrt((R - d) * (R - d) - (w - d) * (w - d))};
        std::printf("\n   expected mitre (%.9f,%.9f) |m1-v1| = %.6f\n", m1.x, m1.y, distance(m1, Vec2{w, -h}));
        Metrics m;
        check(in, out, d, OffsetSide::Left, j, 0.001, mt, m);
        std::printf("   checker: %s\n", m.firstIssue.empty() ? "clean" : m.firstIssue.c_str());
    }
    return 0;
}
