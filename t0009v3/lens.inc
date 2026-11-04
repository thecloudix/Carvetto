// Deterministic sweep of lens widths / lengths: is the thin lens (valid tool positions between the
// two wall offsets) present in the branch / main output? Probe: distance from the lens middle
// (0, d - w/2) to the output, present iff <= w (+1e-9).
int lensMode(int kindSel, double tx) {
    const double tol = 1e-3;
    double ws[] = {tol / 64, tol / 32, tol / 16, tol / 10, tol / 8, 0.15 * tol, 0.19 * tol, tol / 4, 0.3 * tol, tol / 3, 0.4 * tol, 0.45 * tol, tol / 2, 0.55 * tol, 0.7 * tol, tol, 1.5 * tol, 2 * tol};
    double Ls[] = {0.005, 0.02, 0.1, 0.5, 2, 5, 10};
    double ds[] = {0.05, 0.3, 1.0, 3.0};
    const char* jnm[] = {"R", "Sh", "Sq"};
    for (int kind = 0; kind <= 2; ++kind) {
        if (kindSel >= 0 && kind != kindSel) continue;
        for (double w : ws) {
            std::printf("kind %d w/tol %-6.4g:", kind, w / tol);
            for (double L : Ls) {
                int bPresent = 0, mPresent = 0, cases = 0, bOnlyDrop = 0;
                std::string ex;
                for (double d : ds)
                    for (int j = 0; j < 3; ++j) {
                        Rng g(1234 + kind * 100 + j);
                        Contour in = lensSlot(g, d, w, L, kind);
                        if (tx != 0) transformC(in, 0.0, Vec2{tx, -tx});
                        auto out = offsetOpenContour(in, d, OffsetSide::Left, static_cast<JoinType>(j), Tolerance{tol, 1e-9, 0.01}, 2.0);
                        auto mp = mainOffset(toP(in), d, 0, j, tol, 2.0);
                        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
                        Vec2 P{tx, -tx + d - w / 2};
                        auto near = [&](const std::vector<Contour>& o) {
                            double best = 1e300;
                            for (auto& k : o) for (auto& s : k.spans) best = std::min(best, carvetto::geometry::testing::distanceToSpanExact(P, s));
                            return best;
                        };
                        double db = near(out), dm = near(mo);
                        cases++;
                        bool bp = db <= w + 1e-9 * std::max(1.0, std::fabs(tx)), mp2 = dm <= w + 1e-9 * std::max(1.0, std::fabs(tx));
                        bPresent += bp; mPresent += mp2;
                        if (mp2 && !bp) { bOnlyDrop++; if (ex.size() < 40) { char b[64]; snprintf(b, sizeof b, "%s/d%g ", jnm[j], d); ex += b; } }
                    }
                std::printf(" L=%g b%d m%d%s%s |", L, bPresent, mPresent, bOnlyDrop ? " DROP:" : "", ex.c_str());
            }
            std::printf("\n");
            std::fflush(stdout);
        }
    }
    return 0;
}
int lensOne(int kind, double wRel, double L, double d, int j) {
    const double tol = 1e-3, w = wRel * tol;
    Rng g(1234 + kind * 100 + j);
    Contour in = lensSlot(g, d, w, L, kind);
    auto dump = [](const Contour& k) {
        for (auto& s : k.spans) {
            if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g) len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(s));
            else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(s));
        }
    };
    dump(in);
    auto out = offsetOpenContour(in, d, OffsetSide::Left, static_cast<JoinType>(j), Tolerance{tol, 1e-9, 0.01}, 2.0);
    for (auto& k : out) { std::printf("branch chain closed=%d\n", k.closed); dump(k); }
    auto mp = mainOffset(toP(in), d, 0, j, tol, 2.0);
    std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
    for (auto& k : mo) {
        std::printf("main chain closed=%d", k.closed);
        if (k.closed) { double P = 0, A2 = 0; Vec2 o = startPoint(k.spans[0]); for (auto& s : k.spans) { P += length(s); for (int q = 0; q < 64; ++q) A2 += (pointAt(s, q / 64.0) - o).cross(pointAt(s, (q + 1) / 64.0) - o); } std::printf(" perimeter %.4g meanWidth %.3g loopWidth %.3g", P, std::fabs(A2) / P, loopWidth(k)); }
        std::printf("\n"); dump(k);
    }
    return 0;
}
