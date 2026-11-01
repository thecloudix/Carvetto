// Hairpin with U-turn radius rho = d (1 + eps): Round offset inside the turn; Round-over / gap vs main.
int hpMode(int variant, int joinSel) {
    const double tol = 1e-3;
    for (double tx : {0.0, 100.0, 1000.0, 5000.0, 1e4}) {
        for (double eps : {-1e-3, -1e-4, -1e-5, -1e-6, -1e-7, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3}) {
            int dB = 0, dM = 0; double uB = 0, uM = 0; int nB = 0, nM = 0, gB = 0, gM = 0, cases = 0, chB = 0, chM = 0;
            double worstB = 0, worstM = 0;
            for (int t = 0; t < 40; ++t) {
                Rng g(777 + t);
                double d = logU(g, 0.05, 3.0);
                double rho = d * (1 + eps);
                Builder b; b.h = g.range(-kPi, kPi);
                if (variant & 1) b.arc(g.range(2, 50), g.range(0.02, 0.2) * (g.uni() < 0.5 ? -1 : 1)); else b.line(g.range(0.5, 5));
                double sw = kPi * (1 + g.range(-3e-3, 3e-3));
                b.arc(rho, sw);
                if (variant & 2) b.arc(g.range(2, 50), g.range(0.02, 0.2) * (g.uni() < 0.5 ? -1 : 1)); else b.line(g.range(0.5, 5));
                Contour in = b.c;
                transformC(in, 0.0, Vec2{tx, -0.7 * tx});
                Metrics m, mm;
                auto out = offsetOpenContour(in, d, OffsetSide::Left, static_cast<JoinType>(joinSel), Tolerance{tol, 1e-9, 0.01}, 2.0);
                check(in, out, d, OffsetSide::Left, static_cast<JoinType>(joinSel), tol, 2.0, m, false, true);
                auto mp = mainOffset(toP(in), d, 0, joinSel, tol, 2.0);
                std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
                check(in, mo, d, OffsetSide::Left, static_cast<JoinType>(joinSel), tol, 2.0, mm, false, true);
                cases++;
                if (std::getenv("HPV") && ((m.lineJunctionNotBitwise + m.arcGapBeyond) || m.distViol)) std::printf("   t=%d d=%.17g: %s (under %.3g)\n", t, d, m.firstIssue.c_str(), m.worstUnder);
                dB += m.distViol > 0; dM += mm.distViol > 0; uB = std::max(uB, m.worstUnder); uM = std::max(uM, mm.worstUnder); nB += m.roundViol > 0; nM += mm.roundViol > 0;
                gB += (m.lineJunctionNotBitwise + m.arcGapBeyond) > 0; gM += (mm.lineJunctionNotBitwise + mm.arcGapBeyond) > 0;
                chB += (int)out.size(); chM += (int)mo.size();
                worstB = std::max(worstB, m.worstOver); worstM = std::max(worstM, mm.worstOver);
            }
            std::printf("tx %-6g eps %-7.0e: Round-over>allow branch %2d/%d (worst %.2g) main %2d (worst %.2g) | gaps branch %d main %d | under>allow b %d (%.2g) m %d (%.2g) | chains b%d m%d\n",
                        tx, eps, nB, cases, worstB, nM, worstM, gB, gM, dB, uB, dM, uM, chB, chM);
            std::fflush(stdout);
        }
    }
    return 0;
}
int hpOne(int variant, int joinSel, double tx, double eps, int t) {
    const double tol = 1e-3;
    Rng g(777 + t);
    double d = logU(g, 0.05, 3.0);
    double rho = d * (1 + eps);
    Builder b; b.h = g.range(-kPi, kPi);
    if (variant & 1) b.arc(g.range(2, 50), g.range(0.02, 0.2) * (g.uni() < 0.5 ? -1 : 1)); else b.line(g.range(0.5, 5));
    double sw = kPi * (1 + g.range(-3e-3, 3e-3));
    b.arc(rho, sw);
    if (variant & 2) b.arc(g.range(2, 50), g.range(0.02, 0.2) * (g.uni() < 0.5 ? -1 : 1)); else b.line(g.range(0.5, 5));
    Contour in = b.c;
    transformC(in, 0.0, Vec2{tx, -0.7 * tx});
    auto dump = [](const Contour& k) {
        for (auto& s : k.spans) {
            if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g) len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(s));
            else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(s));
        }
    };
    std::printf("d=%.17g join %d\n", d, joinSel); dump(in);
    for (std::size_t i = 0; i + 1 < in.spans.size(); ++i) std::printf("input gap %zu: %.3g\n", i, distance(endPoint(in.spans[i]), startPoint(in.spans[i + 1])));
    auto out = offsetOpenContour(in, d, OffsetSide::Left, static_cast<JoinType>(joinSel), Tolerance{tol, 1e-9, 0.01}, 2.0);
    for (auto& k : out) { std::printf("branch chain closed=%d\n", k.closed); dump(k); for (std::size_t i = 0; i + 1 < k.spans.size(); ++i) std::printf("   junction gap %zu: %.3g\n", i, distance(endPoint(k.spans[i]), startPoint(k.spans[i + 1]))); }
    Metrics m; check(in, out, d, OffsetSide::Left, static_cast<JoinType>(joinSel), tol, 2.0, m, false, true);
    std::printf("branch: %s under %.3g over %.3g\n", m.firstIssue.c_str(), m.worstUnder, m.worstOver);
    auto mp = mainOffset(toP(in), d, 0, joinSel, tol, 2.0);
    std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
    for (auto& k : mo) { std::printf("main chain closed=%d\n", k.closed); dump(k); }
    Metrics mm; check(in, mo, d, OffsetSide::Left, static_cast<JoinType>(joinSel), tol, 2.0, mm, false, true);
    std::printf("main: %s under %.3g over %.3g\n", mm.firstIssue.c_str(), mm.worstUnder, mm.worstOver);
    return 0;
}
