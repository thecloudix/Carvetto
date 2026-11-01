// axisCusp probe: exact input of fam5 0xf50011fd regenerated with overlap g and translation T
int acMode() {
    long n = 0, bad = 0, badM = 0;
    for (double T : {0.0, 10.0, 100.0, 2545.0, 1e4})
        for (int k = 0; k <= 40; ++k) {
            double g = -std::pow(10.0, -10 + k * 0.15);
            for (int dv = 0; dv < 3; ++dv)
                for (int rot = 0; rot < 2; ++rot) {
                    double d = dv == 0 ? 0.81999987565196308 : dv == 1 ? 0.2 : 2.5;
                    Rng gg(0xf50011fdULL + dv);
                    // reproduce wall variant with fixed angles
                    double r = d + g / 2, a1 = 0.7, a2 = 0.6, L = 6 + d;
                    Vec2 P1{-r, 0}, P2{r, 0};
                    Vec2 A1 = P1 + Vec2::fromAngle(kPi + a1, L), A2 = P2 + Vec2::fromAngle(-a2, L);
                    double yb = std::min(A1.y, A2.y) - 2 * d - 1;
                    std::vector<Vec2> pts{A1 + Vec2{0.0, -0.3}, P1, A1, {A1.x, yb}, {A2.x, yb}, A2, P2, A2 + Vec2{0.3, 0.0}};
                    Contour in = fromPts(pts, false);
                    transformC(in, rot ? kPi / 2 : 0.0, Vec2{0.0, T});
                    for (int side = 0; side < 2; ++side) {
                        auto out = offsetOpenContour(in, d, side ? OffsetSide::Right : OffsetSide::Left, JoinType::Round, Tolerance{1e-3, 1e-9, 0.01}, 2.0);
                        Metrics m; check(in, out, d, side ? OffsetSide::Right : OffsetSide::Left, JoinType::Round, 1e-3, 2.0, m, false, true);
                        auto mp = mainOffset(toP(in), d, side, 0, 1e-3, 2.0);
                        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
                        Metrics mm; check(in, mo, d, side ? OffsetSide::Right : OffsetSide::Left, JoinType::Round, 1e-3, 2.0, mm, false, true);
                        n++; bad += m.distViol > 0; badM += mm.distViol > 0;
                        if (m.distViol || mm.distViol)
                            std::printf("AC T=%g g=%.3g d=%g rot=%d side=%d: br dv %d under %.3g (allow %.3g) | main dv %d under %.3g | br %s\n", T, g, d, rot, side, m.distViol, m.worstUnder, allowance(d, 1e-3), mm.distViol, mm.worstUnder, m.firstIssue.c_str());
                    }
                }
        }
    std::printf("ac: %ld offsets, branch dv cases %ld, main dv cases %ld\n", n, bad, badM);
    return 0;
}
