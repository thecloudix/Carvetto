// list output spans closer than d - allowance, with neighbours
int dvMode(std::uint64_t seed, bool useMain) {
    Case2 cc = makeCaseX(seed);
    Case& c = cc.c;
    std::vector<Contour> out;
    if (useMain) { auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, (int)c.join, c.tol, c.mitre); for (auto& p : mp) out.push_back(fromP(p)); }
    else out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    std::vector<Seg> inS; for (auto& s : c.in.spans) inS.push_back(toSeg(s));
    double A = allowance(c.d, c.tol);
    std::printf("%s %s side=%d d=%.17g mitre=%.6g allowance %.3g, input spans %zu, subtol %d\n", cc.name, jn(c.join), (int)c.side, c.d, c.mitre, A, c.in.spans.size(), subTolIn(c));
    for (std::size_t ci = 0; ci < out.size(); ++ci) for (std::size_t i = 0; i < out[ci].spans.size(); ++i) {
        Seg g = toSeg(out[ci].spans[i]);
        double worst = 1e300, wt = 0;
        for (int k = 0; k <= 2000; ++k) { double t = k / 2000.0; Vec2 p = segPoint(g, t); double m = 1e300; for (auto& s : inS) m = std::min(m, distPt(s, p)); if (m < worst) { worst = m; wt = t; } }
        if (c.d - worst > A + 1e-12 * std::max(1.0, g.a.length())) {
            std::printf("chain %zu span %zu/%zu kind %d len %.4g under %.4g at t=%.4f  (%.12g,%.12g)->(%.12g,%.12g)\n", ci, i, out[ci].spans.size(), g.kind, segLen(g), c.d - worst, wt, g.a.x, g.a.y, g.b.x, g.b.y);
            for (int q = -2; q <= 2; ++q) { long j = (long)i + q; if (j < 0 || j >= (long)out[ci].spans.size()) continue; Seg h = toSeg(out[ci].spans[j]); std::printf("    [%ld] kind %d len %.4g (%.12g,%.12g)->(%.12g,%.12g)\n", j, h.kind, segLen(h), h.a.x, h.a.y, h.b.x, h.b.y); }
        }
    }
    return 0;
}
