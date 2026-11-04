double worstUnderOf(const Contour& in, const std::vector<Contour>& out, double d) {
    std::vector<Seg> inS; for (auto& s : in.spans) inS.push_back(toSeg(s));
    double w = 0;
    for (auto& c : out) for (auto& s : c.spans) { Seg g = toSeg(s); for (int k = 0; k <= 400; ++k) { Vec2 p = segPoint(g, k / 400.0); double m = 1e300; for (auto& q : inS) m = std::min(m, distPt(q, p)); w = std::max(w, d - m); } }
    return w;
}
void dumpSpans(const Contour& in) {
    std::printf("Contour in; in.closed = false;\n");
    for (auto& sp : in.spans) {
        if (auto* l = std::get_if<LineSpan>(&sp)) std::printf("in.spans.emplace_back(LineSpan{{%.17g, %.17g}, {%.17g, %.17g}});\n", l->start.x, l->start.y, l->end.x, l->end.y);
        else if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf("in.spans.emplace_back(ArcSpan{{%.17g, %.17g}, %.17g, %.17g, %.17g, ArcDirection::%s});\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW");
    }
}
void dumpPts(const Contour& in) {
    bool allLines = true; for (auto& sp : in.spans) if (!std::holds_alternative<LineSpan>(sp)) allLines = false;
    if (!allLines) { dumpSpans(in); return; }
    std::printf("const std::vector<Vec2> pts{{%.17g, %.17g}", startPoint(in.spans[0]).x, startPoint(in.spans[0]).y);
    for (auto& s : in.spans) std::printf(", {%.17g, %.17g}", endPoint(s).x, endPoint(s).y);
    std::printf("};\n");
}
int shrinkMode(std::uint64_t seed) {
    Case2 cc = makeCaseX(seed);
    Case c = cc.c;
    auto under = [&](const Contour& in) { auto out = offsetOpenContour(in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre); return worstUnderOf(in, out, c.d); };
    double thr = (std::getenv("THR") ? std::atof(std::getenv("THR")) : 10.0) * allowance(c.d, c.tol);
    Contour cur = c.in; cur.closed = false;
    std::printf("start under %.3g (thr %.3g) spans %zu\n", under(cur), thr, cur.spans.size());
    bool progress = true;
    while (progress) {
        progress = false;
        for (std::size_t chunk : {64u, 16u, 4u, 1u}) {
            while (cur.spans.size() > chunk + 1) { Contour t = cur; t.spans.erase(t.spans.begin(), t.spans.begin() + (long)chunk); if (under(t) > thr) { cur = t; progress = true; } else break; }
            while (cur.spans.size() > chunk + 1) { Contour t = cur; t.spans.erase(t.spans.end() - (long)chunk, t.spans.end()); if (under(t) > thr) { cur = t; progress = true; } else break; }
        }
    }
    std::printf("// d=%.17g join=%s mitre=%.17g side=%s under=%.4g spans=%zu\n", c.d, jn(c.join), c.mitre, c.side == OffsetSide::Left ? "Left" : "Right", under(cur), cur.spans.size());
    dumpPts(cur);
    auto mp = mainOffset(toP(cur), c.d, c.side == OffsetSide::Left ? 0 : 1, (int)c.join, c.tol, c.mitre);
    std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
    std::printf("main under on shrunk input: %.3g\n", worstUnderOf(cur, mo, c.d));
    return 0;
}
