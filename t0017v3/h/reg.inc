// classify main's valid points that the branch lacks
int regMode(int argc, char** argv) {
    for (int ai = 2; ai < argc; ++ai) {
        std::uint64_t seed = std::strtoull(argv[ai], nullptr, 0);
        Case2 cc = makeCaseX(seed);
        Case& c = cc.c;
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
        std::vector<Seg> inS, bS;
        for (auto& s : c.in.spans) inS.push_back(toSeg(s));
        for (auto& k : out) for (auto& s : k.spans) bS.push_back(toSeg(s));
        Grid IG, BG; IG.build(inS, std::max(c.d / 2, 0.05)); if (bS.empty()) { std::printf("seed 0x%llx: branch empty\n", (unsigned long long)seed); continue; } BG.build(bS, std::max(c.d / 2, 0.05));
        const double sgn = c.side == OffsetSide::Left ? 1.0 : -1.0;
        const int ni = (int)inS.size();
        long miss = 0, inCorner = 0, other = 0; double worstOther = 0; Vec2 atO{};
        double lim = 2 * c.tol;
        for (auto& k : mo) for (auto& s : k.spans) {
            Seg g = toSeg(s);
            int ns = (int)std::clamp(std::ceil(segLen(g) / (0.25 * lim)), 4.0, 400.0);
            for (int q = 0; q <= ns; ++q) {
                Vec2 p = segPoint(g, (double)q / ns);
                if (IG.minDist(p, c.d + 1) < c.d - 1e-7) continue;
                double dd = BG.minDist(p, 10 * lim + 1);
                if (dd <= lim) continue;
                miss++;
                bool corner = false;
                for (int kk = 0; kk < ni && !corner; ++kk) {
                    int nx = kk + 1; if (nx == ni) { if (!c.in.closed) break; nx = 0; }
                    Vec2 t1 = segTan(inS[kk], 1), t2 = segTan(inS[nx], 0);
                    double cr = t1.cross(t2);
                    bool convex = c.side == OffsetSide::Left ? cr < -1e-12 : cr > 1e-12;
                    if (!convex) continue;
                    Vec2 V = inS[kk].b, v = p - V;
                    double L = v.length();
                    if (L > std::max(c.mitre, 1.5) * c.d + 1e-9) continue;
                    double a1 = (t1.perp() * sgn).angle(), sw = std::remainder((t2.perp() * sgn).angle() - a1, kTwoPi);
                    double rel = std::remainder(v.angle() - a1, kTwoPi);
                    double sl = (inS[kk].kind == 1 || inS[nx].kind == 1) ? 0.3 : 1e-6;
                    if (sw >= 0 ? (rel >= -sl && rel <= sw + sl) : (rel <= sl && rel >= sw - sl)) corner = true;
                }
                if (corner) inCorner++; else { other++; if (dd > worstOther) { worstOther = dd; atO = p; } }
            }
        }
        std::printf("seed 0x%llx %s %s d=%.6g: missing %ld (in convex-corner wedge %ld, elsewhere %ld worst %.3g at (%.9g,%.9g))\n", (unsigned long long)seed, cc.name, jn(c.join), c.d, miss, inCorner, other, worstOther, atO.x, atO.y);
    }
    return 0;
}
