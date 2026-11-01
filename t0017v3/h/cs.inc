// centre-shift probe: 0xe300070e input (Round arcs about vertices 2d apart), upper row moved by g, translated by T
static double g_csOnlyG = 1e300, g_csOnlyT = -1; static int g_csDump = 0;
int csMode(int dirSel) {
    if (std::getenv("CSG")) g_csOnlyG = std::atof(std::getenv("CSG")); if (std::getenv("CST")) g_csOnlyT = std::atof(std::getenv("CST")); g_csDump = std::getenv("CSDUMP") != nullptr;
    std::vector<Vec2> P = {{6.6283223938981575, -1.2779953526139643e-11},  {5.023667459767422, 2.1623209668586711e-11},
         {6.9756096886500769, 34.51051481312367},        {6.3189901091486398, 34.312480314617368},
         {4.6667279880904458, 2.0657439862650513},       {0.69533962297162577, 2.0657439862566656},
         {1.9562893692302896, 33.119825834028326},       {7.04511858784476, 33.809684447840162},
         {6.6283223939411231, 2.0657439862576599},       {5.0236674597854529, 2.0657439862438181},
         {5.4657050614391105, 36.723239966815832},       {4.5703929732694029, 36.057839455949605},
         {4.8076418394176894, 3.1120034057046923e-12},   {8.8588515885397729, -8.7094539924375882e-13},
         {9.7907278097679189, -35.00186354855979},       {1.9705389447012767, -34.43692474497638},
         {0.69533962299185814, -2.6581188530012263e-11}, {4.6667279880936041, -3.0346127572489825e-11},
         {3.1154143986946496, 34.81196163001529},        {4.5941198790408642, 35.289788649021553},
         {5.0236674597518789, 1.9744273286251705e-11},   {6.6283223939582836, -1.3296097279548633e-11},
         {6.980860505868165, 32.133493994828015},        {9.603450489764624, 31.205313249352219},
         {8.858851588581409, 2.0657439862506815},        {4.8076418393909703, 2.0657439862946823}};
    const double d0 = 1.0328719931380388, tol = 1e-3;
    long bad = 0, n = 0;
    double gs[] = {0, 1e-13, -1e-13, 1e-12, -1e-12, 1e-11, -1e-11, 3e-11, 1e-10, -1e-10, 1e-9, -1e-9, 1e-8, -1e-8, 1e-7, -1e-7, 1e-6, -1e-6};
    std::vector<double> gv(std::begin(gs), std::end(gs));
    if (std::getenv("CSFINE")) { gv.clear(); for (int k = 0; k <= 70; ++k) { double v = std::pow(10.0, -13 + k * 0.1); gv.push_back(v); gv.push_back(-v); } }
    for (double T : {0.0, 1e2, 1e3, 1e4, 3e4, 1e5}) if (!std::getenv("CST") || T == std::atof(std::getenv("CST")))
        for (int dir = 0; dir < 4; ++dir) {
            if (dirSel >= 0 && dir != dirSel) continue;
            for (double gg : gv) { if (g_csOnlyG < 1e299 && gg != g_csOnlyG) continue; if (g_csOnlyT >= 0 && T != g_csOnlyT) continue;
                for (int dv = 0; dv < 3; ++dv) {
                    double d = dv == 0 ? d0 : dv == 1 ? d0 * 2.5 : d0 * 0.2;
                    double sc = d / d0;
                    std::vector<Vec2> Q;
                    for (Vec2 p : P) { Vec2 q{p.x * sc, (std::fabs(p.y - 2 * d0) < 1e-6 ? 2 * d + gg : (std::fabs(p.y) < 1e-6 ? p.y : p.y * sc))}; Q.push_back(q); }
                    Contour in = fromPts(Q, false);
                    Vec2 t = dir == 0 ? Vec2{T, 0} : dir == 1 ? Vec2{0, T} : dir == 2 ? Vec2{T, -0.7 * T} : Vec2{-T, -T};
                    double th = dir == 3 ? kPi / 2 : 0.0;
                    transformC(in, th, t);
                    for (int side = 0; side < 2; ++side)
                        for (int j = 0; j < 3; ++j) {
                            auto out = offsetOpenContour(in, d, side ? OffsetSide::Right : OffsetSide::Left, (JoinType)j, Tolerance{tol, 1e-9, 0.01}, 3.8787999757578069);
                            Metrics m; check(in, out, d, side ? OffsetSide::Right : OffsetSide::Left, (JoinType)j, tol, 3.8787999757578069, m, false, true);
                            n++;
                            int b = m.distViol + m.roundViol + m.lineJunctionNotBitwise + m.arcGapBeyond + m.coderBack;
                            if (b) { bad++; auto mp = mainOffset(toP(in), d, side, j, tol, 3.8787999757578069); std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p)); Metrics mm; check(in, mo, d, side ? OffsetSide::Right : OffsetSide::Left, (JoinType)j, tol, 3.8787999757578069, mm, false, true); std::printf("MAIN dv %d rv %d gaps %d (maxArcGapRel %.3g) back %d chains %zu vs br %zu (br maxArcGapRel %.3g)\n", mm.distViol, mm.roundViol, mm.lineJunctionNotBitwise + mm.arcGapBeyond, mm.maxArcGapRel, mm.coderBack, mo.size(), out.size(), m.maxArcGapRel); std::printf("CS T=%g dir=%d g=%g d=%.6g side=%d join=%d: dv %d (%.3g) rv %d (%.3g) gaps %d back %d | %s\n", T, dir, gg, d, side, j, m.distViol, m.worstUnder, m.roundViol, m.worstOver, m.lineJunctionNotBitwise + m.arcGapBeyond, m.coderBack, m.firstIssue.c_str()); }
                            if (b && g_csDump) { for (std::size_t ci = 0; ci < out.size(); ++ci) { std::printf(" chain %zu closed %d\n", ci, out[ci].closed); for (std::size_t k = 0; k < out[ci].spans.size(); ++k) { auto& s = out[ci].spans[k]; if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g)\n", l->start.x, l->start.y, l->end.x, l->end.y); else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW"); if (k + 1 < out[ci].spans.size()) std::printf("     gap %.3g\n", distance(endPoint(s), startPoint(out[ci].spans[k+1]))); } } }
                        }
                } }
        }
    std::printf("cs: %ld offsets, %ld with violations\n", n, bad);
    return 0;
}
