// r2 verifier: AC1 for thin loops / needles with many spans. A closed slot (2d + L) x (2d + w) whose long walls are
// polylines of M segments (args: M d w join seglen closed(1)|U(0) shift). Reports the chains and the uncovered centre line.
#include "v9lib.cpp"
int main(int argc, char** argv) {
    int M = atoi(argv[1]); double d = atof(argv[2]), w = atof(argv[3]); int j = atoi(argv[4]); double seg = atof(argv[5]); int closed = atoi(argv[6]); double sh = argc > 7 ? atof(argv[7]) : 0;
    double L = M * seg, W = 2 * d + w; std::vector<Vec2> P;
    const bool single = std::getenv("SINGLE") != nullptr;  // bottom wall one line
    if (std::getenv("TWO")) { P.push_back({sh, sh}); P.push_back({sh + 0.5 * L, sh}); } else if (single) P.push_back({sh, sh}); else for (int i = 0; i <= M; ++i) P.push_back({sh + i * seg, sh});
    P.push_back({sh + L + 2 * d, sh}); P.push_back({sh + L + 2 * d, sh + W});
    for (int i = M; i >= 0; --i) P.push_back({sh + i * seg, sh + W});
    if (closed) P.push_back({sh - 2 * d, sh + W}), P.push_back({sh - 2 * d, sh});
    for (auto& p : P) p.x -= closed ? 0 : 0;
    Contour in = fromPts(P, closed == 1);
    auto ob = offsetOpenContour(in, d, OffsetSide::Left, (JoinType)j, Tolerance{1e-3, 1e-9, 0.01}, 2.0);
    auto om = mainOffset(toP(in), d, 0, j, 1e-3, 2.0);
    double tot = 0; size_t sp = 0; for (auto& c : ob) { for (auto& s : c.spans) tot += length(s); sp += c.spans.size(); }
    int miss = 0, K = 2000; for (int k = 0; k <= K; ++k) { Vec2 q{sh + 0.05 + (L - 0.1) * k / K, sh + W / 2}; double b = 1e300; for (auto& c : ob) for (auto& s : c.spans) b = std::min(b, distPt(toSeg(s), q)); if (b > 1e-3) ++miss; }
    std::printf("M=%d d=%g w=%g %s seg=%g %s shift=%g: branch chains %zu spans %zu total len %.6g, centre line samples uncovered %d/%d | main chains %zu\n", M, d, w, jn((JoinType)j), seg, closed ? "closed" : "U", sh, ob.size(), sp, tot, miss, K + 1, om.size());
}
