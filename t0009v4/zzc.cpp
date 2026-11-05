#define main vdrv4_main
#include "vchk.cpp"
#undef main
int main(int argc, char** argv) {
    int n = std::atoi(argv[1]);
    Contour c; Vec2 p{0, 0};
    for (int i = 0; i < n; ++i) { Vec2 q = p + Vec2{2e-4, (i % 2 ? 1 : -1) * 3e-4}; c.spans.push_back(LineSpan{p, q}); p = q; }
    for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
        auto o = offsetOpenContour(c, 1.0, s, JoinType::Round);
        Metrics m; check(c, o, 1.0, s, JoinType::Round, 0.001, 2.0, m);
        std::printf("n=%d side %d chains %zu: %s [dv%d cm%d miss %.3g covSamples %d]\n", n, (int)s, o.size(), m.firstIssue.c_str(), m.distViol, m.covMiss, m.worstMiss, m.covSamples);
    }
}
