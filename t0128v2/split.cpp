// r2 verifier: extra splits of strays put back. Channel 2d + w wide whose walls are polylines of unit segments
// (args: N d w join jitter): prints each chain (start, end, spans).
#include "v9lib.cpp"
int main(int argc, char** argv) {
    int N = atoi(argv[1]); double d = atof(argv[2]), w = atof(argv[3]); int j = atoi(argv[4]); double jit = argc > 5 ? atof(argv[5]) : 0; double seg = argc > 6 ? atof(argv[6]) : 1.0;
    double W = 2 * d + w; std::vector<Vec2> P;
    Rng g(5);
    for (int i = 0; i <= N; ++i) P.push_back({i * seg, jit * g.range(-1, 1)});
    for (int i = N; i >= 0; --i) P.push_back({i * seg, W + jit * g.range(-1, 1)});
    Contour in = fromPts(P, false);
    auto ob = offsetOpenContour(in, d, OffsetSide::Left, (JoinType)j, Tolerance{1e-3, 1e-9, 0.01}, 2.0);
    std::printf("chains %zu\n", ob.size());
    for (size_t k = 0; k < ob.size() && k < 12; ++k) { auto& c = ob[k]; double l = 0; for (auto& s : c.spans) l += length(s);
        Vec2 a = startPoint(c.spans.front()), b = endPoint(c.spans.back());
        std::printf(" %zu: spans %zu len %.6g  (%.9g,%.9g)->(%.9g,%.9g)\n", k, c.spans.size(), l, a.x, a.y, b.x, b.y); }
}
