// gap <v9 seed> <join> <side>: chains of branch and main with closed flag, span count and end-to-start gap.
#include "v9lib.cpp"
int main(int argc, char** argv) {
    std::uint64_t seed = std::strtoull(argv[1], nullptr, 0); C9 cc = make9(seed); Case c = cc.c;
    int j = std::atoi(argv[2]), s = std::atoi(argv[3]); Contour ck = polyOf(c.in, c.tol);
    auto ob = offsetOpenContour(ck, c.d, (OffsetSide)s, (JoinType)j, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    auto mp = mainOffset(toP(ck), c.d, s, j, c.tol, c.mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    for (int w = 0; w < 2; ++w) { auto& o = w ? om : ob; std::printf("%s:", w ? "main  " : "branch");
        for (auto& k : o) std::printf(" [%s %zu spans, end-start gap %.4g]", k.closed ? "closed" : "open", k.spans.size(), distance(k.spans.back() == k.spans.back() ? endPoint(k.spans.back()) : Vec2{}, startPoint(k.spans.front()))); std::printf("\n"); }
}
