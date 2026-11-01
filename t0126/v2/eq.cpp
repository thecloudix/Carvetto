#define main scan2_main
#include "scan2.cpp"
#undef main
// pocketCentreRegion == pocketToolpath plan centreRegion, and determinism of pocketCentreRegion
int main(int argc, char** argv) {
    long n = 0, diff = 0, nondet = 0, msgDiff = 0;
    for (int kind : {0, 3, 4, 5, 9, 12, 14, 15}) for (uint64_t seed = 400; seed < 403; ++seed) {
        World W; if (!makeWorld(kind, seed, W)) continue;
        for (double ch : {0.01, 0.2}) for (int k = 0; k < 40; ++k) {
            double d = 0.3 + k * 0.45, al = (k % 3) * 0.1;
            toolpath::PocketParams P; P.tolerance.chordal = ch; P.allowance = al; P.cutDepth = 1;
            std::vector<std::string> ms; auto C = toolpath::pocketCentreRegion(W.input, d, P, &ms);
            auto C2 = toolpath::pocketCentreRegion(W.input, d, P);
            toolpath::PocketTool T; T.tool.name = "T"; T.tool.diameter = d; T.stepover = d * 0.9; T.passDepth = 1;
            P.strategy = toolpath::PocketStrategy::Raster;
            auto res = toolpath::pocketToolpath(W.input, {T}, P);
            ++n; if (!(res.plans[0].centreRegion == C)) { ++diff; if (diff < 5) printf("DIFF kind %d seed %llu ch %g d %g: %zu vs %zu  msgs %zu/%zu\n", kind, (unsigned long long)seed, ch, d, C.size(), res.plans[0].centreRegion.size(), ms.size(), res.messages.size()); }
            if (!(C == C2)) ++nondet;
        }
    }
    printf("eq: %ld cases, %ld differ, %ld nondeterministic\n", n, diff, nondet);
}
