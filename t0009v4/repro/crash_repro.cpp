// T-0009 round-3 verifier: uncaught std::length_error in offsetOpenContour (branch 8b4e69e), g2 seed 0x32000123:
// closed loop of 134 lines (133 shorter than tol), d = 0.041100762317998148, Left, any join, default Tolerance.
// main 0fa0699 returns 1 chain. Build: g++ -std=c++20 -I<wt>/core/include crash_repro.cpp <build>/core/libcarvetto_core.a <build>/_deps/clipper2-build/libClipper2.a
#include <cstdio>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
int main() {
#include "case_g2_32000123.inc"
    for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        try { auto o = offsetOpenContour(in, 0.041100762317998148, OffsetSide::Left, j, Tolerance{}, 2.0); std::printf("join %d: %zu chains\n", (int)j, o.size()); }
        catch (const std::exception& e) { std::printf("join %d: EXCEPTION %s\n", (int)j, e.what()); }
    }
}
