// T-0128 verifier: AC1 edge - closed rectangle (2d + e) x (2d + w): inner thin loop of extent ~e. Dropped iff extent <= tol.
// Also an out-and-back needle path (open) of length e whose ends coincide.
#include "v9lib.cpp"
int main() {
    const double d = 1.0, tol = 1e-3;
    for (double w : {0.0, 1e-9, 1e-6, 1e-4}) for (double e : {5e-4, 9e-4, 9.9e-4, 1.01e-3, 1.1e-3, 1.5e-3, 2e-3, 5e-3, 2e-2}) for (int j = 0; j < 3; ++j) for (Vec2 sh : {Vec2{0, 0}, Vec2{1.2e4, -3e4}}) {
        Contour in = fromPts({{0, 0}, {2 * d + e, 0}, {2 * d + e, 2 * d + w}, {0, 2 * d + w}}, true); transformC(in, 0.0, sh);
        auto ob = offsetOpenContour(in, d, OffsetSide::Left, (JoinType)j, Tolerance{tol, 1e-9, 0.01}, 2.0);
        auto mp = mainOffset(toP(in), d, 0, j, tol, 2.0);
        double L = 0, mind = 1e300, maxd = 0; size_t sp = 0; for (auto& c : ob) for (auto& s : c.spans) { L += length(s); sp++; for (int k = 0; k <= 16; ++k) { Vec2 q = pointAt(s, k / 16.0); double m = 1e300; for (auto& t : in.spans) m = std::min(m, carvetto::geometry::testing::distanceToSpanExact(q, t)); mind = std::min(mind, m); maxd = std::max(maxd, m); } }
        std::printf("w=%-6g e=%-7g %-6s sh=%g | branch chains %zu spans %zu len %.6g closed %d dist [%.12g, %.12g] | main chains %zu\n", w, e, jn((JoinType)j), sh.x, ob.size(), sp, L, ob.empty() ? -1 : (int)ob[0].closed, mind, maxd, mp.size());
    }
}
