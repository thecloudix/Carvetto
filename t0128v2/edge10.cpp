// T-0128 r2 verifier: the 10 x tol stray minimum at its edge.
//  A: isolated thin closed rectangle loop (2d+e) x (2d+w): strand length vs e around 10 tol.
//  B: isolated U channel exactly 2d (+w) wide, inside stray length L-d around 10 tol (opposite copies).
//  C: a slot 2d (+w) wide and s deep opening off a wide pocket (stray touches a chain at one end), s around tol..20 tol.
//  D: a needle slot 2d wide between two pockets (stray touching chains at both ends).
#include "v9lib.cpp"
static void report(const char* tag, double par, const std::vector<Contour>& ob, const std::vector<PContour>& om, JoinType j, double shift) {
    double total = 0; size_t n = ob.size(); std::string lens;
    for (auto& c : ob) { double l = 0; for (auto& s : c.spans) l += length(s); total += l; char b[64]; std::snprintf(b, sizeof b, " %.5g", l); lens += b; }
    std::printf("%s %s shift %g par %.6g: branch chains %zu [%s] main chains %zu\n", tag, jn(j), shift, par, n, lens.c_str(), om.size());
}
int main() {
    Tolerance tol{1e-3, 1e-9, 0.01};
    const double d = 1.0;
    for (double shift : {0.0, 1.2e4}) for (int jj = 0; jj < 3; ++jj) {
        JoinType j = (JoinType)jj;
        for (double w : {0.0, 1e-6}) for (double e : {9.0e-3, 9.9e-3, 9.99e-3, 1.0e-2, 1.001e-2, 1.01e-2, 1.1e-2}) {
            std::vector<Vec2> P{{0, 0}, {2 * d + e, 0}, {2 * d + e, 2 * d + w}, {0, 2 * d + w}};
            for (auto& p : P) p = p + Vec2{shift, shift};
            Contour in = fromPts(P, true);
            auto ob = offsetOpenContour(in, d, OffsetSide::Left, j, tol, 2.0);
            auto om = mainOffset(toP(in), d, 0, jj, 1e-3, 2.0);
            char tag[64]; std::snprintf(tag, sizeof tag, "A w=%g", w); report(tag, e, ob, om, j, shift);
        }
        for (double w : {0.0, 1e-8}) for (double s : {9.0e-3, 9.9e-3, 1.0e-2, 1.001e-2, 1.01e-2, 1.1e-2}) {
            double W = 2 * d + w, L = d + s;
            std::vector<Vec2> P{{0, 0}, {L, 0}, {L, W}, {0, W}};
            for (auto& p : P) p = p + Vec2{shift, shift};
            Contour in = fromPts(P, false);
            auto ob = offsetOpenContour(in, d, OffsetSide::Left, j, tol, 2.0);
            auto om = mainOffset(toP(in), d, 0, jj, 1e-3, 2.0);
            char tag[64]; std::snprintf(tag, sizeof tag, "B w=%g", w); report(tag, s, ob, om, j, shift);
        }
        for (double w : {0.0, 1e-8}) for (double s : {5e-4, 1.5e-3, 3e-3, 5e-3, 9e-3, 1.1e-2, 2e-2}) {
            // pocket 10 x 10 (open path, inside = Left of CCW-ish traversal), slot of width W and depth s+d off its right wall.
            double W = 2 * d + w, D = d + s;
            std::vector<Vec2> P{{0, 0}, {10, 0}, {10, 5 - W / 2}, {10 + D, 5 - W / 2}, {10 + D, 5 + W / 2}, {10, 5 + W / 2}, {10, 10}, {0, 10}};
            for (auto& p : P) p = p + Vec2{shift, shift};
            Contour in = fromPts(P, false);
            auto ob = offsetOpenContour(in, d, OffsetSide::Left, j, tol, 2.0);
            auto om = mainOffset(toP(in), d, 0, jj, 1e-3, 2.0);
            char tag[64]; std::snprintf(tag, sizeof tag, "C w=%g", w); report(tag, s, ob, om, j, shift);
        }
        for (double s : {1.5e-3, 5e-3, 2e-2, 1.0}) {
            // two pockets joined by a channel 2d wide, s long
            double W = 2 * d;
            std::vector<Vec2> P{{0, 0}, {10, 0}, {10, 5 - W / 2}, {10 + s, 5 - W / 2}, {10 + s, 0}, {20 + s, 0}, {20 + s, 10}, {10 + s, 10}, {10 + s, 5 + W / 2}, {10, 5 + W / 2}, {10, 10}, {0, 10}};
            for (auto& p : P) p = p + Vec2{shift, shift};
            Contour in = fromPts(P, false);
            auto ob = offsetOpenContour(in, d, OffsetSide::Left, j, tol, 2.0);
            auto om = mainOffset(toP(in), d, 0, jj, 1e-3, 2.0);
            report("D", s, ob, om, j, shift);
        }
    }
}
