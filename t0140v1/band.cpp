// T-0140 verifier: bandOutline (profile.cpp, external linkage) vs the exact band {q : dist(q, path) <= w}, and Clipper2's
// offsetContours round band as a baseline. Random open paths of lines and arcs (tight arcs rho < w, reversals, tiny spans).
// usage: band seed N   -> per case: points sampled; UNDER = d < w - tolU but outside the union (false hole risk),
//   OVER = d > w + tolO but inside (piece shrinks). Prints worst depth of each error, for bandOutline (B) and Clipper2 (C).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <random>
#include <vector>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
#include "bandoutline_copy.inc"
using namespace carvetto; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static double LU(double a, double b) { return std::exp(U(std::log(a), std::log(b))); }
int main(int argc, char** argv) {
    unsigned long seed = std::strtoul(argv[1], nullptr, 0); int N = std::atoi(argv[2]);
    int kind = std::getenv("KIND") ? std::atoi(std::getenv("KIND")) : -1;
    double worstUB = 0, worstOB = 0, worstUC = 0, worstOC = 0; long negPts = 0; long nU = 0, nO = 0, nUC = 0, nOC = 0, nNull = 0;
    for (int c = 0; c < N; ++c) {
        rng.seed(seed * 7919 + c);
        double w = I(0, 2) == 0 ? LU(0.05, 0.5) : LU(0.5, 6);
        double chordal = I(0, 3) == 0 ? 1e-3 : 0.01;
        Contour path; path.closed = false;
        Vec2 p{0, 0}; double dir = U(0, 2 * PI);
        int ns = I(1, 8);
        for (int s = 0; s < ns; ++s) {
            int k = kind >= 0 ? kind : I(0, 5);
            if (s > 0) { int tk = I(0, 4); dir += tk == 0 ? U(-0.05, 0.05) : tk == 1 ? (I(0, 1) ? 1 : -1) * U(3.0, PI) : tk == 2 ? (I(0, 1) ? 1 : -1) * U(1.4, 1.8) : U(-PI, PI); }
            if (k <= 2) {  // line
                double L = k == 0 ? LU(1e-4, 0.5) * w : LU(0.01, 20);
                Vec2 q = p + Vec2::fromAngle(dir, L); path.spans.push_back(geometry::LineSpan{p, q}); p = q;
            } else {  // arc, often tighter than the band
                double rho = k == 3 ? LU(0.01, 1.0) * w : LU(0.5, 20);
                double sw = (I(0, 1) ? 1 : -1) * (I(0, 3) == 0 ? U(PI, 1.95 * PI) : U(0.05, PI));
                Vec2 n = Vec2::fromAngle(dir + (sw > 0 ? PI / 2 : -PI / 2), rho);
                Vec2 cen = p + n; double a0 = std::atan2(p.y - cen.y, p.x - cen.x);
                geometry::ArcSpan a{cen, rho, a0, a0 + sw, sw > 0 ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW};
                path.spans.push_back(a); p = a.endPoint(); dir += sw;
            }
        }
        auto band = toolpath::bandOutline(path, w, chordal);
        if (!band) { ++nNull; continue; }
        geometry::Tolerance tol; tol.chordal = chordal;
        auto UB = geometry::unionContours({*band}, tol, geometry::FillRule::NonZero);
        auto UC0 = geometry::offsetContours({path}, w, geometry::JoinType::Round, tol, 2.0);
        auto UC = geometry::unionContours(UC0, tol, geometry::FillRule::NonZero);
        geometry::Box2 bb = path.bounds().inflated(w + 0.1);
        int M = 4000; double eU = 0, eO = 0, eUC = 0, eOC = 0; Vec2 atU{}, atO{};
        for (int m = 0; m < M; ++m) {
            Vec2 q{U(bb.min.x, bb.max.x), U(bb.min.y, bb.max.y)};
            if (m % 2) { // sample near the band edge
                q = q; }
            double d = geometry::distanceTo(path, q);
            if (std::fabs(d - w) > 3 * chordal + 0.05 && m % 2) continue;
            bool inB = geometry::locatePoint(UB, q, geometry::FillRule::NonZero, geometry::Tolerance{}) == geometry::PointLocation::Inside;
            bool inC = geometry::locatePoint(UC, q, geometry::FillRule::NonZero, geometry::Tolerance{}) == geometry::PointLocation::Inside;
            if (!inB && d < w) { if (w - d > eU) { eU = w - d; atU = q; } }
            if (inB && d > w) { if (d - w > eO) { eO = d - w; atO = q; } }
            { int wn = geometry::windingNumber(*band, q, geometry::Tolerance{}); if (wn < 0) { ++negPts; std::printf("NEG case %d w %.4f q (%.5f,%.5f) wn %d d %.5f\n", c, w, q.x, q.y, wn, d); } }
            if (!inC && d < w) eUC = std::max(eUC, w - d);
            if (inC && d > w) eOC = std::max(eOC, d - w);
        }
        bool bad = eU > chordal * 1.05 + 1e-6 || eO > chordal * 1.05 + 1e-6;
        if (eU > chordal * 1.05 + 1e-6) ++nU; if (eO > chordal * 1.05 + 1e-6) ++nO;
        if (eUC > chordal * 1.05 + 1e-6) ++nUC; if (eOC > chordal * 1.05 + 1e-6) ++nOC;
        worstUB = std::max(worstUB, eU / chordal); worstOB = std::max(worstOB, eO / chordal); worstUC = std::max(worstUC, eUC / chordal); worstOC = std::max(worstOC, eOC / chordal);
        double aB = 0, aC = 0; for (auto& x : UB) aB += geometry::signedArea(x); for (auto& x : UC) aC += geometry::signedArea(x);
        if (bad || std::getenv("V")) std::printf("case %lu %d w %.4f chordal %g spans %zu | B under %.5f @(%.4f,%.4f) over %.5f @(%.4f,%.4f) | C under %.5f over %.5f | area B %.5f C %.5f\n", seed, c, w, chordal, path.spans.size(), eU, atU.x, atU.y, eO, atO.x, atO.y, eUC, eOC, aB, aC);
    }
    std::printf("SUMMARY neg %ld seed %lu N %d null %ld | B under>chordal %ld over>chordal %ld worst/chordal %.3f %.3f | C under %ld over %ld worst/chordal %.3f %.3f\n", negPts, seed, N, nNull, nU, nO, worstUB, worstOB, nUC, nOC, worstUC, worstOC);
}
