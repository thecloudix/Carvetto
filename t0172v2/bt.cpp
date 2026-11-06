// BoxTree attack: exactness vs brute force on adversarial families, incremental merges, memory, work.
// usage: bt exact N seed   |  bt scale fam  (work per empty/nonempty query vs n)  | bt big N fam (time/mem of N adds + N queries)
#include "box_tree.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <algorithm>
#include <sys/resource.h>
using namespace carvetto::geometry;
using detail::BoxTree;
static std::mt19937_64 R;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(R); }
static double LU(double a, double b) { return std::exp(U(std::log(a), std::log(b))); }
static Box2 C(Vec2 c, double w, double h) { return Box2{c - Vec2{w / 2, h / 2}, c + Vec2{w / 2, h / 2}}; }
static const double inf = INFINITY, nan_ = NAN;
static const int NF = 22;
static Box2 gen(int f, size_t i, size_t n) {
    double k = (double)i;
    switch (f) {
        case 0: return C({0, 0}, k + 1, k + 1);                           // concentric
        case 1: { double r = (double)(i / 2 + 1); return C(i % 2 ? Vec2{0, r / 2} : Vec2{0, -r / 2}, 2 * r, r); }
        case 2: return Box2{Vec2{0, 0}, Vec2{LU(1e-12, 1e3), 0}};         // collinear nested from origin
        case 3: return Box2{Vec2{1, 1}, Vec2{2, 2}};                      // identical
        case 4: return C({U(0, 1), U(0, 1)}, 1e6, 1e6);                   // all huge
        case 5: return C({U(0, 1e3), U(0, 1e3)}, 1e-300, 0);              // all tiny
        case 6: return C({U(0, 100), U(0, 100)}, LU(1e-12, 1e4), LU(1e-12, 1e4)); // mixed
        case 7: return C({U(-1, 1) * 1e300, U(-1, 1) * 1e300}, LU(1, 1e299), LU(1, 1e299)); // huge coords
        case 8: { // nan/inf/empty mixed into normal
            int t = (int)(i % 6);
            if (t == 0) return Box2{};
            if (t == 1) return Box2{Vec2{nan_, 0}, Vec2{1, 1}};
            if (t == 2) return Box2{Vec2{-inf, -inf}, Vec2{inf, inf}};
            if (t == 3) return Box2{Vec2{0, nan_}, Vec2{nan_, 1}};
            if (t == 4) { Box2 b; b.min = {inf, 0}; b.max = {inf, 1}; return b; }
            return C({U(0, 10), U(0, 10)}, U(0, 1), U(0, 1));
        }
        case 9: return C({1e8 + U(0, 1e-6), 1e8}, 0, 0);                    // points at 1e8, sub-ulp jitter
        case 10: return Box2{Vec2{-k - 1, k}, Vec2{k + 1, k}};              // flat boxes widening upward (pyramid)
        case 11: { double s = std::ldexp(1.0, -(int)(i % 1000)); return C({0, 0}, s, s); } // nested over 1000 binades
        case 12: { // cross: long horizontals and long verticals offset, plus holes
            double c = U(-1, 1);
            return i % 2 ? Box2{Vec2{-1e3, c}, Vec2{1e3, c}} : Box2{Vec2{c, -1e3}, Vec2{c, 1e3}};
        }
        case 13: { // ring of thin boxes around origin (annulus) — node bounds cover centre
            double a = U(0, 2 * M_PI); Vec2 p{100 * std::cos(a), 100 * std::sin(a)}; return C(p, LU(1e-3, 1), LU(1e-3, 1));
        }
        case 14: return Box2{Vec2{k, k}, Vec2{k, k}};                       // diagonal points
        case 15: return Box2{Vec2{-k, -k}, Vec2{k + 1e-9, -k}};             // L staircase
        case 16: { double x = U(0, 1); return Box2{Vec2{x, 0}, Vec2{x + 1, 1}}; } // overlapping slabs
        case 17: { double z = (double)(n - i); return Box2{Vec2{k, -z}, Vec2{k + 2 * z, z}}; } // shifting nested
        case 18: { double m = U(0, 1); return Box2{Vec2{m, m}, Vec2{1 - m, 1 - m}}; } // concentric random, some empty
        case 19: return Box2{Vec2{-0.0, 0.0}, Vec2{0.0, -0.0}};             // signed zeros
        case 20: { double x = std::nextafter(1.0, 2.0 + k); return Box2{Vec2{1, 1}, Vec2{x, x}}; } // ulp-differing
        default: return C({U(0, 1e-3), U(0, 1e-3)}, LU(1e-9, 1e-3), LU(1e-9, 1e-3)); // clustered
    }
}
static Box2 randq(const std::vector<Box2>& v) {
    int t = (int)(R() % 8);
    const Box2& b = v[R() % v.size()];
    switch (t) {
        case 0: return b;
        case 1: return Box2{b.min, b.min};
        case 2: return Box2{b.max, b.max};
        case 3: return b.inflated(LU(1e-300, 1e10));
        case 4: return Box2{Vec2{-inf, -inf}, Vec2{inf, inf}};
        case 5: { Box2 q; q.min = {nan_, 0}; q.max = {1, 1}; return q; }
        case 6: { Vec2 p{U(-2, 2), U(-2, 2)}; return Box2{p, p}; }
        default: return Box2{b.min, b.max + Vec2{U(-1, 1) * std::fabs(b.max.x - b.min.x), 0}};
    }
}
static double peakMB() { rusage u; getrusage(RUSAGE_SELF, &u); return u.ru_maxrss / 1024.0; }
int main(int argc, char** argv) {
    std::string mode = argv[1];
    if (mode == "exact") {
        size_t N = atol(argv[2]); R.seed(atol(argv[3]));
        long bad = 0, qs = 0;
        for (int f = 0; f < NF; ++f) for (int mix = 0; mix < 3; ++mix) {
            size_t n = 1 + R() % N;
            std::vector<Box2> v;
            for (size_t i = 0; i < n; ++i) v.push_back(mix == 2 ? gen((int)(R() % NF), i, n) : gen(f, i, n));
            size_t bulk = mix == 0 ? 0 : R() % (n + 1);
            BoxTree t(std::vector<Box2>(v.begin(), v.begin() + bulk));
            for (size_t i = bulk; i <= n; ++i) {
                int nq = (i == n) ? 200 : ((i & (i - 1)) == 0 || i % 7 == 0 ? 3 : 0);
                for (int q = 0; q < nq; ++q) {
                    Box2 Q = randq(v);
                    size_t lo = R() % (i + 1), hi = (R() % 3 == 0) ? (size_t)-1 : lo + R() % (i + 1);
                    std::vector<size_t> want, got, got2;
                    for (size_t j = 0; j < i; ++j) if (v[j].intersects(Q)) want.push_back(j);
                    t.query(Q, [&](size_t j) { got.push_back(j); });
                    std::sort(got.begin(), got.end());
                    std::vector<size_t> wr; for (size_t j : want) if (j >= lo && j <= hi) wr.push_back(j);
                    t.queryUntil(Q, lo, hi, [&](size_t j) { got2.push_back(j); return false; });
                    std::sort(got2.begin(), got2.end());
                    bool until = t.queryUntil(Q, [](size_t) { return true; });
                    ++qs;
                    if (got != want || got2 != wr || until != !want.empty()) {
                        if (++bad < 10) printf("BAD f%d mix%d n%zu i%zu got %zu want %zu range %zu/%zu\n", f, mix, n, i, got.size(), want.size(), got2.size(), wr.size());
                    }
                }
                if (i < n && t.add(v[i]) != i) { ++bad; printf("BAD id\n"); }
            }
        }
        printf("exact N=%zu seed=%s queries %ld bad %ld\n", N, argv[3], qs, bad);
        return bad != 0;
    }
    if (mode == "scale") {  // work per query for empty-result and all queries vs n
        int f = atoi(argv[2]); R.seed(7);
        for (size_t n = 1000; n <= (size_t)atol(argv[3]); n *= 4) {
            std::vector<Box2> v; for (size_t i = 0; i < n; ++i) v.push_back(gen(f, i, n));
            BoxTree t; for (auto& b : v) t.add(b);
            size_t w0 = t.work(), maxw = 0, emptyq = 0, rep = 0, maxwE = 0; size_t Q = 2000;
            double sumw = 0;
            for (size_t q = 0; q < Q; ++q) {
                Box2 qq = randq(v); size_t before = t.work(), k = 0;
                t.query(qq, [&](size_t) { ++k; });
                size_t w = t.work() - before; sumw += w; rep += k;
                maxw = std::max(maxw, w > k ? w - k : 0);
                if (!k) { ++emptyq; maxwE = std::max(maxwE, w); }
            }
            (void)w0;
            printf("fam %2d n %8zu trees %2zu nodes %8zu avgwork %10.1f avgrep %10.1f max(work-rep) %8zu maxworkEmpty %8zu (n^.75=%.0f) empties %zu\n",
                   f, n, t.treeCount(), t.nodeCount(), sumw / Q, (double)rep / Q, maxw, maxwE, std::pow((double)n, 0.75), emptyq);
        }
        return 0;
    }
    if (mode == "big") {  // N adds interleaved with queries; time + peak memory
        size_t N = atol(argv[2]); int f = atoi(argv[3]); R.seed(9);
        auto t0 = std::chrono::steady_clock::now();
        BoxTree t; size_t rep = 0;
        for (size_t i = 0; i < N; ++i) { Box2 b = gen(f, i, N); t.add(b); Box2 q{b.min, b.min}; t.queryUntil(q, [&](size_t) { ++rep; return true; }); }
        double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        printf("big fam %d N %zu: %.3f s, peak %.0f MB, trees %zu nodes %zu work %zu (/N %.1f) rep %zu\n", f, N, s, peakMB(), t.treeCount(), t.nodeCount(), t.work(), (double)t.work() / N, rep);
        return 0;
    }
    if (mode == "adv") {  // search for the worst empty query on adversarial structures: grid of thin boxes
        size_t n = atol(argv[2]); R.seed(3);
        // "Comb" family: n/2 vertical thin boxes at x=i, y in [0,1]; n/2 horizontal thin boxes y=i, x in[-1,0]; empty queries in between
        for (int f = 0; f < 3; ++f) {
            std::vector<Box2> v;
            for (size_t i = 0; i < n; ++i) {
                double k = (double)i;
                if (f == 0) v.push_back(Box2{Vec2{k, 0}, Vec2{k, 1}});
                else if (f == 1) v.push_back(i % 2 ? Box2{Vec2{k, 0}, Vec2{k, 1}} : Box2{Vec2{0, k}, Vec2{1, k}});
                else v.push_back(Box2{Vec2{k, -k}, Vec2{k + 1, k}}); // fanning
            }
            BoxTree t(v);
            size_t worst = 0;
            for (int q = 0; q < 20000; ++q) {
                Vec2 p{U(-1, (double)n), U(-1, (double)n)};
                if (f == 0) p = {std::floor(p.x) + 0.5, U(0, 1)};
                if (f == 1) p = {std::floor(p.x) + 0.5, std::floor(p.y) + 0.5};
                double w = LU(1e-6, 0.4);
                Box2 Q{p - Vec2{w, w}, p + Vec2{w, w}};
                size_t before = t.work(), k = 0; t.query(Q, [&](size_t) { ++k; });
                size_t wk = t.work() - before; if (wk - k > worst) worst = wk - k;
            }
            printf("adv f%d n %zu worst overhead %zu (sqrt n %.0f, n^.75 %.0f)\n", f, n, worst, std::sqrt((double)n), std::pow((double)n, .75));
        }
        return 0;
    }
}
