#include "robust_predicates_old.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace carvetto::geometry::detail {

namespace {

// Floating-point expansions: a value is the exact sum of its components, stored in increasing
// order of magnitude and pairwise non-overlapping, so the sign of the sum is the sign of the
// last (largest) non-zero component. Zero components are dropped as they appear.
using Expansion = std::vector<double>;

// x + y == a + b exactly, x = fl(a + b).
inline void twoSum(double a, double b, double& x, double& y) {
    x = a + b;
    const double bv = x - a;
    const double av = x - bv;
    const double br = b - bv;
    const double ar = a - av;
    y = ar + br;
}

// x + y == a - b exactly, x = fl(a - b).
inline void twoDiff(double a, double b, double& x, double& y) {
    x = a - b;
    const double bv = a - x;
    const double av = x + bv;
    const double br = bv - b;
    const double ar = a - av;
    y = ar + br;
}

// Splits a into hi + lo, each with at most 26 significant bits.
inline void split(double a, double& hi, double& lo) {
    constexpr double kSplitter = 134217729.0;  // 2^27 + 1
    const double c = kSplitter * a;
    const double abig = c - a;
    hi = c - abig;
    lo = a - hi;
}

// x + y == a * b exactly, x = fl(a * b).
inline void twoProduct(double a, double b, double& x, double& y) {
    x = a * b;
    double ahi = 0.0;
    double alo = 0.0;
    double bhi = 0.0;
    double blo = 0.0;
    split(a, ahi, alo);
    split(b, bhi, blo);
    const double err1 = x - ahi * bhi;
    const double err2 = err1 - alo * bhi;
    const double err3 = err2 - ahi * blo;
    y = alo * blo - err3;
}

Expansion twoTermDiff(double a, double b) {
    double x = 0.0;
    double y = 0.0;
    twoDiff(a, b, x, y);
    Expansion e;
    if (y != 0.0) {
        e.push_back(y);
    }
    if (x != 0.0) {
        e.push_back(x);
    }
    return e;
}

// h = e + f (merge by magnitude, then one TwoSum pass; zero components eliminated).
Expansion expansionSum(const Expansion& e, const Expansion& f) {
    if (e.empty()) {
        return f;
    }
    if (f.empty()) {
        return e;
    }
    Expansion g;
    g.reserve(e.size() + f.size());
    std::size_t i = 0;
    std::size_t j = 0;
    while (i < e.size() && j < f.size()) {
        if (std::fabs(e[i]) < std::fabs(f[j])) {
            g.push_back(e[i++]);
        } else {
            g.push_back(f[j++]);
        }
    }
    while (i < e.size()) {
        g.push_back(e[i++]);
    }
    while (j < f.size()) {
        g.push_back(f[j++]);
    }
    Expansion h;
    h.reserve(g.size());
    double q = g[0];
    for (std::size_t k = 1; k < g.size(); ++k) {
        double sum = 0.0;
        double err = 0.0;
        twoSum(q, g[k], sum, err);
        if (err != 0.0) {
            h.push_back(err);
        }
        q = sum;
    }
    if (q != 0.0 || h.empty()) {
        h.push_back(q);
    }
    return h;
}

// h = e * b.
Expansion scaleExpansion(const Expansion& e, double b) {
    Expansion h;
    if (e.empty() || b == 0.0) {
        return h;
    }
    h.reserve(2 * e.size());
    double q = 0.0;
    double lo = 0.0;
    twoProduct(e[0], b, q, lo);
    if (lo != 0.0) {
        h.push_back(lo);
    }
    for (std::size_t i = 1; i < e.size(); ++i) {
        double prodHi = 0.0;
        double prodLo = 0.0;
        twoProduct(e[i], b, prodHi, prodLo);
        double sum = 0.0;
        double err = 0.0;
        twoSum(q, prodLo, sum, err);
        if (err != 0.0) {
            h.push_back(err);
        }
        double sum2 = 0.0;
        double err2 = 0.0;
        twoSum(prodHi, sum, sum2, err2);
        if (err2 != 0.0) {
            h.push_back(err2);
        }
        q = sum2;
    }
    if (q != 0.0) {
        h.push_back(q);
    }
    return h;
}

// h = e * f.
Expansion expansionProduct(const Expansion& e, const Expansion& f) {
    Expansion h;
    for (double c : f) {
        h = expansionSum(h, scaleExpansion(e, c));
    }
    return h;
}

Expansion negate(Expansion e) {
    for (double& c : e) {
        c = -c;
    }
    return e;
}

int expansionSign(const Expansion& e) {
    for (std::size_t k = e.size(); k-- > 0;) {
        if (e[k] > 0.0) {
            return 1;
        }
        if (e[k] < 0.0) {
            return -1;
        }
    }
    return 0;
}

Expansion productTerm(double a, double b) {
    double x = 0.0;
    double y = 0.0;
    twoProduct(a, b, x, y);
    Expansion e;
    if (y != 0.0) {
        e.push_back(y);
    }
    if (x != 0.0) {
        e.push_back(x);
    }
    return e;
}

int orient2dExact(Vec2 a, Vec2 b, Vec2 c) {
    // (ax-cx)(by-cy) - (ay-cy)(bx-cx), expanded so every term is a product of two inputs
    // (the cx*cy terms cancel).
    Expansion s = productTerm(a.x, b.y);
    s = expansionSum(s, negate(productTerm(a.x, c.y)));
    s = expansionSum(s, negate(productTerm(c.x, b.y)));
    s = expansionSum(s, negate(productTerm(a.y, b.x)));
    s = expansionSum(s, productTerm(a.y, c.x));
    s = expansionSum(s, productTerm(c.y, b.x));
    return expansionSign(s);
}

int incircleExact(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const Expansion adx = twoTermDiff(a.x, d.x);
    const Expansion ady = twoTermDiff(a.y, d.y);
    const Expansion bdx = twoTermDiff(b.x, d.x);
    const Expansion bdy = twoTermDiff(b.y, d.y);
    const Expansion cdx = twoTermDiff(c.x, d.x);
    const Expansion cdy = twoTermDiff(c.y, d.y);
    const Expansion alift = expansionSum(expansionProduct(adx, adx), expansionProduct(ady, ady));
    const Expansion blift = expansionSum(expansionProduct(bdx, bdx), expansionProduct(bdy, bdy));
    const Expansion clift = expansionSum(expansionProduct(cdx, cdx), expansionProduct(cdy, cdy));
    const Expansion bc = expansionSum(expansionProduct(bdx, cdy), negate(expansionProduct(cdx, bdy)));
    const Expansion ca = expansionSum(expansionProduct(cdx, ady), negate(expansionProduct(adx, cdy)));
    const Expansion ab = expansionSum(expansionProduct(adx, bdy), negate(expansionProduct(bdx, ady)));
    Expansion det = expansionProduct(alift, bc);
    det = expansionSum(det, expansionProduct(blift, ca));
    det = expansionSum(det, expansionProduct(clift, ab));
    return expansionSign(det);
}

constexpr double kEps = 1.1102230246251565e-16;  // 2^-53
// Forward error bounds of the double evaluations below, with some margin over the published
// (3 + 16 eps) eps and (10 + 96 eps) eps.
constexpr double kOrientBound = 4.0 * kEps;
constexpr double kIncircleBound = 12.0 * kEps;

}  // namespace

int orient2dSign(Vec2 a, Vec2 b, Vec2 c) {
    const double detLeft = (a.x - c.x) * (b.y - c.y);
    const double detRight = (a.y - c.y) * (b.x - c.x);
    const double det = detLeft - detRight;
    const double bound = kOrientBound * (std::fabs(detLeft) + std::fabs(detRight));
    if (det > bound) {
        return 1;
    }
    if (-det > bound) {
        return -1;
    }
    return orient2dExact(a, b, c);
}

int incircleSign(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const double adx = a.x - d.x;
    const double ady = a.y - d.y;
    const double bdx = b.x - d.x;
    const double bdy = b.y - d.y;
    const double cdx = c.x - d.x;
    const double cdy = c.y - d.y;
    const double bdxcdy = bdx * cdy;
    const double cdxbdy = cdx * bdy;
    const double alift = adx * adx + ady * ady;
    const double cdxady = cdx * ady;
    const double adxcdy = adx * cdy;
    const double blift = bdx * bdx + bdy * bdy;
    const double adxbdy = adx * bdy;
    const double bdxady = bdx * ady;
    const double clift = cdx * cdx + cdy * cdy;
    const double det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
    const double permanent = (std::fabs(bdxcdy) + std::fabs(cdxbdy)) * alift +
                             (std::fabs(cdxady) + std::fabs(adxcdy)) * blift +
                             (std::fabs(adxbdy) + std::fabs(bdxady)) * clift;
    const double bound = kIncircleBound * permanent;
    if (det > bound) {
        return 1;
    }
    if (-det > bound) {
        return -1;
    }
    return incircleExact(a, b, c, d);
}

}  // namespace carvetto::geometry::detail
