#include <limits>
struct SX { long sxNew = 0, sxOld = 0, gaps = 0, back = 0, dv = 0, rv = 0, endShort = 0; double maxNoise = 0, maxGap = 0; double maxRealLow = 1e300; };
static void sxCount(const std::vector<Contour>& chains, SX& r, bool verbose = false) {
    const Tolerance exact{1e-9, 1e-9, 0.01};
    const double eps = std::numeric_limits<double>::epsilon();
    for (const Contour& c : chains) {
        const std::size_t n = c.spans.size();
        for (std::size_t i = 0; i < n; ++i) for (std::size_t j = i + 1; j < n; ++j) {
            const bool next = j == i + 1;
            const bool wrap = c.closed && i == 0 && j + 1 == n;
            for (const Intersection& x : intersect(c.spans[i], c.spans[j], exact).points) {
                const double slack = 1e-9 * std::max({1.0, std::fabs(x.point.x), std::fabs(x.point.y)});
                const bool atJunction = (next && distance(x.point, endPoint(c.spans[i])) <= slack) ||
                    (wrap && distance(x.point, startPoint(c.spans[i])) <= slack) ||
                    (n == 2 && c.closed && distance(x.point, endPoint(c.spans[j])) <= slack);
                bool nearJunction = false;
                if (j - i <= 3 || (c.closed && n - j + i <= 3)) for (const Span& sp : c.spans) nearJunction = nearJunction || distance(x.point, startPoint(sp)) <= slack;
                bool noise = false; double ratio = 1e300;
                if (next || wrap) {
                    const double sine = std::fabs(tangentAt(c.spans[i], x.tA).cross(tangentAt(c.spans[j], x.tB)));
                    const double unit = eps * std::max({1.0, std::fabs(x.point.x), std::fabs(x.point.y), length(c.spans[i]), length(c.spans[j])});
                    std::vector<Vec2> js; if (next) js.push_back(endPoint(c.spans[i])); if (wrap) js.push_back(startPoint(c.spans[i]));
                    for (Vec2 jn : js) { ratio = std::min(ratio, distance(x.point, jn) * sine / unit); noise = noise || distance(x.point, jn) * sine <= 100.0 * unit; }
                }
                if (!atJunction && !nearJunction) {
                    r.sxOld++;
                    if (noise) r.maxNoise = std::max(r.maxNoise, ratio); else { r.sxNew++; if (ratio < 1e300) r.maxRealLow = std::min(r.maxRealLow, ratio); }
                    if (verbose) std::printf("   X spans %zu,%zu next %d at (%.17g,%.17g) ratio %.4g %s\n", i, j, (int)next, x.point.x, x.point.y, ratio, noise ? "NOISE" : "REAL");
                }
            }
        }
    }
}
