// T-0167: the block shortcut of tabHolds' raster (src/toolpath/block_settle.hpp). One distance query
// settles a block of 4 x 4 cells only when every cell of the block would get the same verdict from
// its own, exact distance. The constants (the block radius, the anchor cell, the two comparisons) are
// pinned here: a block radius of half a cell or none, a threshold loosened by cells, or an anchor
// outside the block each settle a block some cell of which disagrees.

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "block_settle.hpp"
#include "carvetto/geometry/vec2.hpp"

using carvetto::geometry::Vec2;
using carvetto::toolpath::detail::BlockVerdict;
using carvetto::toolpath::detail::kSettleBlock;
using carvetto::toolpath::detail::settleBlock;

namespace {

using Seg = std::pair<Vec2, Vec2>;

double segDistance(Vec2 p, const Seg& s) {
    const Vec2 d = s.second - s.first;
    const double l2 = d.x * d.x + d.y * d.y;
    double t = l2 > 0.0 ? ((p.x - s.first.x) * d.x + (p.y - s.first.y) * d.y) / l2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const double ex = p.x - s.first.x - t * d.x;
    const double ey = p.y - s.first.y - t * d.y;
    return std::sqrt(ex * ex + ey * ey);
}

double fieldDistance(Vec2 p, const std::vector<Seg>& segs) {
    double best = std::numeric_limits<double>::infinity();
    for (const Seg& s : segs) best = std::min(best, segDistance(p, s));
    return best;
}

std::vector<Seg> polygon(Vec2 c, double r, int n) {
    std::vector<Seg> out;
    for (int i = 0; i < n; ++i) {
        const double a0 = 2.0 * 3.14159265358979323846 * i / n;
        const double a1 = 2.0 * 3.14159265358979323846 * (i + 1) / n;
        out.push_back({c + Vec2{r * std::cos(a0), r * std::sin(a0)}, c + Vec2{r * std::cos(a1), r * std::sin(a1)}});
    }
    return out;
}

struct Counts {
    long far{0};
    long vector{0};
    long open{0};
    long wrong{0};
};

// Sweeps thresholds round each block's exact extremes and counts verdicts that some cell contradicts.
Counts sweep(const std::vector<Seg>& segs, double h, int n) {
    const Vec2 origin{-0.5 * n * h, -0.5 * n * h};
    auto centre = [&](int i, int j) { return Vec2{origin.x + (i + 0.5) * h, origin.y + (j + 0.5) * h}; };
    const int blocks = (n + kSettleBlock - 1) / kSettleBlock;
    Counts c;
    for (int bj = 0; bj < blocks; ++bj) {
        for (int bi = 0; bi < blocks; ++bi) {
            double dmin = std::numeric_limits<double>::infinity();
            double dmax = 0.0;
            for (int j = bj * kSettleBlock; j < std::min(n, (bj + 1) * kSettleBlock); ++j) {
                for (int i = bi * kSettleBlock; i < std::min(n, (bi + 1) * kSettleBlock); ++i) {
                    const double d = fieldDistance(centre(i, j), segs);
                    dmin = std::min(dmin, d);
                    dmax = std::max(dmax, d);
                }
            }
            auto dist = [&](Vec2 q) { return fieldDistance(q, segs); };
            std::vector<double> offsets;
            for (int k = -40; k <= 40; ++k) offsets.push_back(0.01 * h * k);  // within +-0.4 cell
            for (int k = -12; k <= 12; ++k) offsets.push_back(0.5 * h * k);   // out to 6 cells
            for (const double off : offsets) {
                for (const double base : {dmin, dmax}) {
                    for (const double t : {base + off, std::nextafter(base + off, 1e9), std::nextafter(base + off, -1e9)}) {
                        // as `beyond`: every cell must be at least t away
                        if (settleBlock(bi, bj, n, n, h, origin, t, -1.0, dist) == BlockVerdict::Far) {
                            ++c.far;
                            if (!(dmin >= t)) ++c.wrong;
                        }
                        // as `vectorSide`: every cell must be less than t away
                        const BlockVerdict v = settleBlock(bi, bj, n, n, h, origin, 1e18, t, dist);
                        if (v == BlockVerdict::VectorSide) {
                            ++c.vector;
                            if (!(dmax < t)) ++c.wrong;
                        } else {
                            ++c.open;
                        }
                    }
                }
            }
        }
    }
    return c;
}

}  // namespace

TEST_CASE("T-0167: a block is settled only when every cell would get the same verdict", "[toolpath][profile][tabs]") {
    std::vector<std::pair<const char*, std::vector<Seg>>> fields;
    fields.push_back({"straight, 45 degrees", {{Vec2{-30, -30}, Vec2{30, 30}}}});
    fields.push_back({"straight, shallow", {{Vec2{-30, -9}, Vec2{30, 0}}}});
    fields.push_back({"axis-parallel", {{Vec2{-30, 0.03}, Vec2{30, 0.03}}}});
    fields.push_back({"curved", polygon({0, 0}, 3.1, 160)});
    fields.push_back({"two curves", [] {
                          std::vector<Seg> a = polygon({-1.5, 0}, 1.4, 96);
                          const std::vector<Seg> b = polygon({1.5, 0.3}, 0.9, 64);
                          a.insert(a.end(), b.begin(), b.end());
                          return a;
                      }()});
    // Features about the size of a cell: a square 0.12 mm across and a hairline segment.
    fields.push_back({"tiny", {{Vec2{0, 0}, Vec2{0.12, 0}}, {Vec2{0.12, 0}, Vec2{0.12, 0.12}}, {Vec2{0.12, 0.12}, Vec2{0, 0.12}},
                               {Vec2{0, 0.12}, Vec2{0, 0}}, {Vec2{1.3, 1.1}, Vec2{1.35, 1.1}}}});
    for (const double h : {0.05, 0.15}) {
        long far = 0;
        long vector = 0;
        for (const auto& [name, segs] : fields) {
            INFO(name << ", cell " << h);
            const Counts c = sweep(segs, h, h < 0.1 ? 60 : 40);
            CHECK(c.wrong == 0);
            far += c.far;
            vector += c.vector;
        }
        // (The shortcut does settle blocks: a rule that never settles passes the check above.)
        CHECK(far > 1000);
        CHECK(vector > 1000);
    }
}
