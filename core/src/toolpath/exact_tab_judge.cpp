#include "exact_tab_judge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "enclosed_pieces.hpp"
#include "work_meter.hpp"

namespace carvetto::toolpath::detail {

namespace {

using geometry::Vec2;

constexpr double kBandSlack = 5e-4;   // the flood oracle's own slack: bands that just touch are sealed
constexpr double kSameSample = 1e-9;  // samples closer than this are one
constexpr double kMinPiece = 0.01;    // the oracle's smallest piece (mm2)
constexpr double kMaxBandSag = 0.005;

// A disc of radius r (+ the slack) about c as a counter-clockwise polygon reaching at most `sag` beyond it.
PointRing discRing(Vec2 c, double r, double sag) {
    r += kBandSlack;
    int n = 8;  // vertices on the circumscribed circle of radius r / cos(pi / n)
    while (n < 4096 && r * (1.0 / std::cos(geometry::kPi / n) - 1.0) > sag) n *= 2;
    const double rc = r / std::cos(geometry::kPi / n);
    PointRing ring;
    for (int i = 0; i < n; ++i) {
        const double ang = 2.0 * geometry::kPi * i / n;
        ring.push_back(c + Vec2{rc * std::cos(ang), rc * std::sin(ang)});
    }
    return ring;
}

}  // namespace

ToolSide classifyTool(bool isDisc, const std::vector<std::size_t>& coveredBy, std::size_t g,
                      const std::vector<bool>& alive) {
    bool mine = false;
    bool other = false;
    for (std::size_t o : coveredBy) {
        if (o == g) {
            mine = true;
        } else if (alive[o]) {
            other = true;
        }
    }
    if (isDisc && other) return ToolSide::Skip;
    return mine && !other ? ToolSide::Own : ToolSide::Cut;
}

bool exactNoWasteHolds(const geometry::Box2& window, const ExactTools& ex, double rk,
                       const geometry::Tolerance& tol, double mitreLimit) {
    const double bandSag = std::min(tol.chordal, kMaxBandSag);  // as the loose-piece search's bands
    const double reach = rk + kBandSlack;
    std::vector<PointRing> rings;
    auto addRun = [&](const std::vector<Vec2>& run) {
        std::vector<Vec2> pts;
        for (const Vec2& q : run) {
            if (pts.empty() || geometry::distance(pts.back(), q) > kSameSample) pts.push_back(q);
        }
        if (pts.empty()) return;
        if (pts.size() == 1) {
            rings.push_back(discRing(pts[0], rk, bandSag));
            return;
        }
        geometry::Contour path;
        path.closed = false;
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) path.spans.emplace_back(geometry::LineSpan{pts[i], pts[i + 1]});
        for (PointRing& r : bandOutline(path, reach, bandSag, false)) rings.push_back(std::move(r));
    };
    auto addDisc = [&](const ToolDisc& d) { rings.push_back(discRing(d.p, d.rho, bandSag)); };
    for (const auto& run : ex.cutRuns) addRun(run);
    for (const ToolDisc& d : ex.cutX) addDisc(d);
    // The frame: four bars along the window's sides, overlapping at the corners.
    const double lo_x = window.min.x;
    const double hi_x = window.max.x;
    const double lo_y = window.min.y;
    const double hi_y = window.max.y;
    auto bar = [&](double x0, double y0, double x1, double y1) {
        rings.push_back({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
    };
    bar(lo_x - 1.0, lo_y - 1.0, hi_x + 1.0, lo_y);
    bar(lo_x - 1.0, hi_y, hi_x + 1.0, hi_y + 1.0);
    bar(lo_x - 1.0, lo_y - 0.5, lo_x, hi_y + 0.5);
    bar(hi_x, lo_y - 0.5, hi_x + 1.0, hi_y + 0.5);
    std::uint64_t vertices = 0;
    for (const PointRing& r : rings) vertices += r.size();
    EnclosedOptions opt;
    opt.minArea = kMinPiece;
    opt.clearance = 0.0;
    opt.tol = tol;
    opt.mitreLimit = mitreLimit;
    chargeWork(60 * vertices + 1);
    const std::size_t before = enclosedPieces(rings, opt).size();
    for (const auto& run : ex.ownRuns) addRun(run);
    for (const ToolDisc& d : ex.ownX) addDisc(d);
    for (const PointRing& r : rings) vertices += r.size();
    chargeWork(60 * vertices + 1);
    const std::size_t after = enclosedPieces(std::move(rings), opt).size();
    if (after > before) return true;
    return before < 2;
}

}  // namespace carvetto::toolpath::detail
