#include "carvetto/toolpath/clearance.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "work_meter.hpp"

namespace carvetto::toolpath {

namespace {

/// True when the distance from `p` to box `b` (infinite for an empty box), hypot(dx, dy) of the
/// gaps per axis, is >= `limit`. Without the hypot where one axis decides it (T-0131: hypot was
/// much of the time of the tab check's distance queries), with the same answer in every case:
/// hypot(dx, dy) >= max(dx, dy), and hypot(d, 0) == d exactly.
bool boxAtLeast(const geometry::Box2& b, geometry::Vec2 p, double limit) {
    if (b.isEmpty()) {
        return std::numeric_limits<double>::infinity() >= limit;
    }
    const double dx = std::max({b.min.x - p.x, 0.0, p.x - b.max.x});
    const double dy = std::max({b.min.y - p.y, 0.0, p.y - b.max.y});
    if (std::isnan(dx) || std::isnan(dy)) {
        return std::hypot(dx, dy) >= limit;
    }
    if (dx >= limit || dy >= limit) {
        return true;
    }
    if (dx == 0.0 || dy == 0.0) {
        return std::max(dx, dy) >= limit;  // false here; hypot(d, 0) == d
    }
    return std::hypot(dx, dy) >= limit;
}

/// True when the segments ab and cd cross (proper crossing; touching ends do not count).
bool segmentsCross(geometry::Vec2 a, geometry::Vec2 b, geometry::Vec2 c, geometry::Vec2 d) {
    const auto side = [](geometry::Vec2 p, geometry::Vec2 q, geometry::Vec2 r) {
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    };
    const double d1 = side(a, b, c);
    const double d2 = side(a, b, d);
    const double d3 = side(c, d, a);
    const double d4 = side(c, d, b);
    return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0)) && d1 != 0.0 && d2 != 0.0 && d3 != 0.0 &&
           d4 != 0.0;
}

/// Work units of one exact closest-point evaluation on `s` (T-0186; one unit is about 30 ns of
/// CPU): a line costs 1, an arc 4 (a square root and angle tests). A Bezier costs 40 (1.2 us: the
/// root isolation of an ordinary curve measured 0.6 to 1.5 us), 110 (3 us) when its control
/// polygon crosses itself, a looped or cusped curve, which isolates many more roots (a thin
/// self-intersecting loop measured 2 to 3 us a test at every size from 73 mm to 7 m).
std::uint64_t closestPointUnits(const geometry::Span& s) {
    if (const auto* bz = std::get_if<geometry::BezierSpan>(&s)) {
        return segmentsCross(bz->p0, bz->p1, bz->p2, bz->p3) ? 110 : 40;
    }
    return std::holds_alternative<geometry::ArcSpan>(s) ? 4 : 1;
}

struct Sample {
    double u;
    double d;
};

// Appends samples strictly inside (a, b); `fa`/`fb` are the end distances.
void refine(const ArcLengthPath& path, const ClearanceField& field, double threshold, double resolution, double a,
            double fa, double b, double fb, std::vector<Sample>& out, int depth) {
    const double half = 0.5 * (b - a);
    if (std::min(fa, fb) - half >= threshold) {
        return;  // certified clear
    }
    if (std::max(fa, fb) + half < threshold) {
        return;  // certified violating throughout (end samples carry it)
    }
    if (b - a <= resolution || depth > 60) {
        return;
    }
    const double m = 0.5 * (a + b);
    const double fm = field.distance(path.pointAt(m));
    refine(path, field, threshold, resolution, a, fa, m, fm, out, depth + 1);
    out.push_back({m, fm});
    refine(path, field, threshold, resolution, m, fm, b, fb, out, depth + 1);
}

}  // namespace

ClearanceField::ClearanceField(std::vector<geometry::Contour> obstacles) {
    auto data = std::make_shared<Data>();
    data->obstacles = std::move(obstacles);
    std::vector<geometry::Box2> boxes;
    boxes.reserve(data->obstacles.size());
    for (const geometry::Contour& c : data->obstacles) {
        boxes.push_back(c.bounds());
    }
    data->index = BoxIndex(std::move(boxes));
    // A box round every span, containing every point closestPoint can return on it (a line's
    // ends; an arc's whole circle; a Bezier's control points), up to rounding.
    double scale = 0.0;
    data->spanStart.reserve(data->obstacles.size() + 1);
    for (const geometry::Contour& c : data->obstacles) {
        data->spanStart.push_back(data->spanBoxes.size());
        for (const geometry::Span& sp : c.spans) {
            geometry::Box2 b;
            if (const auto* line = std::get_if<geometry::LineSpan>(&sp)) {
                b = geometry::Box2{line->start, line->end};
            } else if (const auto* arc = std::get_if<geometry::ArcSpan>(&sp)) {
                const double r = std::fabs(arc->radius);
                b = geometry::Box2{arc->centre - Vec2{r, r}, arc->centre + Vec2{r, r}};
            } else {
                const auto& bz = std::get<geometry::BezierSpan>(sp);
                b = geometry::Box2{bz.p0, bz.p1};
                b.expand(bz.p2);
                b.expand(bz.p3);
            }
            data->spanBoxes.push_back(b);
            scale = std::max(
                {scale, std::fabs(b.min.x), std::fabs(b.min.y), std::fabs(b.max.x), std::fabs(b.max.y)});
        }
    }
    data->spanStart.push_back(data->spanBoxes.size());
    data->margin = 1e-9 * (1.0 + scale);
    detail::chargeWork(data->spanBoxes.size() + data->obstacles.size() + 1);  // work meter (T-0186)
    data_ = std::move(data);
}

// min(best, distanceTo(obstacle i, p)) (T-0131). The span whose box lies nearest along x or y is
// measured first, then every other span whose box lies more than the best distance so far (plus
// a rounding margin) away along x or y is skipped: its distance is above that best, so it could
// not lower the minimum. The others are measured exactly as distanceTo measures them, and the
// minimum of the same values does not depend on the order they are taken in.
double ClearanceField::obstacleDistance(std::size_t i, geometry::Vec2 p, double best) const {
    const geometry::Contour& c = data_->obstacles[i];
    const std::size_t n = c.spans.size();
    // Work meter (T-0186): the box scan costs about a quarter of a unit per span.
    detail::chargeWork(n / 4 + 1);
    const geometry::Box2* boxes = data_->spanBoxes.data() + data_->spanStart[i];
    auto gap = [&](const geometry::Box2& b) {
        return std::max({b.min.x - p.x, p.x - b.max.x, b.min.y - p.y, p.y - b.max.y});
    };
    std::size_t first = 0;
    double firstGap = std::numeric_limits<double>::infinity();
    for (std::size_t k = 0; k < n; ++k) {
        const double g = gap(boxes[k]);
        if (g < firstGap) {
            firstGap = g;
            first = k;
        }
    }
    double d = std::numeric_limits<double>::infinity();
    if (n > 0) {
        detail::chargeWork(closestPointUnits(c.spans[first]));
        d = geometry::closestPoint(c.spans[first], p).distance;
        if (!(d < std::numeric_limits<double>::infinity())) {
            d = std::numeric_limits<double>::infinity();  // NaN counts as no value, as in distanceTo
        }
    }
    const double slack = data_->margin + 1e-9 * (std::fabs(p.x) + std::fabs(p.y));
    for (std::size_t k = 0; k < n; ++k) {
        if (k == first || gap(boxes[k]) > std::min(best, d) + slack) {
            continue;
        }
        detail::chargeWork(closestPointUnits(c.spans[k]));
        const double dk = geometry::closestPoint(c.spans[k], p).distance;
        if (dk < d) {
            d = dk;
        }
    }
    return std::min(best, d);
}

std::size_t ClearanceField::size() const {
    return data_ ? data_->obstacles.size() - skip_.size() : 0;
}

const geometry::Box2& ClearanceField::bounds() const {
    static const geometry::Box2 kEmpty;
    return data_ ? data_->index.bounds() : kEmpty;
}

bool ClearanceField::skipped(std::size_t i) const {
    return !skip_.empty() && std::binary_search(skip_.begin(), skip_.end(), i);
}

ClearanceField ClearanceField::excluding(std::vector<std::size_t> indices) const {
    ClearanceField out = *this;
    out.skip_.insert(out.skip_.end(), indices.begin(), indices.end());
    std::sort(out.skip_.begin(), out.skip_.end());
    out.skip_.erase(std::unique(out.skip_.begin(), out.skip_.end()), out.skip_.end());
    if (data_) {
        out.skip_.erase(std::remove_if(out.skip_.begin(), out.skip_.end(),
                                       [&](std::size_t i) { return i >= data_->obstacles.size(); }),
                        out.skip_.end());
    }
    return out;
}

double ClearanceField::distance(geometry::Vec2 p) const {
    double best = std::numeric_limits<double>::infinity();
    if (isEmpty()) {
        return best;
    }
    const BoxIndex& index = data_->index;
    const geometry::Box2& all = index.bounds();
    // Squares of half-width r round p: an obstacle whose box misses the square is more than r
    // away, so once the best distance is <= r every other obstacle is ruled out. The square
    // grows until it covers every box.
    const double reach = std::max({std::fabs(p.x - all.min.x), std::fabs(p.x - all.max.x),
                                   std::fabs(p.y - all.min.y), std::fabs(p.y - all.max.y)});
    double r = std::max(index.size() > 64 ? 0.5 * std::max(all.width(), all.height()) /
                                                std::sqrt(static_cast<double>(index.size()))
                                          : reach,
                        1e-6);
    for (;;) {
        const bool last = r >= reach || !std::isfinite(r);
        const std::vector<std::size_t> hits =
            last ? std::vector<std::size_t>{} : index.query(geometry::Box2{p - Vec2{r, r}, p + Vec2{r, r}});
        detail::chargeWork(2 + hits.size());
        auto visit = [&](std::size_t i) {
            if (skipped(i) || boxAtLeast(index.box(i), p, best)) {
                return;  // excluded, or cannot be nearer (a repeat visit from a smaller square)
            }
            best = obstacleDistance(i, p, best);
        };
        if (last) {
            for (std::size_t i = 0; i < index.size(); ++i) {
                visit(i);
            }
            return best;
        }
        for (std::size_t i : hits) {
            visit(i);
        }
        if (best <= r) {
            return best;
        }
        r *= 4.0;
    }
}

/// Most evenly spaced samples findViolations lays along a path (T-0184): 2^21 points, about 100 MB.
constexpr std::size_t kMaxGridSamples = std::size_t{1} << 21;

std::vector<std::pair<double, double>> findViolations(const ArcLengthPath& path, double u0, double u1,
                                                      const ClearanceField& field, double threshold,
                                                      double resolution) {
    std::vector<std::pair<double, double>> result;
    if (path.isEmpty() || field.isEmpty() || u1 <= u0) {
        return result;
    }
    // Whole path far from every obstacle: nothing to do.
    {
        geometry::Box2 pb = path.contour().bounds();
        if (!pb.inflated(threshold).intersects(field.bounds())) {
            return result;
        }
    }
    resolution = std::max(resolution, 1e-9);
    // Initial samples at span boundaries (and at most every 1 mm) so Lipschitz bounds are tight.
    std::vector<double> seeds{u0, u1};
    const double len = path.length();
    for (std::size_t i = 0; i < path.spanCount(); ++i) {
        for (double base = std::floor(u0 / std::max(len, 1e-300)) * len; base <= u1; base += len) {
            const double s = base + path.spanStart(i);
            if (s > u0 && s < u1) {
                seeds.push_back(s);
            }
            if (!path.closed() || len <= 0.0) {
                break;
            }
        }
    }
    std::sort(seeds.begin(), seeds.end());
    // Work meter (T-0186): about one sample per millimetre, charged before the grid is built.
    detail::chargeWork(detail::unitsOf(u1 - u0) + seeds.size());
    std::vector<double> grid;
    // T-0184: the grid holds at most about kMaxGridSamples points whatever the length. Up to that
    // many millimetres of path the spacing is the 1 mm above (unchanged); a longer path (a huge
    // arc, 6e9 mm round) gets a proportionally wider spacing instead of an int overflow and an
    // allocation of gigabytes. Profile refuses such vectors up front, so this guards direct callers.
    const double stride = std::max(1.0, (u1 - u0) / static_cast<double>(kMaxGridSamples));
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        grid.push_back(seeds[i]);
        if (i + 1 < seeds.size()) {
            const double gap = seeds[i + 1] - seeds[i];
            const double wanted = std::floor(gap / stride);
            const auto extra = wanted >= 1.0 ? static_cast<std::size_t>(std::min(wanted, 1e9)) : std::size_t{0};
            for (std::size_t k = 1; k <= extra; ++k) {
                grid.push_back(seeds[i] + gap * static_cast<double>(k) / static_cast<double>(extra + 1));
            }
        }
    }
    std::vector<Sample> samples;
    samples.reserve(grid.size() * 2);
    double prevU = grid.front();
    double prevD = field.distance(path.pointAt(prevU));
    samples.push_back({prevU, prevD});
    for (std::size_t i = 1; i < grid.size(); ++i) {
        const double u = grid[i];
        if (u - prevU <= 0.0) {
            continue;
        }
        const double d = field.distance(path.pointAt(u));
        refine(path, field, threshold, resolution, prevU, prevD, u, d, samples, 0);
        samples.push_back({u, d});
        prevU = u;
        prevD = d;
    }
    // Runs of violating samples, widened to the neighbouring clear samples.
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (samples[i].d >= threshold) {
            continue;
        }
        std::size_t j = i;
        while (j + 1 < samples.size() && samples[j + 1].d < threshold) {
            ++j;
        }
        const double a = i > 0 ? samples[i - 1].u : samples[i].u;
        const double b = j + 1 < samples.size() ? samples[j + 1].u : samples[j].u;
        if (!result.empty() && a <= result.back().second) {
            result.back().second = b;
        } else {
            result.push_back({a, b});
        }
        i = j;
    }
    return result;
}

bool isClear(const ArcLengthPath& path, const ClearanceField& field, double threshold, double resolution) {
    return findViolations(path, 0.0, path.length(), field, threshold, resolution).empty();
}

}  // namespace carvetto::toolpath
