#include "carvetto/toolpath/clearance.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"

namespace carvetto::toolpath {

namespace {

double boxDistance(const geometry::Box2& b, geometry::Vec2 p) {
    if (b.isEmpty()) {
        return std::numeric_limits<double>::infinity();
    }
    const double dx = std::max({b.min.x - p.x, 0.0, p.x - b.max.x});
    const double dy = std::max({b.min.y - p.y, 0.0, p.y - b.max.y});
    return std::hypot(dx, dy);
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
    data_ = std::move(data);
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
        auto visit = [&](std::size_t i) {
            if (skipped(i) || boxDistance(index.box(i), p) >= best) {
                return;  // excluded, or cannot be nearer (a repeat visit from a smaller square)
            }
            best = std::min(best, geometry::distanceTo(data_->obstacles[i], p));
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
    std::vector<double> grid;
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        grid.push_back(seeds[i]);
        if (i + 1 < seeds.size()) {
            const double gap = seeds[i + 1] - seeds[i];
            const int extra = static_cast<int>(std::floor(gap / 1.0));
            for (int k = 1; k <= extra; ++k) {
                grid.push_back(seeds[i] + gap * k / (extra + 1));
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
