#include "carvetto/relief/modify.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace carvetto::relief {

namespace {
bool opaque(float v) { return !std::isnan(v); }
constexpr std::size_t kMaxSegments = 65536;
constexpr std::size_t kMaxRadiusCells = 512;
constexpr double kMaxWork = 4.0e9;  // cells * window taps
}  // namespace

Component componentFromField(const Component& like, HeightField field) {
    Component c;
    c.name = like.name;
    c.visible = like.visible;
    c.mode = CombineMode::Add;
    c.field = std::move(field);
    return c;
}

Status bakeComponents(const std::vector<Component>& components, Component& out) {
    out = Component{};
    Level level;
    level.components = components;
    std::vector<Level> levels{level};
    Grid g;
    Status s = unionGrid(levels, g);
    if (s != Status::Ok) return s;
    HeightField f;
    s = compose(levels, g, f);
    if (s != Status::Ok) return s;
    out.name = components.empty() ? std::string() : components.front().name;
    out.field = std::move(f);
    return Status::Ok;
}

Status replaceBelow(const HeightField& in, const ReplaceBelowParams& p, HeightField& out) {
    out = HeightField{};
    if (!std::isfinite(p.threshold) || !std::isfinite(p.planeHeight) || !std::isfinite(p.lowerBy)) {
        return Status::BadParameter;
    }
    if (in.empty()) return Status::ZeroSize;
    std::vector<float> d = in.data();
    const float repl = p.with == ReplaceWith::Plane ? static_cast<float>(p.planeHeight) : kTransparent;
    for (float& v : d) {
        if (!opaque(v)) continue;
        if (static_cast<double>(v) < p.threshold) v = repl;
        if (opaque(v)) v = static_cast<float>(static_cast<double>(v) - p.lowerBy);
    }
    return HeightField::fromData(in.grid(), std::move(d), out);
}

Status resetByVectors(const HeightField& in, const std::vector<geometry::Polyline>& vectors, ResetRegion region,
                      HeightField& out) {
    out = HeightField{};
    if (in.empty()) return Status::ZeroSize;
    std::size_t segs = 0;
    for (const geometry::Polyline& v : vectors) {
        if (!v.closed || v.points.size() < 3) return Status::BadParameter;
        for (const geometry::Vec2& q : v.points) {
            if (!std::isfinite(q.x) || !std::isfinite(q.y)) return Status::BadParameter;
        }
        segs += v.points.size();
        if (segs > kMaxSegments) return Status::BadParameter;
    }
    std::vector<float> d = in.data();
    const std::size_t w = in.width(), h = in.height();
    const double cs = in.cellSize();
    std::vector<double> xs;
    for (std::size_t j = 0; j < h; ++j) {
        const double y = in.origin().y + static_cast<double>(j) * cs;
        xs.clear();
        for (const geometry::Polyline& v : vectors) {
            const std::size_t n = v.points.size();
            for (std::size_t k = 0; k < n; ++k) {
                const geometry::Vec2 a = v.points[k], b = v.points[(k + 1) % n];
                if ((a.y <= y) != (b.y <= y)) xs.push_back(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
            }
        }
        std::sort(xs.begin(), xs.end());
        for (std::size_t i = 0; i < w; ++i) {
            const double x = in.origin().x + static_cast<double>(i) * cs;
            // Crossings strictly right of x toggle; the total is even, so parity of those at or left matches.
            const std::size_t left = static_cast<std::size_t>(std::upper_bound(xs.begin(), xs.end(), x) - xs.begin());
            const bool inside = (left % 2) == 1;
            if (inside == (region == ResetRegion::Inside)) d[j * w + i] = kTransparent;
        }
    }
    return HeightField::fromData(in.grid(), std::move(d), out);
}

Status offsetModel(const HeightField& in, const OffsetParams& p, HeightField& out) {
    out = HeightField{};
    if (!std::isfinite(p.distance)) return Status::BadParameter;
    if (in.empty()) return Status::ZeroSize;
    const std::size_t w = in.width(), h = in.height();
    const double cs = in.cellSize();
    const double r = std::fabs(p.distance);
    const double rc = std::floor(r / cs);  // whole cells reachable along an axis
    if (rc > static_cast<double>(kMaxRadiusCells)) return Status::BadParameter;
    const std::size_t R = static_cast<std::size_t>(rc);
    const double win = 2.0 * static_cast<double>(R) + 1.0;
    if (static_cast<double>(w) * static_cast<double>(h) * win * win > kMaxWork) return Status::TooLarge;

    std::vector<float> d = in.data();
    if (p.distance != 0.0) {
        // Ball profile: lift(q) = sqrt(r^2 - |q|^2). Grow = max(f + lift), shrink = min(f - lift).
        const bool grow = p.distance > 0.0;
        const std::size_t side = 2 * R + 1;
        std::vector<double> lift(side * side, 0.0);
        std::vector<bool> inBall(side * side, false);
        for (std::size_t b = 0; b < side; ++b) {
            for (std::size_t a = 0; a < side; ++a) {
                const double dx = (static_cast<double>(a) - static_cast<double>(R)) * cs;
                const double dy = (static_cast<double>(b) - static_cast<double>(R)) * cs;
                const double q = r * r - dx * dx - dy * dy;
                if (q >= 0.0) {
                    inBall[b * side + a] = true;
                    lift[b * side + a] = std::sqrt(q);
                }
            }
        }
        for (std::size_t j = 0; j < h; ++j) {
            for (std::size_t i = 0; i < w; ++i) {
                if (!opaque(in.at(i, j))) continue;
                double best = grow ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
                const std::size_t j0 = j > R ? j - R : 0, j1 = std::min(h - 1, j + R);
                const std::size_t i0 = i > R ? i - R : 0, i1 = std::min(w - 1, i + R);
                for (std::size_t jj = j0; jj <= j1; ++jj) {
                    for (std::size_t ii = i0; ii <= i1; ++ii) {
                        const float v = in.at(ii, jj);
                        if (!opaque(v)) continue;
                        const std::size_t k = (jj + R - j) * side + (ii + R - i);
                        if (!inBall[k]) continue;
                        const double c = grow ? static_cast<double>(v) + lift[k] : static_cast<double>(v) - lift[k];
                        best = grow ? std::max(best, c) : std::min(best, c);
                    }
                }
                d[j * w + i] = static_cast<float>(best);
            }
        }
    }
    if (p.clipToZero) {
        for (float& v : d) {
            if (opaque(v) && v < 0.0f) v = 0.0f;
        }
    }
    return HeightField::fromData(in.grid(), std::move(d), out);
}

namespace {
// Normalised Gaussian convolution over opaque cells: sum(w f) / sum(w) in double, fixed order.
Status gaussian(const HeightField& in, double sigmaMm, std::vector<double>& num, std::vector<double>& den) {
    const std::size_t w = in.width(), h = in.height();
    const double sc = sigmaMm / in.cellSize();
    const double rcd = std::ceil(3.0 * sc);
    if (rcd > static_cast<double>(kMaxRadiusCells)) return Status::BadParameter;
    const std::size_t R = static_cast<std::size_t>(rcd);
    std::vector<double> k(R + 1);
    for (std::size_t t = 0; t <= R; ++t) k[t] = std::exp(-0.5 * static_cast<double>(t * t) / (sc * sc));
    num.assign(w * h, 0.0);
    den.assign(w * h, 0.0);
    std::vector<double> tn(w * h, 0.0), td(w * h, 0.0);
    for (std::size_t j = 0; j < h; ++j) {  // horizontal pass
        for (std::size_t i = 0; i < w; ++i) {
            double sn = 0.0, sd = 0.0;
            const std::size_t i0 = i > R ? i - R : 0, i1 = std::min(w - 1, i + R);
            for (std::size_t ii = i0; ii <= i1; ++ii) {
                const float v = in.at(ii, j);
                if (!opaque(v)) continue;
                const double wt = k[ii > i ? ii - i : i - ii];
                sn += wt * static_cast<double>(v);
                sd += wt;
            }
            tn[j * w + i] = sn;
            td[j * w + i] = sd;
        }
    }
    for (std::size_t j = 0; j < h; ++j) {  // vertical pass
        for (std::size_t i = 0; i < w; ++i) {
            double sn = 0.0, sd = 0.0;
            const std::size_t j0 = j > R ? j - R : 0, j1 = std::min(h - 1, j + R);
            for (std::size_t jj = j0; jj <= j1; ++jj) {
                const double wt = k[jj > j ? jj - j : j - jj];
                sn += wt * tn[jj * w + i];
                sd += wt * td[jj * w + i];
            }
            num[j * w + i] = sn;
            den[j * w + i] = sd;
        }
    }
    return Status::Ok;
}

Status checkSmooth(double percent, double radiusMm) {
    if (!std::isfinite(percent) || !std::isfinite(radiusMm) || percent < 0.0 || percent > 100.0 || radiusMm <= 0.0) {
        return Status::BadParameter;
    }
    return Status::Ok;
}
}  // namespace

Status smooth(const HeightField& in, const SmoothParams& p, HeightField& out) {
    out = HeightField{};
    Status s = checkSmooth(p.percent, p.radiusMm);
    if (s != Status::Ok) return s;
    if (in.empty()) return Status::ZeroSize;
    const double sigma = p.percent / 100.0 * p.radiusMm;
    if (sigma <= 0.0) return HeightField::fromData(in.grid(), in.data(), out);
    std::vector<double> num, den;
    s = gaussian(in, sigma, num, den);
    if (s != Status::Ok) return s;
    std::vector<float> d(in.data().size());
    for (std::size_t k = 0; k < d.size(); ++k) {
        const bool wasOpaque = opaque(in.data()[k]);
        if (wasOpaque) {
            d[k] = static_cast<float>(num[k] / den[k]);
        } else if (!p.preserveTransparency && den[k] >= 0.5) {
            d[k] = static_cast<float>(num[k] / den[k]);
        } else {
            d[k] = kTransparent;
        }
    }
    return HeightField::fromData(in.grid(), std::move(d), out);
}

Status emboss(const HeightField& in, const EmbossParams& p, HeightField& out) {
    out = HeightField{};
    if (!std::isfinite(p.scale) || p.scale < 0.0) return Status::BadParameter;
    SmoothParams sp;
    sp.percent = p.detailSmoothnessPercent;
    sp.radiusMm = p.radiusMm;
    sp.preserveTransparency = true;
    HeightField low;
    const Status s = smooth(in, sp, low);
    if (s != Status::Ok) return s;
    std::vector<float> d = in.data();
    for (std::size_t k = 0; k < d.size(); ++k) {
        if (!opaque(d[k])) continue;
        d[k] = static_cast<float>(static_cast<double>(d[k]) - (1.0 - p.scale) * static_cast<double>(low.data()[k]));
    }
    return HeightField::fromData(in.grid(), std::move(d), out);
}

}  // namespace carvetto::relief
