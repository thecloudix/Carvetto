#include "carvetto/relief/revolve.hpp"

#include <algorithm>
#include <cmath>

#include "carvetto/relief/shape.hpp"

namespace carvetto::relief {

namespace {

constexpr std::size_t kMaxProfilePoints = 100000;

/// Profile as a function of the position p along the axis: values at non-decreasing positions.
struct Prof {
    std::vector<double> p;
    std::vector<double> v;

    /// Value at position q; false outside [p.front(), p.back()]. At a position shared by several
    /// points (a step) the largest value wins; elsewhere linear between neighbours.
    bool eval(double q, double& out) const {
        const std::size_t n = p.size();
        const std::size_t k = static_cast<std::size_t>(std::lower_bound(p.begin(), p.end(), q) - p.begin());
        if (k == n) return false;
        if (p[k] == q) {
            double best = v[k];
            for (std::size_t m = k + 1; m < n && p[m] == q; ++m) best = std::max(best, v[m]);
            out = best;
            return true;
        }
        if (k == 0) return false;
        const double f = (q - p[k - 1]) / (p[k] - p[k - 1]);
        out = v[k - 1] + (v[k] - v[k - 1]) * f;
        return true;
    }
};

/// Makes positions non-decreasing; rejects a real backwards move (more than `eps`).
bool makeMonotone(Prof& pr, double eps) {
    for (std::size_t i = 1; i < pr.p.size(); ++i) {
        if (pr.p[i] < pr.p[i - 1] - eps) return false;
        if (pr.p[i] < pr.p[i - 1]) pr.p[i] = pr.p[i - 1];
    }
    return true;
}

}  // namespace

const char* revolveStatusMessage(RevolveStatus s) {
    switch (s) {
        case RevolveStatus::Ok: return "ok";
        case RevolveStatus::BadGrid: return "invalid or oversized grid";
        case RevolveStatus::BadProfile: return "profile needs two or more finite points moving one way";
        case RevolveStatus::Degenerate: return "profile has no length or no radius";
        case RevolveStatus::BadParameter: return "revolve parameter out of range";
        case RevolveStatus::Cancelled: return "cancelled";
    }
    return "unknown";
}

RevolveStatus buildRevolve(const Revolve& rv, const std::atomic<bool>* cancel, Component& out) {
    out = Component{};
    const RevolveParams& prm = rv.params;
    const std::vector<Point>& pts = rv.profile;

    if (!std::isfinite(prm.baseHeight) || !std::isfinite(prm.center.x) || !std::isfinite(prm.center.y) ||
        (prm.exactHeight && !std::isfinite(*prm.exactHeight)))
        return RevolveStatus::BadParameter;
    if (!(prm.cellSize > 0.0) || !std::isfinite(prm.cellSize)) return RevolveStatus::BadGrid;
    if (pts.size() < 2 || pts.size() > kMaxProfilePoints) return RevolveStatus::BadProfile;
    for (const Point& q : pts)
        if (!std::isfinite(q.x) || !std::isfinite(q.y)) return RevolveStatus::BadProfile;

    const bool turn = prm.kind == RevolveKind::Turn;
    Prof pr;
    pr.p.resize(pts.size());
    pr.v.resize(pts.size());
    double ux = 1.0, uy = 0.0;      // Turn axis direction
    double ox = 0.0, oy = 0.0;      // axis origin (Turn: first point; Spin: centre)
    double span = 0.0;              // extent along the axis
    double rawMax = 0.0;            // analytic maximum height
    double reach = 0.0;             // largest radius from the axis (footprint half-width)

    if (turn) {
        ox = pts.front().x;
        oy = pts.front().y;
        const double dx = pts.back().x - ox, dy = pts.back().y - oy;
        const double len = std::hypot(dx, dy);
        if (!(len > 0.0)) return RevolveStatus::Degenerate;
        ux = dx / len;
        uy = dy / len;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const double ex = pts[i].x - ox, ey = pts[i].y - oy;
            pr.p[i] = ex * ux + ey * uy;
            pr.v[i] = std::fabs(ex * uy - ey * ux);
        }
        pr.p.front() = 0.0;
        pr.v.front() = 0.0;
        pr.p.back() = len;
        pr.v.back() = 0.0;
        if (!makeMonotone(pr, len * 1e-9)) return RevolveStatus::BadProfile;
        span = len;
        for (double r : pr.v) reach = std::max(reach, r);
        rawMax = reach;
        if (!(reach > len * 1e-12)) return RevolveStatus::Degenerate;
    } else {
        ox = prm.center.x;
        oy = prm.center.y;
        const double x0 = pts.front().x;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            pr.p[i] = pts[i].x - x0;
            pr.v[i] = pts[i].y;
        }
        const double ext = pr.p.back();
        if (!makeMonotone(pr, std::max(1e-9 * std::fabs(ext), 0.0))) return RevolveStatus::BadProfile;
        span = pr.p.back();
        if (!(span > 0.0)) return RevolveStatus::Degenerate;
        reach = span;
        rawMax = *std::max_element(pr.v.begin(), pr.v.end());
    }

    // Grid.
    Grid g;
    if (prm.grid) {
        g = *prm.grid;
        if (validate(g) != Status::Ok) return RevolveStatus::BadGrid;
    } else {
        const double cs = prm.cellSize;
        double minX, minY, maxX, maxY;
        if (turn) {
            const double nx = -uy, ny = ux;
            minX = minY = 1e300;
            maxX = maxY = -1e300;
            for (double s : {0.0, span})
                for (double w : {-reach, reach}) {
                    const double cx = ox + ux * s + nx * w, cy = oy + uy * s + ny * w;
                    minX = std::min(minX, cx);
                    maxX = std::max(maxX, cx);
                    minY = std::min(minY, cy);
                    maxY = std::max(maxY, cy);
                }
        } else {
            minX = ox - reach;
            maxX = ox + reach;
            minY = oy - reach;
            maxY = oy + reach;
        }
        const double cw = std::ceil((maxX - minX) / cs) + 1.0, ch = std::ceil((maxY - minY) / cs) + 1.0;
        if (!(cw * ch <= static_cast<double>(kMaxCells))) return RevolveStatus::BadGrid;
        g.cellSize = cs;
        if (turn) {
            g.origin = {minX, minY};
            g.width = static_cast<std::size_t>(cw);
            g.height = static_cast<std::size_t>(ch);
        } else {
            // Centre on a cell so the axis is sampled exactly.
            const double n = std::ceil(reach / cs);
            if (!((2.0 * n + 1.0) * (2.0 * n + 1.0) <= static_cast<double>(kMaxCells))) return RevolveStatus::BadGrid;
            g.origin = {ox - n * cs, oy - n * cs};
            g.width = g.height = static_cast<std::size_t>(2.0 * n + 1.0);
        }
        if (validate(g) != Status::Ok) return RevolveStatus::BadGrid;
    }

    double scale = 100.0, base = prm.baseHeight;
    if (prm.exactHeight) {
        if (setTotalHeight(rawMax, *prm.exactHeight, TotalHeightAdjust::Scale, scale, base) != ShapeStatus::Ok)
            return RevolveStatus::BadParameter;
        base = prm.baseHeight;
    }

    HeightField f;
    if (HeightField::create(g, kTransparent, f) != Status::Ok) return RevolveStatus::BadGrid;
    const double nx = -uy, ny = ux;
    for (std::size_t j = 0; j < g.height; ++j) {
        if (cancel && cancel->load()) return RevolveStatus::Cancelled;
        const double y = g.origin.y + static_cast<double>(j) * g.cellSize;
        for (std::size_t i = 0; i < g.width; ++i) {
            const double x = g.origin.x + static_cast<double>(i) * g.cellSize;
            const double dx = x - ox, dy = y - oy;
            double val = 0.0;
            if (turn) {
                const double t = dx * ux + dy * uy, w = dx * nx + dy * ny;
                double r = 0.0;
                if (!pr.eval(t, r) || std::fabs(w) > r) continue;
                val = std::sqrt(std::max(r * r - w * w, 0.0));
            } else {
                if (!pr.eval(std::hypot(dx, dy), val)) continue;
            }
            f.set(i, j, static_cast<float>(val));
        }
    }

    out.name = prm.name;
    out.field = std::move(f);
    out.mode = prm.mode;
    out.shapeHeightPercent = scale;
    out.baseHeight = base;
    return RevolveStatus::Ok;
}

}  // namespace carvetto::relief
