#include "carvetto/relief/sweep.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace carvetto::relief {

const char* sweepStatusMessage(SweepStatus s) {
    switch (s) {
        case SweepStatus::Ok: return "ok";
        case SweepStatus::BadRail: return "a rail needs at least 2 vertices (3 if closed), finite coordinates and non-zero length";
        case SweepStatus::RailKindMismatch: return "both rails must be open or both closed";
        case SweepStatus::NoSections: return "at least one cross section is needed";
        case SweepStatus::BadSection: return "a cross section is invalid or out of order";
        case SweepStatus::NodeCountMismatch: return "the rails must have the same number of nodes";
        case SweepStatus::BadParameter: return "invalid sweep parameter";
        case SweepStatus::BadGrid: return "invalid grid";
        case SweepStatus::TooLarge: return "the sweep is too large";
        case SweepStatus::Cancelled: return "cancelled";
        case SweepStatus::FlatResult: return "the swept surface has no positive height to scale";
    }
    return "unknown";
}

namespace {

bool finite(double v) { return std::isfinite(v); }

// Arc-length parametrised polyline (closed adds the closing segment).
struct Rail {
    std::vector<Point> pts;
    std::vector<double> cum;  // cumulative length at each segment end; cum.back() = total
    std::vector<double> vertexFrac;
    bool closed = false;
    double total = 0.0;

    void build(const std::vector<Point>& p, bool c) {
        pts = p;
        closed = c;
        const std::size_t n = pts.size();
        const std::size_t segs = closed ? n : n - 1;
        cum.assign(segs, 0.0);
        double acc = 0.0;
        for (std::size_t i = 0; i < segs; ++i) {
            const Point& a = pts[i];
            const Point& b = pts[(i + 1) % n];
            acc += std::hypot(b.x - a.x, b.y - a.y);
            cum[i] = acc;
        }
        total = acc;
        vertexFrac.assign(n, 0.0);
        for (std::size_t i = 1; i < n; ++i) vertexFrac[i] = total > 0 ? cum[i - 1] / total : 0.0;
    }

    Point at(double frac) const {
        const std::size_t n = pts.size();
        const double d = std::clamp(frac, 0.0, 1.0) * total;
        std::size_t i = static_cast<std::size_t>(std::lower_bound(cum.begin(), cum.end(), d) - cum.begin());
        if (i >= cum.size()) i = cum.size() - 1;
        const double s0 = i == 0 ? 0.0 : cum[i - 1];
        const double len = cum[i] - s0;
        const double w = len > 0 ? (d - s0) / len : 0.0;
        const Point& a = pts[i];
        const Point& b = pts[(i + 1) % n];
        return Point{a.x + (b.x - a.x) * w, a.y + (b.y - a.y) * w};
    }
};

SweepStatus validateRail(const std::vector<Point>& r, bool closed) {
    if (r.size() < (closed ? 3u : 2u) || r.size() > kMaxSweepRailVertices) return SweepStatus::BadRail;
    for (const Point& p : r)
        if (!finite(p.x) || !finite(p.y)) return SweepStatus::BadRail;
    Rail tmp;
    tmp.build(r, closed);
    if (!(tmp.total > 0.0)) return SweepStatus::BadRail;
    return SweepStatus::Ok;
}

// A profile prepared for evaluation over normalised u in [0,1].
struct Prof {
    std::vector<double> u;  // normalised, strictly increasing, u[0]=0, u.back()=1
    std::vector<double> z;
    std::vector<double> d;  // slopes (dz/du normalised) for smooth
    double width = 1.0;
    bool smooth = false;

    void build(const SweepSection& s) {
        const std::size_t n = s.profile.size();
        const double u0 = s.profile.front().x;
        width = s.profile.back().x - u0;
        u.resize(n);
        z.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            u[i] = i + 1 == n ? 1.0 : (s.profile[i].x - u0) / width;
            z[i] = s.profile[i].y;
        }
        u[0] = 0.0;
        smooth = s.smooth && n > 2;
        if (!smooth) return;
        // Fritsch-Carlson monotone cubic Hermite tangents.
        std::vector<double> sec(n - 1);
        for (std::size_t i = 0; i + 1 < n; ++i) sec[i] = (z[i + 1] - z[i]) / (u[i + 1] - u[i]);
        d.assign(n, 0.0);
        d[0] = sec[0];
        d[n - 1] = sec[n - 2];
        for (std::size_t i = 1; i + 1 < n; ++i) d[i] = sec[i - 1] * sec[i] > 0 ? 0.5 * (sec[i - 1] + sec[i]) : 0.0;
        for (std::size_t i = 0; i + 1 < n; ++i) {
            if (sec[i] == 0.0) {
                d[i] = 0.0;
                d[i + 1] = 0.0;
                continue;
            }
            const double a = d[i] / sec[i];
            const double b = d[i + 1] / sec[i];
            const double h = a * a + b * b;
            if (h > 9.0) {
                const double tau = 3.0 / std::sqrt(h);
                d[i] = tau * a * sec[i];
                d[i + 1] = tau * b * sec[i];
            }
        }
    }

    double eval(double x) const {
        const std::size_t n = u.size();
        std::size_t i = static_cast<std::size_t>(std::upper_bound(u.begin(), u.end(), x) - u.begin());
        i = i == 0 ? 0 : i - 1;
        if (i + 1 >= n) i = n - 2;
        const double h = u[i + 1] - u[i];
        const double t = std::clamp((x - u[i]) / h, 0.0, 1.0);
        if (!smooth) return z[i] + (z[i + 1] - z[i]) * t;
        const double t2 = t * t;
        const double t3 = t2 * t;
        return (2 * t3 - 3 * t2 + 1) * z[i] + (t3 - 2 * t2 + t) * h * d[i] + (-2 * t3 + 3 * t2) * z[i + 1] +
               (t3 - t2) * h * d[i + 1];
    }
};

struct Key {
    double f;
    double t1;
    double t2;
    std::size_t prof;
};

struct Rung {
    double t1;
    double t2;
    std::size_t a;
    std::size_t b;
    double w;
};

double polyArea(const std::vector<Point>& p) {
    double s = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Point& a = p[i];
        const Point& b = p[(i + 1) % p.size()];
        s += a.x * b.y - b.x * a.y;
    }
    return 0.5 * std::fabs(s);
}

bool inside(const std::vector<Point>& poly, double x, double y) {
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Point& a = poly[i];
        const Point& b = poly[j];
        if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

struct V3 {
    double x, y, z;
};

constexpr double kEps = 1e-9;

void raster(HeightField& f, const V3& a, const V3& b, const V3& c) {
    const Grid& g = f.grid();
    const double det = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    const double scale = std::max({std::fabs(b.x - a.x), std::fabs(c.x - a.x), std::fabs(b.y - a.y), std::fabs(c.y - a.y)});
    if (!(std::fabs(det) > 1e-12 * scale * scale) || !(scale > 0.0)) return;
    const double minx = std::min({a.x, b.x, c.x});
    const double maxx = std::max({a.x, b.x, c.x});
    const double miny = std::min({a.y, b.y, c.y});
    const double maxy = std::max({a.y, b.y, c.y});
    const double pad = kEps * g.cellSize;
    auto lo = [&](double v, double o) { return std::max(0.0, std::ceil((v - pad - o) / g.cellSize)); };
    auto hi = [&](double v, double o, std::size_t n) {
        return std::min(static_cast<double>(n) - 1.0, std::floor((v + pad - o) / g.cellSize));
    };
    const double i0 = lo(minx, g.origin.x);
    const double i1 = hi(maxx, g.origin.x, g.width);
    const double j0 = lo(miny, g.origin.y);
    const double j1 = hi(maxy, g.origin.y, g.height);
    for (double jj = j0; jj <= j1; jj += 1.0) {
        const double py = g.origin.y + jj * g.cellSize;
        for (double ii = i0; ii <= i1; ii += 1.0) {
            const double px = g.origin.x + ii * g.cellSize;
            const double l1 = ((b.y - c.y) * (px - c.x) + (c.x - b.x) * (py - c.y)) / det;
            const double l2 = ((c.y - a.y) * (px - c.x) + (a.x - c.x) * (py - c.y)) / det;
            const double l3 = 1.0 - l1 - l2;
            if (l1 < -kEps || l2 < -kEps || l3 < -kEps) continue;
            const float v = static_cast<float>(l1 * a.z + l2 * b.z + l3 * c.z);
            const std::size_t i = static_cast<std::size_t>(ii);
            const std::size_t j = static_cast<std::size_t>(jj);
            const float old = f.at(i, j);
            if (std::isnan(old) || v > old) f.set(i, j, v);
        }
    }
}

}  // namespace

SweepStatus validateSweep(const SweepParams& p) {
    if (p.closed1 != p.closed2) return SweepStatus::RailKindMismatch;
    if (SweepStatus s = validateRail(p.rail1, p.closed1); s != SweepStatus::Ok) return s;
    if (SweepStatus s = validateRail(p.rail2, p.closed2); s != SweepStatus::Ok) return s;
    if (p.sections.empty()) return SweepStatus::NoSections;
    if (p.sections.size() > kMaxSweepSections) return SweepStatus::BadSection;
    if (!finite(p.exactHeight) || p.exactHeight < 0.0) return SweepStatus::BadParameter;
    double prev1 = 0.0;
    double prev2 = 0.0;
    for (const SweepSection& s : p.sections) {
        if (s.profile.size() < 2 || s.profile.size() > kMaxSweepProfilePoints) return SweepStatus::BadSection;
        if (!finite(s.t1) || !finite(s.t2) || s.t1 < 0 || s.t1 > 1 || s.t2 < 0 || s.t2 > 1) return SweepStatus::BadSection;
        if (s.t1 < prev1 || s.t2 < prev2) return SweepStatus::BadSection;
        prev1 = s.t1;
        prev2 = s.t2;
        for (std::size_t i = 0; i < s.profile.size(); ++i) {
            if (!finite(s.profile[i].x) || !finite(s.profile[i].y)) return SweepStatus::BadSection;
            if (i > 0 && !(s.profile[i].x > s.profile[i - 1].x)) return SweepStatus::BadSection;
        }
    }
    return SweepStatus::Ok;
}

void swapRails(SweepParams& p) {
    std::swap(p.rail1, p.rail2);
    std::swap(p.closed1, p.closed2);
    for (SweepSection& s : p.sections) std::swap(s.t1, s.t2);
}

SweepStatus reverseRail(SweepParams& p, int which) {
    if (which != 1 && which != 2) return SweepStatus::BadParameter;
    std::vector<Point>& r = which == 1 ? p.rail1 : p.rail2;
    const bool closed = which == 1 ? p.closed1 : p.closed2;
    if (closed && r.size() > 1)
        std::reverse(r.begin() + 1, r.end());  // keep the start vertex
    else
        std::reverse(r.begin(), r.end());
    for (SweepSection& s : p.sections) {
        double& t = which == 1 ? s.t1 : s.t2;
        t = (closed && t == 0.0) ? 0.0 : 1.0 - t;
    }
    // Both rails reversed leaves the list descending: restore ascending order.
    if (p.sections.size() > 1 && p.sections.front().t1 > p.sections.back().t1 &&
        p.sections.front().t2 > p.sections.back().t2)
        std::reverse(p.sections.begin(), p.sections.end());
    return SweepStatus::Ok;
}

SweepStatus addSectionToAllNodes(SweepParams& p, const std::vector<Point>& profile, bool smooth) {
    if (p.rail1.size() != p.rail2.size()) return SweepStatus::NodeCountMismatch;
    if (p.closed1 != p.closed2) return SweepStatus::RailKindMismatch;
    if (SweepStatus s = validateRail(p.rail1, p.closed1); s != SweepStatus::Ok) return s;
    if (SweepStatus s = validateRail(p.rail2, p.closed2); s != SweepStatus::Ok) return s;
    if (p.sections.size() + p.rail1.size() > kMaxSweepSections) return SweepStatus::BadSection;
    Rail a;
    Rail b;
    a.build(p.rail1, p.closed1);
    b.build(p.rail2, p.closed2);
    for (std::size_t k = 0; k < p.rail1.size(); ++k) {
        SweepSection s;
        s.profile = profile;
        s.t1 = a.vertexFrac[k];
        s.t2 = b.vertexFrac[k];
        s.smooth = smooth;
        p.sections.push_back(std::move(s));
    }
    return SweepStatus::Ok;
}

SweepStatus sweep(const SweepParams& p, const Grid& grid, const std::atomic<bool>* cancel, HeightField& out) {
    out = HeightField();
    if (SweepStatus s = validateSweep(p); s != SweepStatus::Ok) return s;
    if (validate(grid) != Status::Ok) return SweepStatus::BadGrid;

    Rail r1;
    Rail r2;
    r1.build(p.rail1, p.closed1);
    r2.build(p.rail2, p.closed2);
    std::vector<Prof> profs(p.sections.size());
    for (std::size_t i = 0; i < profs.size(); ++i) profs[i].build(p.sections[i]);

    // Keys: implicit anchors at both ends plus the sections.
    std::vector<Key> keys;
    if (p.sections.front().t1 > 0.0 || p.sections.front().t2 > 0.0) keys.push_back({0.0, 0.0, 0.0, 0});
    for (std::size_t i = 0; i < p.sections.size(); ++i)
        keys.push_back({0.5 * (p.sections[i].t1 + p.sections[i].t2), p.sections[i].t1, p.sections[i].t2, i});
    if (p.sections.back().t1 < 1.0 || p.sections.back().t2 < 1.0)
        keys.push_back({1.0, 1.0, 1.0, p.sections.size() - 1});

    // Rungs.
    const double step = 0.5 * grid.cellSize;
    std::vector<Rung> rungs;
    for (std::size_t k = 0; k + 1 < keys.size(); ++k) {
        const Key& ka = keys[k];
        const Key& kb = keys[k + 1];
        if (!(kb.f > ka.f)) continue;
        const double span1 = (kb.t1 - ka.t1) * r1.total;
        const double span2 = (kb.t2 - ka.t2) * r2.total;
        const double n = std::ceil(std::max(span1, span2) / step);
        if (n > static_cast<double>(kMaxSweepQuads)) return SweepStatus::TooLarge;
        std::vector<double> ss;
        const std::size_t cnt = static_cast<std::size_t>(std::max(1.0, n));
        for (std::size_t i = 0; i <= cnt; ++i) ss.push_back(static_cast<double>(i) / static_cast<double>(cnt));
        auto addVerts = [&](const Rail& r, double a, double b) {
            if (!(b > a)) return;
            for (double v : r.vertexFrac)
                if (v > a && v < b) ss.push_back((v - a) / (b - a));
        };
        addVerts(r1, ka.t1, kb.t1);
        addVerts(r2, ka.t2, kb.t2);
        std::sort(ss.begin(), ss.end());
        ss.erase(std::unique(ss.begin(), ss.end(), [](double x, double y) { return y - x < 1e-12; }), ss.end());
        for (double s : ss) {
            Rung r;
            r.t1 = ka.t1 + (kb.t1 - ka.t1) * s;
            r.t2 = ka.t2 + (kb.t2 - ka.t2) * s;
            r.a = ka.prof;
            r.b = p.sweepBetweenSpans ? kb.prof : ka.prof;
            r.w = p.sweepBetweenSpans ? s : 0.0;
            rungs.push_back(r);
            if (rungs.size() > kMaxSweepQuads) return SweepStatus::TooLarge;
        }
    }

    // Shared sample set across u: uniform plus every profile vertex.
    double maxGap = 0.0;
    for (const Rung& r : rungs) {
        const Point a = r1.at(r.t1);
        const Point b = r2.at(r.t2);
        maxGap = std::max(maxGap, std::hypot(a.x - b.x, a.y - b.y));
    }
    const double mu = std::ceil(maxGap / step);
    if (mu > static_cast<double>(kMaxSweepQuads)) return SweepStatus::TooLarge;
    const std::size_t uniform = static_cast<std::size_t>(std::max(1.0, mu));
    std::vector<double> us;
    for (std::size_t i = 0; i <= uniform; ++i) us.push_back(static_cast<double>(i) / static_cast<double>(uniform));
    for (const Prof& pr : profs) us.insert(us.end(), pr.u.begin(), pr.u.end());
    std::sort(us.begin(), us.end());
    us.erase(std::unique(us.begin(), us.end(), [](double x, double y) { return y - x < 1e-12; }), us.end());
    if (rungs.size() * us.size() > kMaxSweepQuads) return SweepStatus::TooLarge;

    HeightField f;
    if (Status st = HeightField::create(grid, kTransparent, f); st != Status::Ok) return SweepStatus::BadGrid;

    std::vector<V3> prev;
    std::vector<V3> cur(us.size());
    bool havePrev = false;
    for (const Rung& r : rungs) {
        if (cancel != nullptr && cancel->load(std::memory_order_relaxed)) return SweepStatus::Cancelled;
        const Point a = r1.at(r.t1);
        const Point b = r2.at(r.t2);
        const double gap = std::hypot(a.x - b.x, a.y - b.y);
        const double kA = p.scaleWithWidth ? gap / profs[r.a].width : 1.0;
        const double kB = p.scaleWithWidth ? gap / profs[r.b].width : 1.0;
        for (std::size_t i = 0; i < us.size(); ++i) {
            double z = profs[r.a].eval(us[i]) * kA;
            if (r.w > 0.0) z = z + (profs[r.b].eval(us[i]) * kB - z) * r.w;
            cur[i] = V3{a.x + (b.x - a.x) * us[i], a.y + (b.y - a.y) * us[i], z};
        }
        if (havePrev) {
            for (std::size_t i = 0; i + 1 < us.size(); ++i) {
                raster(f, prev[i], prev[i + 1], cur[i]);
                raster(f, prev[i + 1], cur[i + 1], cur[i]);
            }
        }
        prev = cur;
        havePrev = true;
    }

    if (p.fillCenter && p.closed1) {
        const bool inner2 = polyArea(p.rail2) < polyArea(p.rail1);
        const std::vector<Point>& inner = inner2 ? p.rail2 : p.rail1;
        // Mean height of the profile end that touches the inner rail, over the rungs.
        double sum = 0.0;
        for (const Rung& r : rungs) {
            const Point a = r1.at(r.t1);
            const Point b = r2.at(r.t2);
            const double gap = std::hypot(a.x - b.x, a.y - b.y);
            const double u = inner2 ? 1.0 : 0.0;
            const double kA = p.scaleWithWidth ? gap / profs[r.a].width : 1.0;
            const double kB = p.scaleWithWidth ? gap / profs[r.b].width : 1.0;
            double z = profs[r.a].eval(u) * kA;
            if (r.w > 0.0) z = z + (profs[r.b].eval(u) * kB - z) * r.w;
            sum += z;
        }
        const float fillZ = static_cast<float>(sum / static_cast<double>(rungs.size()));
        for (std::size_t j = 0; j < grid.height; ++j)
            for (std::size_t i = 0; i < grid.width; ++i) {
                if (!std::isnan(f.at(i, j))) continue;
                const double x = grid.origin.x + static_cast<double>(i) * grid.cellSize;
                const double y = grid.origin.y + static_cast<double>(j) * grid.cellSize;
                if (inside(inner, x, y)) f.set(i, j, fillZ);
            }
    }

    if (p.exactHeight > 0.0) {
        float lo = 0.0f;
        float hi = 0.0f;
        if (!f.minMax(lo, hi) || !(hi > 0.0f)) return SweepStatus::FlatResult;
        const double k = p.exactHeight / static_cast<double>(hi);
        for (std::size_t j = 0; j < grid.height; ++j)
            for (std::size_t i = 0; i < grid.width; ++i) {
                const float v = f.at(i, j);
                if (!std::isnan(v)) f.set(i, j, static_cast<float>(static_cast<double>(v) * k));
            }
    }
    out = std::move(f);
    return SweepStatus::Ok;
}

}  // namespace carvetto::relief
