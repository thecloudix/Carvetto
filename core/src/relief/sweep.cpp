#include "carvetto/relief/sweep.hpp"

#include "sweep_detail.hpp"

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

using detail::Prof;
using detail::V3;
using detail::raster;

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
