#include "carvetto/relief/extrude.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "sweep_detail.hpp"

namespace carvetto::relief {

const char* extrudeStatusMessage(ExtrudeStatus s) {
    switch (s) {
        case ExtrudeStatus::Ok: return "ok";
        case ExtrudeStatus::NoRails: return "at least one rail is needed";
        case ExtrudeStatus::BadRail: return "a rail needs at least 2 vertices (3 if closed), finite coordinates and non-zero length";
        case ExtrudeStatus::NoSections: return "at least one cross section is needed";
        case ExtrudeStatus::BadSection: return "a cross section is invalid or out of order";
        case ExtrudeStatus::BadComponent: return "the component must have at least 2x2 cells and some opaque cells";
        case ExtrudeStatus::BadParameter: return "invalid extrude parameter";
        case ExtrudeStatus::BadGrid: return "invalid grid";
        case ExtrudeStatus::TooLarge: return "the extrusion is too large";
        case ExtrudeStatus::Cancelled: return "cancelled";
        case ExtrudeStatus::FlatResult: return "the extruded surface has no positive height to scale";
    }
    return "unknown";
}

namespace {

using detail::Prof;
using detail::raster;
using detail::V3;

constexpr double kSameS = 1e-9;       // arc positions closer than this (mm) are one rung
constexpr double kMiterLimit = 8.0;   // sharp corners beyond this miter ratio are bevelled
constexpr double kMinCross = 0.1;     // smallest crossing sine used for a weave window

bool finite(double v) { return std::isfinite(v); }
double cross(const Point& a, const Point& b) { return a.x * b.y - a.y * b.x; }
double dot(const Point& a, const Point& b) { return a.x * b.x + a.y * b.y; }
Point sub(const Point& a, const Point& b) { return Point{a.x - b.x, a.y - b.y}; }

// Rail polyline without repeated vertices, parametrised by arc length s in mm.
struct XRail {
    std::vector<Point> pts;
    std::vector<double> cum;  // cumulative length at each segment end
    std::vector<Point> dir;   // unit direction of each segment
    bool closed = false;
    double total = 0.0;

    std::size_t segs() const { return cum.size(); }
    double segStart(std::size_t i) const { return i == 0 ? 0.0 : cum[i - 1]; }

    bool build(const ExtrudeRail& r) {
        if (r.points.size() > kMaxExtrudeRailVertices) return false;
        closed = r.closed;
        pts.clear();
        for (const Point& p : r.points) {
            if (!finite(p.x) || !finite(p.y)) return false;
            if (!pts.empty() && std::hypot(p.x - pts.back().x, p.y - pts.back().y) <= 1e-12) continue;
            pts.push_back(p);
        }
        if (closed && pts.size() > 1 && std::hypot(pts.front().x - pts.back().x, pts.front().y - pts.back().y) <= 1e-12)
            pts.pop_back();
        if (pts.size() < (closed ? 3u : 2u)) return false;
        const std::size_t n = pts.size();
        const std::size_t sc = closed ? n : n - 1;
        cum.assign(sc, 0.0);
        dir.assign(sc, Point{});
        double acc = 0.0;
        for (std::size_t i = 0; i < sc; ++i) {
            const Point& a = pts[i];
            const Point& b = pts[(i + 1) % n];
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            dir[i] = Point{(b.x - a.x) / len, (b.y - a.y) / len};
            acc += len;
            cum[i] = acc;
        }
        total = acc;
        return total > 0.0 && finite(total);
    }

    std::size_t segAt(double s) const {
        std::size_t i = static_cast<std::size_t>(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
        return i >= cum.size() ? cum.size() - 1 : i;
    }
    Point pointAt(std::size_t seg, double s) const {
        const std::size_t n = pts.size();
        const double len = cum[seg] - segStart(seg);
        const double w = std::clamp((s - segStart(seg)) / len, 0.0, 1.0);
        const Point& a = pts[seg];
        const Point& b = pts[(seg + 1) % n];
        return Point{a.x + (b.x - a.x) * w, a.y + (b.y - a.y) * w};
    }
};

// Piecewise-linear weave level along one rail: 1 outside the crossings on an open rail, periodic on a closed one.
struct WeaveLevel {
    std::vector<std::pair<double, double>> knots;  // (arc position, factor), ascending
    bool closed = false;
    double total = 0.0;
    std::vector<std::pair<double, double>> ext;  // knots shifted by -total, 0, +total (closed only)

    void finish() {
        if (!closed) {
            ext = knots;
            return;
        }
        ext.clear();
        for (int k = -1; k <= 1; ++k)
            for (const auto& kn : knots) ext.emplace_back(kn.first + k * total, kn.second);
    }
    double at(double s) const {
        if (ext.empty()) return 1.0;
        const auto it = std::upper_bound(ext.begin(), ext.end(), s,
                                         [](double v, const std::pair<double, double>& k) { return v < k.first; });
        if (it == ext.begin()) return ext.front().second;
        if (it == ext.end()) return ext.back().second;
        const auto& hi = *it;
        const auto& lo = *(it - 1);
        const double span = hi.first - lo.first;
        if (!(span > 0.0)) return hi.second;
        return lo.second + (hi.second - lo.second) * ((s - lo.first) / span);
    }
};

struct Window {
    double c;  // centre (arc position)
    double h;  // half width
    double v;  // factor
};

struct Occurrence {
    std::size_t rail;
    double s;
    double sinTh;
    std::size_t cid;
    int which;
};

// Crossings of every pair of rail segments (half-open parameter ranges so a crossing at a vertex
// counts once). Fills one Level per rail.
bool buildWeave(const std::vector<XRail>& rails, const WeaveParams& wp, double srcWidth, std::vector<WeaveLevel>& levels) {
    levels.assign(rails.size(), WeaveLevel{});
    for (std::size_t r = 0; r < rails.size(); ++r) {
        levels[r].closed = rails[r].closed;
        levels[r].total = rails[r].total;
    }
    struct SegRef {
        std::size_t rail;
        std::size_t seg;
    };
    std::vector<SegRef> refs;
    for (std::size_t r = 0; r < rails.size(); ++r)
        for (std::size_t i = 0; i < rails[r].segs(); ++i) refs.push_back({r, i});
    if (refs.size() > kMaxWeaveSegments) return false;

    struct Crossing {
        Occurrence occ[2];
        std::size_t idx[2] = {0, 0};
    };
    std::vector<Crossing> crossings;
    for (std::size_t a = 0; a < refs.size(); ++a)
        for (std::size_t b = a + 1; b < refs.size(); ++b) {
            const XRail& ra = rails[refs[a].rail];
            const XRail& rb = rails[refs[b].rail];
            const std::size_t ia = refs[a].seg;
            const std::size_t ib = refs[b].seg;
            if (refs[a].rail == refs[b].rail) {
                const std::size_t n = ra.segs();
                if (ib == ia + 1 || (ra.closed && ia == 0 && ib == n - 1)) continue;  // share a vertex
            }
            const Point p0 = ra.pts[ia];
            const Point p1 = ra.pts[(ia + 1) % ra.pts.size()];
            const Point q0 = rb.pts[ib];
            const Point q1 = rb.pts[(ib + 1) % rb.pts.size()];
            const Point d1 = sub(p1, p0);
            const Point d2 = sub(q1, q0);
            const double rr = cross(d1, d2);
            const double l1 = ra.cum[ia] - ra.segStart(ia);
            const double l2 = rb.cum[ib] - rb.segStart(ib);
            const double sinTh = std::fabs(rr) / (l1 * l2);
            if (!(sinTh > 1e-9)) continue;
            const Point w = sub(q0, p0);
            const double t = cross(w, d2) / rr;
            const double u = cross(w, d1) / rr;
            if (!(t >= 0.0 && t < 1.0 && u >= 0.0 && u < 1.0)) continue;  // also rejects NaN
            Crossing c;
            c.occ[0] = {refs[a].rail, ra.segStart(ia) + t * l1, sinTh, crossings.size(), 0};
            c.occ[1] = {refs[b].rail, rb.segStart(ib) + u * l2, sinTh, crossings.size(), 1};
            if (c.occ[0].rail == c.occ[1].rail && c.occ[1].s < c.occ[0].s) {
                std::swap(c.occ[0], c.occ[1]);
                c.occ[0].which = 0;
                c.occ[1].which = 1;
            }
            crossings.push_back(c);
        }

    std::vector<std::vector<Occurrence>> perRail(rails.size());
    for (const Crossing& c : crossings)
        for (const Occurrence& o : c.occ) perRail[o.rail].push_back(o);
    for (std::size_t r = 0; r < rails.size(); ++r) {
        auto& v = perRail[r];
        std::sort(v.begin(), v.end(), [](const Occurrence& x, const Occurrence& y) {
            if (x.s != y.s) return x.s < y.s;
            if (x.cid != y.cid) return x.cid < y.cid;
            return x.which < y.which;
        });
        for (std::size_t i = 0; i < v.size(); ++i) crossings[v[i].cid].idx[v[i].which] = i;
    }

    // Each rail alternates over/under along its length: over at crossing index i iff (i + phase) is
    // even. Phases are propagated over the crossings so the two strands at a crossing differ; the
    // lowest rail of every connected group starts over (startOver) or under.
    std::vector<int> phase(rails.size(), -1);
    for (std::size_t r0 = 0; r0 < rails.size(); ++r0) {
        if (phase[r0] >= 0) continue;
        phase[r0] = wp.startOver ? 0 : 1;
        std::vector<std::size_t> queue{r0};
        for (std::size_t qi = 0; qi < queue.size(); ++qi) {
            const std::size_t a = queue[qi];
            for (const Crossing& c : crossings)
                for (int w = 0; w < 2; ++w) {
                    if (c.occ[w].rail != a) continue;
                    const Occurrence& other = c.occ[1 - w];
                    if (phase[other.rail] >= 0) continue;
                    const bool overA = (c.idx[w] + static_cast<std::size_t>(phase[a])) % 2 == 0;
                    phase[other.rail] = static_cast<int>((c.idx[1 - w] + (overA ? 1u : 0u)) % 2);
                    queue.push_back(other.rail);
                }
        }
    }

    for (std::size_t r = 0; r < rails.size(); ++r) {
        std::vector<Window> wins;
        for (const Occurrence& o : perRail[r]) {
            const Crossing& c = crossings[o.cid];
            const bool over = (c.idx[o.which] + static_cast<std::size_t>(phase[r])) % 2 == 0;
            const double h = std::min(0.5 * srcWidth / std::max(o.sinTh, kMinCross), 0.5 * rails[r].total);
            wins.push_back({o.s, h, (over ? wp.zOverPercent : wp.zUnderPercent) / 100.0});
        }
        WeaveLevel& lv = levels[r];
        const double tot = rails[r].total;
        for (std::size_t k = 0; k < wins.size(); ++k) {
            double left = wins[k].c - wins[k].h;
            double right = wins[k].c + wins[k].h;
            const bool hasPrev = k > 0 || (lv.closed && wins.size() > 1);
            const bool hasNext = k + 1 < wins.size() || (lv.closed && wins.size() > 1);
            if (hasPrev) {
                const double pc = k > 0 ? wins[k - 1].c : wins.back().c - tot;
                left = std::max(left, 0.5 * (pc + wins[k].c));
            }
            if (hasNext) {
                const double nc = k + 1 < wins.size() ? wins[k + 1].c : wins.front().c + tot;
                right = std::min(right, 0.5 * (wins[k].c + nc));
            }
            lv.knots.emplace_back(left, wins[k].v);
            lv.knots.emplace_back(right, wins[k].v);
        }
        if (!lv.closed && !wins.empty()) {
            // Open rails return to full height one window half-width beyond the first/last crossing.
            const double l0 = lv.knots.front().first - wins.front().h;
            const double l1 = lv.knots.back().first + wins.back().h;
            lv.knots.insert(lv.knots.begin(), {l0, 1.0});
            lv.knots.emplace_back(l1, 1.0);
        }
        lv.finish();
    }
    return true;
}

struct Rung {
    double s;
    Point p;
    Point d;  // offset direction per mm of across-rail offset (left normal, mitre scaled)
};

Point leftNormal(const Point& t) { return Point{-t.y, t.x}; }

// Rungs over [sa, sb] of the rail: uniform steps, every vertex (corner joints expanded to a
// mitre / bevel / round fan), plus `breaks`.
void makeRungs(const XRail& R, double sa, double sb, double step, double halfMax, bool sharp, double cell,
               const std::vector<double>& breaks, std::vector<Rung>& rungs) {
    rungs.clear();
    struct Pos {
        double s;
        long joint;  // vertex index or -1
    };
    std::vector<Pos> pos;
    const double n = std::max(1.0, std::ceil((sb - sa) / step));
    const std::size_t cnt = static_cast<std::size_t>(n);
    for (std::size_t i = 0; i <= cnt; ++i) pos.push_back({sa + (sb - sa) * static_cast<double>(i) / n, -1});
    const std::size_t nv = R.pts.size();
    auto addJoint = [&](double s, std::size_t k) {
        if (s >= sa - kSameS && s <= sb + kSameS) pos.push_back({s, static_cast<long>(k)});
    };
    if (R.closed) {
        addJoint(0.0, 0);
        addJoint(R.total, 0);
        for (std::size_t k = 1; k < nv; ++k) addJoint(R.cum[k - 1], k);
    } else {
        for (std::size_t k = 1; k + 1 < nv; ++k) addJoint(R.cum[k - 1], k);
    }
    for (double b : breaks)
        if (b > sa && b < sb) pos.push_back({b, -1});
    std::stable_sort(pos.begin(), pos.end(), [](const Pos& a, const Pos& b) { return a.s < b.s; });
    std::vector<Pos> uniq;
    for (const Pos& p : pos) {
        if (!uniq.empty() && p.s - uniq.back().s < kSameS) {
            if (p.joint >= 0) uniq.back().joint = p.joint;
            continue;
        }
        uniq.push_back(p);
    }
    const double dTheta = std::clamp(0.5 * cell / std::max(halfMax, 1e-12), 0.02, 0.2);
    for (const Pos& p : uniq) {
        const double s = std::clamp(p.s, sa, sb);
        if (p.joint < 0) {
            const std::size_t seg = R.segAt(s);
            rungs.push_back({s, R.pointAt(seg, s), leftNormal(R.dir[seg])});
            continue;
        }
        const std::size_t k = static_cast<std::size_t>(p.joint);
        const std::size_t prev = R.closed ? (k + nv - 1) % nv : k - 1;
        const std::size_t next = k;
        const Point nA = leftNormal(R.dir[prev]);
        const Point nB = leftNormal(R.dir[next]);
        const Point at = R.pts[k];
        const double c = dot(nA, nB);
        const double phi = std::atan2(cross(R.dir[prev], R.dir[next]), dot(R.dir[prev], R.dir[next]));
        if (std::fabs(phi) < 1e-9) {
            rungs.push_back({s, at, nA});
        } else if (sharp) {
            if (1.0 + c >= 2.0 / (kMiterLimit * kMiterLimit)) {
                rungs.push_back({s, at, Point{(nA.x + nB.x) / (1.0 + c), (nA.y + nB.y) / (1.0 + c)}});
            } else {
                rungs.push_back({s, at, nA});
                rungs.push_back({s, at, nB});
            }
        } else {
            const std::size_t steps = static_cast<std::size_t>(std::max(1.0, std::ceil(std::fabs(phi) / dTheta)));
            for (std::size_t i = 0; i <= steps; ++i) {
                const double a = phi * static_cast<double>(i) / static_cast<double>(steps);
                const double ca = std::cos(a);
                const double sa2 = std::sin(a);
                rungs.push_back({s, at, Point{nA.x * ca - nA.y * sa2, nA.x * sa2 + nA.y * ca}});
            }
        }
    }
}

struct Key {
    double t;
    std::size_t prof;
};

}  // namespace

ExtrudeStatus validateExtrude(const ExtrudeParams& p) {
    if (p.rails.empty()) return ExtrudeStatus::NoRails;
    if (p.rails.size() > kMaxExtrudeRails) return ExtrudeStatus::BadRail;
    for (const ExtrudeRail& r : p.rails) {
        XRail x;
        if (!x.build(r)) return ExtrudeStatus::BadRail;
    }
    if (!finite(p.exactHeight) || p.exactHeight < 0.0) return ExtrudeStatus::BadParameter;
    if (p.weave.enabled) {
        if (!finite(p.weave.zUnderPercent) || p.weave.zUnderPercent < 0.0) return ExtrudeStatus::BadParameter;
        if (!finite(p.weave.zOverPercent) || p.weave.zOverPercent < 0.0) return ExtrudeStatus::BadParameter;
    }
    if (p.source == ExtrudeSource::CrossSections) {
        if (p.sections.empty()) return ExtrudeStatus::NoSections;
        if (p.sections.size() > kMaxExtrudeSections) return ExtrudeStatus::BadSection;
        double lastT = -1.0;
        for (const ExtrudeSection& s : p.sections) {
            if (!finite(s.t) || s.t < 0.0 || s.t > 1.0 || !(s.t > lastT)) return ExtrudeStatus::BadSection;
            lastT = s.t;
            if (s.profile.size() < 2 || s.profile.size() > kMaxExtrudeProfilePoints) return ExtrudeStatus::BadSection;
            for (std::size_t i = 0; i < s.profile.size(); ++i) {
                if (!finite(s.profile[i].x) || !finite(s.profile[i].y)) return ExtrudeStatus::BadSection;
                if (i > 0 && !(s.profile[i].x > s.profile[i - 1].x)) return ExtrudeStatus::BadSection;
            }
        }
    } else {
        float lo = 0.0f;
        float hi = 0.0f;
        if (p.component.width() < 2 || p.component.height() < 2 || !p.component.minMax(lo, hi))
            return ExtrudeStatus::BadComponent;
        if (!p.stretchToRail && (!finite(p.overlapPercent) || p.overlapPercent < 0.0 || p.overlapPercent >= 100.0))
            return ExtrudeStatus::BadParameter;
    }
    return ExtrudeStatus::Ok;
}

ExtrudeStatus extrude(const ExtrudeParams& p, const Grid& grid, const std::atomic<bool>* cancel, HeightField& out) {
    out = HeightField();
    if (ExtrudeStatus s = validateExtrude(p); s != ExtrudeStatus::Ok) return s;
    if (validate(grid) != Status::Ok) return ExtrudeStatus::BadGrid;

    std::vector<XRail> rails(p.rails.size());
    for (std::size_t i = 0; i < rails.size(); ++i) rails[i].build(p.rails[i]);

    const double step = 0.5 * grid.cellSize;
    const bool useComp = p.source == ExtrudeSource::Component;

    // Source description.
    std::vector<Prof> profs;
    std::vector<double> us;  // shared normalised across-samples (vector source)
    double srcWidth = 0.0;   // across extent in mm (max over sections)
    double heightMax = 0.0;  // highest source z, for Add Base
    double compLx = 0.0;
    double compLy = 0.0;
    std::size_t compCols = 0;  // across intervals (component source)
    if (!useComp) {
        profs.resize(p.sections.size());
        double hm = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < profs.size(); ++i) {
            profs[i].build(SweepSection{p.sections[i].profile, 0.0, 0.0, p.sections[i].smooth});
            srcWidth = std::max(srcWidth, profs[i].width);
            for (const Point& q : p.sections[i].profile) hm = std::max(hm, q.y);
        }
        heightMax = hm;
        const double mu = std::ceil(srcWidth / step);
        if (mu > static_cast<double>(kMaxExtrudeQuads)) return ExtrudeStatus::TooLarge;
        const std::size_t uniform = static_cast<std::size_t>(std::max(1.0, mu));
        for (std::size_t i = 0; i <= uniform; ++i) us.push_back(static_cast<double>(i) / static_cast<double>(uniform));
        for (const Prof& pr : profs) us.insert(us.end(), pr.u.begin(), pr.u.end());
        std::sort(us.begin(), us.end());
        us.erase(std::unique(us.begin(), us.end(), [](double x, double y) { return y - x < 1e-12; }), us.end());
    } else {
        float lo = 0.0f;
        float hi = 0.0f;
        p.component.minMax(lo, hi);
        heightMax = static_cast<double>(hi);
        compLx = static_cast<double>(p.component.width() - 1) * p.component.cellSize();
        compLy = static_cast<double>(p.component.height() - 1) * p.component.cellSize();
        srcWidth = compLy;
        const double mc = std::ceil(compLy / step);
        if (mc > static_cast<double>(kMaxExtrudeQuads)) return ExtrudeStatus::TooLarge;
        compCols = static_cast<std::size_t>(std::max(1.0, mc));
    }
    const std::size_t samples = useComp ? compCols + 1 : us.size();

    std::vector<WeaveLevel> levels;
    if (p.weave.enabled && !buildWeave(rails, p.weave, srcWidth, levels)) return ExtrudeStatus::TooLarge;

    HeightField f;
    if (Status st = HeightField::create(grid, kTransparent, f); st != Status::Ok) return ExtrudeStatus::BadGrid;

    std::size_t work = 0;
    std::vector<Rung> rungs;
    std::vector<V3> prev;
    std::vector<V3> cur(samples);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto weaveZ = [&](double z, double fac) {
        return p.weave.mode == WeaveMode::ScaleShape ? z * fac : z + (fac - 1.0) * heightMax;
    };
    auto finiteTri = [](const V3& a, const V3& b, const V3& c) {
        return std::isfinite(a.z) && std::isfinite(b.z) && std::isfinite(c.z);
    };

    // One span: arc interval [sa, sb] of rail r. `fill` computes cur[] for a rung.
    auto runSpan = [&](std::size_t r, double sa, double sb, auto&& fill) -> ExtrudeStatus {
        const XRail& R = rails[r];
        std::vector<double> breaks;
        if (p.weave.enabled)
            for (const auto& k : levels[r].ext) breaks.push_back(k.first);
        makeRungs(R, sa, sb, step, 0.5 * srcWidth, p.sharpCorners, grid.cellSize, breaks, rungs);
        work += rungs.size() * samples;
        if (work > kMaxExtrudeQuads) return ExtrudeStatus::TooLarge;
        bool havePrev = false;
        for (const Rung& rg : rungs) {
            if (cancel != nullptr && cancel->load(std::memory_order_relaxed)) return ExtrudeStatus::Cancelled;
            const double fac = p.weave.enabled ? levels[r].at(rg.s) : 1.0;
            fill(rg, sa, sb, fac);
            if (havePrev) {
                for (std::size_t i = 0; i + 1 < samples; ++i) {
                    if (finiteTri(prev[i], prev[i + 1], cur[i])) raster(f, prev[i], prev[i + 1], cur[i]);
                    if (finiteTri(prev[i + 1], cur[i + 1], cur[i])) raster(f, prev[i + 1], cur[i + 1], cur[i]);
                }
            }
            prev = cur;
            havePrev = true;
        }
        return ExtrudeStatus::Ok;
    };

    for (std::size_t r = 0; r < rails.size(); ++r) {
        const XRail& R = rails[r];
        if (!useComp) {
            std::vector<Key> keys;
            if (p.sections.front().t > 0.0) keys.push_back({0.0, 0});
            for (std::size_t i = 0; i < p.sections.size(); ++i) keys.push_back({p.sections[i].t, i});
            if (p.sections.back().t < 1.0) keys.push_back({1.0, p.sections.size() - 1});
            for (std::size_t k = 0; k + 1 < keys.size(); ++k) {
                const Key& ka = keys[k];
                const Key& kb = keys[k + 1];
                const double sa = ka.t * R.total;
                const double sb = kb.t * R.total;
                const std::size_t pa = ka.prof;
                const std::size_t pb = p.sweepBetweenSpans ? kb.prof : ka.prof;
                auto fill = [&](const Rung& rg, double a, double b, double fac) {
                    const double w = p.sweepBetweenSpans ? (rg.s - a) / (b - a) : 0.0;
                    const double width = profs[pa].width + (profs[pb].width - profs[pa].width) * w;
                    for (std::size_t i = 0; i < us.size(); ++i) {
                        double z = profs[pa].eval(us[i]);
                        if (w > 0.0) z = z + (profs[pb].eval(us[i]) - z) * w;
                        if (p.weave.enabled) z = weaveZ(z, fac);
                        const double off = (us[i] - 0.5) * width;
                        cur[i] = V3{rg.p.x + rg.d.x * off, rg.p.y + rg.d.y * off, z};
                    }
                };
                if (ExtrudeStatus st = runSpan(r, sa, sb, fill); st != ExtrudeStatus::Ok) return st;
            }
        } else {
            std::size_t copies = 1;
            double over = 0.0;
            double lc = R.total;
            if (!p.stretchToRail) {
                over = p.overlapPercent / 100.0;
                const double stepNat = compLx * (1.0 - over);
                const double nc = std::max(1.0, std::round((R.total - compLx) / stepNat) + 1.0);
                if (nc > static_cast<double>(kMaxExtrudeCopies)) return ExtrudeStatus::TooLarge;
                copies = static_cast<std::size_t>(nc);
                lc = R.total / (1.0 + static_cast<double>(copies - 1) * (1.0 - over));
            }
            const Grid& cg = p.component.grid();
            for (std::size_t k = 0; k < copies; ++k) {
                const double sa = static_cast<double>(k) * lc * (1.0 - over);
                const double sb = std::min(sa + lc, R.total);
                const bool flip = p.flipAlternate && (k % 2 == 1);
                auto fill = [&](const Rung& rg, double a, double b, double fac) {
                    double xl = (rg.s - a) / (b - a) * compLx;
                    xl = std::clamp(xl, 0.0, compLx);
                    if (flip) xl = compLx - xl;
                    for (std::size_t j = 0; j < samples; ++j) {
                        const double yl = compLy * static_cast<double>(j) / static_cast<double>(compCols);
                        const float v = p.component.sample(cg.origin.x + xl, cg.origin.y + yl);
                        double z = std::isnan(v) ? nan : static_cast<double>(v);
                        if (p.weave.enabled && !std::isnan(z)) z = weaveZ(z, fac);
                        const double off = yl - 0.5 * compLy;
                        cur[j] = V3{rg.p.x + rg.d.x * off, rg.p.y + rg.d.y * off, z};
                    }
                };
                if (ExtrudeStatus st = runSpan(r, sa, sb, fill); st != ExtrudeStatus::Ok) return st;
            }
        }
    }

    if (p.exactHeight > 0.0) {
        float lo = 0.0f;
        float hi = 0.0f;
        if (!f.minMax(lo, hi) || !(hi > 0.0f)) return ExtrudeStatus::FlatResult;
        const double k = p.exactHeight / static_cast<double>(hi);
        for (std::size_t j = 0; j < grid.height; ++j)
            for (std::size_t i = 0; i < grid.width; ++i) {
                const float v = f.at(i, j);
                if (!std::isnan(v)) f.set(i, j, static_cast<float>(static_cast<double>(v) * k));
            }
    }
    out = std::move(f);
    return ExtrudeStatus::Ok;
}

}  // namespace carvetto::relief
