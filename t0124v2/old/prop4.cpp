// Round-3 verifier sweep for T-0120 (throwaway; new generators, new seeds, own oracle).
// Checks: structure, gouge (end mill disc / V cone), side, per-point wall coverage at final
// depth, tab integrity (wall + kerf interior columns), tab counts, tab lengths on straight spans,
// finishing order (median-distance classification), level structure without ramps, determinism.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <unordered_map>

#include "oracle.hpp"

using namespace v;
using namespace carvetto::toolpath;

static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

static Contour poly(const std::vector<Vec2>& pts, bool closed = true) {
    Contour c;
    for (size_t i = 0; i + 1 < pts.size(); ++i) c.append(geometry::LineSpan{pts[i], pts[i + 1]});
    if (closed) c.append(geometry::LineSpan{pts.back(), pts.front()});
    c.closed = closed;
    return c;
}
static Contour translate(Contour c, Vec2 off) {
    for (auto& sp : c.spans) {
        if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start + off; l->end = l->end + off; }
        else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) a->centre = a->centre + off;
        else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = b->p0 + off; b->p1 = b->p1 + off; b->p2 = b->p2 + off; b->p3 = b->p3 + off; }
    }
    return c;
}
static Contour reversed(const Contour& c) {
    Contour o;
    o.closed = c.closed;
    for (size_t i = c.spans.size(); i-- > 0;) o.append(geometry::reversed(c.spans[i]));
    return o;
}
// gear: n teeth, alternating radii with flat tops
static Contour gear(Vec2 c, double R, int n, double depth) {
    std::vector<Vec2> p;
    for (int i = 0; i < n; ++i) {
        double a0 = 2 * PI * i / n, w = 2 * PI / n;
        double rr[4] = {R - depth, R, R, R - depth};
        double aa[4] = {a0, a0 + w * 0.2, a0 + w * 0.5, a0 + w * 0.7};
        for (int k = 0; k < 4; ++k) p.push_back({c.x + rr[k] * std::cos(aa[k]), c.y + rr[k] * std::sin(aa[k])});
    }
    return poly(p);
}
// letters from orthogonal outlines
static Contour letter(Vec2 o, double s) {
    double t = s * U(0.2, 0.35);
    switch (I(0, 3)) {
        case 0: return poly({{o.x, o.y}, {o.x + s, o.y}, {o.x + s, o.y + t}, {o.x + t, o.y + t}, {o.x + t, o.y + s * 1.4}, {o.x, o.y + s * 1.4}});  // L
        case 1: return poly({{o.x + s / 2 - t / 2, o.y}, {o.x + s / 2 + t / 2, o.y}, {o.x + s / 2 + t / 2, o.y + s - t}, {o.x + s, o.y + s - t}, {o.x + s, o.y + s}, {o.x, o.y + s}, {o.x, o.y + s - t}, {o.x + s / 2 - t / 2, o.y + s - t}});  // T
        case 2: return poly({{o.x, o.y}, {o.x + s, o.y}, {o.x + s, o.y + s}, {o.x + s - t, o.y + s}, {o.x + s - t, o.y + t}, {o.x + t, o.y + t}, {o.x + t, o.y + s}, {o.x, o.y + s}});  // U
        default: return poly({{o.x, o.y}, {o.x + t, o.y}, {o.x + t, o.y + s / 2 - t / 2}, {o.x + s - t, o.y + s / 2 - t / 2}, {o.x + s - t, o.y}, {o.x + s, o.y}, {o.x + s, o.y + s}, {o.x + s - t, o.y + s}, {o.x + s - t, o.y + s / 2 + t / 2}, {o.x + t, o.y + s / 2 + t / 2}, {o.x + t, o.y + s}, {o.x, o.y + s}});  // H
    }
}
// flower: closed loop of alternating convex/concave arcs through points on a circle
static Contour flower(Vec2 c, double R, int n) {
    Contour out;
    out.closed = true;
    std::vector<Vec2> p;
    for (int i = 0; i < n; ++i) p.push_back({c.x + R * std::cos(2 * PI * i / n), c.y + R * std::sin(2 * PI * i / n)});
    for (int i = 0; i < n; ++i) {
        Vec2 a = p[(size_t)i], b = p[(size_t)((i + 1) % n)];
        double L = geometry::distance(a, b);
        bool convex = (i % 2 == 0) || I(0, 1);
        double sag = U(0.1, 0.45) * L;
        double rad = (L * L / 4 + sag * sag) / (2 * sag);
        Vec2 mid = (a + b) * 0.5, dir = (b - a).normalized(), right{dir.y, -dir.x};
        double h = rad - sag;
        Vec2 cen = convex ? mid - right * h : mid + right * h;
        out.append(geometry::ArcSpan::fromCentre(cen, a, b, convex ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW));
    }
    return out;
}
// spikes: star with needle-sharp points
static Contour spikes(Vec2 c, double R, int n) {
    std::vector<Vec2> p;
    for (int i = 0; i < 2 * n; ++i) {
        double a = PI * i / n + (i % 2 ? U(-0.05, 0.05) : 0);
        double rr = i % 2 ? R * U(0.2, 0.35) : R * U(0.8, 1.0);
        p.push_back({c.x + rr * std::cos(a), c.y + rr * std::sin(a)});
    }
    return poly(p);
}
// chamfered rectangle with short edges
static Contour chamfered(Vec2 o, double w, double h, double ch) {
    return poly({{o.x + ch, o.y}, {o.x + w - ch, o.y}, {o.x + w, o.y + ch}, {o.x + w, o.y + h - ch}, {o.x + w - ch, o.y + h}, {o.x + ch, o.y + h}, {o.x, o.y + h - ch}, {o.x, o.y + ch}});
}
static Contour slot(Vec2 c, double L, double rr, double ang) {
    Vec2 d{std::cos(ang), std::sin(ang)}, n{-d.y, d.x};
    Vec2 a = c - d * (L / 2), b = c + d * (L / 2);
    Contour o;
    o.closed = true;
    o.append(geometry::LineSpan{a - n * rr, b - n * rr});
    o.append(geometry::ArcSpan::fromCentre(b, b - n * rr, b + n * rr, geometry::ArcDirection::CCW));
    o.append(geometry::LineSpan{b + n * rr, a + n * rr});
    o.append(geometry::ArcSpan::fromCentre(a, a + n * rr, a - n * rr, geometry::ArcDirection::CCW));
    return o;
}
// D-shape: line + arc
static Contour dshape(Vec2 c, double R) {
    Contour o;
    o.closed = true;
    double a0 = U(-0.6, -0.1) * PI;
    Vec2 s = c + Vec2::fromAngle(a0, R), e = c + Vec2::fromAngle(PI - a0, R);
    o.append(geometry::ArcSpan::fromCentre(c, s, e, geometry::ArcDirection::CCW));
    o.append(geometry::LineSpan{e, s});
    return o;
}
static Contour anyClosed(Vec2 c, double R) {
    switch (I(0, 8)) {
        case 0: return gear(c, R, I(5, 14), R * U(0.1, 0.35));
        case 1: return letter(c - Vec2{R / 2, R / 2}, R * 1.2);
        case 2: return flower(c, R, 2 * I(2, 4));
        case 3: return spikes(c, R, I(3, 7));
        case 4: return chamfered(c - Vec2{R, R * 0.6}, 2 * R, 1.2 * R, U(0.3, R * 0.3));
        case 5: return slot(c, U(0.5, 2) * R, U(0.25, 0.6) * R, U(0, PI));
        case 6: return dshape(c, R);
        case 7: return Contour::circle(c, R * U(0.4, 1.0), I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW);
        default: return Contour::rectangle(geometry::Box2{{c.x - R, c.y - R * U(0.3, 1)}, {c.x + R * U(0.3, 1), c.y + R}});
    }
}
static Contour anyOpen(Vec2 c, double R) {
    switch (I(0, 3)) {
        case 0: {  // S-curve of two arcs + line
            Contour o;
            double r1 = U(4, R / 2);
            o.append(geometry::ArcSpan{c, r1, PI, 0, geometry::ArcDirection::CW});
            Vec2 e = o.endPoint();
            o.append(geometry::ArcSpan{e + Vec2{r1, 0}, r1, PI, 2 * PI, geometry::ArcDirection::CCW});
            o.append(geometry::LineSpan{o.endPoint(), o.endPoint() + Vec2{U(3, 15), U(-10, 10)}});
            return o;
        }
        case 1: return poly({c, c + Vec2{U(10, 2 * R), U(-5, 5)}}, false);
        case 2: {  // staircase
            std::vector<Vec2> p{c};
            int n = I(2, 5);
            for (int i = 0; i < n; ++i) {
                p.push_back(p.back() + Vec2{U(5, 15), 0});
                p.push_back(p.back() + Vec2{0, U(5, 15) * (I(0, 1) ? 1 : -1)});
            }
            return poly(p, false);
        }
        default: {  // arc-line-arc hook
            Contour o;
            o.append(geometry::LineSpan{c, c + Vec2{R, 0}});
            double rr = U(3, 12);
            o.append(geometry::ArcSpan{c + Vec2{R, rr}, rr, -PI / 2, PI / 2 * U(0.2, 1.8), geometry::ArcDirection::CCW});
            return o;
        }
    }
}

// ---- sample hash: XY quantised to 0.004 mm keeping min z; buckets of 0.5 mm
struct SampleHash {
    double cell = 0.5;
    std::unordered_map<long long, std::vector<std::array<double, 3>>> b;
    std::unordered_map<long long, size_t> keyIdx;
    std::unordered_map<long long, long long> keyBucket;
    static long long k2(long long i, long long j) { return i * 1000003LL + j; }
    void add(Vec2 q, double z) {
        long long qi = std::llround(q.x / 1e-6), qj = std::llround(q.y / 1e-6);
        long long kq = qi * 1000000007LL + qj * 7919LL;
        long long bi = (long long)std::floor(q.x / cell), bj = (long long)std::floor(q.y / cell);
        long long bk = k2(bi, bj);
        auto& v = b[bk];
        auto it = keyIdx.find(kq);
        if (it != keyIdx.end()) {
            auto& s = b[keyBucket[kq]][it->second];
            if (z < s[2]) s[2] = z;
            return;
        }
        keyIdx[kq] = v.size();
        keyBucket[kq] = bk;
        v.push_back({q.x, q.y, z});
    }
    // min z among samples within rho of X; returns +inf if none. Also returns nearest distance.
    std::pair<double, double> query(Vec2 X, double rho) const {
        double zmin = 1e18, dmin = 1e18;
        long long i0 = (long long)std::floor((X.x - rho) / cell), i1 = (long long)std::floor((X.x + rho) / cell);
        long long j0 = (long long)std::floor((X.y - rho) / cell), j1 = (long long)std::floor((X.y + rho) / cell);
        for (long long i = i0; i <= i1; ++i)
            for (long long j = j0; j <= j1; ++j) {
                auto it = b.find(k2(i, j));
                if (it == b.end()) continue;
                for (auto& s : it->second) {
                    double d = std::hypot(s[0] - X.x, s[1] - X.y);
                    if (d < dmin) dmin = d;
                    if (d <= rho && s[2] < zmin) zmin = s[2];
                }
            }
        return {zmin, dmin};
    }
};

struct WallPt {
    Vec2 p, n;  // n = left normal of the vector
    size_t contour, span;
    bool line;
};
static std::vector<WallPt> wallPoints(const std::vector<Contour>& cs, double step) {
    std::vector<WallPt> out;
    for (size_t ci = 0; ci < cs.size(); ++ci)
        for (size_t si = 0; si < cs[ci].spans.size(); ++si) {
            const Span& s = cs[ci].spans[si];
            double len = 0;
            Vec2 prev = geometry::pointAt(s, 0);
            for (int k = 1; k <= 64; ++k) { Vec2 q = geometry::pointAt(s, k / 64.0); len += geometry::distance(prev, q); prev = q; }
            int n = std::max(2, (int)std::ceil(len / step));
            for (int k = 0; k < n; ++k) {
                double t = (double)k / n;
                Vec2 tg = geometry::tangentAt(s, t).normalized();
                out.push_back({geometry::pointAt(s, t), Vec2{-tg.y, tg.x}, ci, si, std::holds_alternative<geometry::LineSpan>(s)});
            }
        }
    return out;
}

struct Stats {
    int scenes = 0, empty = 0, gouge = 0, soft = 0, allowViol = 0, side = 0, zFail = 0, rapid = 0, vert = 0, descent = 0, arc = 0, det = 0, diag = 0,
        coverage = 0, covChecked = 0, tabVal = 0, tabInterior = 0, tabCount = 0, tabCountChecked = 0, tabLen = 0, tabLenChecked = 0, tabIntLen = 0,
        tabMerged = 0, finOrder = 0, finChecked = 0, levels = 0, levelsChecked = 0, fullCircle = 0, tinyArc = 0, vCorner = 0, plungeSide = 0;
    double worst = 1e9, worstCone = 1e9;
};

int main(int argc, char** argv) {
    unsigned long seed = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 1;
    int N = argc > 2 ? std::atoi(argv[2]) : 200;
    int target = argc > 3 ? std::atoi(argv[3]) : -1;
    rng.seed(seed);
    Stats st;
    std::map<std::string, int> kinds;
    auto t0 = std::chrono::steady_clock::now();
    for (int sc = 0; sc < N; ++sc) {
        std::vector<Contour> cs;
        int kind = I(0, 9);
        double R = U(8, 30);
        bool hasCircle = false, hasBez = false, mayCross = false, oneOpen = false;
        if (kind == 0) cs.push_back(anyClosed({0, 0}, R));
        else if (kind == 1) {  // separated set (no merging) on a jittered grid
            int n = I(2, 6);
            for (int i = 0; i < n; ++i) cs.push_back(anyClosed({(i % 3) * 3.2 * 30 + U(-3, 3), (i / 3) * 3.2 * 30 + U(-3, 3)}, U(8, 25)));
        } else if (kind == 2) {  // concentric rings
            double r0 = U(20, 40);
            Vec2 c{U(-5, 5), U(-5, 5)};
            int n = I(2, 4);
            double rr = r0;
            for (int i = 0; i < n; ++i) {
                cs.push_back(I(0, 1) ? Contour::circle(c + Vec2{U(-1, 1), U(-1, 1)}, rr) : chamfered(c - Vec2{rr, rr}, 2 * rr, 2 * rr, U(0.5, rr / 3)));
                rr *= U(0.4, 0.75);
            }
        } else if (kind == 3) {  // close neighbours (merge path)
            Contour a = anyClosed({0, 0}, R);
            Contour b2 = anyClosed({0, 0}, U(6, 20));
            double gap = std::pow(10.0, U(-1.5, 1.0));
            b2 = translate(b2, {a.bounds().max.x - b2.bounds().min.x + gap, U(-6, 6)});
            cs = {a, b2};
        } else if (kind == 4) {  // single open
            cs.push_back(anyOpen({0, 0}, R));
            oneOpen = true;
        } else if (kind == 5) {  // hole in plate with letters/gears inside
            Contour plate = Contour::rectangle(geometry::Box2{{-60, -45}, {60, 45}});
            cs.push_back(plate);
            int n = I(1, 3);
            for (int i = 0; i < n; ++i) cs.push_back(anyClosed({-35.0 + 35 * i, U(-10, 10)}, U(6, 14)));
        } else if (kind == 6) {  // reversed orientation inputs
            cs.push_back(reversed(anyClosed({0, 0}, R)));
            if (I(0, 1)) cs.push_back(reversed(anyClosed({3.5 * R + 10, 0}, R)));
        } else if (kind == 7) {  // open + closed mix
            cs.push_back(anyClosed({0, 0}, R));
            cs.push_back(anyOpen({U(-R, R), U(-R, R)}, R));
            mayCross = true;
        } else if (kind == 8) {  // overlapping closed vectors
            cs.push_back(anyClosed({0, 0}, R));
            cs.push_back(anyClosed({U(-R / 2, R / 2), U(-R / 2, R / 2)}, R * U(0.5, 1)));
            mayCross = true;
        } else {  // big rectangle with many small circles (tab/order test)
            int nx = I(2, 3), ny = I(2, 3);
            for (int i = 0; i < nx; ++i)
                for (int j = 0; j < ny; ++j) cs.push_back(I(0, 1) ? Contour::circle({i * 40.0, j * 40.0}, U(8, 15)) : Contour::rectangle(geometry::Box2{{i * 40.0 - 12, j * 40.0 - 9}, {i * 40.0 + 12, j * 40.0 + 9}}));
        }
        for (auto& c : cs)
            for (auto& s : c.spans) {
                if (std::holds_alternative<geometry::BezierSpan>(s)) hasBez = true;
                if (auto* a = std::get_if<geometry::ArcSpan>(&s)) if (std::fabs(std::fabs(mySweep(*a)) - 2 * PI) < 1e-9) hasCircle = true;
            }
        // ---- params
        ProfileParams p;
        bool useV = I(0, 4) == 0;
        double toolD = std::vector<double>{2.0, 3.175, 6.0, 8.0, 12.0}[(size_t)I(0, 4)];
        double vAng = std::vector<double>{30, 60, 90, 120}[(size_t)I(0, 3)];
        double vD = std::vector<double>{6.0, 12.7, 25.0}[(size_t)I(0, 2)];
        tooling::Tool tool = useV ? vbit(vD, vAng) : endMill(toolD);
        const double vTan = std::tan(vAng * PI / 360);
        int sk = I(0, 9);
        p.side = sk < 4 ? ProfileSide::Outside : sk < 8 ? ProfileSide::Inside : ProfileSide::On;
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = I(0, 1) ? 0.0 : U(-0.25, 0.8);
        p.passes.startDepth = I(0, 3) == 0 ? U(0, 5) : 0.0;
        p.passes.cutDepth = useV ? U(0.5, 6) : U(1, 18);
        p.passes.passDepth = U(0.8, 7);
        if (I(0, 4) == 0) p.passes.lastPassThickness = U(0.2, 1.0);
        p.safeZ = U(3, 15);
        p.separateLastPass = I(0, 3) == 0;
        p.lastPassAllowance = U(0.05, 0.6);
        p.lastPassReverse = I(0, 1);
        p.tabs.enabled = I(0, 1) == 0;
        p.tabs.length = U(1, 15);
        p.tabs.thickness = U(0.3, 5);
        p.tabs.spacing = I(0, 2) ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance;
        p.tabs.count = I(1, 8);
        p.tabs.distance = U(15, 150);
        p.tabs.avoidCorners = I(0, 1);
        p.tabs.threeD = I(0, 3) == 0;
        int rk = I(0, 4);
        p.ramp.type = rk <= 1 ? RampType::None : rk == 2 ? RampType::Smooth : rk == 3 ? RampType::ZigZag : RampType::Spiral;
        p.ramp.distance = U(1, 50);
        p.ramp.useAngle = I(0, 2) == 0;
        p.ramp.angle = U(1, 40) * PI / 180;
        p.ramp.onLeadIn = I(0, 2) == 0;
        int lk = I(0, 2);
        p.leads.type = lk == 0 ? LeadType::None : lk == 1 ? LeadType::Straight : LeadType::Circular;
        p.leads.length = U(0.5, 20);
        p.leads.angle = U(0.02, PI / 2);
        p.leads.radius = U(0.5, 15);
        p.leads.sweep = U(0.05, 180) * PI / 180;
        p.leads.leadOut = I(0, 1);
        p.leads.overcut = I(0, 1) ? U(0, 25) : 0.0;
        p.order.selectionOrder = I(0, 3) == 0;
        p.order.leftToRight = I(0, 3) == 0;
        p.order.bottomToTop = I(0, 3) == 0;
        p.order.grid = I(0, 3) == 0;
        int sa = I(0, 2);
        p.startAt = sa == 0 ? StartAt::VectorStart : sa == 1 ? StartAt::Optimise : StartAt::ClosestOnBoundingBox;
        p.startAnchor = (BoxAnchor)I(0, 8);
        p.startPosition = {U(-80, 80), U(-80, 80)};
        p.corners.sharpExternal = useV && I(0, 1);
        p.corners.sharpInternal = useV && I(0, 2) > 0;
        p.corners.mitreLimit = U(1.2, 8);
        if (target >= 0 && sc != target) continue;

        auto res = profileToolpath(cs, tool, p);
        auto res2 = profileToolpath(cs, tool, p);
        ++st.scenes;
        std::vector<std::string> errs, notes;
        char b[400];
        if (fmt(res.toolpath) != fmt(res2.toolpath) || res.diagnostics.size() != res2.diagnostics.size()) { ++st.det; errs.push_back("nondeterministic"); }
        if (res.has(ProfileIssue::DiscontinuousContour) || res.has(ProfileIssue::EmptyContour)) notes.push_back("generator produced invalid contour");
        const auto& mv = res.toolpath.moves;
        if (mv.empty()) { ++st.empty; if (res.diagnostics.empty()) { ++st.diag; errs.push_back("empty without diagnostic"); } }
        const double r = profileToolRadius(tool, p.passes);
        const double tol = 1e-3;
        const double bottom = p.passes.bottomZ();
        const bool merged = res.has(ProfileIssue::MergedWithNeighbour) || res.has(ProfileIssue::TrimmedNearVector);
        // ---- structure
        for (size_t i = 0; i < mv.size(); ++i) {
            const Move& m = mv[i];
            if (m.to.z < bottom - 1e-9 || m.to.z > p.safeZ + 1e-12) { ++st.zFail; errs.push_back("Z out of [bottom, safeZ]"); break; }
            if (i == 0) { if (m.kind != MoveKind::Rapid) { ++st.rapid; errs.push_back("first move not rapid"); } continue; }
            const auto A = mv[i - 1].to;
            if (m.kind == MoveKind::Rapid && (m.to.z != p.safeZ || A.z != p.safeZ)) { ++st.rapid; errs.push_back("rapid not at safe Z"); break; }
            if (m.kind == MoveKind::Plunge && (A.x != m.to.x || A.y != m.to.y || m.to.z >= A.z)) { ++st.vert; errs.push_back("plunge not vertical-down"); break; }
            if (m.kind == MoveKind::Retract && (A.x != m.to.x || A.y != m.to.y || m.to.z != p.safeZ)) { ++st.vert; errs.push_back("retract not vertical to safe Z"); break; }
            if (m.kind == MoveKind::Feed && m.to.z < A.z && A.x == m.to.x && A.y == m.to.y) { ++st.vert; errs.push_back("vertical descent as Feed"); break; }
            if ((m.kind == MoveKind::Feed || isArc(m.kind)) && m.to.z < A.z - 1e-12 && (A.x != m.to.x || A.y != m.to.y)) {
                bool allowed = p.ramp.type != RampType::None || p.ramp.onLeadIn || p.tabs.threeD || p.corners.sharpInternal;
                if (!allowed) { ++st.descent; snprintf(b, sizeof b, "sloped descent without ramp/3D tab at move %zu", i); errs.push_back(b); break; }
            }
            if (isArc(m.kind)) {
                double r0 = geometry::distance(A.xy(), m.centre), r1 = geometry::distance(m.to.xy(), m.centre);
                if (std::fabs(r0 - r1) > 1e-6) { ++st.arc; errs.push_back("arc radius mismatch"); break; }
                double ch = geometry::distance(A.xy(), m.to.xy());
                if (ch > 0 && ch < 0.005) { ++st.tinyArc; snprintf(b, sizeof b, "arc with chord %.3g (full-circle hazard) at move %zu", ch, i); errs.push_back(b); }
                if (ch == 0 && !hasCircle) { ++st.fullCircle; snprintf(b, sizeof b, "full-circle arc move %zu but no circle input", i); notes.push_back(b); }
            }
        }
        for (size_t i = 0; i + 1 < mv.size(); ++i) if (mv[i].kind == MoveKind::Retract && mv[i + 1].kind != MoveKind::Rapid) { ++st.rapid; errs.push_back("retract not followed by rapid"); break; }
        if (!mv.empty() && mv.back().kind != MoveKind::Retract) { ++st.rapid; errs.push_back("does not end with retract"); }

        Obstacles ob(cs);
        // ---- gouge / allowance / side
        if (p.side != ProfileSide::On && !mv.empty()) {
            double worst = 1e9, worstA = 1e9;
            Vec2 wAt{};
            double wZ = 0;
            size_t wMove = 0;
            int sideBad = 0, sideN = 0;
            sampleMoves(res.toolpath, 0.02, [&](Vec2 q, double z, size_t mi) {
                if (z >= 0) return;
                double rad = useV ? std::min(-z * vTan, vD / 2) : r;
                double d = ob.dist(q, rad + std::max(0.0, p.allowance) + 1);
                double m = d - (rad + std::min(0.0, p.allowance));
                if (m < worst) { worst = m; wAt = q; wZ = z; wMove = mi; }
                worstA = std::min(worstA, d - (rad + p.allowance));
                if (!mayCross && !oneOpen && kind != 7 && d > 0.5 * r && (++sideN % 7 == 0)) {
                    bool in = ob.depth(q) % 2 == 1;
                    if ((p.side == ProfileSide::Outside) == in) ++sideBad;
                }
            });
            if (useV) st.worstCone = std::min(st.worstCone, worst); else st.worst = std::min(st.worst, worst);
            if (worst < -tol) {
                ++st.gouge;
                snprintf(b, sizeof b, "GOUGE %.5g at (%.4f,%.4f) z %.4f move %zu %s%s", worst, wAt.x, wAt.y, wZ, wMove, kn(mv[wMove].kind), useV ? " [V cone]" : "");
                errs.push_back(b);
            } else if (worst < 0) ++st.soft;
            if (worstA < -2 * tol && worst >= -tol) { ++st.allowViol; snprintf(b, sizeof b, "allowance violated by %.4g", -worstA); notes.push_back(b); }
            if (sideBad) { ++st.side; snprintf(b, sizeof b, "tool on wrong side at %d samples", sideBad); errs.push_back(b); }
        }
        // ---- blocks
        struct Blk { size_t a, e; };
        std::vector<Blk> bls;
        { size_t a = 0; for (size_t i = 0; i < mv.size(); ++i) { if (mv[i].kind == MoveKind::Rapid) a = i; if (mv[i].kind == MoveKind::Retract) bls.push_back({a, i}); } }
        // ---- level structure (no ramp/leads/overcut/effective tabs, end mill)
        const bool tabsEff = p.tabs.enabled && p.tabs.thickness > 0;
        if (!useV && p.ramp.type == RampType::None && !p.ramp.onLeadIn && p.leads.type == LeadType::None && p.leads.overcut == 0 && !tabsEff && !mv.empty()) {
            ++st.levelsChecked;
            auto lv = p.passes.levels();
            for (size_t bi = 0; bi < bls.size(); ++bi) {
                std::map<double, double> flat;
                for (size_t k = bls[bi].a + 1; k <= bls[bi].e; ++k)
                    if ((mv[k].kind == MoveKind::Feed || isArc(mv[k].kind)) && mv[k].to.z == mv[k - 1].to.z) flat[mv[k].to.z] += xyLen(res.toolpath, k);
                bool isFinBlock = p.separateLastPass && flat.size() == 1 && flat.begin()->first == bottom && lv.size() > 1;
                std::set<double> want(lv.begin(), lv.end());
                if (isFinBlock) want = {bottom};
                std::set<double> got;
                for (auto& [z, L] : flat) got.insert(z);
                double L0 = flat.empty() ? 0 : flat.begin()->second;
                bool eq = true;
                for (auto& [z, L] : flat) if (std::fabs(L - L0) > 1e-6 * std::max(1.0, L0)) eq = false;
                if (got != want || !eq) {
                    ++st.levels;
                    std::string s = "level structure: block " + std::to_string(bi) + " levels/lengths:";
                    for (auto& [z, L] : flat) { snprintf(b, sizeof b, " %.4f:%.4f", z, L); s += b; }
                    s += " want";
                    for (double z : want) { snprintf(b, sizeof b, " %.4f", z); s += b; }
                    errs.push_back(s);
                    break;
                }
            }
        }
        // ---- per-point coverage + tabs (end mill, not On, not merged)
        if (!useV && p.side != ProfileSide::On && !merged && !mv.empty()) {
            SampleHash H;
            sampleMoves(res.toolpath, 0.02, [&](Vec2 q, double z, size_t) { if (z < 0) H.add(q, z); });
            const double d0 = r + p.allowance;
            const double delta = hasBez ? 1.5e-3 : 2.5e-4;
            auto W = wallPoints(cs, 0.05);
            const double zTab = bottom + std::min(p.tabs.thickness, p.passes.cutDepth);
            const double eps = 1e-6;
            std::vector<int> state(W.size(), -1);  // -1 untouched, 0 bottom, 1 tab, 2 bad
            std::vector<Vec2> wdir(W.size());
            int bad = 0, badV = 0;
            size_t touched = 0;
            std::string firstBad;
            for (size_t i = 0; i < W.size(); ++i) {
                auto [zmin, dmin] = H.query(W[i].p, d0 + delta);
                if (zmin > 1e17) continue;
                if (dmin < d0 - 1e-3) continue;  // something (a lead) came closer than the cut: not a cut-wall point
                ++touched;
                if (zmin <= bottom + eps) { state[i] = 0; continue; }
                if (!tabsEff) { state[i] = 2; ++bad; if (firstBad.empty()) { snprintf(b, sizeof b, "wall point (%.4f,%.4f) of vector %zu only cut to %.5f (bottom %.5f)", W[i].p.x, W[i].p.y, W[i].contour, zmin, bottom); firstBad = b; } continue; }
                if (p.tabs.threeD ? zmin <= zTab + eps : std::fabs(zmin - zTab) <= eps) state[i] = 1;
                else { state[i] = 2; ++badV; if (firstBad.empty()) { snprintf(b, sizeof b, "wall point (%.4f,%.4f) of vector %zu cut to %.5f: neither bottom %.5f nor tab top %.5f", W[i].p.x, W[i].p.y, W[i].contour, zmin, bottom, zTab); firstBad = b; } }
            }
            ++st.covChecked;
            if (bad) { ++st.coverage; snprintf(b, sizeof b, "COVERAGE: %d wall points not at final depth; first: %s", bad, firstBad.c_str()); errs.push_back(b); }
            if (badV) { ++st.tabVal; snprintf(b, sizeof b, "TAB HEIGHT: %d wall points at a wrong height; first: %s", badV, firstBad.c_str()); errs.push_back(b); }
            if (tabsEff) {
                // components per contour, skipping untouched points
                struct Comp { size_t contour; std::vector<size_t> pts; };
                std::vector<Comp> comps;
                std::vector<size_t> perContour(cs.size(), 0);
                for (size_t ci = 0; ci < cs.size(); ++ci) {
                    std::vector<size_t> idx;
                    for (size_t i = 0; i < W.size(); ++i) if (W[i].contour == ci && state[i] >= 0) idx.push_back(i);
                    if (idx.empty()) continue;
                    std::vector<Comp> cc;
                    bool open = false;
                    for (size_t k = 0; k < idx.size(); ++k) {
                        bool t = state[idx[k]] != 0;
                        if (t) { if (!open) { cc.push_back({ci, {}}); open = true; } cc.back().pts.push_back(idx[k]); }
                        else open = false;
                    }
                    if (cs[ci].closed && cc.size() >= 2 && state[idx.front()] != 0 && state[idx.back()] != 0) {
                        for (size_t x : cc.front().pts) cc.back().pts.push_back(x);
                        cc.erase(cc.begin());
                    }
                    if (cs[ci].closed && cc.size() == 1 && cc[0].pts.size() == idx.size()) {}  // whole loop raised
                    perContour[ci] = cc.size();
                    for (auto& c : cc) comps.push_back(c);
                }
                // dropped per contour from diagnostics
                std::vector<int> dropped(cs.size(), 0);
                for (auto& d : res.diagnostics) {
                    if (d.issue != ProfileIssue::TabsNotPlaced || !d.contour) continue;
                    auto pos = d.message.find(" tab(s) left out");
                    if (pos == std::string::npos) continue;
                    size_t bb = d.message.rfind(": ", pos);
                    dropped[*d.contour] += std::atoi(d.message.c_str() + bb + 2);
                }
                if (p.tabs.spacing == TabSpacing::ConstantNumber && zTab < -1e-9) {
                    for (size_t ci = 0; ci < cs.size(); ++ci) {
                        // cuts per contour: count blocks whose bottom samples are nearest this contour (single cut assumed when not merged)
                        size_t touchedC = 0;
                        for (size_t i = 0; i < W.size(); ++i) if (W[i].contour == ci && state[i] >= 0) ++touchedC;
                        if (touchedC == 0) continue;
                        ++st.tabCountChecked;
                        if ((int)perContour[ci] + dropped[ci] != p.tabs.count) {
                            ++st.tabCount;
                            snprintf(b, sizeof b, "TAB COUNT vector %zu: placed %zu + dropped %d != requested %d", ci, perContour[ci], dropped[ci], p.tabs.count);
                            errs.push_back(b);
                        }
                    }
                }
                // length checks on tabs lying on one straight vector span
                for (auto& c : comps) {
                    bool oneLine = true;
                    for (size_t x : c.pts) if (!W[x].line || W[x].span != W[c.pts[0]].span) oneLine = false;
                    if (!oneLine) continue;
                    {
                        const Span& sp0 = cs[c.contour].spans[W[c.pts[0]].span];
                        Vec2 e0 = geometry::startPoint(sp0), e1 = geometry::endPoint(sp0);
                        double mind = 1e9;
                        for (size_t x : c.pts) mind = std::min({mind, geometry::distance(W[x].p, e0), geometry::distance(W[x].p, e1)});
                        if (mind < 0.3) continue;
                        c.contour += 0;  // keep
                        if (mind < 2 * r + 0.5) { /* too near a corner for interior columns */ c.pts.reserve(c.pts.size()); }
                    }
                    // ignore components touching the span ends (corner neighbourhood)
                    double L = 0;
                    for (size_t k = 1; k < c.pts.size(); ++k) L = std::max(L, geometry::distance(W[c.pts[0]].p, W[c.pts[k]].p));
                    L += 0.05;
                    ++st.tabLenChecked;
                    const double want = p.tabs.length + 2 * r + (p.tabs.threeD ? 2 * std::min(p.tabs.thickness, p.passes.cutDepth) : 0.0);
                    if (std::fabs(L - want) > 0.15) {
                        ++st.tabLen;
                        snprintf(b, sizeof b, "TAB LENGTH at wall: %.3f vs expected %.3f (vector %zu near (%.3f,%.3f))", L, want, c.contour, W[c.pts[0]].p.x, W[c.pts[0]].p.y);
                        errs.push_back(b);
                    }
                    // interior columns
                    if (p.tabs.threeD) continue;
                    // waste/path side: direction from wall to nearest sample
                    size_t mid = c.pts[c.pts.size() / 2];
                    Vec2 n = W[mid].n;
                    auto qa = H.query(W[mid].p + n * d0, 1e-3), qb = H.query(W[mid].p - n * d0, 1e-3);
                    if (qb.second < qa.second) n = n * -1.0;
                    // extend the column scan a little beyond the wall component
                    Vec2 t = Vec2{-n.y, n.x};
                    Vec2 a0 = W[c.pts.front()].p, a1 = W[c.pts.back()].p;
                    if ((a1 - a0).dot(t) < 0) t = t * -1.0;
                    bool nearCorner = false;
                    { const Span& sp0 = cs[c.contour].spans[W[c.pts[0]].span]; Vec2 e0 = geometry::startPoint(sp0), e1 = geometry::endPoint(sp0);
                      for (size_t x : c.pts) if (geometry::distance(W[x].p, e0) < 2 * r + 0.5 || geometry::distance(W[x].p, e1) < 2 * r + 0.5) nearCorner = true; }
                    if (nearCorner) continue;
                    for (double f : {0.3, 0.6, 1.0}) {
                        double tt = p.allowance + f * r;
                        double lenT = 0;
                        int vals = 0;
                        for (double s = -0.5; s <= L + 0.5; s += 0.02) {
                            Vec2 X = a0 + t * s + n * tt;
                            auto [z, dm] = H.query(X, r + 2.5e-4);
                            (void)dm;
                            if (z > bottom + eps) {
                                lenT += 0.02;
                                if (std::fabs(z - zTab) > eps && z < 1e17) ++vals;
                            }
                        }
                        double h = std::sqrt(std::max(0.0, r * r - (d0 - tt) * (d0 - tt)));
                        double hr = 0;
                        if (p.separateLastPass) { double dd = d0 + p.lastPassAllowance - tt; hr = std::sqrt(std::max(0.0, r * r - dd * dd)); }
                        double wantT = p.tabs.length + 2 * r - 2 * std::max(h, hr);
                        if (vals) { ++st.tabInterior; snprintf(b, sizeof b, "TAB INTERIOR: %d kerf points at f=%.1f cut to an intermediate height (vector %zu near (%.3f,%.3f))", vals, f, c.contour, a0.x, a0.y); errs.push_back(b); break; }
                        if (lenT < wantT - 0.12) { ++st.tabIntLen; snprintf(b, sizeof b, "TAB INTERIOR LENGTH f=%.1f: %.3f < expected %.3f (vector %zu near (%.3f,%.3f))", f, lenT, wantT, c.contour, a0.x, a0.y); errs.push_back(b); break; }
                    }
                }
            }
        }
        // ---- finishing order (median-distance classification)
        if (p.separateLastPass && p.side != ProfileSide::On && bls.size() >= 2 && !mv.empty()) {
            std::vector<std::vector<Vec2>> pts(bls.size());
            std::vector<double> med(bls.size(), 1e9);
            for (size_t bi = 0; bi < bls.size(); ++bi) {
                Toolpath sub{"", {}, {}};
                for (size_t k = bls[bi].a; k <= bls[bi].e; ++k) sub.moves.push_back(mv[k]);
                std::vector<double> ds;
                sampleMoves(sub, 0.25, [&](Vec2 q, double z, size_t) { if (z != bottom) return; pts[bi].push_back(q); ds.push_back(ob.dist(q, 1e9)); });
                if (!ds.empty()) { std::nth_element(ds.begin(), ds.begin() + (long)ds.size() / 2, ds.end()); med[bi] = ds[ds.size() / 2]; }
            }
            const double rf = (useV ? r : r) + p.allowance;
            std::vector<int> isFin(bls.size());
            for (size_t bi = 0; bi < bls.size(); ++bi) isFin[bi] = std::fabs(med[bi] - rf) < 0.5 * p.lastPassAllowance;
            size_t firstFin = bls.size();
            for (size_t bi = 0; bi < bls.size(); ++bi) if (isFin[bi]) { firstFin = bi; break; }
            bool clean = firstFin > 0 && firstFin < bls.size();
            for (size_t bi = firstFin; bi < bls.size(); ++bi) if (!isFin[bi]) clean = false;
            if (clean) {
                ++st.finChecked;
                int prev = -1;
                bool ok = true;
                std::string seq;
                for (size_t fi = firstFin; fi < bls.size(); ++fi) {
                    if (pts[fi].empty()) continue;
                    // counterpart: roughing block with smallest median nearest-distance from finishing samples
                    double best = 1e18;
                    int rank = -1;
                    for (size_t ri = 0; ri < firstFin; ++ri) {
                        if (pts[ri].empty()) continue;
                        std::vector<double> ds;
                        for (size_t k = 0; k < pts[fi].size(); k += 4) {
                            double m = 1e18;
                            for (auto& q : pts[ri]) m = std::min(m, geometry::distance(q, pts[fi][k]));
                            ds.push_back(m);
                        }
                        std::nth_element(ds.begin(), ds.begin() + (long)ds.size() / 2, ds.end());
                        if (ds[ds.size() / 2] < best) { best = ds[ds.size() / 2]; rank = (int)ri; }
                    }
                    seq += " " + std::to_string(rank);
                    if (rank < prev) ok = false;
                    prev = std::max(prev, rank);
                }
                if (!ok) { ++st.finOrder; errs.push_back("FINISH ORDER: finishing counterparts' roughing ranks:" + seq); }
            } else if (firstFin < bls.size() || true) {
                notes.push_back("finishing classification unclear");
            }
        }
        if (!errs.empty() || target >= 0) {
            std::printf("scene %d kind %d side %d dir %d tool %s%.3f%s allow %.3f sep %d(lpa %.3f) tabs %d(3d %d n %d len %.2f th %.2f sp %d) ramp %d lead %d out %d oc %.2f start %d moves %zu diags:",
                        sc, kind, (int)p.side, (int)p.direction, useV ? "V" : "EM", useV ? vD : toolD, useV ? (std::string("@") + std::to_string((int)vAng)).c_str() : "", p.allowance, p.separateLastPass, p.lastPassAllowance,
                        p.tabs.enabled, p.tabs.threeD, p.tabs.count, p.tabs.length, p.tabs.thickness, (int)p.tabs.spacing, (int)p.ramp.type, (int)p.leads.type, p.leads.leadOut, p.leads.overcut, (int)p.startAt, mv.size());
            for (auto& d : res.diagnostics) std::printf(" %d", (int)d.issue);
            std::printf("\n");
            for (auto& e : errs) std::printf("   - %s\n", e.c_str());
            if (target >= 0) {
                for (auto& e : notes) std::printf("   . %s\n", e.c_str());
                for (auto& d : res.diagnostics) std::printf("   diag %d %s\n", (int)d.issue, d.message.c_str());
                std::printf("   r %.4f bottom %.4f top %.4f pass %.3f lpt %.3f safe %.3f sharpInt %d sharpExt %d avoid %d\n", r, bottom, -p.passes.startDepth, p.passes.passDepth, p.passes.lastPassThickness, p.safeZ, p.corners.sharpInternal, p.corners.sharpExternal, p.tabs.avoidCorners);
                for (auto& c : cs) { std::printf("   contour closed=%d spans=%zu\n", c.closed, c.spans.size()); for (auto& s2 : c.spans) { auto a = geometry::startPoint(s2), e = geometry::endPoint(s2); std::printf("     span %zu (%.4f,%.4f)->(%.4f,%.4f)\n", s2.index(), a.x, a.y, e.x, e.y); } }
                FILE* f = std::fopen("replay4.moves", "w"); dump(res.toolpath, f); std::fclose(f);
            }
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    std::printf("seed %lu: scenes %d empty %d | gouge %d soft %d allowViol %d side %d z %d rapid %d vert %d descent %d arc %d tinyArc %d fullCircle(note) %d det %d diag %d\n", seed, st.scenes, st.empty, st.gouge, st.soft, st.allowViol, st.side, st.zFail, st.rapid, st.vert, st.descent, st.arc, st.tinyArc, st.fullCircle, st.det, st.diag);
    std::printf("seed %lu: coverage %d/%d tabHeight %d tabInterior %d tabIntLen %d tabCount %d/%d tabLen %d/%d finOrder %d/%d levels %d/%d | worst EM %.6g cone %.6g | %.1fs\n", seed, st.coverage, st.covChecked, st.tabVal, st.tabInterior, st.tabIntLen, st.tabCount, st.tabCountChecked, st.tabLen, st.tabLenChecked, st.finOrder, st.finChecked, st.levels, st.levelsChecked, st.worst, st.worstCone, std::chrono::duration<double>(t1 - t0).count());
}
