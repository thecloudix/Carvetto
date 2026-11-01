// Randomised gouge / invariant sweep for T-0120 (verifier throwaway, own generators, own seeds).
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "oracle.hpp"
#include "scene_io.hpp"
#include <functional>
#include <set>
#include <map>
#include "carvetto/toolpath/lead_moves.hpp"

using namespace v;
using namespace carvetto::toolpath;

static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

static Contour polygon(const std::vector<Vec2>& pts, bool closed) {
    Contour c;
    for (size_t i = 0; i + 1 < pts.size(); ++i) c.append(geometry::LineSpan{pts[i], pts[i + 1]});
    if (closed) c.append(geometry::LineSpan{pts.back(), pts.front()});
    c.closed = closed;
    return c;
}
// star-shaped random polygon around centre, radius R; angles sorted, mild radius variation
static std::vector<Vec2> starPts(Vec2 c, double R, int n, double var) {
    std::vector<double> ang;
    for (int i = 0; i < n; ++i) ang.push_back(2 * PI * (i + U(-0.3, 0.3)) / n);
    std::sort(ang.begin(), ang.end());
    std::vector<Vec2> pts;
    for (double a : ang) {
        double r = R * U(1 - var, 1.0);
        pts.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
    }
    return pts;
}
// polygon with some edges replaced by arcs bulging outward by a small sagitta (stays star-shaped-ish)
static Contour arcPoly(Vec2 c, double R, int n) {
    auto pts = starPts(c, R, n, 0.25);
    Contour out;
    out.closed = true;
    for (size_t i = 0; i < pts.size(); ++i) {
        Vec2 a = pts[i], b = pts[(i + 1) % pts.size()];
        if (I(0, 1)) {
            double L = geometry::distance(a, b);
            double sag = U(0.05, 0.3) * L * (I(0, 3) == 0 ? -0.5 : 1.0);
            // circle through a, b with sagitta sag on the outer side (left of a->b is inside for CCW)
            double rad = (L * L / 4 + sag * sag) / (2 * std::fabs(sag));
            Vec2 mid = (a + b) * 0.5;
            Vec2 dir = (b - a).normalized();
            Vec2 right{dir.y, -dir.x};  // outward for CCW polygon
            double h = rad - std::fabs(sag);
            Vec2 cen = sag > 0 ? mid - right * h : mid + right * h;
            auto arcDir = sag > 0 ? geometry::ArcDirection::CW : geometry::ArcDirection::CCW;
            out.append(geometry::ArcSpan::fromCentre(cen, a, b, arcDir));
        } else
            out.append(geometry::LineSpan{a, b});
    }
    return out;
}
static Contour roundedRect(Vec2 o, double w, double h, double rr) {
    Contour c;
    c.closed = true;
    using geometry::ArcSpan;
    using geometry::LineSpan;
    auto ccw = geometry::ArcDirection::CCW;
    c.append(LineSpan{{o.x + rr, o.y}, {o.x + w - rr, o.y}});
    c.append(ArcSpan{{o.x + w - rr, o.y + rr}, rr, -PI / 2, 0, ccw});
    c.append(LineSpan{{o.x + w, o.y + rr}, {o.x + w, o.y + h - rr}});
    c.append(ArcSpan{{o.x + w - rr, o.y + h - rr}, rr, 0, PI / 2, ccw});
    c.append(LineSpan{{o.x + w - rr, o.y + h}, {o.x + rr, o.y + h}});
    c.append(ArcSpan{{o.x + rr, o.y + h - rr}, rr, PI / 2, PI, ccw});
    c.append(LineSpan{{o.x, o.y + h - rr}, {o.x, o.y + rr}});
    c.append(ArcSpan{{o.x + rr, o.y + rr}, rr, PI, 1.5 * PI, ccw});
    return c;
}
static Contour bezBlob(Vec2 c, double R) {
    const double k = 0.5523 * R;
    Contour out;
    out.closed = true;
    std::vector<Vec2> P;
    for (int i = 0; i < 4; ++i) {
        double a = i * PI / 2;
        double r = R * U(0.8, 1.1);
        P.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
    }
    for (int i = 0; i < 4; ++i) {
        Vec2 a = P[(size_t)i], b = P[(size_t)((i + 1) % 4)];
        double aa = i * PI / 2, ab = (i + 1) * PI / 2;
        Vec2 ta{-std::sin(aa), std::cos(aa)}, tb{-std::sin(ab), std::cos(ab)};
        double ka = k * U(0.7, 1.2), kb = k * U(0.7, 1.2);
        out.append(geometry::BezierSpan{a, a + ta * ka, b - tb * kb, b});
    }
    return out;
}
static Contour randomClosed(Vec2 c, double R) {
    switch (I(0, 5)) {
        case 0: return Contour::rectangle(geometry::Box2{{c.x - R, c.y - R * 0.7}, {c.x + R, c.y + R * 0.7}});
        case 1: return Contour::circle(c, R, I(0, 1) ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW);
        case 2: return polygon(starPts(c, R, I(5, 12), 0.4), true);
        case 3: return arcPoly(c, R, I(4, 8));
        case 4: return roundedRect({c.x - R, c.y - R * 0.6}, 2 * R, 1.2 * R, U(0.5, 0.45 * R));
        default: return bezBlob(c, R);
    }
}
static Contour scaled(const Contour& src, Vec2 c, double s) {
    Contour out = src;
    for (auto& sp : out.spans) {
        auto f = [&](Vec2 p) { return c + (p - c) * s; };
        if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = f(l->start); l->end = f(l->end); }
        else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { a->centre = f(a->centre); a->radius *= s; }
        else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) { b->p0 = f(b->p0); b->p1 = f(b->p1); b->p2 = f(b->p2); b->p3 = f(b->p3); }
    }
    return out;
}
static Contour randomOpen(Vec2 c, double R) {
    switch (I(0, 2)) {
        case 0: {
            std::vector<Vec2> pts{c};
            double ang = U(0, 2 * PI);
            int n = I(1, 5);
            for (int i = 0; i < n; ++i) {
                ang += U(-1.2, 1.2);
                double L = U(5, R);
                pts.push_back(pts.back() + Vec2{std::cos(ang), std::sin(ang)} * L);
            }
            return polygon(pts, false);
        }
        case 1: {
            Contour o;
            double a0 = U(0, 2 * PI), sw = U(0.5, 4.0);
            o.append(geometry::ArcSpan{c, R * 0.6, a0, a0 + sw, geometry::ArcDirection::CCW});
            o.append(geometry::LineSpan{o.endPoint(), o.endPoint() + Vec2{U(-8, 8), U(-8, 8)}});
            return o;
        }
        default: {
            Contour o;
            Vec2 a = c, d = c + Vec2{R, U(-R / 2, R / 2)};
            o.append(geometry::BezierSpan{a, a + Vec2{R / 3, R / 2}, d - Vec2{R / 3, -R / 2}, d});
            return o;
        }
    }
}

static Contour comb(Vec2 o, double W, double H) {
    // random comb: teeth of random widths/depths along the top edge (orthogonal, concave)
    std::vector<Vec2> pts{{o.x, o.y}, {o.x + W, o.y}, {o.x + W, o.y + H}};
    double x = o.x + W;
    int n = I(2, 5);
    double step = W / (2 * n + 1);
    for (int i = 0; i < n; ++i) {
        double x1 = x - step * U(0.8, 1.2), x2 = x1 - step * U(0.8, 1.2);
        double d = H * U(0.2, 0.7);
        pts.push_back({x1, o.y + H});
        pts.push_back({x1, o.y + H - d});
        pts.push_back({x2, o.y + H - d});
        pts.push_back({x2, o.y + H});
        x = x2;
    }
    pts.push_back({o.x, o.y + H});
    return polygon(pts, true);
}
static Contour stadium(Vec2 c, double L, double rr, double ang) {
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
static Contour zigzagOpen(Vec2 c, double R) {
    std::vector<Vec2> pts{c};
    int n = I(2, 7);
    double ang = U(0, 2 * PI);
    for (int i = 0; i < n; ++i) {
        ang += (i % 2 ? 1 : -1) * U(1.5, 2.6);
        pts.push_back(pts.back() + Vec2{std::cos(ang), std::sin(ang)} * U(4, R));
    }
    return polygon(pts, false);
}

struct Stats {
    int scenes = 0, empty = 0, gougeFail = 0, sideFail = 0, zFail = 0, rapidFail = 0, vertFail = 0, arcFail = 0,
        detFail = 0, diagFail = 0, descentFail = 0, tabFail = 0, softGouge = 0, vGouge = 0, tabCount = 0, notch = 0, notchStart = 0, bottomFail = 0, finOrder = 0, startMovedUnneeded = 0, startMoved = 0, finChecked = 0, tabChecked = 0;
    double worstMargin = 1e9;
};

int main(int argc, char** argv) {
    unsigned long seed = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 777;
    int N = argc > 2 ? std::atoi(argv[2]) : 500;
    int target = argc > 3 ? std::atoi(argv[3]) : -1;
    bool verbose = target >= 0;
    rng.seed(seed);
    Stats st;
    auto t0 = std::chrono::steady_clock::now();
    for (int sc = 0; sc < N; ++sc) {
        // ---- scene
        std::vector<Contour> cs;
        bool crossingFree = true;
        (void)crossingFree;
        bool onlyOneOpen = false;
        int kind = I(0, 9);
        double R = U(8, 30);
        if (kind == 0) cs.push_back(randomClosed({0, 0}, R));
        else if (kind == 1) {  // nested
            Contour outer = randomClosed({0, 0}, R);
            double s = U(0.25, 0.8);
            Contour inner = scaled(outer, {0, 0}, s);
            cs = {outer, inner};
            if (I(0, 1)) cs.push_back(scaled(outer, {0, 0}, s * U(0.2, 0.7)));
        } else if (kind == 2) {  // neighbours, near touching
            double gap = std::pow(10.0, U(-1.3, 1.2));
            Contour a = randomClosed({0, 0}, R);
            auto ba = a.bounds();
            double R2 = U(6, 25);
            Contour b = randomClosed({0, 0}, R2);
            auto bb = b.bounds();
            double dx = ba.max.x - bb.min.x + gap;
            const Vec2 off{dx, U(-5, 5)};
            for (auto& sp : b.spans) {
                if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { l->start = l->start + off; l->end = l->end + off; }
                else if (auto* ar = std::get_if<geometry::ArcSpan>(&sp)) ar->centre = ar->centre + off;
                else if (auto* bz = std::get_if<geometry::BezierSpan>(&sp)) { bz->p0 = bz->p0 + off; bz->p1 = bz->p1 + off; bz->p2 = bz->p2 + off; bz->p3 = bz->p3 + off; }
            }
            cs = {a, b};
        } else if (kind == 3) {  // single open vector
            cs.push_back(randomOpen({0, 0}, R));
            onlyOneOpen = true;
        } else if (kind == 4) {  // open + closed mix (may cross)
            cs.push_back(randomClosed({0, 0}, R));
            cs.push_back(randomOpen({U(-R, R), U(-R, R)}, R));
            crossingFree = false;
        } else if (kind == 5) {  // small grid
            int nx = I(2, 4), ny = I(1, 3);
            double r = U(3, 8), sp = 2 * r + std::pow(10.0, U(-1, 1.2));
            for (int i = 0; i < nx; ++i)
                for (int j = 0; j < ny; ++j) cs.push_back(randomClosed({i * sp * 1.5, j * sp * 1.5}, r));
        } else if (kind == 7) {  // comb, possibly with a stadium slot inside
            cs.push_back(comb({0, 0}, U(30, 90), U(20, 50)));
            if (I(0, 1)) cs.push_back(stadium({U(10, 20), U(4, 8)}, U(4, 10), U(1.5, 3.5), U(0, 0.3)));
        } else if (kind == 8) {  // cluster of slots at random angles (may be close)
            int n = I(2, 5);
            for (int i = 0; i < n; ++i) cs.push_back(stadium({i * U(8, 20), U(-4, 4)}, U(3, 20), U(1, 5), U(0, PI)));
            crossingFree = false;  // may overlap
        } else if (kind == 9) {  // open zigzag
            cs.push_back(zigzagOpen({0, 0}, R));
            onlyOneOpen = true;
        } else {  // hole with island and siblings
            Contour outer = Contour::rectangle(geometry::Box2{{-R * 2, -R * 1.5}, {R * 2, R * 1.5}});
            cs.push_back(outer);
            cs.push_back(randomClosed({-R, 0}, R * U(0.4, 0.8)));
            cs.push_back(randomClosed({R, 0}, R * U(0.4, 0.8)));
            cs.push_back(scaled(cs[1], {-R, 0}, U(0.2, 0.6)));
        }
        // ---- params
        ProfileParams p;
        double toolD = std::vector<double>{3.0, 6.0, 6.35, 10.0}[(size_t)I(0, 3)];
        bool useV = I(0, 5) == 0;
        double vAng = std::vector<double>{30, 60, 90, 120}[(size_t)I(0, 3)];
        double vD = I(0, 1) ? 12.0 : 6.0;
        tooling::Tool tool = useV ? vbit(vD, vAng) : endMill(toolD);
        const double vTan = std::tan(vAng * PI / 360);
        int sideK = I(0, 9);
        p.side = sideK < 4 ? ProfileSide::Outside : sideK < 8 ? ProfileSide::Inside : ProfileSide::On;
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = I(0, 2) == 0 ? 0.0 : U(-0.3, 0.6);
        p.passes.startDepth = I(0, 3) == 0 ? 1.0 : 0.0;
        p.passes.cutDepth = useV ? U(1, 5) : U(2, 14);
        p.passes.passDepth = U(1, 6);
        p.safeZ = 5;
        p.separateLastPass = I(0, 3) == 0;
        p.lastPassAllowance = U(0.05, 0.5);
        p.lastPassReverse = I(0, 1);
        p.tabs.enabled = I(0, 2) == 0;
        p.tabs.length = U(2, 12);
        p.tabs.thickness = U(0.5, 4);
        p.tabs.spacing = I(0, 1) ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance;
        p.tabs.count = I(1, 6);
        p.tabs.distance = U(20, 120);
        p.tabs.avoidCorners = I(0, 1);
        p.tabs.threeD = I(0, 2) == 0;
        int rk = I(0, 3);
        p.ramp.type = rk == 0 ? RampType::None : rk == 1 ? RampType::Smooth : rk == 2 ? RampType::ZigZag : RampType::Spiral;
        p.ramp.distance = U(2, 40);
        p.ramp.useAngle = I(0, 2) == 0;
        p.ramp.angle = U(2, 30) * PI / 180;
        p.ramp.onLeadIn = I(0, 2) == 0;
        int lk = I(0, 2);
        p.leads.type = lk == 0 ? LeadType::None : lk == 1 ? LeadType::Straight : LeadType::Circular;
        p.leads.length = U(1, 15);
        p.leads.angle = U(0.05, PI / 2);
        p.leads.radius = U(1, 12);
        p.leads.sweep = U(0.1, 90) * PI / 180;
        p.leads.leadOut = I(0, 1);
        p.leads.overcut = I(0, 1) ? U(0, 10) : 0.0;
        p.order.selectionOrder = I(0, 3) == 0;
        p.order.leftToRight = I(0, 3) == 0;
        p.order.bottomToTop = I(0, 3) == 0;
        p.order.grid = I(0, 3) == 0;
        int sa = I(0, 2);
        p.startAt = sa == 0 ? StartAt::VectorStart : sa == 1 ? StartAt::Optimise : StartAt::ClosestOnBoundingBox;
        p.startAnchor = (BoxAnchor)I(0, 8);
        p.startPosition = {U(-50, 50), U(-50, 50)};
        p.corners.sharpExternal = useV && I(0, 1);
        p.corners.sharpInternal = useV && I(0, 1);

        if (target >= 0 && sc != target) continue;
        if (target >= 0 && std::getenv("DUMPSCENE")) writeScene(std::getenv("DUMPSCENE"), cs, p, useV ? vD : toolD, useV ? (int)vAng : 0);
        auto res = profileToolpath(cs, tool, p);
        auto res2 = profileToolpath(cs, tool, p);
        ++st.scenes;
        if (res.has(ProfileIssue::DiscontinuousContour) || res.has(ProfileIssue::EmptyContour)) std::printf("scene %d: generator produced invalid contour\n", sc);
        std::vector<std::string> errs;
        if (fmt(res.toolpath) != fmt(res2.toolpath) || res.diagnostics.size() != res2.diagnostics.size()) {
            ++st.detFail;
            errs.push_back("nondeterministic");
        }
        const auto& mv = res.toolpath.moves;
        if (mv.empty()) {
            ++st.empty;
            if (res.diagnostics.empty()) { ++st.diagFail; errs.push_back("empty without diagnostic"); }
        }
        const double r = profileToolRadius(tool, p.passes);
        const double tol = 1e-3;
        const double bottom = p.passes.bottomZ();
        const double top = -p.passes.startDepth;
        // ---- structural checks
        for (size_t i = 0; i < mv.size(); ++i) {
            const Move& m = mv[i];
            if (m.to.z < bottom - 1e-9) { ++st.zFail; errs.push_back("below bottom"); break; }
            if (m.to.z > p.safeZ + 1e-12) { ++st.zFail; errs.push_back("above safe Z"); break; }
            if (m.kind == MoveKind::Rapid) {
                if (m.to.z != p.safeZ || (i > 0 && mv[i - 1].to.z != p.safeZ)) { ++st.rapidFail; errs.push_back("rapid not at safe Z"); break; }
            }
            if (i == 0) {
                if (m.kind != MoveKind::Rapid) { ++st.rapidFail; errs.push_back("first not rapid"); }
                continue;
            }
            const auto A = mv[i - 1].to;
            if (m.kind == MoveKind::Plunge || m.kind == MoveKind::Retract) {
                if (A.x != m.to.x || A.y != m.to.y) { ++st.vertFail; errs.push_back("plunge/retract not vertical"); break; }
                if (m.kind == MoveKind::Plunge && m.to.z > A.z) { ++st.vertFail; errs.push_back("plunge goes up"); break; }
                if (m.kind == MoveKind::Retract && m.to.z != p.safeZ) { ++st.vertFail; errs.push_back("retract not to safe Z"); break; }
            }
            if ((m.kind == MoveKind::Feed || isArc(m.kind)) && m.to.z < A.z - 1e-12 &&
                (A.x != m.to.x || A.y != m.to.y)) {
                bool allowed = p.ramp.type != RampType::None || p.tabs.threeD || p.corners.sharpInternal;
                if (!allowed) { ++st.descentFail; errs.push_back("sloped descent without ramp/3D tab"); break; }
            }
            if (m.kind == MoveKind::Feed && m.to.z < A.z - 1e-12 && A.x == m.to.x && A.y == m.to.y) {
                ++st.vertFail; errs.push_back("vertical feed downwards (should be plunge)"); break;
            }
            if (isArc(m.kind)) {
                double r0 = geometry::distance(A.xy(), m.centre), r1 = geometry::distance(m.to.xy(), m.centre);
                if (std::fabs(r0 - r1) > 1e-6) {
                    ++st.arcFail;
                    char b[120]; std::snprintf(b, sizeof b, "arc radius mismatch %.3g (r=%.4f)", r0 - r1, r0);
                    errs.push_back(b); break;
                }
            }
        }
        // every retract is followed by a rapid or ends the toolpath
        for (size_t i = 0; i + 1 < mv.size(); ++i)
            if (mv[i].kind == MoveKind::Retract && mv[i + 1].kind != MoveKind::Rapid) { ++st.rapidFail; errs.push_back("retract not followed by rapid"); break; }
        if (!mv.empty() && mv.back().kind != MoveKind::Retract) { ++st.rapidFail; errs.push_back("does not end with retract"); }
        // final depth reached
        if (!mv.empty()) {
            double zmin = 1e9;
            for (auto& m : mv) zmin = std::min(zmin, m.to.z);
            if (std::fabs(zmin - bottom) > 1e-9) { if (res.has(ProfileIssue::TabsNotPlaced)) errs.push_back("note: final depth not reached (TabsNotPlaced reported)"); else { ++st.zFail; errs.push_back("final depth not reached, no diagnostic"); } }
        }
        // ---- gouge + side
        if (p.side != ProfileSide::On && !mv.empty()) {
            Obstacles ob(cs);
            double thr = r + std::min(0.0, p.allowance) - tol;
            double worst = 1e9;
            bool side = false;
            int sideSamples = 0;
            Vec2 worstAt{};
            size_t worstMove = 0;
            sampleMoves(res.toolpath, 0.02, [&](Vec2 q, double z, size_t mi) {
                if (useV ? z >= 0 : z >= top - 1e-12) return;
                double thrz = useV ? std::min(-z * vTan, vD / 2) + std::min(0.0, p.allowance) - tol : thr;
                double d = ob.dist(q, r + 1);
                if (d - thrz < worst) { worst = d - thrz; worstAt = q; worstMove = mi; }
                if (crossingFree && !onlyOneOpen && kind != 4 && (++sideSamples % 10 == 0) && d > r * 0.5) {
                    int dep = ob.depth(q);
                    bool inRegion = dep % 2 == 1;
                    if ((p.side == ProfileSide::Outside && inRegion) || (p.side == ProfileSide::Inside && !inRegion)) side = true;
                }
            });
            if (onlyOneOpen) {
                // local side test against the open vector: right for Outside, left for Inside
                const Contour& c0 = cs[0];
                sampleMoves(res.toolpath, 0.2, [&](Vec2 q, double z, size_t) {
                    if (z >= top) return;
                    // find nearest point on c0 by dense sampling
                    double bd = 1e9, bt = 0; size_t bs = 0;
                    for (size_t si = 0; si < c0.spans.size(); ++si)
                        for (int k = 0; k <= 400; ++k) {
                            Vec2 pp = geometry::pointAt(c0.spans[si], k / 400.0);
                            double d = geometry::distance(pp, q);
                            if (d < bd) { bd = d; bt = k / 400.0; bs = si; }
                        }
                    if (bt < 0.01 || bt > 0.99) return;  // vertex wedges: sign test ambiguous
                    Vec2 pp = geometry::pointAt(c0.spans[bs], bt);
                    Vec2 tg = geometry::tangentAt(c0.spans[bs], bt);
                    double cr = tg.cross(q - pp);
                    if (bd > 0.5 * r && ((p.side == ProfileSide::Outside && cr > 0) || (p.side == ProfileSide::Inside && cr < 0))) side = true;
                });
            }
            st.worstMargin = std::min(st.worstMargin, worst);
            if (worst < 0) {
                bool soft = worst > -tol;
                if (p.corners.sharpInternal && !soft) { ++st.vGouge; errs.push_back("[V-bit sharp internal]"); } else
                (soft ? st.softGouge : st.gougeFail)++;
                char b[300];
                std::snprintf(b, sizeof b, "gouge margin %.6g at (%.4f,%.4f) move %zu kind %s", worst, worstAt.x, worstAt.y,
                              worstMove, kn(mv[worstMove].kind));
                errs.push_back(b);
            }
            if (side) { ++st.sideFail; errs.push_back("cut on wrong side"); }
        }
        // ---- blocks
        struct Blk { size_t a, b; };
        std::vector<Blk> bls;
        { size_t a = 0; for (size_t i = 0; i < mv.size(); ++i) { if (mv[i].kind == MoveKind::Rapid) a = i; if (mv[i].kind == MoveKind::Retract) bls.push_back({a, i}); } }
        // every block reaches the bottom with XY travel there
        for (auto& bl : bls) {
            bool ok = false;
            for (size_t k = bl.a + 1; k <= bl.b; ++k) if (mv[k].to.z == bottom && mv[k - 1].to.z == bottom && (isArc(mv[k].kind) || !(mv[k].to.x == mv[k - 1].to.x && mv[k].to.y == mv[k - 1].to.y))) { ok = true; break; }
            if (!ok) { ++st.bottomFail; errs.push_back("a cut never travels at the bottom"); break; }
        }
        // ---- tabs: plateau unions, count, notches
        if (p.tabs.enabled && !mv.empty() && p.tabs.thickness < p.passes.cutDepth - 1e-6) {
            const double zTab = bottom + p.tabs.thickness;
            auto key = [](Vec2 q) { return std::make_pair(std::llround(q.x * 1e5), std::llround(q.y * 1e5)); };
            using K = std::pair<long long, long long>;
            int placedTotal = 0;
            for (auto& bl : bls) {
                // plateau moves (deduplicated by endpoints, direction-free)
                std::set<std::pair<K, K>> segs;
                std::vector<std::pair<size_t, size_t>> segMoves;  // (i) move indices
                for (size_t k = bl.a + 1; k <= bl.b; ++k) {
                    if ((mv[k].kind == MoveKind::Feed || isArc(mv[k].kind)) && mv[k].to.z == zTab && mv[k - 1].to.z == zTab &&
                        !(mv[k].to.x == mv[k - 1].to.x && mv[k].to.y == mv[k - 1].to.y)) {
                        K k1 = key(mv[k - 1].to.xy()), k2 = key(mv[k].to.xy());
                        if (k2 < k1) std::swap(k1, k2);
                        if (segs.insert({k1, k2}).second) segMoves.push_back({k, 0});
                    }
                }
                // union-find over shared endpoints; degree per vertex
                std::map<K, int> deg;
                std::map<K, K> par;
                std::function<K(K)> fd = [&](K x) { auto it = par.find(x); if (it == par.end() || it->second == x) return x; K r2 = fd(it->second); par[x] = r2; return r2; };
                for (auto& sg : segs) { deg[sg.first]++; deg[sg.second]++; if (!par.count(sg.first)) par[sg.first] = sg.first; if (!par.count(sg.second)) par[sg.second] = sg.second; par[fd(sg.first)] = fd(sg.second); }
                std::set<K> roots;
                for (auto& [k, v] : par) roots.insert(fd(k));
                placedTotal += (int)roots.size();
                // descent below zTab at an interior plateau vertex (degree 2) or inside a plateau segment
                for (size_t k = bl.a + 1; k < bl.b; ++k) {
                    if (mv[k].to.z >= zTab - 1e-9) continue;
                    const Vec2 q = mv[k].to.xy();
                    std::vector<Vec2> dirs;
                    for (auto& [mi, unused] : segMoves) {
                        (void)unused;
                        Vec2 A = mv[mi - 1].to.xy(), B = mv[mi].to.xy();
                        for (int e = 0; e < 2; ++e) {
                            Vec2 P = e ? B : A;
                            if (geometry::distance(P, q) > 1e-6) continue;
                            Vec2 dvec;
                            if (isArc(mv[mi].kind)) {
                                Vec2 rr = P - mv[mi].centre;
                                Vec2 t = mv[mi].kind == MoveKind::ArcCCW ? Vec2{-rr.y, rr.x} : Vec2{rr.y, -rr.x};
                                dvec = e ? t * -1.0 : t;
                            } else dvec = e ? A - B : B - A;
                            dirs.push_back(dvec.normalized());
                        }
                    }
                    bool interior = false;
                    for (size_t x = 0; x < dirs.size(); ++x) for (size_t y = x + 1; y < dirs.size(); ++y) if (dirs[x].dot(dirs[y]) < 0.9) interior = true;
                    if (!interior) {
                        // inside a straight plateau segment (not at its end points)
                        for (auto& [mi, unused] : segMoves) {
                            (void)unused;
                            if (mv[mi].kind != MoveKind::Feed) continue;
                            Vec2 A = mv[mi - 1].to.xy(), B = mv[mi].to.xy(), q = mv[k].to.xy();
                            if (segDist(q, A, B) < 1e-7 && geometry::distance(q, A) > 1e-5 && geometry::distance(q, B) > 1e-5) { interior = true; break; }
                        }
                    }
                    if (interior) {
                        bool atStart = geometry::distance(mv[k].to.xy(), mv[bl.a].to.xy()) < 1e-6 || (k > bl.a + 1 && mv[k - 1].kind == MoveKind::Plunge);
                        ++st.notch;
                        if (atStart) ++st.notchStart;
                        char b[200]; std::snprintf(b, sizeof b, "tab notched: tool at z %.4f below tab top %.4f inside a raised stretch at (%.4f,%.4f) move %zu %s", mv[k].to.z, zTab, mv[k].to.x, mv[k].to.y, k, kn(mv[k].kind));
                        errs.push_back(b);
                        break;
                    }
                }
            }
            if (p.ramp.type == RampType::None && !p.separateLastPass && p.tabs.spacing == TabSpacing::ConstantNumber && !p.corners.sharpInternal) {
                int dropped = 0;
                for (auto& d : res.diagnostics) {
                    if (d.issue != ProfileIssue::TabsNotPlaced) continue;
                    auto pos = d.message.find(" tab(s) left out");
                    if (pos == std::string::npos) continue;
                    size_t b = d.message.rfind(": ", pos);
                    dropped += std::atoi(d.message.c_str() + b + 2);
                }
                ++st.tabChecked;
                if (placedTotal + dropped != p.tabs.count * (int)bls.size()) {
                    ++st.tabCount;
                    char b[200]; std::snprintf(b, sizeof b, "tabs placed %d + dropped %d != %d x %zu cuts", placedTotal, dropped, p.tabs.count, bls.size());
                    errs.push_back(b);
                }
            }
        }
        // ---- finishing order follows roughing order
        if (p.separateLastPass && bls.size() >= 2 && p.side != ProfileSide::On) {
            Obstacles ob(cs);
            std::vector<double> mind(bls.size(), 1e9);
            std::vector<std::vector<Vec2>> pts(bls.size());
            for (size_t bi = 0; bi < bls.size(); ++bi) {
                Toolpath sub{"", {}, {}};
                for (size_t k = bls[bi].a; k <= bls[bi].b; ++k) sub.moves.push_back(mv[k]);
                sampleMoves(sub, 0.25, [&](Vec2 q, double z, size_t) { if (z != bottom) return; pts[bi].push_back(q); mind[bi] = std::min(mind[bi], ob.dist(q, 1e9)); });
            }
            const double rf = r + p.allowance;
            std::vector<int> isFin(bls.size());
            for (size_t bi = 0; bi < bls.size(); ++bi) isFin[bi] = std::fabs(mind[bi] - rf) < 0.5 * p.lastPassAllowance;
            size_t firstFin = bls.size();
            for (size_t bi = 0; bi < bls.size(); ++bi) if (isFin[bi]) { firstFin = bi; break; }
            bool clean = true;
            for (size_t bi = firstFin; bi < bls.size(); ++bi) if (!isFin[bi]) clean = false;
            if (clean && firstFin > 0 && firstFin < bls.size()) {
                ++st.finChecked;
                int prevRank = -1;
                bool ok = true;
                for (size_t fi = firstFin; fi < bls.size(); ++fi) {
                    if (pts[fi].empty()) continue;
                    Vec2 q = pts[fi].front();
                    double best = 1e18; int rank = -1;
                    for (size_t ri = 0; ri < firstFin; ++ri) for (auto& pp : pts[ri]) { double d = geometry::distance(pp, q); if (d < best) { best = d; rank = (int)ri; } }
                    if (rank < prevRank) ok = false;
                    prevRank = std::max(prevRank, rank);
                }
                if (!ok) { ++st.finOrder; errs.push_back("finishing order differs from roughing order"); }
            }
        }
        // ---- StartMoved only when needed
        if (res.has(ProfileIssue::StartMoved) && !p.separateLastPass && p.side != ProfileSide::On && p.leads.type != LeadType::None) {
            ++st.startMoved;
            ProfileParams q = p;
            q.leads.type = LeadType::None;
            q.leads.leadOut = false;
            auto r0 = profileToolpath(cs, tool, q);
            std::vector<std::pair<Vec2, Vec2>> starts;  // start, tangent
            const auto& m0 = r0.toolpath.moves;
            for (size_t k = 0; k + 2 < m0.size(); ++k) {
                if (m0[k].kind != MoveKind::Rapid) continue;
                size_t j = k + 1;
                while (j < m0.size() && m0[j].to.x == m0[k].to.x && m0[j].to.y == m0[k].to.y) ++j;
                if (j >= m0.size()) break;
                Vec2 a = m0[k].to.xy(), t;
                if (isArc(m0[j].kind)) { Vec2 rr = a - m0[j].centre; t = m0[j].kind == MoveKind::ArcCCW ? Vec2{-rr.y, rr.x} : Vec2{rr.y, -rr.x}; }
                else t = m0[j].to.xy() - a;
                starts.push_back({a, t.normalized()});
            }
            // which of these starts are not the start of any with-lead cut?
            Obstacles ob(cs);
            const bool wasteLeft = p.direction == CutDirection::Climb;
            for (auto& [a, t] : starts) {
                bool kept = false;
                for (auto& bl : bls) for (size_t k = bl.a; k <= std::min(bl.b, bl.a + 60); ++k) if (geometry::distance(mv[k].to.xy(), a) < 1e-6) kept = true;
                if (kept) continue;
                std::vector<LeadOptions> vars{p.leads};
                LeadOptions alt = p.leads;
                if (alt.type == LeadType::Straight) { alt.angle = PI / 2; vars.push_back(alt); } else { alt.sweep *= 0.5; vars.push_back(alt); }
                bool fits = false;
                double bestMargin = -1e9;
                for (auto& o : vars) for (double scl : {1.0, 0.5, 0.25, 0.125}) {
                    Contour lead = leadIn(a, t, wasteLeft, o, scl);
                    if (lead.isEmpty()) continue;
                    Toolpath lt{"", {}, {}};
                    lt.moves.push_back(Move::rapid(toolpath::Vec3{lead.startPoint(), -1}));
                    for (auto& sp : lead.spans) {
                        if (auto* l = std::get_if<geometry::LineSpan>(&sp)) lt.moves.push_back(Move::feed(toolpath::Vec3{l->end, -1}));
                        else if (auto* ar = std::get_if<geometry::ArcSpan>(&sp)) { Move mm = Move::feed(toolpath::Vec3{geometry::endPoint(sp), -1}); mm.kind = ar->direction == geometry::ArcDirection::CCW ? MoveKind::ArcCCW : MoveKind::ArcCW; mm.centre = ar->centre; lt.moves.push_back(mm); }
                    }
                    double mn = 1e9;
                    sampleMoves(lt, 0.01, [&](Vec2 qq, double, size_t) { mn = std::min(mn, ob.dist(qq, r + 1) - r); });
                    bestMargin = std::max(bestMargin, mn);
                    if (mn > 1e-3) fits = true;
                }
                if (fits) { ++st.startMovedUnneeded; char b[200]; std::snprintf(b, sizeof b, "StartMoved although a lead fits at the original start (%.4f,%.4f), margin %.4g", a.x, a.y, bestMargin); errs.push_back(b); }
            }
        }
        if (!errs.empty() || verbose) {
            std::printf("scene %d kind %d side %d dir %d tool %s%.2f allow %.3f sep %d tabs %d(3d %d) ramp %d lead %d start %d moves %zu diags:",
                        sc, kind, (int)p.side, (int)p.direction, useV ? "V" : "EM", toolD, p.allowance, p.separateLastPass,
                        p.tabs.enabled, p.tabs.threeD, (int)p.ramp.type, (int)p.leads.type, (int)p.startAt, mv.size());
            for (auto& d : res.diagnostics) std::printf(" %d", (int)d.issue);
            std::printf("\n");
            for (auto& e : errs) std::printf("   - %s\n", e.c_str());
            if (verbose) { for (auto& d : res.diagnostics) std::printf("   diag %d %s\n", (int)d.issue, d.message.c_str());
              std::printf("   V %d ang %.0f vD %.2f sharpInt %d sharpExt %d avoid %d spacing %d count %d dist %.3f startDepth %.3f lead len %.3f ang %.3f rad %.3f sweep %.3f out %d overcut %.3f\n", useV, vAng, vD, p.corners.sharpInternal, p.corners.sharpExternal, p.tabs.avoidCorners, (int)p.tabs.spacing, p.tabs.count, p.tabs.distance, p.passes.startDepth, p.leads.length, p.leads.angle, p.leads.radius, p.leads.sweep, p.leads.leadOut, p.leads.overcut);
              std::printf("   bottom %.4f top %.4f cut %.4f pass %.4f thick %.4f len %.3f r %.4f\n", bottom, top, p.passes.cutDepth, p.passes.passDepth, p.tabs.thickness, p.tabs.length, r);
              for (auto& c : cs) { std::printf("   contour closed=%d spans=%zu start(%.4f,%.4f)\n", c.closed, c.spans.size(), c.startPoint().x, c.startPoint().y); for (auto& s2: c.spans) { auto a=geometry::startPoint(s2), b=geometry::endPoint(s2); std::printf("     span %zu (%.4f,%.4f)->(%.4f,%.4f)\n", s2.index(), a.x,a.y,b.x,b.y);} }
              FILE* f = std::fopen("replay.moves", "w"); dump(res.toolpath, f); std::fclose(f); }
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    std::printf("seed %lu EXTRA: vGouge(sharpInt) %d | bottomFail %d | tabCount %d/%d | notch %d (at start %d) | finOrder %d/%d | startMoved %d unneeded %d\n", seed, st.vGouge, st.bottomFail, st.tabCount, st.tabChecked, st.notch, st.notchStart, st.finOrder, st.finChecked, st.startMoved, st.startMovedUnneeded);
    std::printf("seed %lu: scenes %d empty %d | gouge %d (soft<tol %d) side %d z %d rapid %d vert %d descent %d arc %d det %d diag %d tab %d | worst margin %.6g | %.1fs\n",
                seed, st.scenes, st.empty, st.gougeFail, st.softGouge, st.sideFail, st.zFail, st.rapidFail, st.vertFail,
                st.descentFail, st.arcFail, st.detFail, st.diagFail, st.tabFail, st.worstMargin,
                std::chrono::duration<double>(t1 - t0).count());
    return 0;
}
