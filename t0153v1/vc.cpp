// T-0153 verifier: independent V-carve oracle (does not use core closestPoint or the branch's oracle).
// usage: vc <seed> <N> [verbose]
// For each random case: (1) certified cone/wall check on every non-rapid move of the carving tool
// (dist(centre, boundary) - footprint(depth) >= -eps, interval subdivision, exact span distances from
// oracle.hpp); (2) clearance end mills: dist - R - limitRadius >= -eps and dist - R >= -eps;
// (3) structure: rapids at safe Z, plunge/retract vertical, no lateral move above the start plane
// other than at safe Z, depths within the limit; (4) depth/coverage: at random interior points the
// carved depth (max over densely sampled tool positions of all tools) vs the ideal
// min(limit, finv(dist)), reported as lateral shortfall in units of chordal.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>
#include "oracle.hpp"
#include "shapes.hpp"
#include "carvetto/toolpath/vcarve.hpp"
using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Contour; using geometry::Vec2;
static std::mt19937_64 rng; static bool CROSS = getenv("VCCROSS") != nullptr;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

struct TS { int type; double R; double tanh; }; // type 0 vbit 1 ball 2 endmill
static double fp(const TS& t, double d) { // footprint radius at depth d below start plane
    if (d <= 0) return 0;
    if (t.type == 0) return std::min(t.R, d * t.tanh);
    if (t.type == 1) { if (d >= t.R) return t.R; return std::sqrt(d * (2 * t.R - d)); }
    return t.R;
}
static double finv(const TS& t, double s) { // depth whose footprint is s (vbit/ball)
    if (s <= 0) return 0;
    if (t.type == 0) return std::min(s, t.R) / t.tanh;
    if (s >= t.R) return t.R;
    return t.R - std::sqrt(t.R * t.R - s * s);
}
static double fullDepth(const TS& t) { return t.type == 0 ? t.R / t.tanh : t.R; }

static tooling::Tool mk(int type, double D, double angDeg, const std::string& name, int num) {
    tooling::Tool t; t.id = name; t.name = name; t.diameter = D;
    t.type = type == 0 ? tooling::ToolType::VBit : type == 1 ? tooling::ToolType::BallNose : tooling::ToolType::EndMill;
    if (type == 0) t.includedAngle = angDeg * vo::PI / 180;
    auto& c = t.defaultCuttingData; c.toolNumber = num; c.feedRate = 1000; c.plungeRate = 300; c.spindleSpeed = 18000; c.passDepth = U(0.5, 3); c.stepover = 0.4 * D;
    return t;
}

static Contour xform(const Contour& c, double s, double ang, Vec2 off) {
    Contour o; o.closed = c.closed; double ca = std::cos(ang), sa = std::sin(ang);
    auto P = [&](Vec2 p) { return Vec2{off.x + s * (ca * p.x - sa * p.y), off.y + s * (sa * p.x + ca * p.y)}; };
    for (auto& sp : c.spans) {
        if (auto* l = std::get_if<geometry::LineSpan>(&sp)) o.append(geometry::LineSpan{P(l->start), P(l->end)});
        else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { geometry::ArcSpan b = *a; b.centre = P(a->centre); b.radius = a->radius * s; b.startAngle += ang; b.endAngle += ang; o.append(b); }
        else { auto b = std::get<geometry::BezierSpan>(sp); o.append(geometry::BezierSpan{P(b.p0), P(b.p1), P(b.p2), P(b.p3)}); }
    }
    return o;
}

static std::vector<Contour> genShape(std::string& name) {
    static auto lib = vs::all();
    int k = I(0, 9);
    std::vector<Contour> cs;
    if (k <= 3) { auto& s = lib[I(0, (int)lib.size() - 1)]; name = s.name; double sc = U(0.2, 1.5), an = U(0, 2 * vo::PI); Vec2 o{U(-50, 50), U(-50, 50)}; for (auto& c : s.cs) cs.push_back(xform(c, sc, an, o)); }
    else if (k == 4) { name = "star"; int n = I(3, 12); double R = U(2, 20); cs.push_back(vs::star({U(-20, 20), U(-20, 20)}, R, R * U(0.2, 0.9), n, U(0, 6))); }
    else if (k == 5) { name = "holedstar"; int n = I(4, 9); double R = U(8, 20); Vec2 o{U(-20, 20), U(-20, 20)}; cs.push_back(vs::star(o, R, R * U(0.5, 0.9), n, U(0, 6))); cs.push_back(vs::rev(vs::star(o, R * 0.35, R * U(0.1, 0.3), I(3, 7), U(0, 6)))); }
    else if (k == 6) { name = "thinstrip"; double w = U(0.02, 1.0), L = U(5, 40); cs.push_back(xform(vs::poly({{0, 0}, {L, 0}, {L, w}, {0, w}}), 1, U(0, 6), {U(-9, 9), U(-9, 9)})); }
    else if (k == 7) { name = "ringneigh"; Vec2 o{U(-10, 10), U(-10, 10)}; double R = U(4, 12); cs.push_back(vs::circle(o, R)); cs.push_back(vs::rev(vs::circle({o.x + U(-1, 1), o.y + U(-1, 1)}, R * U(0.3, 0.8)))); { double rn = U(1, 4); cs.push_back(vs::circle({o.x + R + (CROSS ? U(-0.5, 0.5) * rn : rn + U(0.05, 3)), o.y}, rn)); } }
    else if (k == 8) { name = "text"; double s = U(3, 12); double x = 0; int n = I(2, 5); for (int i = 0; i < n; ++i) { int g = I(0, 3); auto& sh = lib[g == 0 ? 17 : g == 1 ? 15 : g == 2 ? 16 : 14];
            std::vector<Contour> gl; for (auto& c : sh.cs) gl.push_back(xform(c, s / 20.0, 0, {0, 0})); double mnx = 1e300, mxx = -1e300; for (auto& c : gl) { auto b = c.bounds(); mnx = std::min(mnx, b.min.x); mxx = std::max(mxx, b.max.x); }
            for (auto& c : gl) cs.push_back(xform(c, 1, 0, {x - mnx, 0})); x += (mxx - mnx) + (CROSS ? U(-0.3, 0.3) * s : U(0.05, 0.4) * s); } }
    else { name = "bezblob"; Contour c; c.closed = true; int n = I(3, 6); Vec2 o{U(-10, 10), U(-10, 10)}; std::vector<Vec2> p; for (int i = 0; i < n; ++i) { double a = 2 * vo::PI * i / n; double r = U(4, 12); p.push_back({o.x + r * std::cos(a), o.y + r * std::sin(a)}); }
        for (int i = 0; i < n; ++i) { Vec2 a = p[i], b = p[(i + 1) % n]; Vec2 m = (a + b) * 0.5; Vec2 d = b - a; Vec2 nn{-d.y, d.x}; c.append(geometry::BezierSpan{a, a + d * 0.3 + nn * U(-0.3, 0.6), b - d * 0.3 + nn * U(-0.3, 0.6), b}); (void)m; } cs.push_back(c); }
    return cs;
}

static void dump(const std::vector<Contour>& cs) {
    for (size_t i = 0; i < cs.size(); ++i) { std::printf("  contour %zu closed=%d\n", i, (int)cs[i].closed);
        for (auto& sp : cs[i].spans) {
            if (auto* l = std::get_if<geometry::LineSpan>(&sp)) std::printf("    L %.17g %.17g %.17g %.17g\n", l->start.x, l->start.y, l->end.x, l->end.y);
            else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) std::printf("    A %.17g %.17g r %.17g %.17g %.17g dir %d\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, (int)a->direction);
            else { auto b = std::get<geometry::BezierSpan>(sp); std::printf("    B %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", b.p0.x, b.p0.y, b.p1.x, b.p1.y, b.p2.x, b.p2.y, b.p3.x, b.p3.y); }
        } }
}
static bool segX(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    auto o = [](Vec2 p, Vec2 q, Vec2 r) { return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x); };
    double d1 = o(a, b, c), d2 = o(a, b, d), d3 = o(c, d, a), d4 = o(c, d, b);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}
// true when any two boundary polylines (or one with itself, non-adjacent) cross
static bool crossing(const std::vector<Contour>& cs) {
    vo::Boundary B = vo::makeBoundary(cs, 0.05);
    std::vector<std::pair<Vec2, Vec2>> segs; std::vector<std::pair<int, int>> id;
    for (int c = 0; c < (int)B.poly.size(); ++c) { auto& p = B.poly[c]; for (size_t i = 0; i < p.size(); ++i) { segs.push_back({p[i], p[(i + 1) % p.size()]}); id.push_back({c, (int)i}); } }
    for (size_t i = 0; i < segs.size(); ++i) for (size_t j = i + 1; j < segs.size(); ++j) {
        if (id[i].first == id[j].first) { int n = (int)B.poly[id[i].first].size(); int di = std::abs(id[i].second - id[j].second); if (di <= 1 || di == n - 1) continue; }
        if (segX(segs[i].first, segs[i].second, segs[j].first, segs[j].second)) return true; }
    return false;
}
struct Samp { double x, y, d; int tool; };
int main(int argc, char** argv) {
    unsigned long seed = strtoul(argv[1], nullptr, 10); int N = atoi(argv[2]); bool verbose = argc > 3;
    rng.seed(seed);
    double worstLatMode[3] = {-1e300, -1e300, -1e300}; double worstCone = 1e300, worstEM = 1e300, worstEMa = 1e300, worstLat = -1e300, worstOver = -1e300, worstFlatShort = -1e300;
    int skippedCross = 0; int cases = 0, okc = 0, fails = 0, refused = 0;
    std::map<std::string, int> diagCount;
    for (int ci = 0; ci < N; ++ci) {
        std::string sname; std::vector<Contour> cs = genShape(sname);
        int tt = I(0, 3) == 0 ? 1 : 0; double ang = U(20, 160); double D = tt == 0 ? U(1, 25) : U(0.5, 12);
        tooling::Tool tool = mk(tt, D, ang, "Carve", 1);
        TS ts{tt, D / 2, tt == 0 ? std::tan(tool.includedAngle / 2) : 0};
        VCarveParams p; p.safeZ = U(1, 10); p.startDepth = I(0, 2) == 0 ? U(0, 3) : 0;
        double ch[] = {0.005, 0.01, 0.02, 0.05, 0.1}; p.tolerance.chordal = ch[I(0, 4)]; p.tolerance.distance = std::min(p.tolerance.distance, p.tolerance.chordal);
        int fm = I(0, 2); if (fm > 0) p.flatDepth = U(0.1, 1.2) * fullDepth(ts);
        std::vector<PocketTool> cl; int ncl = fm == 2 ? I(1, 2) : 0;
        double emD[2] = {0, 0};
        for (int i = 0; i < ncl; ++i) { emD[i] = i == 0 ? U(1, 8) : U(0.3, emD[0]); cl.push_back(PocketTool{mk(2, emD[i], 0, "EM" + std::to_string(i), 2 + i), std::nullopt}); }
        if (fm == 1) tool.defaultCuttingData.stepover = U(0.2, 0.9) * 2 * fp(ts, std::min(p.flatDepth, fullDepth(ts)));
        p.clearanceStrategy = I(0, 1) ? PocketStrategy::Raster : PocketStrategy::Offset; p.clearanceRasterAngle = U(0, 3); p.rampPlunge = I(0, 3) == 0;
        if (!CROSS && crossing(cs)) { ++skippedCross; continue; }
        ++cases;
        VCarveResult r = vcarveToolpath(cs, tool, std::nullopt, cl, p);
        for (auto& d : r.diagnostics) diagCount[std::to_string((int)d.issue)]++;
        if (!r.ok()) { ++refused; if (verbose) { std::printf("case %d %s refused/empty:", ci, sname.c_str()); for (auto& d : r.diagnostics) std::printf(" [%d %s]", (int)d.issue, d.message.c_str()); std::printf("\n"); } continue; }
        ++okc;
        double limit = std::min(p.flatDepth > 0 ? p.flatDepth : 1e300, fullDepth(ts));
        double limR = fp(ts, limit);
        vo::Boundary B = vo::makeBoundary(cs, p.tolerance.chordal * 0.5);
        auto dist = [&](Vec2 q) { return vo::exactDist(B, q).d; };
        const double eps = 1e-6;
        bool bad = false; std::string why;
        std::vector<Samp> samples;
        auto tps = r.toolpaths();
        if (getenv("VCMUT")) { double f = atof(getenv("VCMUT")); for (auto& m : tps.back().moves) if (m.to.z < -p.startDepth) m.to.z = -p.startDepth + (m.to.z + p.startDepth) * f; } // oracle sanity: deepen the carve
        for (size_t ti = 0; ti < tps.size(); ++ti) {
            const Toolpath& tp = tps[ti];
            bool isCarve = ti + 1 == tps.size();
            TS tsx = ts; if (!isCarve) { int idx = -1; for (int i = 0; i < ncl; ++i) if (tp.tool.diameter == emD[i]) idx = i; tsx = TS{2, emD[std::max(idx, 0)] / 2, 0}; }
            geometry::Vec3 at{p.startPosition.x, p.startPosition.y, p.safeZ}; bool first = true;
            for (const Move& m : tp.moves) {
                geometry::Vec3 a = at, b = m.to;
                if (first && m.kind != MoveKind::Rapid) { bad = true; why = "first move not rapid"; }
                first = false;
                if (m.kind == MoveKind::Rapid) { if (std::fabs(b.z - p.safeZ) > 1e-9 || (std::fabs(a.z - p.safeZ) > 1e-9 && (a.x != b.x || a.y != b.y))) { bad = true; why = "rapid not at safe Z"; } at = b; continue; }
                if (m.kind == MoveKind::ArcCW || m.kind == MoveKind::ArcCCW || m.kind == MoveKind::Dwell) { bad = true; why = "arc/dwell"; at = b; continue; }
                if ((m.kind == MoveKind::Plunge || m.kind == MoveKind::Retract) && (a.x != b.x || a.y != b.y)) { bad = true; why = "plunge/retract not vertical"; }
                if (m.kind == MoveKind::Plunge && b.z > a.z) { bad = true; why = "plunge up"; }
                if (m.kind == MoveKind::Retract && b.z < a.z) { bad = true; why = "retract down"; }
                double da = -a.z - p.startDepth, db = -b.z - p.startDepth;
                bool lateral = a.x != b.x || a.y != b.y;
                if (lateral && std::max(a.z, b.z) > -p.startDepth + 1e-9 && !(std::fabs(a.z - p.safeZ) < 1e-9 && std::fabs(b.z - p.safeZ) < 1e-9)) { bad = true; why = "lateral feed above start plane z=" + std::to_string(std::max(a.z, b.z)); }
                double maxD = isCarve ? limit : limit; if (std::max(da, db) > maxD + 1e-9) { bad = true; why = "deeper than limit " + std::to_string(std::max(da, db)) + " > " + std::to_string(maxD); }
                // certified subdivision
                double Lxy = std::hypot(b.x - a.x, b.y - a.y);
                auto depthAt = [&](double t) { return da + (db - da) * t; };
                auto cen = [&](double t) { return Vec2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}; };
                double need = isCarve ? 0 : 0; (void)need;
                std::vector<std::pair<double, double>> stk{{0, 1}}; int iters = 0;
                double localMin = 1e300, localMinA = 1e300;
                while (!stk.empty() && iters < 2000000) {
                    ++iters; auto [t0, t1] = stk.back(); stk.pop_back();
                    double dmax = std::max(depthAt(t0), depthAt(t1)); if (dmax <= 1e-12) continue;
                    double tm = 0.5 * (t0 + t1); double dc = dist(cen(tm)); double half = 0.5 * Lxy * (t1 - t0);
                    double lb, val;
                    if (isCarve) { lb = dc - half - fp(tsx, dmax); val = dc - fp(tsx, depthAt(tm)); }
                    else { lb = dc - half - tsx.R - limR; val = dc - tsx.R - limR; }
                    localMin = std::min(localMin, val);
                    if (!isCarve) localMinA = std::min(localMinA, dc - tsx.R);
                    if (lb >= -eps) continue;
                    if (val < -eps) break; // a real violation found: stop refining this move
                    if (t1 - t0 < 1e-9 || half < 1e-10) { localMin = std::min(localMin, lb); continue; }
                    stk.push_back({t0, tm}); stk.push_back({tm, t1});
                }
                if (isCarve) { worstCone = std::min(worstCone, localMin); if (localMin < -eps) { bad = true; why = "CONE VIOLATION " + std::to_string(localMin) + " at move to (" + std::to_string(b.x) + "," + std::to_string(b.y) + "," + std::to_string(b.z) + ")"; } }
                else { worstEMa = std::min(worstEMa, localMin); if (localMinA < 1e299) worstEM = std::min(worstEM, localMinA); if (localMin < -eps) { bad = true; why = "ENDMILL inside allowance " + std::to_string(localMin); } }
                // inside region check (centre at depth must be inside)
                if (std::max(da, db) > 1e-6 && !vo::insideEO(B, cen(0.5)) && dist(cen(0.5)) > 1e-3) { bad = true; char buf[256]; std::snprintf(buf, sizeof buf, "centre outside region tool %zu kind %d (%.4f,%.4f,%.4f)->(%.4f,%.4f,%.4f) dist %.4f", ti, (int)m.kind, a.x,a.y,a.z,b.x,b.y,b.z, dist(cen(0.5))); why = buf; }
                // samples for depth
                int ns = std::max(1, (int)std::ceil(Lxy / (p.tolerance.chordal / 8)));
                for (int i = 0; i <= ns; ++i) { double t = double(i) / ns; double d = depthAt(t); if (d > 1e-9) { Vec2 c = cen(t); samples.push_back({c.x, c.y, d, isCarve ? -1 : (int)ti}); } }
                at = b;
            }
            if (std::fabs(at.z - p.safeZ) > 1e-9) { bad = true; why = "does not end at safe Z"; }
        }
        // depth/coverage at random interior points
        double cell = std::max(0.5, std::max(limR, (ncl ? emD[0] / 2 : 0.0)) + 1e-9);
        std::unordered_map<long long, std::vector<int>> grid;
        auto key = [&](long long ix, long long iy) { return ix * 1000003LL + iy; };
        for (int i = 0; i < (int)samples.size(); ++i) grid[key((long long)std::floor(samples[i].x / cell), (long long)std::floor(samples[i].y / cell))].push_back(i);
        double caseLat = -1e300, caseOver = -1e300, caseFlat = -1e300; Vec2 latAt{0,0}; double latD = 0, latI = 0;
        for (int k = 0; k < 400; ++k) {
            Vec2 q{U(B.minx, B.maxx), U(B.miny, B.maxy)};
            if (!vo::insideEO(B, q)) continue;
            double dq = dist(q);
            double ideal = std::min(limit, finv(ts, dq));
            double Dq = 0;
            long long ix = (long long)std::floor(q.x / cell), iy = (long long)std::floor(q.y / cell);
            for (long long gx = ix - 1; gx <= ix + 1; ++gx) for (long long gy = iy - 1; gy <= iy + 1; ++gy) {
                auto it = grid.find(key(gx, gy)); if (it == grid.end()) continue;
                for (int si : it->second) { const Samp& s = samples[si]; double rr = std::hypot(q.x - s.x, q.y - s.y);
                    double cut = -1;
                    if (s.tool < 0) { if (rr <= fp(ts, s.d)) { cut = ts.type == 0 ? s.d - rr / ts.tanh : s.d - (ts.R - std::sqrt(std::max(0.0, ts.R * ts.R - rr * rr))); } }
                    else { const Toolpath& tp = tps[s.tool]; if (rr <= tp.tool.diameter / 2) cut = s.d; }
                    Dq = std::max(Dq, cut); }
            }
            if (ts.type == 0) caseOver = std::max(caseOver, Dq - ideal);
            if (ideal < limit - 1e-9) { double lat = dq - fp(ts, Dq); if (lat / p.tolerance.chordal > caseLat) { caseLat = lat / p.tolerance.chordal; latAt = q; latD = Dq; latI = ideal; } }
            else { caseFlat = std::max(caseFlat, (limit - Dq)); }
        }
        worstLat = std::max(worstLat, caseLat); worstLatMode[fm] = std::max(worstLatMode[fm], caseLat); worstOver = std::max(worstOver, caseOver); worstFlatShort = std::max(worstFlatShort, caseFlat);
        bool covBad = (fm == 0 && caseLat > 1.2) || caseLat > 3.0 || caseOver > 1e-4;
        if (bad || covBad || verbose) std::printf("case %d seed %lu %s tool=%d D=%.3f ang=%.1f chord=%.3f flat=%.3f start=%.3f ncl=%d emD=%.3f/%.3f strat=%d: %s lat=%.3f*ch at (%.4f,%.4f) D=%.4f ideal=%.4f over=%.2e flatShort=%.3f\n", ci, seed, sname.c_str(), tt, D, ang, p.tolerance.chordal, p.flatDepth, p.startDepth, ncl, emD[0], emD[1], (int)p.clearanceStrategy, bad ? why.c_str() : "ok", caseLat, latAt.x, latAt.y, latD, latI, caseOver, std::max(caseFlat, -1.0));
        if ((bad || covBad) && getenv("VCDUMP")) dump(cs);
        if ((bad || covBad) && getenv("VCMOVES")) { std::printf("  stepover=%.4f limR=%.4f dist(latAt)=%.4f\n", tool.defaultCuttingData.stepover, limR, dist(latAt));
            for (size_t ti = 0; ti < tps.size(); ++ti) { geometry::Vec3 at{0, 0, 0}; for (auto& m : tps[ti].moves) { if (m.kind != MoveKind::Rapid && std::hypot(m.to.x - latAt.x, m.to.y - latAt.y) < 2.5 * limR) std::printf("   tp%zu k%d (%.4f,%.4f,%.4f)->(%.4f,%.4f,%.4f)\n", ti, (int)m.kind, at.x, at.y, at.z, m.to.x, m.to.y, m.to.z); at = m.to; } } }
        if (bad || covBad) ++fails;
    }
    std::printf("skipped crossing inputs: %d\n", skippedCross); std::printf("SUMMARY seed=%lu cases=%d ok=%d refused=%d fails=%d worstCone=%.3e worstEMallow=%.3e worstEMwall=%.3e worstLat=%.3f*ch worstOver=%.2e worstFlatShort=%.3f\n", seed, cases, okc, refused, fails, worstCone, worstEMa, worstEM, worstLat, worstOver, worstFlatShort);
    std::printf("  worst lateral shortfall by mode (no flat / flat self-clear / flat + clearance tools): %.3f %.3f %.3f *chordal\n", worstLatMode[0], worstLatMode[1], worstLatMode[2]);
    for (auto& [k, v] : diagCount) std::printf("  issue %s: %d\n", k.c_str(), v);
}
