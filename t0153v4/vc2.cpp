// T-0153 verifier round 2: independent V-carve oracle over gen2.hpp families (crossing input allowed).
// usage: vc2 <seed> <N> [verbose]   env: VC_FAM=<id> (default random 0..15), VC_FAR=<offset mm>
// VC_FILE=<contours in the dump format> (every case uses that input, random tools/params),
// VCMUT=<f> scales the carve depths by f (oracle sanity: f > 1 must report cone violations),
// VC_FIX="<type> <D> <angle> <chordal> <flat> <start>" fixes the carving tool and parameters (no clearance tools),
// (translate every case by (off, -0.7 off)), VC_SCALE=<s> (scale every case), VC_PTS=<coverage points>.
// Checks per case, against the ORIGINAL curves (exact span distances, oracle.hpp):
//  cone: every non-rapid carving move, certified subdivision: dist(centre, all curves) - footprint(depth) >= -1e-6,
//        and the deepest point of every move is inside the even-odd region. dist to ALL curves <= dist to the true
//        even-odd boundary, so this is conservative; a flagged move is re-checked with a witness search (points
//        of the footprint disc outside the region) to separate coincident-edge false alarms from real ones.
//  clearance end mills: dist - R - limitR >= -1e-6.  structure: rapids at safe Z, vertical plunge/retract,
//  no lateral feed above the start plane, depths within the limit.
//  coverage: at random region points, lateral shortfall dist - footprint(carved depth) for points whose ideal
//  depth is above the limit (reported by bucket: all, near the clamp kink), and flat floor shortfall.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include "gen2.hpp"
#include <fstream>
#include <sstream>
static std::vector<carvetto::geometry::Contour> loadFile(const char* path) { using namespace carvetto; std::vector<geometry::Contour> cs; std::ifstream in(path); std::string line;
    while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
        else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(geometry::LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double x, y, r, s0, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s0 >> e >> dd >> dir; cs.back().append(geometry::ArcSpan{{x, y}, r, s0, e, dir ? geometry::ArcDirection::CW : geometry::ArcDirection::CCW}); }
        else if (t == "B") { double v[8]; for (double& q : v) is >> q; cs.back().append(geometry::BezierSpan{{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}}); } }
    return cs; }
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
using geometry::Contour; using geometry::Vec2; using g2::U; using g2::I;
struct TS { int type; double R; double tanh; };
static double fp(const TS& t, double d) { if (d <= 0) return 0; if (t.type == 0) return std::min(t.R, d * t.tanh); if (t.type == 1) { if (d >= t.R) return t.R; return std::sqrt(d * (2 * t.R - d)); } return t.R; }
static double finv(const TS& t, double s) { if (s <= 0) return 0; if (t.type == 0) return std::min(s, t.R) / t.tanh; if (s >= t.R) return t.R; return t.R - std::sqrt(t.R * t.R - s * s); }
static double fullDepth(const TS& t) { return t.type == 0 ? t.R / t.tanh : t.R; }
static tooling::Tool mk(int type, double D, double angDeg, const std::string& name, int num) {
    tooling::Tool t; t.id = name; t.name = name; t.diameter = D;
    t.type = type == 0 ? tooling::ToolType::VBit : type == 1 ? tooling::ToolType::BallNose : tooling::ToolType::EndMill;
    if (type == 0) t.includedAngle = angDeg * vo::PI / 180;
    auto& c = t.defaultCuttingData; c.toolNumber = num; c.feedRate = 1000; c.plungeRate = 300; c.spindleSpeed = 18000; c.passDepth = U(0.5, 3); c.stepover = 0.4 * D;
    return t;
}
static void dump(const std::vector<Contour>& cs) {
    for (size_t i = 0; i < cs.size(); ++i) { std::printf("contour %zu closed=%d\n", i, (int)cs[i].closed);
        for (auto& sp : cs[i].spans) {
            if (auto* l = std::get_if<geometry::LineSpan>(&sp)) std::printf("    L %.17g %.17g %.17g %.17g\n", l->start.x, l->start.y, l->end.x, l->end.y);
            else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) std::printf("    A %.17g %.17g r %.17g %.17g %.17g dir %d\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, (int)a->direction);
            else { auto b = std::get<geometry::BezierSpan>(sp); std::printf("    B %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", b.p0.x, b.p0.y, b.p1.x, b.p1.y, b.p2.x, b.p2.y, b.p3.x, b.p3.y); }
        } }
}
struct Samp { double x, y, d; int tool; };
int main(int argc, char** argv) {
    unsigned long seed = strtoul(argv[1], nullptr, 10); int N = atoi(argv[2]); bool verbose = argc > 3;
    g2::rng.seed(seed);
    int famFix = getenv("VC_FAM") ? atoi(getenv("VC_FAM")) : -1;
    double far = getenv("VC_FAR") ? atof(getenv("VC_FAR")) : 0.0; double scl = getenv("VC_SCALE") ? atof(getenv("VC_SCALE")) : 1.0;
    int npts = getenv("VC_PTS") ? atoi(getenv("VC_PTS")) : 1500;
    double worstCone = 1e300, worstEMa = 1e300, worstLat[2] = {-1e300, -1e300}, worstKink[2] = {-1e300, -1e300}, worstOver = -1e300, worstFlat = -1e300, worstFlatSelf = -1e300;
    int cases = 0, okc = 0, fails = 0, refused = 0, realViol = 0, falseAlarm = 0, resolvedCases = 0, startPlaneOutside = 0;
    std::map<std::string, int> diagCount, famFails;
    for (int ci = 0; ci < N; ++ci) {
        int fam = famFix >= 0 ? famFix : I(0, 15);
        double ch[] = {0.005, 0.01, 0.02, 0.05, 0.1}; double chord = ch[I(0, 4)];
        std::string sname; std::vector<Contour> cs = getenv("VC_FILE") ? (sname = getenv("VC_FILE"), loadFile(getenv("VC_FILE"))) : g2::gen(fam, chord, sname);
        if (far != 0.0 || scl != 1.0) for (auto& c : cs) c = g2::xform(c, scl, 0, {far, -0.7 * far});
        int tt = I(0, 3) == 0 ? 1 : 0; double ang = U(20, 160); double D = (tt == 0 ? U(1, 25) : U(0.5, 12)) * scl;
        tooling::Tool tool = mk(tt, D, ang, "Carve", 1);
        TS ts{tt, D / 2, tt == 0 ? std::tan(tool.includedAngle / 2) : 0};
        VCarvingParams p; p.safeZ = U(1, 10); p.startDepth = I(0, 2) == 0 ? U(0, 3) : 0;
        p.tolerance.chordal = chord; p.tolerance.distance = std::min(p.tolerance.distance, chord);
        int fm = I(0, 2); if (fm > 0) p.flatDepth = U(0.1, 1.2) * fullDepth(ts);
        std::vector<PocketTool> cl; int ncl = fm == 2 ? I(1, 2) : 0; double emD[2] = {0, 0};
        for (int i = 0; i < ncl; ++i) { emD[i] = i == 0 ? U(1, 8) * scl : U(0.3, emD[0]); cl.push_back(PocketTool{mk(2, emD[i], 0, "EM" + std::to_string(i), 2 + i), std::nullopt}); }
        if (fm == 1) tool.defaultCuttingData.stepover = U(0.2, 0.9) * 2 * fp(ts, std::min(p.flatDepth, fullDepth(ts)));
        p.clearanceStrategy = I(0, 1) ? PocketStrategy::Raster : PocketStrategy::Offset; p.clearanceRasterAngle = U(0, 3); p.rampPlunge = I(0, 3) == 0;
        if (getenv("VC_FIX")) { // "<type 0 vbit|1 ball> <D> <angle deg> <chordal> <flat> <start>": fixed carving tool, no clearance tools
            double fx[6] = {0, 6, 90, 0.02, 0, 0}; std::sscanf(getenv("VC_FIX"), "%lf %lf %lf %lf %lf %lf", &fx[0], &fx[1], &fx[2], &fx[3], &fx[4], &fx[5]);
            tt = (int)fx[0]; D = fx[1]; ang = fx[2]; tool = mk(tt, D, ang, "Carve", 1); ts = TS{tt, D / 2, tt == 0 ? std::tan(tool.includedAngle / 2) : 0};
            chord = fx[3]; p.tolerance.chordal = chord; p.tolerance.distance = std::min(0.001, chord); p.flatDepth = fx[4]; p.startDepth = fx[5]; fm = fx[4] > 0 ? 1 : 0; cl.clear(); ncl = 0;
            if (fm == 1) tool.defaultCuttingData.stepover = 0.5 * 2 * fp(ts, std::min(p.flatDepth, fullDepth(ts))); }
        ++cases;
        VCarvingResult r = vCarvingToolpath(cs, tool, std::nullopt, cl, p);
        bool resolved = r.has(ToolpathIssue::CrossingVectorsResolved); if (resolved) ++resolvedCases;
        for (auto& d : r.diagnostics) diagCount[std::to_string((int)d.issue)]++;
        if (!r.ok()) { ++refused; if (verbose) { std::printf("case %d fam %d %s refused/empty:", ci, fam, sname.c_str()); for (auto& d : r.diagnostics) std::printf(" [%d %s]", (int)d.issue, d.message.substr(0, 120).c_str()); std::printf("\n"); } continue; }
        ++okc;
        double limit = std::min(p.flatDepth > 0 ? p.flatDepth : 1e300, fullDepth(ts)); double limR = fp(ts, limit);
        vo::Boundary B = vo::makeBoundary(cs, chord * 0.25);
        auto dist = [&](Vec2 q) { return vo::exactDist(B, q).d; };
        const double eps = 1e-6;
        auto witnessOutside = [&](Vec2 c, double f) { // a point of the disc (c, f) strictly outside the region?
            { vo::Foot ft = vo::exactDist(B, c); Vec2 u = ft.q - c; double ul = std::hypot(u.x, u.y); // just beyond the closest curve point
              if (ul > 0 && ul < f) { u = u / ul; for (double t : {1e-6, 1e-5, 1e-4, 0.25 * (f - ul), 0.5 * (f - ul), 0.99 * (f - ul)}) { if (t <= 0 || ul + t > f) continue; Vec2 q = ft.q + u * t; if (!vo::insideExact(cs, q) && dist(q) >= 0.5 * t) return true; } } }
            for (int ri = 1; ri <= 16; ++ri) { double rr = f * ri / 16.0; for (int k = 0; k < 128; ++k) { double a = 2 * vo::PI * k / 128; Vec2 q{c.x + rr * std::cos(a), c.y + rr * std::sin(a)}; if (!vo::insideExact(cs, q) && dist(q) > 1e-5) return true; } }
            return !vo::insideExact(cs, c) && dist(c) > 1e-5; };
        bool bad = false; std::string why; std::vector<Samp> samples;
        auto tps = r.toolpaths();
        if (getenv("VCMUT")) { double fm2 = atof(getenv("VCMUT")); for (auto& m : tps.back().moves) if (m.to.z < -p.startDepth) m.to.z = -p.startDepth + (m.to.z + p.startDepth) * fm2; } // oracle sanity: deepen the carve
        for (size_t ti = 0; ti < tps.size(); ++ti) {
            const Toolpath& tp = tps[ti]; bool isCarve = ti + 1 == tps.size();
            TS tsx = ts; if (!isCarve) { int idx = 0; for (int i = 0; i < ncl; ++i) if (tp.tool.diameter == emD[i]) idx = i; tsx = TS{2, emD[idx] / 2, 0}; }
            geometry::Vec3 at{p.startPosition.x, p.startPosition.y, p.safeZ}; bool first = true;
            for (const Move& m : tp.moves) {
                geometry::Vec3 a = at, b = m.to;
                if (first && m.kind != MoveKind::Rapid) { bad = true; why = "first move not rapid"; } first = false;
                if (m.kind == MoveKind::Rapid) { if (std::fabs(b.z - p.safeZ) > 1e-9 || (std::fabs(a.z - p.safeZ) > 1e-9 && (a.x != b.x || a.y != b.y))) { bad = true; why = "rapid not at safe Z"; } at = b; continue; }
                if (m.kind == MoveKind::ArcCW || m.kind == MoveKind::ArcCCW || m.kind == MoveKind::Dwell) { bad = true; why = "arc/dwell"; at = b; continue; }
                if ((m.kind == MoveKind::Plunge || m.kind == MoveKind::Retract) && (a.x != b.x || a.y != b.y)) { bad = true; why = "plunge/retract not vertical"; }
                double da = -a.z - p.startDepth, db = -b.z - p.startDepth; bool lateral = a.x != b.x || a.y != b.y;
                if (lateral && std::max(a.z, b.z) > -p.startDepth + 1e-9 && !(std::fabs(a.z - p.safeZ) < 1e-9 && std::fabs(b.z - p.safeZ) < 1e-9)) { bad = true; why = "lateral feed above start plane"; }
                if (std::max(da, db) > limit + 1e-9) { bad = true; why = "deeper than limit"; }
                if (p.startDepth > 0 && lateral && std::fabs(da) < 1e-9 && std::fabs(db) < 1e-9) { Vec2 mid{(a.x + b.x) / 2, (a.y + b.y) / 2}; if (!vo::insideExact(cs, mid) && dist(mid) > 1e-4) ++startPlaneOutside; }
                double Lxy = std::hypot(b.x - a.x, b.y - a.y);
                auto depthAt = [&](double t) { return da + (db - da) * t; };
                auto cen = [&](double t) { return Vec2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t}; };
                std::vector<std::pair<double, double>> stk{{0, 1}}; int iters = 0; double localMin = 1e300; double vt = -1;
                while (!stk.empty() && iters < 2000000) {
                    ++iters; auto [t0, t1] = stk.back(); stk.pop_back();
                    double dmax = std::max(depthAt(t0), depthAt(t1)); if (dmax <= 1e-12) continue;
                    double tm = 0.5 * (t0 + t1); double dc = dist(cen(tm)); double half = 0.5 * Lxy * (t1 - t0);
                    double lb, val;
                    if (isCarve) { lb = dc - half - fp(tsx, dmax); val = dc - fp(tsx, depthAt(tm)); } else { lb = dc - half - tsx.R - limR; val = dc - tsx.R - limR; }
                    if (val < localMin) { localMin = val; vt = tm; }
                    if (lb >= -eps) continue;
                    if (val < -eps) break;
                    if (t1 - t0 < 1e-9 || half < 1e-10) { localMin = std::min(localMin, lb); continue; }
                    stk.push_back({t0, tm}); stk.push_back({tm, t1});
                }
                if (localMin < -eps) {
                    double f = isCarve ? fp(tsx, depthAt(vt)) : tsx.R + limR;
                    bool real = false; // witness along the whole move (end mills: the tool disc itself)
                    for (int wi = -1; wi <= 64 && !real; ++wi) { double tw = wi < 0 ? vt : wi / 64.0; double dw = depthAt(tw); if (dw <= 1e-9) continue;
                        double fw = isCarve ? fp(tsx, dw) : tsx.R; if (witnessOutside(cen(tw), fw)) { real = true; vt = tw; f = isCarve ? fw : f; } }
                    if (real) { ++realViol; bad = true; char buf[300]; std::snprintf(buf, sizeof buf, "%s VIOLATION %.3e at (%.5f,%.5f) depth %.5f footprint %.5f", isCarve ? "CONE" : "ENDMILL-ALLOWANCE", localMin, cen(vt).x, cen(vt).y, depthAt(vt), f); why = buf; }
                    else { ++falseAlarm; famFails["falseAlarmFam" + std::to_string(fam)]++; if (getenv("VCDBG")) std::printf("    falsealarm %s %.3e at (%.6f,%.6f) depth %.5f f %.5f dist %.6f inside %d\n", isCarve ? "cone" : "em", localMin, cen(vt).x, cen(vt).y, depthAt(vt), f, dist(cen(vt)), (int)vo::insideExact(cs, cen(vt))); localMin = std::max(localMin, 0.0); }
                }
                if (isCarve) worstCone = std::min(worstCone, localMin); else worstEMa = std::min(worstEMa, localMin);
                { double tdeep = da > db ? 0.0 : 1.0; Vec2 c = cen(tdeep); if (std::max(da, db) > 1e-6 && !vo::insideExact(cs, c) && dist(c) > 1e-4) { ++realViol; bad = true; char buf[200]; std::snprintf(buf, sizeof buf, "centre outside region (%.4f,%.4f) depth %.4f dist %.4f", c.x, c.y, std::max(da, db), dist(c)); why = buf; } }
                int ns = std::max(1, (int)std::ceil(Lxy / (chord / 16)));
                for (int i = 0; i <= ns; ++i) { double t = double(i) / ns; double d = depthAt(t); if (d > 1e-9) { Vec2 c = cen(t); samples.push_back({c.x, c.y, d, isCarve ? -1 : (int)ti}); } }
                at = b;
            }
            if (std::fabs(at.z - p.safeZ) > 1e-9) { bad = true; why = "does not end at safe Z"; }
        }
        double cell = std::max(0.5 * scl, std::max(limR, (ncl ? emD[0] / 2 : 0.0)) + 1e-9);
        std::unordered_map<long long, std::vector<int>> grid; auto key = [&](long long ix, long long iy) { return ix * 1000003LL + iy; };
        for (int i = 0; i < (int)samples.size(); ++i) grid[key((long long)std::floor(samples[i].x / cell), (long long)std::floor(samples[i].y / cell))].push_back(i);
        double caseLat = -1e300, caseKink = -1e300, caseOver = -1e300, caseFlat = -1e300; Vec2 latAt{0, 0}, flatAt{0, 0};
        auto carved = [&](Vec2 q) { double Dq = 0; long long ix = (long long)std::floor(q.x / cell), iy = (long long)std::floor(q.y / cell);
            for (long long gx = ix - 1; gx <= ix + 1; ++gx) for (long long gy = iy - 1; gy <= iy + 1; ++gy) { auto it = grid.find(key(gx, gy)); if (it == grid.end()) continue;
                for (int si : it->second) { const Samp& s = samples[si]; double rr = std::hypot(q.x - s.x, q.y - s.y); double cut = -1;
                    if (s.tool < 0) { if (rr <= fp(ts, s.d)) cut = ts.type == 0 ? s.d - rr / ts.tanh : s.d - (ts.R - std::sqrt(std::max(0.0, ts.R * ts.R - rr * rr))); }
                    else if (rr <= tps[s.tool].tool.diameter / 2) cut = s.d;
                    Dq = std::max(Dq, cut); } }
            return Dq; };
        for (int k = 0; k < npts; ++k) {
            Vec2 q{U(B.minx, B.maxx), U(B.miny, B.maxy)}; if (!vo::insideExact(cs, q)) continue;
            double dq = dist(q), ideal = std::min(limit, finv(ts, dq)), Dq = carved(q);
            if (ts.type == 0 && fam != 15) caseOver = std::max(caseOver, Dq - ideal);
            if (ideal < limit - 1e-9) { double lat = (dq - fp(ts, Dq)) / chord; if (lat > caseLat) { caseLat = lat; latAt = q; } if (dq > limR - 3 * chord) caseKink = std::max(caseKink, lat); }
            else if (limit - Dq > caseFlat) { caseFlat = limit - Dq; flatAt = q; }
        }
        int rb = resolved ? 1 : 0;
        worstLat[rb] = std::max(worstLat[rb], caseLat); worstKink[rb] = std::max(worstKink[rb], caseKink); worstOver = std::max(worstOver, caseOver); if (ncl > 0) worstFlat = std::max(worstFlat, caseFlat); else worstFlatSelf = std::max(worstFlatSelf, caseFlat);
        double bound = resolved ? 1.625 : 1.5;
        bool covBad = caseLat > bound || caseOver > 1e-4;
        if (bad || covBad || verbose) std::printf("case %d seed %lu fam %d %s res=%d tool=%d D=%.3f ang=%.1f chord=%.3f flat=%.3f start=%.3f ncl=%d emD=%.3f/%.3f: %s lat=%.3f*ch at (%.5f,%.5f) kink=%.3f over=%.2e flatShort=%.3f at (%.4f,%.4f) dist %.4f limR %.4f\n", ci, seed, fam, sname.c_str(), rb, tt, D, ang, chord, p.flatDepth, p.startDepth, ncl, emD[0], emD[1], bad ? why.c_str() : "ok", caseLat, latAt.x, latAt.y, caseKink, caseOver, std::max(caseFlat, -1.0), flatAt.x, flatAt.y, dist(flatAt), limR);
        if (verbose) { for (auto& d : r.diagnostics) std::printf("    diag %d %s\n", (int)d.issue, d.message.substr(0, 200).c_str()); if (getenv("VCFLATAT")) { /* worst flat point */ } }
        if ((bad || covBad) && getenv("VCDUMP")) dump(cs);
        if (bad || covBad) { ++fails; famFails[std::to_string(fam) + (bad ? ":safety" : ":coverage")]++; }
    }
    std::printf("SUMMARY seed=%lu fam=%d far=%g scale=%g cases=%d ok=%d refused=%d resolved=%d fails=%d realViolations=%d falseAlarms(coincident)=%d worstCone=%.3e worstEMallow=%.3e latNonRes=%.4g latRes=%.4g kinkNonRes=%.4g kinkRes=%.4g worstOver=%.2e worstFlatShortEndMills=%.4g worstFlatShortSelf=%.4g startPlaneOutsideMoves=%d\n",
                seed, famFix, far, scl, cases, okc, refused, resolvedCases, fails, realViol, falseAlarm, worstCone, worstEMa, worstLat[0], worstLat[1], worstKink[0], worstKink[1], worstOver, worstFlat, worstFlatSelf, startPlaneOutside);
    for (auto& [k, v] : famFails) std::printf("  fails %s: %d\n", k.c_str(), v);
    for (auto& [k, v] : diagCount) std::printf("  issue %s: %d\n", k.c_str(), v);
}
