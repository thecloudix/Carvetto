// Round-3 V-bit cone-vs-wall sweep (verifier throwaway): new shapes, new seeds.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include "oracle.hpp"
using namespace v;
using namespace carvetto::toolpath;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"

static Contour bezGear(Vec2 c, double R, int n) {  // polygonised-curve walls with genuine corners
    Contour o;
    o.closed = true;
    std::vector<Vec2> p;
    for (int i = 0; i < n; ++i) p.push_back(c + Vec2::fromAngle(2 * PI * i / n, R));
    for (int i = 0; i < n; ++i) {
        Vec2 a = p[(size_t)i], b = p[(size_t)((i + 1) % n)];
        Vec2 m = (a + b) * 0.5, d = b - a, nn{-d.y, d.x};
        double k = U(-0.25, 0.25);
        o.append(geometry::BezierSpan{a, a + d * 0.3 + nn * k, b - d * 0.3 + nn * k, b});
        (void)m;
    }
    return o;
}

int main(int argc, char** argv) {
    unsigned long seed = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 1;
    int N = argc > 2 ? std::atoi(argv[2]) : 500;
    int target = argc > 3 ? std::atoi(argv[3]) : -1;
    rng.seed(seed);
    int bad = 0, cases = 0, climbs = 0, gouges = 0, allowV = 0, det = 0, missed = 0, expectedCorners = 0;
    double worstAll = 1e9;
    auto t0 = std::chrono::steady_clock::now();
    for (int sc = 0; sc < N; ++sc) {
        std::vector<Contour> cs;
        int k = I(0, 8);
        double R = U(10, 35);
        bool straightOnly = false;
        switch (k) {
            case 0: cs.push_back(gear({0, 0}, R, I(5, 12), R * U(0.1, 0.4))); straightOnly = true; break;
            case 1: cs.push_back(letter({0, 0}, R)); straightOnly = true; break;
            case 2: cs.push_back(flower({0, 0}, R, 2 * I(2, 4))); break;
            case 3: cs.push_back(spikes({0, 0}, R, I(3, 7))); straightOnly = true; break;
            case 4: cs.push_back(dshape({0, 0}, R)); break;
            case 5: cs.push_back(bezGear({0, 0}, R, I(3, 6))); break;
            case 6: {  // plate with letters inside
                cs.push_back(Contour::rectangle(geometry::Box2{{-50, -40}, {50, 40}}));
                cs.push_back(letter({-35, -10}, 20));
                cs.push_back(letter({10, -10}, 20));
                straightOnly = true;
                break;
            }
            case 7: {  // close neighbours: merged polygonal regions
                cs.push_back(gear({0, 0}, R, I(5, 9), R * 0.3));
                Contour b2 = letter({0, 0}, 20);
                cs.push_back(translate(b2, {R + U(0.2, 4), -10}));
                break;
            }
            default: {  // flower with straight chords mixed in
                Contour f = flower({0, 0}, R, 2 * I(2, 4));
                for (size_t i = 0; i < f.spans.size(); i += 2) f.spans[i] = geometry::LineSpan{geometry::startPoint(f.spans[i]), geometry::endPoint(f.spans[i])};
                cs.push_back(f);
            }
        }
        double ang = std::vector<double>{30, 60, 90, 120}[(size_t)I(0, 3)];
        double D = std::vector<double>{6.35, 12.7, 25.4}[(size_t)I(0, 2)];
        auto tool = vbit(D, ang);
        double tn = std::tan(ang * PI / 360);
        ProfileParams p;
        p.side = I(0, 1) ? ProfileSide::Inside : ProfileSide::Outside;
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        int sd = I(0, 3);
        p.passes.startDepth = sd == 0 ? 0.0 : sd == 1 ? (double)I(1, 5) : U(0, 5);
        p.passes.cutDepth = U(0.2, (D / 2) / tn * 1.5);
        p.passes.passDepth = U(0.3, 5);
        p.allowance = I(0, 1) ? 0.0 : U(0.01, 0.8);
        p.corners.sharpInternal = I(0, 5) > 0;
        p.corners.sharpExternal = I(0, 1);
        p.corners.mitreLimit = U(1.1, 10);
        p.separateLastPass = I(0, 4) == 0;
        p.lastPassAllowance = U(0.05, 0.5);
        p.tabs.enabled = I(0, 3) == 0;
        p.tabs.thickness = U(0.2, 2);
        p.tabs.length = U(2, 8);
        p.tabs.count = I(1, 4);
        p.leads.type = (LeadType)I(0, 2);
        p.leads.leadOut = I(0, 1);
        p.ramp.type = (RampType)I(0, 3);
        if (target >= 0 && sc != target) continue;
        auto res = profileToolpath(cs, tool, p);
        auto res2 = profileToolpath(cs, tool, p);
        ++cases;
        std::vector<std::string> errs;
        if (fmt(res.toolpath) != fmt(res2.toolpath)) { ++det; errs.push_back("nondeterministic"); }
        Obstacles ob(cs);
        double worst = 1e9, worstA = 1e9; Vec2 aAt{}; double aZ = 0; size_t aMove = 0;
        Vec2 wAt{};
        double wZ = 0;
        size_t wMove = 0;
        sampleMoves(res.toolpath, 0.01, [&](Vec2 q, double z, size_t mi) {
            if (z >= 0) return;
            double cone = std::min(-z * tn, D / 2);
            double d = ob.dist(q, cone + p.allowance + 1);
            if (d - cone < worst) { worst = d - cone; wAt = q; wZ = z; wMove = mi; }
            if (d - cone - p.allowance < worstA) { worstA = d - cone - p.allowance; aAt = q; aZ = z; aMove = mi; }
        });
        // corner climbs: feed moves ending at Z == 0
        int cl = 0;
        for (size_t i = 1; i < res.toolpath.moves.size(); ++i) if (res.toolpath.moves[i].kind == MoveKind::Feed && res.toolpath.moves[i].to.z == 0.0) ++cl;
        climbs += cl;
        // expected corners on straight-only single shapes, allowance 0, sharp internal, no merge
        if (straightOnly && p.corners.sharpInternal && p.allowance == 0 && !res.has(ProfileIssue::MergedWithNeighbour) && !res.has(ProfileIssue::TrimmedNearVector) && !p.separateLastPass) {
            for (size_t ci = 0; ci < cs.size(); ++ci) {
                const Contour& c = cs[ci];
                bool skip = false;
                for (auto& d : res.diagnostics) if (d.contour && *d.contour == ci && d.issue == ProfileIssue::ToolTooLarge) skip = true;
                if (skip) continue;
                // orientation-free: vertex convexity relative to region = left turn for CCW
                double area = 0;
                for (auto& s : c.spans) { Vec2 a = geometry::startPoint(s), e = geometry::endPoint(s); area += a.cross(e); }
                double sgn = area > 0 ? 1 : -1;
                int depthC = 0;
                for (auto& o : cs) if (&o != &c && ob.depth(geometry::startPoint(c.spans[0])) > 0) (void)o;
                // nesting parity of this contour
                Obstacles self({c});
                (void)self;
                int par = 0;
                { Vec2 pr = geometry::startPoint(c.spans[0]); for (auto& o : cs) if (&o != &c) { Obstacles oo({o}); par += oo.depth(pr); } }
                bool hole = par % 2 == 1;
                (void)depthC;
                size_t n = c.spans.size();
                for (size_t j = 0; j < n; ++j) {
                    Vec2 a = geometry::tangentAt(c.spans[(j + n - 1) % n], 1.0), b2 = geometry::tangentAt(c.spans[j], 0.0);
                    double turn = std::atan2(a.cross(b2), a.dot(b2)) * sgn;  // >0 convex w.r.t. this contour's interior
                    if (std::fabs(turn) < 0.0873) continue;
                    bool convexRegion = (turn > 0) != hole;
                    bool want = (p.side == ProfileSide::Inside) ? convexRegion : !convexRegion;
                    if (!want) continue;
                    Vec2 V = geometry::startPoint(c.spans[j]);
                    ++expectedCorners;
                    bool hit = false;
                    for (auto& m : res.toolpath.moves) if (m.to.z == 0.0 && geometry::distance(m.to.xy(), V) < 1e-6) hit = true;
                    if (!hit) { ++missed; if (target >= 0) std::printf("   missed corner (%.4f,%.4f)\n", V.x, V.y); }
                }
            }
        }
        worstAll = std::min(worstAll, worst);
        if (worst < -1e-3) { ++gouges; char b[300]; std::snprintf(b, sizeof b, "CONE GOUGE %.5g at (%.4f,%.4f) z %.4f move %zu %s", worst, wAt.x, wAt.y, wZ, wMove, kn(res.toolpath.moves[wMove].kind)); errs.push_back(b); }
        else if (worstA < -1e-3) { ++allowV; char b[300]; std::snprintf(b, sizeof b, "allowance violated by %.4g at (%.4f,%.4f) z %.4f move %zu %s", -worstA, aAt.x, aAt.y, aZ, aMove, kn(res.toolpath.moves[aMove].kind)); errs.push_back(b); }
        if (!errs.empty() || target >= 0) {
            ++bad;
            std::printf("case %d shape %d ang %.0f D %.2f side %d dir %d start %.3f cut %.3f pass %.3f allow %.3f sep %d int %d ext %d tabs %d lead %d ramp %d r %.4f diags:", sc, k, ang, D, (int)p.side, (int)p.direction, p.passes.startDepth, p.passes.cutDepth, p.passes.passDepth, p.allowance, p.separateLastPass, p.corners.sharpInternal, p.corners.sharpExternal, p.tabs.enabled, (int)p.leads.type, (int)p.ramp.type, profileToolRadius(tool, p.passes));
            for (auto& d : res.diagnostics) std::printf(" %d", (int)d.issue);
            std::printf("\n");
            for (auto& e : errs) std::printf("   - %s\n", e.c_str());
            if (target >= 0) { FILE* f = std::fopen("vcase3.moves", "w"); dump(res.toolpath, f); std::fclose(f); for (auto& c : cs) for (auto& sp : c.spans) { auto a = geometry::startPoint(sp), e = geometry::endPoint(sp); std::printf("   span %zu (%.4f,%.4f)->(%.4f,%.4f)\n", sp.index(), a.x, a.y, e.x, e.y); } }
        }
    }
    std::printf("vbit3 seed %lu: %d cases, %d flagged, gouges %d, allowance %d, det %d, climbs %d, corners missed %d/%d, worst %.4g, %.1fs\n", seed, cases, bad, gouges, allowV, det, climbs, missed, expectedCorners, worstAll,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
}
