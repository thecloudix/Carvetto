// T-0124 verifier (throwaway): material-band tab oracle + structure/gouge/determinism.
// For every tab plateau of the primary cuts (the finishing cuts with a separate last pass,
// identified by their median distance to the vectors, else every cut), the "bridge" across the
// kerf(s) is checked along the vector normal at every core point of the plateau: points at
// distance d from the vector, d in [a, a + lpa + 2r] (lpa = 0 without separate last pass), must not
// be within r of any cutting sample below the tab top (any block, any move).
// usage: band seed N [family] [forceSep] [target]
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <unordered_map>
#include "oracle.hpp"
using namespace v;
using namespace carvetto::toolpath;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "hash.hpp"

// ---- new generators
static Contour bezBlob(Vec2 c, double R, int n) {  // closed Catmull-Rom blob of cubic Beziers
    std::vector<Vec2> p;
    for (int i = 0; i < n; ++i) {
        double a = 2 * PI * i / n + U(-0.2, 0.2) / n;
        double rr = R * U(0.55, 1.0);
        p.push_back({c.x + rr * std::cos(a), c.y + rr * std::sin(a)});
    }
    Contour o;
    o.closed = true;
    for (int i = 0; i < n; ++i) {
        Vec2 p0 = p[(size_t)((i - 1 + n) % n)], p1 = p[(size_t)i], p2 = p[(size_t)((i + 1) % n)], p3 = p[(size_t)((i + 2) % n)];
        o.append(geometry::BezierSpan{p1, p1 + (p2 - p0) * (1.0 / 6), p2 - (p3 - p1) * (1.0 / 6), p2});
    }
    return o;
}
static Contour bezWave(Vec2 c, double L, int n) {  // open wave of Beziers
    Contour o;
    Vec2 a = c;
    for (int i = 0; i < n; ++i) {
        Vec2 b = a + Vec2{L / n, U(-L / n, L / n)};
        double h = U(-0.6, 0.6) * L / n;
        o.append(geometry::BezierSpan{a, a + Vec2{L / (3 * n), h}, b - Vec2{L / (3 * n), -h}, b});
        a = b;
    }
    return o;
}
// rectangle with a narrow slot cut into one side (a "comb" of k teeth of width w)
static Contour comb(Vec2 o, double W, double H, int k, double slotW, double depth) {
    std::vector<Vec2> pts{{o.x, o.y}, {o.x + W, o.y}, {o.x + W, o.y + H}};
    double pitch = W / (k + 1);
    for (int i = k; i >= 1; --i) {
        double x = o.x + pitch * i;
        pts.push_back({x + slotW / 2, o.y + H});
        pts.push_back({x + slotW / 2, o.y + H - depth});
        pts.push_back({x - slotW / 2, o.y + H - depth});
        pts.push_back({x - slotW / 2, o.y + H});
    }
    pts.push_back({o.x, o.y + H});
    return poly(pts);
}

struct Stats {
    int scenes = 0, empty = 0, det = 0, structure = 0, gouge = 0, clampMissing = 0, countBad = 0, countChecked = 0;
    int grazeScenes = 0, plateaus = 0, midOther = 0, midOwn = 0, lost = 0, shortened = 0, orphan = 0, finClass = 0;
    double worstShort = 0, worstGouge = 1e9;
};

int main(int argc, char** argv) {
    unsigned long seed = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 1;
    int N = argc > 2 ? std::atoi(argv[2]) : 200;
    int fam = argc > 3 ? std::atoi(argv[3]) : 0;
    int forceSep = argc > 4 ? std::atoi(argv[4]) : -1;
    int target = argc > 5 ? std::atoi(argv[5]) : -1;
    const bool verbose = std::getenv("V") != nullptr;
    rng.seed(seed);
    Stats st;
    auto t0 = std::chrono::steady_clock::now();
    for (int sc = 0; sc < N; ++sc) {
        // ---- params first (tool size feeds the generators)
        ProfileParams p;
        double D = std::vector<double>{2.0, 3.175, 6.0, 8.0, 12.0}[(size_t)I(0, 4)];
        double r = D / 2;
        int sk = I(0, 1);
        p.side = sk ? ProfileSide::Outside : ProfileSide::Inside;
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = I(0, 2) == 0 ? 0.0 : U(0.0, 0.5);
        p.passes.startDepth = I(0, 3) == 0 ? U(0, 3) : 0.0;
        p.passes.cutDepth = U(2, 20);
        p.passes.passDepth = I(0, 3) == 0 ? U(0.4, 1.5) : U(1, 8);
        if (I(0, 5) == 0) p.passes.lastPassThickness = U(0.2, 1.0);
        p.separateLastPass = forceSep >= 0 ? forceSep : I(0, 1);
        p.lastPassAllowance = U(0.05, 1.0);
        p.lastPassReverse = I(0, 1);
        p.tabs.enabled = true;
        p.tabs.length = U(1, 20);
        p.tabs.thickness = I(0, 7) == 0 ? U(p.passes.cutDepth, p.passes.cutDepth + 5) : U(0.3, std::min(6.0, p.passes.cutDepth));
        p.tabs.spacing = I(0, 3) ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance;
        p.tabs.count = I(0, 1) ? 2 * I(0, 4) + 1 : I(1, 9);
        p.tabs.distance = U(10, 120);
        p.tabs.avoidCorners = I(0, 1);
        p.tabs.threeD = I(0, 2) == 0;
        p.ramp.type = (RampType)I(0, 3);
        p.ramp.distance = U(1, 40);
        p.ramp.useAngle = I(0, 2) == 0;
        p.ramp.angle = U(1, 30) * PI / 180;
        p.ramp.onLeadIn = I(0, 3) == 0;
        p.leads.type = (LeadType)I(0, 2);
        p.leads.length = U(0.5, 15);
        p.leads.angle = U(0.05, PI / 2);
        p.leads.radius = U(0.5, 12);
        p.leads.sweep = U(5, 90) * PI / 180;
        p.leads.leadOut = I(0, 1);
        p.leads.overcut = I(0, 1) ? U(0, 20) : 0.0;
        p.startAt = (StartAt)I(0, 2);
        p.startAnchor = (BoxAnchor)I(0, 8);
        p.startPosition = {U(-50, 50), U(-50, 50)};
        p.order.leftToRight = I(0, 3) == 0;
        p.order.grid = I(0, 3) == 0;
        // ---- geometry
        std::vector<Contour> cs;
        int f = fam == 0 ? I(1, 7) : fam;
        bool open = false;
        double R = U(8, 35);
        switch (f) {
            case 1: {  // narrow slots: stadium barely wider than the tool (inside) or comb with narrow gaps (outside)
                if (p.side == ProfileSide::Inside) cs.push_back(slot({0, 0}, U(10, 60), r + p.allowance + U(0.02, r + 1.5), U(0, PI)));
                else cs.push_back(comb({0, 0}, U(40, 100), U(20, 50), I(1, 4), U(0.5, 2.5 * D), U(5, 18)));
                break;
            }
            case 2: {  // tight lobes
                int k = I(0, 2);
                if (k == 0) cs.push_back(flower({0, 0}, U(2.5, 10), 2 * I(2, 5)));
                else if (k == 1) cs.push_back(gear({0, 0}, U(8, 25), I(6, 20), U(1, 6)));
                else cs.push_back(spikes({0, 0}, U(8, 25), I(3, 8)));
                break;
            }
            case 3: {  // nested: plate with holes / islands near its edge
                double W = U(50, 120), H = U(40, 90);
                cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {W, H}}));
                int n = I(1, 3);
                for (int i = 0; i < n; ++i) {
                    double rr = U(4, 15);
                    Vec2 c{U(rr + 0.5, W - rr - 0.5), U(rr + 0.5, H - rr - 0.5)};
                    cs.push_back(I(0, 1) ? Contour::circle(c, rr) : Contour::rectangle(geometry::Box2{{c.x - rr, c.y - rr * 0.7}, {c.x + rr, c.y + rr * 0.7}}));
                }
                if (I(0, 1)) {  // island in the first hole region? a ring
                    cs.push_back(Contour::circle({W / 2, H / 2}, std::min(W, H) * 0.1));
                }
                break;
            }
            case 4: cs.push_back(bezBlob({0, 0}, R, I(4, 9))); break;
            case 5: {  // open
                open = true;
                int k = I(0, 1);
                cs.push_back(k ? anyOpen({0, 0}, R) : bezWave({0, 0}, U(30, 120), I(2, 6)));
                break;
            }
            case 6: {  // rectangles / chamfers, odd tab counts, corners
                p.tabs.count = 2 * I(0, 5) + 1;
                p.tabs.spacing = TabSpacing::ConstantNumber;
                cs.push_back(I(0, 1) ? Contour::rectangle(geometry::Box2{{0, 0}, {U(20, 120), U(15, 80)}}) : chamfered({0, 0}, U(30, 100), U(20, 70), U(0.5, 10)));
                break;
            }
            case 8: {  // fixed star from fam-2 seed 24002 scene 89, pinned params, varied count/anchor
                std::vector<Vec2> pts{{20.77171848650805, 0}, {4.5169587328544925, 2.7051918976259475}, {10.491667657202415, 18.172101438401711}, {0.079852792917912016, 4.7270855225453614}, {-9.5128454916249616, 16.476731716046977}, {-5.8595431912535636, 3.2223813701659214}, {-21.657268430262796, 0}, {-5.0331398534122114, -2.8747102338050992}, {-9.8680574985765848, -17.091976959545669}, {0.0092785251418496494, -4.8834229283725632}, {10.517900027426347, -18.217537236432516}, {3.9736897346006068, -2.1356896222425501}};
                cs.push_back(poly(pts));
                D = 6.0; p.side = ProfileSide::Outside; p.allowance = 0.25; p.tabs.length = 1.3945; p.tabs.thickness = 0.946; p.tabs.threeD = false; p.tabs.avoidCorners = false;
                p.tabs.spacing = TabSpacing::ConstantNumber; p.tabs.count = 1 + sc % 12; p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = (BoxAnchor)((sc / 12) % 9);
                p.leads.type = LeadType::None; p.leads.overcut = 0; p.ramp.type = RampType::None; p.passes.startDepth = 0; p.passes.cutDepth = 16.75; p.passes.passDepth = 3.9; p.passes.lastPassThickness = 0; p.lastPassAllowance = 0.097;
                break;
            }
            case 9: {  // rectangle, 4 tabs centred near the corners (start at bottom centre), avoid corners off
                cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {40, 30}}));
                D = std::vector<double>{3.175, 6.0, 12.0}[(size_t)(sc % 3)]; p.side = ProfileSide::Outside; p.allowance = (sc / 3) % 2 ? 0.2 : 0.0;
                p.tabs.length = 0.5 + 0.5 * ((sc / 6) % 12); p.tabs.thickness = 1.5; p.tabs.threeD = false; p.tabs.avoidCorners = false;
                p.tabs.spacing = TabSpacing::ConstantNumber; p.tabs.count = 4; p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::BottomCentre;
                p.leads.type = LeadType::None; p.leads.overcut = 0; p.ramp.type = RampType::None; p.passes.startDepth = 0; p.passes.cutDepth = 6; p.passes.passDepth = 3; p.passes.lastPassThickness = 0;
                break;
            }
            case 10: {  // Inside of a plain circle, short tabs, big tools
                cs.push_back(Contour::circle({0, 0}, U(10, 30)));
                D = std::vector<double>{6.0, 8.0, 12.0}[(size_t)I(0, 2)]; p.side = ProfileSide::Inside; p.allowance = I(0, 1) ? 0.0 : U(0, 0.5);
                p.tabs.length = std::getenv("TLMIN") ? U(std::atof(std::getenv("TLMIN")), std::atof(std::getenv("TLMAX"))) : U(0.5, 4); p.tabs.threeD = false; p.tabs.spacing = TabSpacing::ConstantNumber; p.tabs.count = I(1, 4);
                p.lastPassAllowance = U(0.3, 1.0);
                break;
            }
            default: cs.push_back(anyClosed({0, 0}, R)); if (I(0, 2) == 0) cs.push_back(anyClosed({3.5 * 35, 0}, U(8, 25))); break;
        }
        r = D / 2;
        if (target >= 0 && sc != target) continue;
        ++st.scenes;
        if (target >= 0 && std::getenv("DUMPCS")) {
            for (auto& c : cs) { std::printf("CS closed %d\n", (int)c.closed); for (auto& sp : c.spans) { if (auto* l = std::get_if<geometry::LineSpan>(&sp)) std::printf("L %.17g %.17g %.17g %.17g\n", l->start.x, l->start.y, l->end.x, l->end.y); else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) std::printf("A %.17g %.17g %.17g %.17g %.17g %d\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, (int)a->direction); else if (auto* bz = std::get_if<geometry::BezierSpan>(&sp)) std::printf("B %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", bz->p0.x, bz->p0.y, bz->p1.x, bz->p1.y, bz->p2.x, bz->p2.y, bz->p3.x, bz->p3.y); else std::printf("NONLINE\n"); } }
            std::printf("P D %.17g allow %.17g cut %.17g pass %.17g start %.17g lpt %.17g lpa %.17g len %.17g th %.17g n %d dir %d side %d rev %d sp %d dist %.17g avoid %d threeD %d ramp %d rdist %.17g useAngle %d rangle %.17g onLead %d lead %d llen %.17g lang %.17g lrad %.17g lsweep %.17g lout %d oc %.17g startAt %d anchor %d spx %.17g spy %.17g l2r %d grid %d\n", D, p.allowance, p.passes.cutDepth, p.passes.passDepth, p.passes.startDepth, p.passes.lastPassThickness, p.lastPassAllowance, p.tabs.length, p.tabs.thickness, p.tabs.count, (int)p.direction, (int)p.side, (int)p.lastPassReverse, (int)p.tabs.spacing, p.tabs.distance, (int)p.tabs.avoidCorners, (int)p.tabs.threeD, (int)p.ramp.type, p.ramp.distance, (int)p.ramp.useAngle, p.ramp.angle, (int)p.ramp.onLeadIn, (int)p.leads.type, p.leads.length, p.leads.angle, p.leads.radius, p.leads.sweep, (int)p.leads.leadOut, p.leads.overcut, (int)p.startAt, (int)p.startAnchor, p.startPosition.x, p.startPosition.y, (int)p.order.leftToRight, (int)p.order.grid);
        }
        auto res = profileToolpath(cs, endMill(D), p);
        auto res2 = profileToolpath(cs, endMill(D), p);
        std::vector<std::string> errs;
        char b[600];
        if (fmt(res.toolpath) != fmt(res2.toolpath) || res.diagnostics.size() != res2.diagnostics.size()) { ++st.det; errs.push_back("NONDETERMINISTIC"); }
        const auto& mv = res.toolpath.moves;
        if (mv.empty()) { ++st.empty; if (res.diagnostics.empty()) errs.push_back("EMPTY without diagnostic"); }
        const double bottom = p.passes.bottomZ();
        const double th = std::min(p.tabs.thickness, p.passes.cutDepth);
        const double zTab = bottom + th;
        if (p.tabs.thickness > p.passes.cutDepth && !res.has(ProfileIssue::TabThicknessClamped)) { ++st.clampMissing; errs.push_back("thickness > depth but no TabThicknessClamped"); }
        if (p.tabs.thickness <= p.passes.cutDepth && res.has(ProfileIssue::TabThicknessClamped)) { ++st.clampMissing; errs.push_back("spurious TabThicknessClamped"); }
        // ---- structure
        for (size_t i = 0; i < mv.size(); ++i) {
            const Move& m = mv[i];
            if (!std::isfinite(m.to.x) || !std::isfinite(m.to.y) || !std::isfinite(m.to.z)) { ++st.structure; errs.push_back("non-finite"); break; }
            if (m.to.z < bottom - 1e-9 || m.to.z > p.safeZ + 1e-12) { ++st.structure; errs.push_back("Z out of range"); break; }
            if (i == 0) continue;
            const auto A = mv[i - 1].to;
            if (m.to == A && !(isArc(m.kind))) { ++st.structure; snprintf(b, sizeof b, "zero-length move %zu kind %s", i, kn(m.kind)); errs.push_back(b); break; }
            if (m.kind == MoveKind::Plunge && (A.x != m.to.x || A.y != m.to.y || m.to.z >= A.z)) { ++st.structure; errs.push_back("plunge not vertical-down"); break; }
            if (m.to.z < A.z && A.x == m.to.x && A.y == m.to.y && m.kind != MoveKind::Plunge && !isArc(m.kind)) { ++st.structure; snprintf(b, sizeof b, "vertical descent not a plunge at move %zu kind %s", i, kn(m.kind)); errs.push_back(b); break; }
            if (m.kind == MoveKind::Rapid && (A.z != p.safeZ || m.to.z != p.safeZ)) { ++st.structure; errs.push_back("rapid below safe Z"); break; }
        }
        // ---- gouge (end mill disc): distance to vectors >= r + min(0, allowance)
        Obstacles ob(cs);
        double worst = 1e9;
        sampleMoves(res.toolpath, 0.03, [&](Vec2 q, double z, size_t) {
            if (z >= 0) return;
            double d = ob.dist(q, r + 1);
            worst = std::min(worst, d - r);
        });
        st.worstGouge = std::min(st.worstGouge, worst);
        if (worst < -1e-3) { ++st.gouge; snprintf(b, sizeof b, "GOUGE %.5f", worst); errs.push_back(b); }
        // ---- blocks
        struct Blk { size_t a, e; double med; bool fin; };
        std::vector<Blk> bls;
        { size_t a = 0; for (size_t i = 0; i < mv.size(); ++i) { if (mv[i].kind == MoveKind::Rapid) a = i; if (mv[i].kind == MoveKind::Retract) bls.push_back({a, i, 1e9, true}); } }
        std::vector<int> blockOf(mv.size(), -1);
        for (size_t bi = 0; bi < bls.size(); ++bi) for (size_t k = bls[bi].a; k <= bls[bi].e; ++k) blockOf[k] = (int)bi;
        const double d0 = r + p.allowance;
        if (p.separateLastPass) {
            for (auto& bl : bls) {
                Toolpath sub{"", {}, {}};
                for (size_t k = bl.a; k <= bl.e; ++k) sub.moves.push_back(mv[k]);
                std::vector<double> ds;
                sampleMoves(sub, 0.25, [&](Vec2 q, double z, size_t) { if (z == bottom) ds.push_back(ob.dist(q, 1e9)); });
                if (!ds.empty()) { std::nth_element(ds.begin(), ds.begin() + (long)ds.size() / 2, ds.end()); bl.med = ds[ds.size() / 2]; }
                bl.fin = std::fabs(bl.med - d0) < 0.5 * p.lastPassAllowance;
            }
            if (target >= 0) for (size_t bi = 0; bi < bls.size(); ++bi) std::printf("   block %zu moves %zu-%zu median dist %.4f (finishing expected %.4f, roughing %.4f) fin %d\n", bi, bls[bi].a, bls[bi].e, bls[bi].med, d0, d0 + p.lastPassAllowance, (int)bls[bi].fin);
            // all roughing first, then finishing
            bool seenFin = false;
            for (auto& bl : bls) { if (bl.fin) seenFin = true; else if (seenFin) { ++st.finClass; errs.push_back("roughing block after a finishing block (or unclear classification)"); break; } }
        }
        // ---- sample hash of all cutting samples below the tab top, per block
        SampleHash H;                                   // all blocks
        std::vector<SampleHash> HB(bls.size());         // per block
        sampleMoves(res.toolpath, 0.02, [&](Vec2 q, double z, size_t mi) {
            if (z >= zTab - 1e-6) return;
            H.add(q, z);
            if (blockOf[mi] >= 0) HB[(size_t)blockOf[mi]].add(q, z);
        });
        // ---- plateaus on each block: runs of feed/arc moves flat at zTab, entered from below and left downwards
        struct Plat { int blk; std::vector<Vec2> pts; std::vector<double> s; };
        std::vector<Plat> plats[2];  // [0] primary, [1] secondary (roughing with sep)
        for (size_t bi = 0; bi < bls.size(); ++bi) {
            const auto& bl = bls[bi];
            size_t i = bl.a + 1;
            while (i <= bl.e) {
                auto flat = [&](size_t k) { return (mv[k].kind == MoveKind::Feed || isArc(mv[k].kind)) && mv[k].to.z == zTab && mv[k - 1].to.z == zTab && !(mv[k].to.xy() == mv[k - 1].to.xy()); };
                if (!flat(i)) { ++i; continue; }
                size_t j = i;
                while (j + 1 <= bl.e && flat(j + 1)) ++j;
                bool fromBelow = i >= 2 && mv[i - 2].to.z < zTab;
                bool down = j + 1 <= bl.e && mv[j + 1].to.z < zTab;
                if (fromBelow && down) {
                    Plat pl{(int)bi, {}, {}};
                    Toolpath sub{"", {}, {}};
                    sub.moves.push_back(Move::rapid(mv[i - 1].to));
                    for (size_t k = i; k <= j; ++k) sub.moves.push_back(mv[k]);
                    double s = 0;
                    sampleMoves(sub, 0.02, [&](Vec2 q, double, size_t) { if (!pl.pts.empty()) s += geometry::distance(pl.pts.back(), q); if (pl.pts.empty() || geometry::distance(pl.pts.back(), q) > 1e-9) { pl.pts.push_back(q); pl.s.push_back(s); } });
                    plats[p.separateLastPass && !bls[bi].fin ? 1 : 0].push_back(std::move(pl));
                }
                i = j + 1;
            }
        }
        // ---- band check on primary plateaus
        const double lpaEff = p.separateLastPass ? p.lastPassAllowance : 0.0;
        int midOther = 0, midOwn = 0, lost = 0, shortened = 0, noSlug = 0, grazes = 0;
        double worstShort = 0;
        std::string firstMsg;
        for (const Plat& pl : plats[0]) {
            if (pl.pts.size() < 3) continue;
            const double len = pl.s.back();
            const double m = 0.05;
            if (len < 2 * (r + m) + 0.1) continue;
            ++st.plateaus;
            const double c0 = r + m, c1 = len - r - m;
            int lines = 0, intact = 0;
            double firstIntact = -1, lastIntact = -1;
            bool midO = false, midS = false;
            double maxW = 0;
            Vec2 midAt{};
            int midBy = -1;
            for (size_t k = 1; k + 1 < pl.pts.size(); ++k) {
                double s = pl.s[k];
                if (s < c0 || s > c1) continue;
                if (k % 5) continue;  // every 0.1 mm
                Vec2 c = pl.pts[k];
                Vec2 t = (pl.pts[k + 1] - pl.pts[k - 1]);
                double tl = std::hypot(t.x, t.y);
                if (tl < 1e-9) continue;
                t = t * (1 / tl);
                Vec2 n{-t.y, t.x};
                double da = ob.dist(c + n * d0, 10), dbb = ob.dist(c - n * d0, 10);
                Vec2 nv = da < dbb ? n : n * -1.0;
                if (std::min(da, dbb) > 0.05 + 0.02 * d0) continue;  // not a clean normal to a vector here
                {   // a bridge only matters when there is waste (a slug) beyond the band to hold
                    Vec2 Xo = c + nv * (d0 - (p.allowance + lpaEff + 2 * r + 0.1));
                    if (H.query(Xo, r - 1e-3).first < zTab - 1e-6) { ++noSlug; continue; }
                }
                ++lines;
                bool cutOwn = false, cutOther = false, graze = false;
                int by = -1;
                double cutW = 0, cutWOther = 0;
                for (double d = p.allowance + 0.02; d <= p.allowance + lpaEff + 2 * r - 0.02; d += 0.1) {
                    Vec2 X = c + nv * (d0 - d);
                    if (H.query(X, r - 1e-3).first >= zTab - 1e-6) continue;
                    cutW += 0.1;
                    bool own = HB[(size_t)pl.blk].query(X, r - 1e-3).first < zTab - 1e-6;
                    if (!own) {
                        cutWOther += 0.1;
                        if (by < 0) for (size_t bb = 0; bb < bls.size() && by < 0; ++bb) if (HB[bb].query(X, r - 1e-3).first < zTab - 1e-6) by = (int)bb;
                    }
                    if (target >= 0 && std::getenv("LINES")) {
                        double bestD = 1e9; size_t bestM = 0; double bz = 0;
                        sampleMoves(res.toolpath, 0.02, [&](Vec2 q, double z, size_t mi) { if (z < zTab - 1e-6) { double dd = geometry::distance(q, X); if (dd < r - 1e-3 && dd < bestD) { bestD = dd; bestM = mi; bz = z; } } });
                        std::printf("   cut pt: core (%.3f,%.3f) s %.3f/%.3f band d %.3f by move %zu (%s, blk %d) z %.3f dist %.3f\n", c.x, c.y, s, len, d, bestM, kn(mv[bestM].kind), blockOf[bestM], bz, bestD);
                    }
                }
                maxW = std::max(maxW, cutW);
                graze = cutW > 0 && cutW <= 0.5;
                cutOther = cutWOther > 0.5;
                cutOwn = !cutOther && cutW > 0.5;
                if (graze) ++grazes;
                bool middle = s >= c0 + 0.25 * (c1 - c0) && s <= c1 - 0.25 * (c1 - c0);
                if (!cutOwn && !cutOther) { ++intact; if (firstIntact < 0) firstIntact = s; lastIntact = s; continue; }
                if (middle && cutOther && !midO) { midO = true; midAt = c; midBy = by; }
                if (middle && cutOwn && !cutOther) midS = true;
            }
            if (lines == 0) continue;
            if (std::getenv("CONN") && (midO || midS || intact == 0) && !open) {
                // flood fill: part side (uncut) -> uncut waste -> slug (uncut waste beyond the kerf band)
                double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
                for (auto& q : pl.pts) { x0 = std::min(x0, q.x); y0 = std::min(y0, q.y); x1 = std::max(x1, q.x); y1 = std::max(y1, q.y); }
                const double grow = p.allowance + lpaEff + 2 * r + 1.5, h = 0.1;
                x0 -= grow; y0 -= grow; x1 += grow; y1 += grow;
                int nx = (int)((x1 - x0) / h) + 1, ny = (int)((y1 - y0) / h) + 1;
                const int wasteParity = ob.depth(pl.pts[pl.pts.size() / 2]) % 2;
                const double slugD = p.allowance + lpaEff + 2 * r + 0.3;
                std::vector<signed char> st2((size_t)nx * ny, 0);  // 0 cut, 1 part, 2 waste solid, 3 slug
                int slugCells = 0;
                for (int i = 0; i < nx; ++i) for (int j = 0; j < ny; ++j) {
                    Vec2 P{x0 + i * h, y0 + j * h};
                    const double zc = bottom + th * (std::getenv("CONNF") ? std::atof(std::getenv("CONNF")) : 1.0);
                    bool cut = (std::getenv("CONNOWN") ? HB[(size_t)pl.blk] : H).query(P, r - 1e-3).first < zc - 1e-6;
                    if (std::getenv("DIFFDUMP") && !cut && H.query(P, r - 1e-3).first < zc - 1e-6 && ob.depth(P) % 2 == wasteParity) {
                        double bestD = 1e9; size_t bestM = 0; double bz = 0;
                        sampleMoves(res.toolpath, 0.05, [&](Vec2 qq, double z, size_t mi) { if (z < zc - 1e-6) { double dd = geometry::distance(qq, P); if (dd < r - 1e-3 && dd < bestD) { bestD = dd; bestM = mi; bz = z; } } });
                        std::printf("   DIFF cell (%.2f,%.2f) cut by move %zu (%s blk %d) z %.3f\n", P.x, P.y, bestM, kn(mv[bestM].kind), blockOf[bestM], bz);
                    }
                    if (cut) continue;
                    bool waste = ob.depth(P) % 2 == wasteParity;
                    signed char v = 1;
                    if (waste) { v = ob.dist(P, slugD + 1) >= slugD ? 3 : 2; if (v == 3) ++slugCells; }
                    st2[(size_t)i * ny + j] = v;
                }
                std::vector<int> q; std::vector<char> seen(st2.size(), 0);
                for (size_t k = 0; k < st2.size(); ++k) if (st2[k] == 1) { q.push_back((int)k); seen[k] = 1; }
                bool reached = false;
                for (size_t qi = 0; qi < q.size() && !reached; ++qi) {
                    int k = q[qi], i = k / ny, j = k % ny;
                    const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
                    for (int e = 0; e < 4; ++e) {
                        int a2 = i + di[e], b2 = j + dj[e];
                        if (a2 < 0 || b2 < 0 || a2 >= nx || b2 >= ny) continue;
                        size_t kk = (size_t)a2 * ny + b2;
                        if (seen[kk] || st2[kk] == 0) continue;
                        if (st2[kk] == 3) { reached = true; break; }
                        seen[kk] = 1; q.push_back((int)kk);
                    }
                }
                std::printf("   CONN plateau block %d len %.3f at (%.3f,%.3f): %s (slug cells %d)\n", pl.blk, len, pl.pts[pl.pts.size() / 2].x, pl.pts[pl.pts.size() / 2].y, slugCells == 0 ? "NOSLUG" : reached ? "CONNECTED" : "SEVERED", slugCells);
            }
            if (midO) { ++midOther; if (firstMsg.empty()) { snprintf(b, sizeof b, "MID-CUT-OTHER: plateau block %d len %.3f core (%.3f,%.3f) bridge cut below tab top by block %d (max cut width %.2f of band %.2f)", pl.blk, len, midAt.x, midAt.y, midBy, maxW, lpaEff + 2 * r); firstMsg = b; } }
            else if (midS) { ++midOwn; if (firstMsg.empty()) { snprintf(b, sizeof b, "MID-CUT-OWN: plateau block %d len %.3f bridge cut by its own block (max cut width %.2f of band %.2f)", pl.blk, len, maxW, lpaEff + 2 * r); firstMsg = b; } }
            if (intact == 0) { ++lost; if (firstMsg.empty()) { snprintf(b, sizeof b, "LOST: plateau block %d len %.3f no intact bridge line (%d lines)", pl.blk, len, lines); firstMsg = b; } }
            else {
                double sh = (firstIntact - c0) + (c1 - lastIntact);
                if (sh > 0.3) { ++shortened; worstShort = std::max(worstShort, sh); }
            }
        }
        // ---- orphan roughing plateaus (a roughing tab with no finishing tab near it)
        int orphan = 0;
        if (p.separateLastPass) {
            for (const Plat& rp : plats[1]) {
                if (rp.s.back() < 2 * r + 0.2) continue;
                Vec2 mid = rp.pts[rp.pts.size() / 2];
                bool found = false;
                for (const Plat& fp : plats[0]) {
                    for (auto& q : fp.pts) if (geometry::distance(q, mid) < p.lastPassAllowance + 0.5) { found = true; break; }
                    if (found) break;
                }
                if (!found) ++orphan;
            }
        }
        // ---- count: single closed vector, constant number
        bool countOk = true;
        int keptFin = 0, leftOut = 0;
        if (cs.size() == 1 && !open && p.tabs.spacing == TabSpacing::ConstantNumber && !mv.empty() && th > 0 && !res.has(ProfileIssue::MergedWithNeighbour) && !res.has(ProfileIssue::TrimmedNearVector)) {
            // distinct finishing plateaus by midpoint (overcut repeats the first ones)
            std::vector<Vec2> mids;
            int finBlocks = 0;
            for (auto& bl : bls) finBlocks += bl.fin ? 1 : 0;
            for (const Plat& pl : plats[0]) {
                Vec2 mm = pl.pts[pl.pts.size() / 2];
                bool dup = false;
                for (auto& q : mids) if (geometry::distance(q, mm) < 0.05) dup = true;
                if (!dup) mids.push_back(mm);
            }
            keptFin = (int)mids.size();
            for (auto& d : res.diagnostics) {
                if (d.issue != ProfileIssue::TabsNotPlaced) continue;
                auto pos = d.message.find(" tab(s) left out");
                if (pos == std::string::npos) continue;
                size_t bb = d.message.rfind(": ", pos);
                leftOut += std::atoi(d.message.c_str() + bb + 2);
                auto of = d.message.find(" of ", bb);
                if (of != std::string::npos && std::atoi(d.message.c_str() + of + 4) != p.tabs.count) countOk = false;
            }
            if (finBlocks == (p.separateLastPass ? 1 : (int)bls.size()) && bls.size() >= 1) {
                ++st.countChecked;
                // plateaus shorter than the core margin are not seen as plateaus; count only when every tab is visible
                if (keptFin + leftOut != p.tabs.count || !countOk) {
                    ++st.countBad;
                    snprintf(b, sizeof b, "COUNT: %d finishing plateaus + %d left out != %d requested", keptFin, leftOut, p.tabs.count);
                    errs.push_back(b);
                }
            }
        }
        st.midOther += midOther > 0; st.midOwn += midOwn > 0; st.lost += lost > 0; st.shortened += shortened > 0; st.orphan += orphan > 0;
        st.worstShort = std::max(st.worstShort, worstShort); st.grazeScenes += grazes > 0;
        if (midOther || midOwn || lost) errs.push_back(firstMsg + " [midOther " + std::to_string(midOther) + " midOwn " + std::to_string(midOwn) + " lost " + std::to_string(lost) + " noSlugLines " + std::to_string(noSlug) + "]");
        if (orphan && verbose) errs.push_back("orphan roughing plateaus: " + std::to_string(orphan));
        if (shortened && verbose) { snprintf(b, sizeof b, "shortened plateaus %d worst %.3f", shortened, worstShort); errs.push_back(b); }
        if (!errs.empty() || target >= 0) {
            std::printf("scene %d fam %d side %d dir %d D %.3f allow %.3f sep %d lpa %.3f rev %d tabs n %d sp %d len %.2f th %.2f 3d %d avoid %d ramp %d lead %d out %d oc %.2f start %d depth %.2f/%.2f diags:", sc, f, (int)p.side, (int)p.direction, D, p.allowance, p.separateLastPass, p.lastPassAllowance, p.lastPassReverse, p.tabs.count, (int)p.tabs.spacing, p.tabs.length, p.tabs.thickness, p.tabs.threeD, p.tabs.avoidCorners, (int)p.ramp.type, (int)p.leads.type, p.leads.leadOut, p.leads.overcut, (int)p.startAt, p.passes.cutDepth, p.passes.passDepth);
            for (auto& d : res.diagnostics) std::printf(" %d", (int)d.issue);
            std::printf("\n");
            for (auto& e : errs) std::printf("   - %s\n", e.c_str());
            if (target >= 0) {
                for (auto& d : res.diagnostics) std::printf("   diag %d %s\n", (int)d.issue, d.message.c_str());
                FILE* fo = std::fopen("band.moves", "w"); dump(res.toolpath, fo); std::fclose(fo);
            }
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    std::printf("band seed %lu fam %d sep %d: scenes %d empty %d det %d struct %d gouge %d (worst %.5f) clamp %d finClass %d | plateaus %d: midOther %d midOwn %d lost %d shortened %d (worst %.2f) orphanRough %d grazeScenes %d | count %d/%d | %.1fs\n", seed, fam, forceSep, st.scenes, st.empty, st.det, st.structure, st.gouge, st.worstGouge, st.clampMissing, st.finClass, st.plateaus, st.midOther, st.midOwn, st.lost, st.shortened, st.worstShort, st.orphan, st.grazeScenes, st.countBad, st.countChecked, std::chrono::duration<double>(t1 - t0).count());
}
