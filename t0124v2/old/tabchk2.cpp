// Tab-material survival check (verifier throwaway): for every tab plateau (rise ... plunge at the
// tab top) the material on the cut centreline at least a tool radius inside the plateau ends must
// not be cut below the tab top by ANY move (other passes, other levels, leads, ramps, overcut).
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
#include <unordered_map>
#include "hash.hpp"

int main(int argc, char** argv) {
    unsigned long seed = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 1;
    int N = argc > 2 ? std::atoi(argv[2]) : 300;
    int target = argc > 3 ? std::atoi(argv[3]) : -1;
    int forceSep = argc > 4 ? std::atoi(argv[4]) : -1;
    rng.seed(seed);
    int bad = 0, plateaus = 0, scenes = 0, folded = 0;
    for (int sc = 0; sc < N; ++sc) {
        std::vector<Contour> cs;
        int k = I(0, 3);
        double R = U(8, 30);
        if (k == 0) cs.push_back(anyClosed({0, 0}, R));
        else if (k == 1) cs.push_back(Contour::rectangle(geometry::Box2{{0, 0}, {U(20, 120), U(15, 80)}}));
        else if (k == 2) cs.push_back(Contour::circle({0, 0}, R));
        else { cs.push_back(anyClosed({0, 0}, R)); cs.push_back(anyClosed({3.2 * 30, 0}, U(8, 25))); }
        ProfileParams p;
        double D = std::vector<double>{3.175, 6.0, 8.0, 12.0}[(size_t)I(0, 3)];
        auto tool = endMill(D);
        p.side = I(0, 1) ? ProfileSide::Outside : ProfileSide::Inside;
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = I(0, 1) ? 0.0 : U(-0.2, 0.5);
        p.passes.startDepth = I(0, 3) == 0 ? U(0, 3) : 0.0;
        p.passes.cutDepth = U(3, 20);
        p.passes.passDepth = U(1, 8);
        p.separateLastPass = forceSep >= 0 ? forceSep : I(0, 1);
        p.lastPassAllowance = U(0.05, 1.0);
        p.lastPassReverse = I(0, 1);
        p.tabs.enabled = true;
        p.tabs.length = U(2, 20);
        p.tabs.thickness = U(0.5, std::min(5.0, p.passes.cutDepth - 0.2));
        p.tabs.spacing = I(0, 1) ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance;
        p.tabs.count = I(1, 8);
        p.tabs.distance = U(15, 120);
        p.tabs.avoidCorners = I(0, 1);
        p.tabs.threeD = I(0, 2) == 0;
        int rk = I(0, 3);
        p.ramp.type = (RampType)rk;
        p.ramp.distance = U(2, 40);
        p.ramp.onLeadIn = I(0, 3) == 0;
        p.leads.type = (LeadType)I(0, 2);
        p.leads.length = U(1, 15);
        p.leads.radius = U(1, 12);
        p.leads.leadOut = I(0, 1);
        p.leads.overcut = I(0, 1) ? U(0, 20) : 0.0;
        p.startAt = (StartAt)I(0, 2);
        p.startAnchor = (BoxAnchor)I(0, 8);
        p.startPosition = {U(-50, 50), U(-50, 50)};
        if (target >= 0 && sc != target) continue;
        ++scenes;
        auto res = profileToolpath(cs, tool, p);
        const auto& mv = res.toolpath.moves;
        const double r = D / 2, zTab = p.passes.bottomZ() + p.tabs.thickness;
        // all cutting samples
        std::vector<int> blockOf(mv.size(), 0);
        { int b = -1; for (size_t x = 0; x < mv.size(); ++x) { if (mv[x].kind == MoveKind::Rapid && (x == 0 || mv[x-1].kind == MoveKind::Retract)) ++b; blockOf[x] = std::max(b, 0); } }
        int nb = mv.empty() ? 0 : blockOf.back() + 1;
        std::vector<SampleHash> HB((size_t)nb);
        sampleMoves(res.toolpath, 0.02, [&](Vec2 q, double z, size_t mi) { if (z < 0) HB[(size_t)blockOf[mi]].add(q, z); });
        // plateaus: maximal runs of moves at exactly zTab that start with a vertical rise (feed up) and end with a plunge
        int badHere = 0;
        std::string first;
        for (size_t i = 1; i < mv.size(); ++i) {
            bool rise = mv[i].kind == MoveKind::Feed && mv[i].to.z == zTab && mv[i - 1].to.z < zTab && mv[i].to.xy() == mv[i - 1].to.xy();
            bool rise3d = p.tabs.threeD && mv[i].to.z == zTab && mv[i - 1].to.z < zTab;
            if (!rise && !rise3d) continue;
            size_t j = i + 1;
            std::vector<size_t> run;
            while (j < mv.size() && mv[j].to.z == zTab && mv[j].kind != MoveKind::Retract) { run.push_back(j); ++j; }
            if (j >= mv.size() || mv[j].to.z >= zTab || mv[j].kind == MoveKind::Retract) continue;  // must go back down
            // sample along the run
            std::vector<Vec2> pts;
            Toolpath sub{"", {}, {}};
            sub.moves.push_back(Move::rapid(mv[i].to));
            for (size_t x : run) sub.moves.push_back(mv[x]);
            sampleMoves(sub, 0.05, [&](Vec2 q, double, size_t) { if (pts.empty() || geometry::distance(pts.back(), q) > 1e-7) pts.push_back(q); });
            double len = 0;
            for (size_t a = 1; a < pts.size(); ++a) len += geometry::distance(pts[a - 1], pts[a]);
            if (len < 2 * r + 0.1) continue;
            { bool fold = false; for (size_t a = 2; a < pts.size(); ++a) { Vec2 d1 = pts[a-1]-pts[a-2], d2 = pts[a]-pts[a-1]; double l1 = std::hypot(d1.x,d1.y), l2 = std::hypot(d2.x,d2.y); if (l1 > 1e-9 && l2 > 1e-9 && (d1.x*d2.x+d1.y*d2.y) < -0.9*l1*l2) fold = true; }
              if (fold) { ++folded; continue; } }
            double s = 0;
            ++plateaus;
            for (size_t a = 1; a < pts.size(); ++a) {
                s += geometry::distance(pts[a - 1], pts[a]);
                double margin = r + (p.tabs.threeD ? 0 : 0) + 0.05;
                if (std::getenv("MID") ? std::fabs(s - len / 2) > 0.03 : (s < margin || s > len - margin)) continue;
                double zmin = 1e18; int who = -1;
                for (int bb = 0; bb < nb; ++bb) { if (bb == blockOf[i]) continue; double zz = HB[(size_t)bb].query(pts[a], r - 1e-3).first; if (zz < zmin) { zmin = zz; who = bb; } }
                if (zmin < zTab - 1e-6) {
                    ++badHere;
                    if (first.empty()) { char b[200]; std::snprintf(b, sizeof b, "tab centreline (%.4f,%.4f) (plateau from move %zu block %d/%d, len %.3f, s %.3f, fromEnd %.3f) cut to %.4f < tab top %.4f by block %d", pts[a].x, pts[a].y, i, blockOf[i], nb, len, s, std::min(s, len - s), zmin, zTab, who); first = b; }
                    break;
                }
            }
        }
        if (badHere || target >= 0) {
            ++bad;
            std::printf("scene %d k %d side %d sep %d lpa %.3f D %.3f allow %.3f tabs n %d len %.2f th %.2f 3d %d sp %d ramp %d lead %d out %d oc %.2f: %d bad plateaus; %s diags:", sc, k, (int)p.side, p.separateLastPass, p.lastPassAllowance, D, p.allowance, p.tabs.count, p.tabs.length, p.tabs.thickness, p.tabs.threeD, (int)p.tabs.spacing, rk, (int)p.leads.type, p.leads.leadOut, p.leads.overcut, badHere, first.c_str());
            for (auto& d : res.diagnostics) std::printf(" %d", (int)d.issue);
            std::printf("\n");
            if (target >= 0) { for (auto& d : res.diagnostics) std::printf("   diag %s\n", d.message.c_str()); FILE* f = std::fopen("tab.moves", "w"); dump(res.toolpath, f); std::fclose(f); }
        }
    }
    std::printf("tabchk seed %lu: %d scenes, %d plateaus (%d folded skipped), %d scenes with cut-through tabs\n", seed, scenes, plateaus, folded, bad);
}
