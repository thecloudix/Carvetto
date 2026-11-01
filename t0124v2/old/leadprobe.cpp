// A/B: StartMoved / LeadShortened / LeadDropped and worst clearance vs r + allowance.
#include "oracle.hpp"
using namespace v; using namespace carvetto::toolpath;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
int main(int argc, char** argv) {
    rng.seed(std::strtoul(argv[1], nullptr, 10)); int N = std::atoi(argv[2]);
    for (int sc = 0; sc < N; ++sc) {
        std::vector<Contour> cs;
        double R = U(10, 30);
        cs.push_back(anyClosed({0, 0}, R));
        int k = I(0, 2);
        if (k >= 1) { auto b = cs[0].bounds(); cs.push_back(Contour::rectangle(geometry::Box2{{b.min.x - U(6.2, 18), U(b.min.y, b.max.y - 6)}, {b.min.x - U(5.5, 6.2) + 0.0, U(b.min.y, b.max.y - 6) + 6}})); }
        if (k == 2) { cs.push_back(Contour::circle({0, 0}, R * 0.25)); }
        ProfileParams p;
        double D = std::vector<double>{3.175, 6.0, 8.0}[(size_t)I(0, 2)];
        p.side = I(0, 1) ? ProfileSide::Outside : ProfileSide::Inside;
        p.direction = I(0, 1) ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = I(0, 2) == 0 ? 0.0 : U(0.0, 0.5);
        p.passes.cutDepth = U(3, 10); p.passes.passDepth = U(2, 6);
        p.leads.type = I(0, 1) ? LeadType::Straight : LeadType::Circular;
        p.leads.length = U(2, 15); p.leads.angle = U(0.1, PI / 2); p.leads.radius = U(1, 10); p.leads.sweep = U(10, 90) * PI / 180;
        p.leads.leadOut = I(0, 1);
        p.startAt = (StartAt)I(0, 2); p.startAnchor = (BoxAnchor)I(0, 8);
        auto res = profileToolpath(cs, endMill(D), p);
        Obstacles ob(cs);
        double worst = 1e9;
        sampleMoves(res.toolpath, 0.02, [&](Vec2 q, double z, size_t) { if (z < 0) worst = std::min(worst, ob.dist(q, D) - (D / 2 + p.allowance)); });
        std::printf("%d %d %d %d %.5f %s\n", sc, (int)res.has(ProfileIssue::StartMoved), (int)res.has(ProfileIssue::LeadShortened), (int)res.has(ProfileIssue::LeadDropped), worst, res.has(ProfileIssue::MergedWithNeighbour) || res.has(ProfileIssue::TrimmedNearVector) ? "M" : "-");
    }
}
