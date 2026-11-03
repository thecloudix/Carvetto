// T-0017 verifier: slot ends whose radius is d (1 + 1e-6..1e-5), exact continuous input.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
static void run(const char* name, const Contour& in, double d) {
    auto out = offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, Tolerance::defaults(), 2.0);
    double gapMax = 0, under = 0, over = 0;
    for (auto& c : out) {
        for (std::size_t i = 0; i + 1 < c.spans.size(); ++i) gapMax = std::max(gapMax, distance(endPoint(c.spans[i]), startPoint(c.spans[i + 1])));
        for (auto& s : c.spans)
            for (int k = 0; k <= 200000; ++k) {
                Vec2 p = pointAt(s, k / 200000.0);
                double m = 1e300;
                for (auto& q : in.spans) m = std::min(m, testing::distanceToSpanExact(p, q));
                under = std::max(under, d - m); over = std::max(over, m - d);
            }
    }
    std::printf("%s: %zu chain(s), max junction gap %.3g mm, max inside d %.3g mm, max beyond d (Round) %.3g mm\n", name, out.size(), gapMax, under, over);
}
int main() {
    {   // (a) arc -> U-turn arc (radius d(1+1e-6)) -> line, near the origin: arc-arc junction gap
        Contour in;
        in.spans.push_back(ArcSpan{{-6.1835881590179307, -4.5609137211896442}, 7.6836642620876354, 0.63550719201869088, 0.59034783467233742, ArcDirection::CW});
        in.spans.push_back(ArcSpan{{0.5819774350952871, -0.027581994245153507}, 0.46028897665152524, -2.5512448189174557, 0.58554158006787915, ArcDirection::CCW});
        in.spans.push_back(LineSpan{{0.96558820755480146, 0.22679708759909528}, {-1.0290918042762391, 3.2348303527579176}});
        run("(a) arc/U-turn/line at origin, d=0.46028851636300894", in, 0.46028851636300894);
    }
    {   // (b) line -> U-turn arc -> line at ~5000 mm: Round beyond d
        Contour in;
        in.spans.push_back(LineSpan{{5188.4461692588793, -4233.4568454048558}, {5192.3005941971678, -4231.826230991167}});
        in.spans.push_back(ArcSpan{{5192.2285411430012, -4231.6559129279085}, 0.18493210993996412, -1.1705785180382842, 1.9688563277542461, ArcDirection::CCW});
        in.spans.push_back(LineSpan{{5192.1568557699384, -4231.4854397846402}, {5186.9319562479923, -4233.6825534429363}});
        run("(b) line/U-turn/line at 5e3, d=0.18493026061898363", in, 0.18493026061898363);
    }
    {   // (c) line -> U-turn arc -> arc at ~1e4 mm: line tilted inwards and outwards
        Contour in;
        in.spans.push_back(LineSpan{{10000, -7000}, {9999.7699140805944, -6998.8108121181322}});
        in.spans.push_back(ArcSpan{{9999.295782931682, -6998.9025477496334}, 0.48292418913950697, 0.19112003894738341, 3.3333454522343238, ArcDirection::CCW});
        in.spans.push_back(ArcSpan{{9979.9701657394107, -7002.654385997299}, 19.203512006852304, 0.1917527986445309, 0.10631536567386451, ArcDirection::CW});
        run("(c) line/U-turn/arc at 1e4, d=0.48291935994590746", in, 0.48291935994590746);
    }
}
