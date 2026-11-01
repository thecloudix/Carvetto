// T-0017 round-2 verifier: fam5 0xf50011fd (axisCusp, exact continuous polyline). Two spikes whose
// Round caps' circles overlap by ~1.7e-7 mm; Round, Left, d = 0.81999987565196308.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    const double d = 0.81999987565196308;
    for (double shiftP1 : {0.0, -1e-7, -3e-7, -1e-6, -3e-6, -1e-5, -1e-4, -1e-3, 1e-7})
    for (double ty : {0.0, -2545.0}) {
        std::vector<Vec2> p{{-6.6012367772470677, 2550.4046822452678}, {1.5063151763440656e-16, 2545.0108170201156 + shiftP1},
                            {-6.3012367772470679, 2550.4046822452678}, {-8.9412365285509932, 2550.4046822452678},
                            {-8.9412365285509967, 2536.2105695160385}, {-4.1869101545926952, 2536.2105695160385},
                            {-1.5063151763440656e-16, 2543.370817441034}, {-4.1869101545926952, 2535.9105695160383}};
        Contour in;
        for (auto& q : p) q.y += ty;
        for (std::size_t i = 1; i < p.size(); ++i) in.spans.push_back(LineSpan{p[i - 1], p[i]});
        auto out = offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, Tolerance::defaults(), 2.0);
        double under = 0;
        for (auto& c : out)
            for (auto& s : c.spans)
                for (int k = 0; k <= 40000; ++k) {
                    Vec2 q = pointAt(s, k / 40000.0);
                    double m = 1e300;
                    for (auto& t : in.spans) m = std::min(m, testing::distanceToSpanExact(q, t));
                    under = std::max(under, d - m);
                }
        std::printf("P1 shift %-6g ty %-7g (overlap of the caps' circles %.3g): %zu chain(s), max inside d %.3g mm (allowances: snap %.3g, bridge sagitta %.3g)\n",
                    shiftP1, ty, 2 * d - (p[1].y - p[6].y), out.size(), under, std::min(5e-7 * std::max(1.0, d), 0.1 * 1e-6 / d), 1e-6 / (8 * d));
    }
}
