#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
int main() {
    Contour in;
    in.spans.push_back(LineSpan{{5188.4461692588793,-4233.4568454048558},{5192.3005941971678,-4231.826230991167}});
    in.spans.push_back(ArcSpan{{5192.2285411430012,-4231.6559129279085},0.18493210993996412,-1.1705785180382842,1.9688563277542461,ArcDirection::CCW});
    in.spans.push_back(LineSpan{{5192.1568557699384,-4231.4854397846402},{5186.9319562479923,-4233.6825534429363}});
    for (int i=0;i<2;++i) std::printf("input gap %d: %.3g\n", i, distance(endPoint(in.spans[i]), startPoint(in.spans[i+1])));
    double d = 0.18493026061898363;
    auto out = offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, Tolerance::defaults(), 3.64);
    for (auto& c : out) for (auto& s : c.spans) {
        double wmax=-1e9, wmin=1e9; double tmax=0;
        for (int k=0;k<=20000;++k){ Vec2 p=pointAt(s,k/20000.0); double m=1e300; for(auto& q:in.spans) m=std::min(m,testing::distanceToSpanExact(p,q)); if(m-d>wmax){wmax=m-d;tmax=k/20000.0;} wmin=std::min(wmin,m-d);}
        std::printf("span len %.6g: max over d %.3g at t=%.5f, min %.3g\n", length(s), wmax, tmax, wmin);
    }
}
