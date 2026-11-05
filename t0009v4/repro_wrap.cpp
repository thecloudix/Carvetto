// T-0009 verifier repro: rounding-radius arc between two collinear lines -> near-full-circle arc in Sharp offset.
#include <cstdio>
#include <cmath>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
static double distIn(const Contour& in, Vec2 p) { double b = 1e300; for (auto& s : in.spans) b = std::min(b, carvetto::geometry::testing::distanceToSpanExact(p, s)); return b; }
static void probe(const char* nm, const Contour& in, double d, double mitre) {
    for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
        for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
            auto out = offsetOpenContour(in, d, s, j, Tolerance{}, mitre);
            double worst = 1e300, maxSweep = 0;
            for (auto& c : out) for (auto& sp : c.spans) {
                for (int k = 0; k <= 2000; ++k) worst = std::min(worst, distIn(in, pointAt(sp, k / 2000.0)));
                if (auto* a = std::get_if<ArcSpan>(&sp)) maxSweep = std::max(maxSweep, std::fabs(a->sweepAngle()));
            }
            std::printf("%s j%d s%d: chains %zu, min distance to input %.6g (d %.6g), max arc sweep %.4f rad\n", nm, (int)j, (int)s, out.size(), worst, d, maxSweep);
        }
}
int main() {
    { Contour in;
      in.spans.emplace_back(LineSpan{{-286.64931065391545, -667.2793053148431}, {-292.45208432818686, -664.71106545508371}});
      in.spans.emplace_back(ArcSpan{{-292.45208432818669, -664.71106545508337}, 3.9653404153571554e-13, -2.0344439357957027, -6.8817730429172244, ArcDirection::CW});
      in.spans.emplace_back(LineSpan{{-292.45208432818634, -664.7110654550836}, {-312.65154256130393, -655.77102114319712}});
      probe("c9200e83", in, 0.068653772818644987, 2.0); }
    { Contour in;
      in.spans.emplace_back(LineSpan{{3.6681549236583715, 9.7507350321250676}, {3.6385756065220707, 5.9441539321690033}});
      in.spans.emplace_back(ArcSpan{{3.6385756065220711, 5.9441539321690033}, 5.1267333246122462e-16, 3.1415926535897931, 8.8705911338854726, ArcDirection::CCW});
      in.spans.emplace_back(LineSpan{{3.6385756065220707, 5.9441539321690033}, {3.592169221517957, -0.027913213936717238}});
      probe("c90008db", in, 1.7780116989105186, 2.5774050735653171); }
    { Contour in;
      in.spans.emplace_back(LineSpan{{23.024066380607042, 4.3987616220873802}, {15.760295181442771, 13.776157000264629}});
      in.spans.emplace_back(ArcSpan{{15.760295181442773, 13.776157000264629}, 1.3435550662105094e-15, 3.1415926535897931, -1.8930167051923847, ArcDirection::CW});
      in.spans.emplace_back(LineSpan{{15.760295181442773, 13.776157000264627}, {12.098151102817612, 18.503917830764067}});
      probe("c9201371", in, 0.14125543441318877, 3.9089467380180274); }
    { // clean, at the origin: straight line, 1e-13 arc sweeping 4 rad, straight on
      Contour in; double r = 1e-13;
      in.spans.emplace_back(LineSpan{{-5, 0}, {0, 0}});
      ArcSpan a{{0, -r}, r, 1.5707963267948966, 1.5707963267948966 - 4.0, ArcDirection::CW};
      in.spans.emplace_back(a);
      in.spans.emplace_back(LineSpan{a.endPoint(), {5, 0}});
      probe("synthetic r1e-13 sweep -4", in, 0.5, 2.0); }
}
