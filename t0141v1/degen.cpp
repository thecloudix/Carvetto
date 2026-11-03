#include <cstdio>
#include <limits>
#include "clipper2/clipper.h"
#include "offset_batch.hpp"
#include "scenes.hpp"
namespace G = carvetto::geometry;
int main() {
  const size_t kNever = std::numeric_limits<size_t>::max();
  for (int variant = 0; variant < 6; ++variant) for (double d : {-0.2, 0.2, 0.6}) for (int jn = 0; jn < 3; ++jn) {
    JoinType J = jn == 0 ? JoinType::Round : jn == 1 ? JoinType::Square : JoinType::Sharp;
    std::vector<Contour> cs;
    if (variant % 2 == 0) cs.push_back(rectC({0, 30}, {5, 35}));            // global lowest CCW (largest y)
    else cs.push_back(rectC({0, 30}, {5, 35}).reversed());                  // global lowest CW
    for (int i = 0; i < 40; ++i) {
      double x = 5.0 * i;
      Contour needle; needle.closed = true;                                 // zero-area closed contour, lowest in its batch
      needle.spans.emplace_back(G::LineSpan{{x, 3}, {x + 1, 3}}); needle.spans.emplace_back(G::LineSpan{{x + 1, 3}, {x, 3}});
      Contour dot; dot.closed = true;                                        // closed contour collapsing to one grid point
      dot.spans.emplace_back(G::LineSpan{{x + 2, 2.5}, {x + 2 + 1e-8, 2.5}}); dot.spans.emplace_back(G::LineSpan{{x + 2 + 1e-8, 2.5}, {x + 2, 2.5}});
      Contour circ = Contour::circle({x + 0.5, 1.0}, 0.8, variant < 2 ? ArcDirection::CW : ArcDirection::CCW);
      if (variant >= 4) { cs.push_back(dot); cs.push_back(circ); } else { cs.push_back(needle); cs.push_back(circ); }
    }
    auto one = G::detail::offsetContoursBatched(cs, d, J, Tolerance{0.001, 1e-9, 0.001}, 2.0, kNever);
    auto def = offsetContours(cs, d, J, Tolerance{0.001, 1e-9, 0.001}, 2.0);
    auto ar = [](const std::vector<Contour>& v) { double a = 0; for (auto& c : v) for (auto& s : c.spans) { Vec2 p = startPoint(s), q = endPoint(s); a += 0.5 * (p.x * q.y - q.x * p.y); } return a; }; double a1 = ar(one), a2 = ar(def);
    std::printf("v%d d %+.1f j%d rings %zu %zu area %.6f %.6f %s\n", variant, d, jn, one.size(), def.size(), a1, a2, hashOut(one) == hashOut(def) ? "identical" : "differs");
  }
}
