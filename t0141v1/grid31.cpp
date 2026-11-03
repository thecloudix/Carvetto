// threshold edge: k columns x m rows of rectangles (k batches side by side), and the same with jitter
#include <cstdio>
#include <cstdlib>
#include "scenes.hpp"
int main(int argc, char** argv) {
  for (int k : {30, 31, 32, 33}) for (int m : {1, 50}) for (int jt : {0, 1, 2}) {
    std::vector<Contour> cs; std::mt19937_64 r(k * 100 + m);
    for (int i = 0; i < k; ++i) for (int j = 0; j < m; ++j) { double y = 0.7 * j + (jt ? (r() % 100) * 1e-3 : 0); 
      Contour c = rectC({0.7 * i, y}, {0.7 * i + 0.5, y + 0.3}); if (jt == 2 && (i + j) % 3 == 0) c = c.reversed(); cs.push_back(c); }
    for (double d : {-0.1 + 3e-6, -0.05, 0.05, 0.0999}) for (int jn = 0; jn < 3; ++jn) {
      JoinType J = jn == 0 ? JoinType::Round : jn == 1 ? JoinType::Square : JoinType::Sharp;
      auto out = offsetContours(cs, d, J, Tolerance{0.001, 1e-9, 0.001}, 2.0);
      std::printf("%d %d %d %g %d %zu %016llx\n", k, m, jt, d, jn, out.size(), (unsigned long long)hashOut(out)); } }
}
