// timings: mode row|jit|col|spiral|grid|holes  n -> CPU seconds best of reps, ring count
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include "scenes.hpp"
int main(int argc, char** argv) {
  const char* m = argv[1]; int n = std::atoi(argv[2]); int reps = argc > 3 ? std::atoi(argv[3]) : 3;
  std::vector<Contour> cs; double d = -(0.1 - 3e-6); std::mt19937_64 r(5);
  if (!std::strcmp(m, "row") || !std::strcmp(m, "jit")) for (int i = 0; i < n; ++i) { double x = 0.7 * i, h = i % 2 ? 0.2 : 0.3; double y0 = 0;
      if (m[0] == 'j') { y0 = (r() % 1000) * 1e-4; h += (r() % 1000) * 1e-4; } cs.push_back(rectC({x, y0}, {x + 0.5, y0 + h})); }
  else if (!std::strcmp(m, "col")) for (int i = 0; i < n; ++i) { double y = 0.7 * i; cs.push_back(rectC({0, y}, {0.5, y + (i % 2 ? 0.2 : 0.3)})); }
  else if (!std::strcmp(m, "grid")) { int k = int(std::sqrt(double(n))); for (int i = 0; i < k; ++i) for (int j = 0; j < k; ++j) cs.push_back(rectC({0.7 * i, 0.7 * j}, {0.7 * i + 0.5, 0.7 * j + 0.3})); }
  else if (!std::strcmp(m, "holes")) { // lone CW holes side by side under a CCW lowest contour (acknowledged adversarial)
      cs.push_back(rectC({0, 10}, {1, 11})); for (int i = 1; i < n; ++i) cs.push_back(rectC({0.7 * i, 0}, {0.7 * i + 0.5, 0.3}).reversed()); }
  else if (!std::strcmp(m, "spiral")) { // nested corners: alternating vertical / horizontal bars, each peeled by one cut
      double N = 2.0 * n; for (int k = 0; k < n; ++k) { double o = 2.0 * k;
        if (k % 2 == 0) cs.push_back(rectC({o, 0}, {o + 0.5, N - o})); else cs.push_back(rectC({o + 2, N - o}, {N, N - o + 0.5})); } d = -0.1; }
  if (!std::strcmp(m, "plate")) { cs.push_back(rectC({-1, -1}, {0.7 * n + 1, 1.3})); for (int i = 0; i < n; ++i) cs.push_back(rectC({0.7 * i, 0}, {0.7 * i + 0.5, i % 2 ? 0.2 : 0.3}).reversed()); d = -0.05; }
  if (!std::strcmp(m, "touch")) { for (int i = 0; i < n; ++i) cs.push_back(rectC({0.7 * i, 0}, {0.7 * i + 0.5, i % 2 ? 0.2 : 0.3})); d = 0.15; }
  double best = 1e30; size_t rings = 0;
  for (int k = 0; k < reps; ++k) { std::clock_t t0 = std::clock(); rings = offsetContours(cs, d, JoinType::Round, Tolerance{0.001, 1e-9, 0.001}).size();
    best = std::min(best, double(std::clock() - t0) / CLOCKS_PER_SEC); }
  std::printf("%s %d %.4f s rings %zu\n", m, n, best, rings);
}
