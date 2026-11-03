// prints: idx kind nrings hash   (offsetContours, both main and branch)
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include "scenes.hpp"
int main(int argc, char** argv) {
  int a = std::atoi(argv[1]), b = std::atoi(argv[2]);
  for (int i = a; i < b; ++i) { Gen g(0x141000ull + i); Scene s = g.make(i);
    auto out = offsetContours(s.cs, s.d, s.join, s.tol, s.mitre);
    std::printf("%d %d %zu %zu %016llx\n", i, s.kind, s.cs.size(), out.size(), (unsigned long long)hashOut(out)); }
}
