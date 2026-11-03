#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include "shapes.hpp"
using namespace carvetto::geometry;
static double cpu() { timespec t; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
int main(int argc, char** argv) {
    const char* nm = argv[1]; double s = std::atof(argv[2]); size_t ms = argc > 3 ? std::atol(argv[3]) : 0;
    vs::Shape sh; for (auto& x : vs::all()) if (x.name == nm) sh = x;
    for (auto& c : sh.cs) c.transform(Transform2::scaling(s));
    Tolerance tol; MedialAxisOptions o; o.maxSamples = ms;
    double c0 = cpu(); MedialAxis ax = medialAxis(sh.cs, tol, o); double c1 = cpu() - c0;
    size_t pts = 0; for (auto& b : ax.branches) pts += b.points.size();
    std::printf("%s scale=%g cpu=%.2fs samples=%zu capped=%d branches=%zu pts=%zu\n", nm, s, c1, ax.boundarySamples, (int)ax.accuracyCapped, ax.branches.size(), pts);
}
