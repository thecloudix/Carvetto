// T-0131 verifier: per-doubling ratios with the repo's interleaved CPU timer (cpu_scaling.hpp).
// usage: pt case n pairs [variant]  -> times n vs 2n (env as pf.cpp: ANG, RD, TABS, RASTER, NOSEP, ...)
#define main pf_main_unused
#include "pf.cpp"
#undef main
#include <functional>
#include <memory>
#include "cpu_scaling.hpp"
int main(int argc, char** argv) {
  std::string c = argv[1]; size_t n = strtoul(argv[2], 0, 10); int pairs = atoi(argv[3]); int var = argc > 4 ? atoi(argv[4]) : 0;
  auto pg = [&](size_t m) { rng.seed(77); return var == 0 ? textPage(m) : var == 1 ? page1(m) : page2(m); };
  size_t sink = 0;
  auto make = [&](size_t m) -> std::function<void()> {
    if (c == "comb") {
      auto cb = std::make_shared<Contour>(combC((int)m)); PocketParams p; p.cutDepth = 9; p.strategy = getenv("OFFSET") ? PocketStrategy::Offset : PocketStrategy::Raster;
      p.rasterAngle = getenv("ANG") ? atof(getenv("ANG")) * PI / 180 : 0.0; double d = getenv("TOOL") ? atof(getenv("TOOL")) : 0.3;
      std::vector<PocketTool> tools{ptool("B", 1, d, d * 0.4, 9)}; if (getenv("RD")) { double rd = atof(getenv("RD")); tools.push_back(ptool("S", 2, rd, rd * 0.4, 9)); }
      return [=, &sink] { auto r = pocketToolpath({*cb}, tools, p); sink += r.toolpaths[0].moves.size(); };
    }
    auto cs = std::make_shared<std::vector<Contour>>(pg(m));
    if (c == "sep" || c == "tabs") {
      ProfileParams p; p.passes.cutDepth = 3; if (c == "sep" && !getenv("NOSEP")) { p.separateLastPass = true; p.lastPassAllowance = 0.2; }
      if (c == "tabs" || getenv("TABS")) { p.tabs.enabled = true; p.tabs.length = 1; p.tabs.count = 1; p.tabs.thickness = 1; }
      if (getenv("SIDE")) p.side = (ProfileSide)atoi(getenv("SIDE"));
      return [=, &sink] { auto r = profileToolpath(*cs, ptoolP(), p); sink += r.toolpath.moves.size(); };
    }
    PocketParams p; p.cutDepth = 3; if (getenv("RASTER")) { p.strategy = PocketStrategy::Raster; p.rasterAngle = getenv("ANG") ? atof(getenv("ANG")) * PI / 180 : 0.0; }
    double rd = getenv("RD") ? atof(getenv("RD")) : 0.3; std::vector<PocketTool> tools{ptool("EM 1", 1, 1.0, 0.4, 3.0)}; if (c == "rest") tools.push_back(ptool("EM .3", 2, rd, rd * 0.4, 3.0));
    return [=, &sink] { auto r = pocketToolpath(*cs, tools, p); sink += r.toolpaths.size(); };
  };
  auto s = make(n), l = make(2 * n);
  auto t = carvetto::test::interleavedCpuSeconds(pairs, s, l);
  std::printf("%s var %d n %zu: small %.3f large %.3f ratio %.3f (minima %.3f, pair %.3f) reps %d\n", c.c_str(), var, n, t.smallSeconds, t.largeSeconds, t.ratio(), t.largeSeconds / t.smallSeconds, t.pairRatio, t.reps);
}
