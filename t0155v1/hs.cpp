#include <cstdio>
#include <cmath>
#include <cstring>
#include <random>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/engrave.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static std::mt19937_64 rng; static const double PI = 3.14159265358979323846;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static unsigned long long H(unsigned long long h, const void* p, size_t n){ auto c=(const unsigned char*)p; for(size_t i=0;i<n;++i){h^=c[i]; h*=1099511628211ULL;} return h; }
static Contour rc(Vec2 o) {
  Contour c; int n = I(2, 6); Vec2 p = o, first = o; bool closed = I(0,2) != 0;
  if (I(0,4)==0) { c.append(geometry::ArcSpan::fullCircle({o.x, o.y}, U(0.5, 12), geometry::ArcDirection::CCW)); c.closed = true; return c; }
  for (int i = 0; i < n; ++i) { Vec2 q{p.x + U(-10,10), p.y + U(-10,10)}; if (closed && i == n-1) q = first;
    if (I(0,2)==0 && !(closed && i==n-1)) c.append(geometry::BezierSpan{p, {p.x+U(-5,5), p.y+U(-5,5)}, {q.x+U(-5,5), q.y+U(-5,5)}, q}); else c.append(geometry::LineSpan{p, q}); p = q; }
  c.closed = closed; return c;
}
int main(int, char** argv) {
  rng.seed(std::strtoul(argv[1], nullptr, 10)); int N = std::atoi(argv[2]);
  for (int sc = 0; sc < N; ++sc) {
    std::vector<Contour> cs; int nv = I(1,6); for (int i=0;i<nv;++i) cs.push_back(rc({U(-40,40),U(-40,40)}));
    tooling::Tool t; t.id="t"; t.name = I(0,1)? "my pocket cutter" : "V 60"; int tk = I(0,3);
    t.type = tk==0? tooling::ToolType::VBit : tk==1? tooling::ToolType::EndMill : tk==2? tooling::ToolType::BallNose : tooling::ToolType::Engraving;
    t.diameter = U(0.5, 6); t.includedAngle = U(PI/6, PI*0.6); t.tipDiameter = tk==3 ? 0.2 : 0.0;
    auto& cd = t.defaultCuttingData; cd.toolNumber=1; cd.feedRate=1000; cd.plungeRate=300; cd.spindleSpeed=12000; cd.passDepth=1; cd.stepover=0.3*t.diameter;
    EngraveParams E; E.depth = U(0.02, 2); E.passes = I(0,4)==0? I(1,30) : I(1,4); E.safeZ = U(0.5,10);
    E.strategy = I(0,1)? EngraveStrategy::Fill : EngraveStrategy::Outline; E.fillPattern = I(0,1)? PocketStrategy::Raster : PocketStrategy::Offset;
    E.rasterAngle = U(0, PI); E.stepover = I(0,2)==0? 0.0 : U(0.05, 1.0);
    auto r = engraveToolpath(cs, t, std::nullopt, E);
    unsigned long long h = 1469598103934665603ULL;
    for (auto& m : r.toolpath.moves) { int k=(int)m.kind; h=H(h,&k,sizeof k); h=H(h,&m.to,sizeof m.to); h=H(h,&m.centre,sizeof m.centre); }
    unsigned long long hd = 1469598103934665603ULL; for (auto& d : r.diagnostics) { int k=(int)d.issue; hd=H(hd,&k,sizeof k); }
    std::printf("%d %d moves %zu h %016llx issues %016llx ok %d\n", sc, (int)E.strategy, r.toolpath.moves.size(), h, hd, (int)r.ok());
    for (auto& d : r.diagnostics) std::printf("   %s\n", d.message.c_str());
  }
}
