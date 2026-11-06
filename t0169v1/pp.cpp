// t0169v1 pp: Pocket CPU probes. usage: pp <case> <n> ; prints cpu, moves, retracts, hash
// cases: bigcirc (circle radius n mm, chordal 0.002 = distance, raster, 6 mm + 3 mm rest),
//        bigcircoff (same, offset strategy), gear (n teeth on a 200 mm gear, arcs both ways, 3 mm raster + 1 mm rest),
//        concave (square n mm with 8 big concave arc bites per side, raster + rest), islands (n x n circle islands, offset,
//        2 levels: travel at depth / routes), reflex (star with n points, arc edges, offset + rest), wavy (n Bezier lobes)
// env PP_DFR (distance = chordal*DFR, default 1), PP_CH (chordal, default 0.002), PP_STRAT (0/1 override)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::ArcDirection; using geometry::ArcSpan; using geometry::BezierSpan; using geometry::Contour; using geometry::LineSpan; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846, TAU = 2 * PI;
static PocketTool tool(const char* nm, int num, double d, double so, double pd) { PocketTool t; t.tool.id = nm; t.tool.name = nm; t.tool.type = tooling::ToolType::EndMill; t.tool.diameter = d; auto& cd = t.tool.defaultCuttingData; cd.toolNumber = num; cd.feedRate = 1000; cd.plungeRate = 300; cd.spindleSpeed = 18000; cd.stepover = so; cd.passDepth = pd; return t; }
static geometry::Span arcEdge(Vec2 P, Vec2 Q, double s) { Vec2 d = Q - P; double len = d.length(); Vec2 n = (d * (1.0 / len)).perp(); double h = 0.5 * len, R = (h * h + s * s) / (2 * std::fabs(s)); Vec2 c = (P + Q) * 0.5 - n * ((R - std::fabs(s)) * (s > 0 ? 1 : -1));
  double a0 = std::atan2(P.y - c.y, P.x - c.x), a1 = std::atan2(Q.y - c.y, Q.x - c.x); Vec2 apex = (P + Q) * 0.5 + n * s; ArcSpan ccw{c, R, a0, a1, ArcDirection::CCW}, cw{c, R, a0, a1, ArcDirection::CW}; return distance(ccw.pointAt(0.5), apex) < distance(cw.pointAt(0.5), apex) ? ccw : cw; }
static unsigned long long HH = 1469598103934665603ULL; static void hb(const void* p, size_t n) { auto* q = (const unsigned char*)p; for (size_t i = 0; i < n; ++i) { HH ^= q[i]; HH *= 1099511628211ULL; } }
int main(int argc, char** argv) {
  std::string c = argv[1]; double n = atof(argv[2]); std::vector<Contour> cs; std::vector<PocketTool> tools; PocketParams p; p.cutDepth = 3; p.safeZ = 5;
  double ch = getenv("PP_CH") ? atof(getenv("PP_CH")) : 0.002, dfr = getenv("PP_DFR") ? atof(getenv("PP_DFR")) : 1.0; p.tolerance.chordal = ch; p.tolerance.distance = std::max(1e-6, ch * dfr);
  if (c == "bigcirc" || c == "bigcircoff") { cs.push_back(Contour::circle({0, 0}, n)); p.strategy = c == "bigcirc" ? PocketStrategy::Raster : PocketStrategy::Offset; p.rasterAngle = 0.3; double D = std::max(6.0, n / 100); tools = {tool("A", 1, D, 0.5 * D, 3), tool("B", 2, D / 2, 0.2 * D, 3)}; }
  else if (c == "flower") {  // circle R = PP_R (default 1500) whose wall is n concave arc bites (reflex junctions, long concave walls); raster 15 mm + 5 mm rest
    int k = (int)n; double R = getenv("PP_R") ? atof(getenv("PP_R")) : 1500; Contour g; g.closed = true; std::vector<Vec2> v; for (int i = 0; i < k; ++i) v.push_back(Vec2::fromAngle(TAU * i / k, R));
    for (int i = 0; i < k; ++i) { Vec2 P = v[i], Q = v[(i + 1) % k]; g.append(arcEdge(P, Q, 0.08 * distance(P, Q))); } cs.push_back(g);
    p.strategy = PocketStrategy::Raster; p.rasterAngle = 0.3; double D = std::max(3.0, R / 100); tools = {tool("A", 1, D, 0.5 * D, 3), tool("B", 2, D / 3, 0.4 * D / 3, 3)}; }
  else if (c == "gear") { int k = (int)n; double R = 200, dep = 6; std::vector<Vec2> v; for (int i = 0; i < 2 * k; ++i) v.push_back(Vec2::fromAngle(TAU * i / (2 * k), i % 2 ? R - dep : R)); Contour g; g.closed = true;
    for (int i = 0; i < 2 * k; ++i) { Vec2 P = v[i], Q = v[(i + 1) % (2 * k)]; g.append(arcEdge(P, Q, (i % 2 ? 0.2 : -0.2) * distance(P, Q))); } cs.push_back(g); p.strategy = PocketStrategy::Raster; p.rasterAngle = 0.2; tools = {tool("A", 1, 3, 1.2, 3), tool("B", 2, 1, 0.4, 3)}; }
  else if (c == "concave") { double L = n; Contour g; g.closed = true; Vec2 cr[4] = {{0, 0}, {L, 0}, {L, L}, {0, L}}; for (int e = 0; e < 4; ++e) for (int j = 0; j < 8; ++j) { Vec2 P = cr[e] + (cr[(e + 1) % 4] - cr[e]) * (j / 8.0), Q = cr[e] + (cr[(e + 1) % 4] - cr[e]) * ((j + 1) / 8.0); g.append(arcEdge(P, Q, 0.12 * distance(P, Q))); }
    cs.push_back(g); p.strategy = PocketStrategy::Raster; p.rasterAngle = 0.4; double D = std::max(3.0, L / 200); tools = {tool("A", 1, D, 0.4 * D, 3), tool("B", 2, D / 3, 0.4 * D / 3, 3)}; }
  else if (c == "islands") { int k = (int)n; double pitch = 10; cs.push_back(Contour::rectangle(geometry::Box2{{-5, -5}, {k * pitch + 5, k * pitch + 5}})); for (int i = 0; i < k; ++i) for (int j = 0; j < k; ++j) cs.push_back(Contour::circle({i * pitch + 5 + 0.7 * std::sin(i * 1.3 + j), j * pitch + 5}, 2.5 + 0.5 * std::cos(i + 2.0 * j)));
    p.strategy = PocketStrategy::Offset; p.cutDepth = 6; tools = {tool("A", 1, 2, 0.8, 3), tool("B", 2, 0.8, 0.3, 3)}; }
  else if (c == "reflex") { int k = (int)n; double R = 100, inner = 0.8; std::vector<Vec2> v; for (int i = 0; i < 2 * k; ++i) v.push_back(Vec2::fromAngle(TAU * i / (2 * k), i % 2 ? R * inner : R)); Contour g; g.closed = true;
    for (int i = 0; i < 2 * k; ++i) { Vec2 P = v[i], Q = v[(i + 1) % (2 * k)]; g.append(arcEdge(P, Q, -0.05 * distance(P, Q))); } cs.push_back(g); p.strategy = PocketStrategy::Offset; tools = {tool("A", 1, 4, 1.6, 3), tool("B", 2, 1, 0.4, 3)}; }
  else { int k = (int)n; double R = 100; std::vector<Vec2> v; for (int i = 0; i < k; ++i) v.push_back(Vec2::fromAngle(TAU * i / k, R * (0.8 + 0.2 * std::sin(i * 2.7)))); Contour g; g.closed = true;
    for (int i = 0; i < k; ++i) { Vec2 P = v[i], Q = v[(i + 1) % k], d = Q - P, nn = d.normalized().perp(); g.append(BezierSpan{P, P + d * 0.33 + nn * (0.3 * d.length()), P + d * 0.67 - nn * (0.3 * d.length()), Q}); } cs.push_back(g); p.strategy = PocketStrategy::Raster; p.rasterAngle = 1.0; tools = {tool("A", 1, 4, 1.6, 3), tool("B", 2, 1, 0.4, 3)}; }
  if (getenv("PP_STRAT")) p.strategy = atoi(getenv("PP_STRAT")) ? PocketStrategy::Raster : PocketStrategy::Offset;
  double t0 = double(std::clock()) / CLOCKS_PER_SEC; auto r = pocketToolpath(cs, tools, p); double t1 = double(std::clock()) / CLOCKS_PER_SEC;
  size_t mv = 0, rt = 0; for (auto& tp : r.toolpaths) { mv += tp.moves.size(); rt += tp.count(MoveKind::Retract); for (auto& m : tp.moves) { hb(&m.kind, sizeof m.kind); hb(&m.to, sizeof m.to); } } for (auto& d : r.diagnostics) hb(d.message.data(), d.message.size());
  if (getenv("PP_DIAG")) for (auto& d : r.diagnostics) std::printf("DIAG %s\n", d.message.c_str());
  std::printf("%s n=%g cpu %.3f moves %zu retracts %zu diags %zu hash %016llx\n", c.c_str(), n, t1 - t0, mv, rt, r.diagnostics.size(), HH);
}
