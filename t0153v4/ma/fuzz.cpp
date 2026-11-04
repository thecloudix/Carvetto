#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include "shapes.hpp"
using namespace carvetto::geometry;
using vs::poly; using vs::rev; using vo::PI;
static const double NaN = std::numeric_limits<double>::quiet_NaN(), Inf = std::numeric_limits<double>::infinity();
struct Case { std::string name; std::vector<Contour> cs; bool checkRadius; };
static Case gen(int idx, std::mt19937_64& R) {
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(R); };
    auto I = [&](int a, int b) { return std::uniform_int_distribution<int>(a, b)(R); };
    int kind = idx % 16;
    Case c; c.checkRadius = true;
    switch (kind) {
    case 0: { int n = I(3, 60); std::vector<Vec2> p; for (int i = 0; i < n; ++i) { double a = 2 * PI * i / n; double r = U(1, 10); p.push_back({r * std::cos(a), r * std::sin(a)}); if (I(0, 4) == 0) p.push_back(p.back()); if (I(0,5)==0 && p.size()>1) p.push_back((p[p.size()-1]+p[p.size()-2])*0.5); } c = {"star-dup-collinear", {poly(p)}, true}; break; }
    case 1: { int n = I(3, 30); std::vector<Vec2> p; for (int i = 0; i < n; ++i) p.push_back({U(0, 10), U(0, 10)}); c = {"self-intersecting", {poly(p)}, false}; break; }
    case 2: { int n = I(3, 40); std::vector<Vec2> p; for (int i = 0; i < n; ++i) { double a = 2 * PI * i / n; double r = I(2, 8); p.push_back({std::round(r * std::cos(a)), std::round(r * std::sin(a))}); } c = {"grid-snapped", {poly(p)}, false}; break; }
    case 3: { double g = std::vector<double>{0, 1e-12, 1e-9, 1e-6, 1e-3, 0.019, 0.02, 0.021, 0.05}[I(0, 8)];
        c = {"near-touching-squares g=" + std::to_string(g), {poly({{0, 0}, {5, 0}, {5, 5}, {0, 5}}), poly({{5 + g, U(-2, 2)}, {10, 0}, {10, 5}, {5 + g, 5 + U(-2, 2)}})}, false}; break; }
    case 4: { double g = std::vector<double>{0, 1e-9, 1e-5, 0.01, 0.019, 0.021}[I(0, 5)];
        c = {"hole-near-outer g=" + std::to_string(g), {poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), rev(poly({{g, 3}, {4, 3}, {4, 6}, {g, 6}}))}, g > 0}; break; }
    case 5: { double a = std::pow(10.0, -U(2, 12)); c = {"sliver", {poly({{0, 0}, {30, 0}, {30 * std::cos(a), 30 * std::sin(a)}})}, true}; break; }
    case 6: { double o = std::pow(10.0, I(5, 15)); auto sh = vs::all()[I(0, 20)]; for (auto& ct : sh.cs) ct.transform(Transform2::translation({o, -o})); c = {"huge-offset " + sh.name + " " + std::to_string(o), sh.cs, false}; break; }
    case 7: { double s = std::pow(10.0, I(-120, 8)); auto sh = vs::all()[I(0, 20)]; for (auto& ct : sh.cs) ct.transform(Transform2::scaling(s)); c = {"scaled " + sh.name + " " + std::to_string(s), sh.cs, false}; break; }
    case 8: { Contour k = poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}); int w = I(0, 5);
        if (w == 0) std::get<LineSpan>(k.spans[1]).end.x = NaN, std::get<LineSpan>(k.spans[2]).start.x = NaN;
        if (w == 1) std::get<LineSpan>(k.spans[1]).end.y = Inf, std::get<LineSpan>(k.spans[2]).start.y = Inf;
        if (w == 2) k.spans.push_back(ArcSpan{{5, 5}, NaN, 0, 1, ArcDirection::CCW});
        if (w == 3) k.spans.insert(k.spans.begin() + 1, LineSpan{{10, 0}, {10, 0}});
        if (w == 4) k.spans.insert(k.spans.begin() + 1, BezierSpan{{10, 0}, {10, 0}, {10, 0}, {10, 0}});
        if (w == 5) k.spans.insert(k.spans.begin() + 1, ArcSpan{{10, 0}, 0.0, 0, 1, ArcDirection::CCW});
        c = {"nonfinite/degenerate w=" + std::to_string(w), {k, poly({{20, 0}, {25, 0}, {25, 5}})}, false}; break; }
    case 9: { int n = I(2, 8); Contour k; k.closed = true; std::vector<Vec2> p; for (int i = 0; i < n; ++i) { double a = 2 * PI * i / n; p.push_back({8 * std::cos(a), 8 * std::sin(a)}); }
        for (int i = 0; i < n; ++i) { Vec2 a = p[i], b = p[(i + 1) % n]; k.append(BezierSpan{a, {U(-12, 12), U(-12, 12)}, {U(-12, 12), U(-12, 12)}, b}); } c = {"random-bezier-blob", {k}, false}; break; }
    case 10: { Contour k; k.closed = true; double r = U(0.5, 5); int n = I(1, 6); Vec2 cur{0, 0}; double a0 = 0;
        // chain of arcs closing with a line
        for (int i = 0; i < n; ++i) { ArcSpan a{cur - Vec2::fromAngle(a0, r), r, a0, a0 + U(0.1, 3), I(0,1) ? ArcDirection::CCW : ArcDirection::CW}; k.append(a); cur = a.endPoint(); a0 = U(-PI, PI); }
        k.append(LineSpan{cur, k.startPoint()}); c = {"arc-chain", {k}, false}; break; }
    case 11: { auto sh = vs::all()[I(0, 20)]; auto cs = sh.cs; cs.push_back(cs[0]); c = {"duplicated-contour " + sh.name, cs, false}; break; }
    case 12: { double w = std::pow(10.0, -U(1, 4)); double L = U(10, 400); c = {"long-channel w=" + std::to_string(w) + " L=" + std::to_string(L), {poly({{0, 0}, {L, 0}, {L, w}, {0, w}})}, true}; break; }
    case 13: { int n = I(50, 1500); std::vector<Vec2> p; for (int i = 0; i < n; ++i) { double a = 2 * PI * i / n; double r = (i % 2) ? U(0.1, 1) : U(5, 10); p.push_back({r * std::cos(a), r * std::sin(a)}); } c = {"spiky n=" + std::to_string(n), {poly(p)}, true}; break; }
    case 14: { // contours sharing an edge / hole touching at a vertex / coincident edges
        int w = I(0, 2);
        if (w == 0) c = {"shared-edge", {poly({{0, 0}, {5, 0}, {5, 5}, {0, 5}}), poly({{5, 0}, {10, 0}, {10, 5}, {5, 5}})}, false};
        if (w == 1) c = {"hole-touch-vertex", {poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), rev(poly({{0, 5}, {4, 3}, {4, 7}}))}, false};
        if (w == 2) c = {"hole-on-edge", {poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}}), rev(poly({{0, 3}, {4, 3}, {4, 6}, {0, 6}}))}, false};
        break; }
    default: { // tiny features: zig-zag with micro teeth
        int n = I(10, 400); double amp = std::pow(10.0, -U(1, 7)); std::vector<Vec2> p; for (int i = 0; i <= n; ++i) p.push_back({10.0 * i / n, (i % 2) * amp}); p.push_back({10, 5}); p.push_back({0, 5});
        c = {"micro-teeth amp=" + std::to_string(amp), {poly(p)}, true}; break; }
    }
    return c;
}
int main(int argc, char** argv) {
    int n = argc > 1 ? std::atoi(argv[1]) : 400; unsigned seed = argc > 2 ? std::atoi(argv[2]) : 1; int alarmS = argc > 3 ? std::atoi(argv[3]) : 60;
    int crashes = 0, hangs = 0, bad = 0, capped = 0; double worst = 0; std::string worstName;
    for (int i = 0; i < n; ++i) {
        std::mt19937_64 R(seed * 1000003ull + i);
        Case c = gen(i, R);
        int pfd[2]; if (pipe(pfd) != 0) return 2;
        auto t0 = std::chrono::steady_clock::now();
        pid_t pid = fork();
        if (pid == 0) {
            close(pfd[0]); alarm(alarmS);
            Tolerance tol; MedialAxis ax = medialAxis(c.cs, tol);
            int code = 0; double maxErr = 0;
            bool finiteIn = true; double ext = 0; for (auto& ct : c.cs) for (auto& sp : ct.spans) { Box2 b = bounds(sp); if (!std::isfinite(b.min.x + b.min.y + b.max.x + b.max.y)) finiteIn = false; else ext += b.width() + b.height(); }
            if (ext > 2e4) finiteIn = false;
            vo::Boundary B = finiteIn ? vo::makeBoundary(c.cs, 0.01) : vo::Boundary{};
            size_t np = 0;
            for (auto& br : ax.branches) { if (br.sides.size() + 1 != br.points.size() || br.startNode >= ax.nodes.size() || br.endNode >= ax.nodes.size()) code = 5;
                for (size_t k = 0; k < br.points.size(); ++k) { auto& p = br.points[k]; ++np;
                    if (!std::isfinite(p.point.x) || !std::isfinite(p.point.y) || !std::isfinite(p.radius) || p.radius < 0) code = 3;
                    else if (c.checkRadius && finiteIn && k % 7 == 0) maxErr = std::max(maxErr, std::fabs(vo::exactDist(B, p.point).d - p.radius)); }
                for (auto& s : br.sides) for (auto& r : s) if (r.contour >= c.cs.size() || r.span >= c.cs[r.contour].spans.size() || !(r.t >= 0 && r.t <= 1)) code = 6; }
            for (auto& nd : ax.nodes) if (!std::isfinite(nd.point.x) || !std::isfinite(nd.radius)) code = 3;
            if (code == 0 && !ax.accuracyCapped && maxErr > tol.chordal) code = 4;
            char buf[256]; int len = std::snprintf(buf, sizeof buf, "%d %zu %zu %zu %zu %.3e", (int)ax.accuracyCapped, ax.boundarySamples, ax.ignoredContours, ax.branches.size(), np, maxErr);
            if (write(pfd[1], buf, len) < 0) {}
            _exit(code);
        }
        close(pfd[1]); char buf[256] = {0}; ssize_t rd = read(pfd[0], buf, 255); (void)rd; close(pfd[0]);
        int st = 0; waitpid(pid, &st, 0);
        double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const char* tag = "ok";
        if (WIFSIGNALED(st)) { if (WTERMSIG(st) == SIGALRM) { ++hangs; tag = "HANG"; } else { ++crashes; tag = "CRASH"; } }
        else if (WEXITSTATUS(st) != 0) { ++bad; tag = WEXITSTATUS(st) == 4 ? "RADERR" : "BAD"; }
        int cap = 0; std::sscanf(buf, "%d", &cap); capped += cap;
        if (std::string(tag) != "ok" || secs > 5 || cap || argc > 4) std::printf("%4d %-6s %6.2fs %-50s [capped smp ign br pts err] %s sig=%d exit=%d\n", i, tag, secs, c.name.c_str(), buf, WIFSIGNALED(st) ? WTERMSIG(st) : 0, WIFEXITED(st) ? WEXITSTATUS(st) : -1);
        std::fflush(stdout);
    }
    std::printf("cases=%d crashes=%d hangs=%d bad=%d capped=%d\n", n, crashes, hangs, bad, capped);
}
