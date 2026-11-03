#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include "shapes.hpp"
using namespace carvetto::geometry;
using vo::PI;
static double cpu() { timespec t; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static std::vector<Contour> glyph(int k, Vec2 o, double h) {
    double s = h / 20.0; Transform2 m{s, 0, 0, s, o.x, o.y};
    std::vector<Contour> g;
    switch (k % 5) {
    case 0: g = {vs::bezEllipse({8, 11}, 8, 11), vs::rev(vs::bezEllipse({8, 11}, 5, 8.5))}; break;
    case 1: g = {vs::poly({{0, 0}, {3, 0}, {4, 4}, {8, 4}, {9, 0}, {12, 0}, {7.5, 16}, {4.5, 16}}), vs::rev(vs::poly({{4.8, 6.5}, {7.2, 6.5}, {6, 11.5}}))}; break;
    case 2: g = {vs::sGlyph({0, 0}, 20)}; break;
    case 3: g = {vs::cGlyph({10, 10}, 10, 3)}; break;
    default: g = {vs::poly({{0, 0}, {3, 0}, {3, 8}, {9, 8}, {9, 0}, {12, 0}, {12, 20}, {9, 20}, {9, 11}, {3, 11}, {3, 20}, {0, 20}})}; break; // H
    }
    for (auto& c : g) c.transform(m);
    return g;
}
static size_t spans(const std::vector<Contour>& cs) { size_t n = 0; for (auto& c : cs) n += c.spans.size(); return n; }
int main(int argc, char** argv) {
    const char* mode = argc > 1 ? argv[1] : "text";
    size_t maxSeg = argc > 2 ? std::atol(argv[2]) : 50000;
    double h = argc > 3 ? std::atof(argv[3]) : 5.0;
    Tolerance tol;
    double prev = 0;
    for (size_t target = 1562; target <= maxSeg + 1; target *= 2) {
        std::vector<Contour> cs;
        if (!std::strcmp(mode, "text")) {
            int k = 0; int perRow = 80;
            while (spans(cs) < target) { int r = k / perRow, c = k % perRow; auto g = glyph(k * 7 + r, {c * h * 0.8, -r * h * 1.4}, h); cs.insert(cs.end(), g.begin(), g.end()); ++k; }
        } else if (!std::strcmp(mode, "blob")) { // single polygon, smooth radial noise, N vertices, radius ~ sqrt(N)
            size_t n = target; double R = 5.0 * std::sqrt((double)n / 100.0); std::vector<Vec2> p;
            for (size_t i = 0; i < n; ++i) { double a = 2 * PI * i / n; double r = R * (1 + 0.3 * std::sin(7 * a) + 0.1 * std::sin(31 * a) + 0.03 * std::sin(173 * a)); p.push_back({r * std::cos(a), r * std::sin(a)}); }
            cs = {vs::poly(p)};
        } else if (!std::strcmp(mode, "holes")) { // one region: big rect with many small square/diamond holes
            size_t nh = target / 4; int side = (int)std::ceil(std::sqrt((double)nh)); double pitch = 2.0;
            cs.push_back(vs::poly({{-1, -1}, {side * pitch + 1, -1}, {side * pitch + 1, side * pitch + 1}, {-1, side * pitch + 1}}));
            for (size_t i = 0; i + 1 < nh; ++i) { double x = (i % side) * pitch + 0.5, y = (i / side) * pitch + 0.5; if (i % 2) cs.push_back(vs::rev(vs::poly({{x, y}, {x + 1, y}, {x + 1, y + 1}, {x, y + 1}}))); else cs.push_back(vs::rev(vs::poly({{x + 0.5, y}, {x + 1, y + 0.5}, {x + 0.5, y + 1}, {x, y + 0.5}}))); }
        } else if (!std::strcmp(mode, "bezblob")) { // single closed Bezier outline with N spans (wavy)
            size_t n = target; double R = 5.0 * std::sqrt((double)n / 100.0); Contour k; k.closed = true;
            auto P = [&](double a) { double r = R * (1 + 0.2 * std::sin(9 * a) + 0.05 * std::sin(57 * a)); return Vec2{r * std::cos(a), r * std::sin(a)}; };
            for (size_t i = 0; i < n; ++i) { double a0 = 2 * PI * i / n, a1 = 2 * PI * (i + 1) / n; Vec2 p0 = P(a0), p3 = P(a1); Vec2 d = (p3 - p0) * (1.0 / 3); Vec2 nrm{-d.y, d.x}; double w = (i % 2) ? 0.3 : -0.3; k.append(BezierSpan{p0, p0 + d + nrm * w, p0 + d * 2 + nrm * w, p3}); }
            cs = {k};
        }
        else if (!std::strcmp(mode, "comb")) { // single region: N/4 teeth 1 mm wide, 5 mm long, 1 mm gaps
            size_t nt = target / 4; std::vector<Vec2> p{{0, 0}}; for (size_t i = 0; i < nt; ++i) { double x = 2.0 * i; p.push_back({x, 6}); p.push_back({x + 1, 6}); p.push_back({x + 1, 1}); p.push_back({x + 2, 1}); }
            p.push_back({2.0 * nt, 0}); cs = {vs::poly(p)};
        }
        size_t ns = spans(cs);
        double c0 = cpu(); auto w0 = std::chrono::steady_clock::now();
        MedialAxis ax = medialAxis(cs, tol);
        double c1 = cpu() - c0; double w1 = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
        size_t pts = 0; for (auto& b : ax.branches) pts += b.points.size();
        std::printf("%-8s spans=%7zu contours=%6zu cpu=%8.3fs wall=%8.3fs ratio=%5.2f samples=%8zu smp/span=%5.1f capped=%d pts=%zu\n", mode, ns, cs.size(), c1, w1, prev > 0 ? c1 / prev : 0.0, ax.boundarySamples, (double)ax.boundarySamples / ns, (int)ax.accuracyCapped, pts);
        std::fflush(stdout); prev = c1;
    }
}
