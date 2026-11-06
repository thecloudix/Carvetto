// T-0172 verifier: ClusterGrid build stats for one generated box set: cgone <kind> <n> <seed> <fill>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>
#include <variant>
#include <string>
#include <memory>
#include <array>
#include <numeric>
#include <random>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#define private public
#include "offset.cpp"
#undef private
using namespace carvetto::geometry;
namespace cg = carvetto::geometry;
struct G { std::mt19937_64 r; double u(double a, double b) { return std::uniform_real_distribution<double>(a, b)(r); } int i(int a, int b) { return std::uniform_int_distribution<int>(a, b)(r); } double logu(double a, double b) { return std::exp(u(std::log(a), std::log(b))); } };
int main(int argc, char** argv) {
    int kind = atoi(argv[1]); size_t n = strtoul(argv[2], 0, 0); G g{std::mt19937_64(strtoull(argv[3], 0, 0))}; bool fill = atoi(argv[4]);
    std::vector<Box2> b;
    auto add = [&](Vec2 c, double w, double h) { b.push_back(Box2{c - Vec2{w / 2, h / 2}, c + Vec2{w / 2, h / 2}}); };
    if (kind == 0) for (size_t i = 0; i < n; ++i) add({g.u(0, 100), g.u(0, 100)}, g.logu(1e-3, 5), g.logu(1e-3, 5));
    if (kind == 5) { for (size_t i = 0; i < n; ++i) add({5, 0.004 * double(i)}, 10, 0); add({1e5, 1e5}, 2, 2); }
    if (kind == 3) for (size_t i = 0; i < n; ++i) add({g.u(0, 1e-9), 3.0}, 0, g.logu(1e-12, 1));
    if (kind == 10) { for (size_t i = 0; i < n; ++i) { double a = g.u(0, 10), c = g.u(0, 10); add({(a + c) / 2, g.u(0, 0.01)}, std::fabs(a - c), 0); } add({1e5, 1e5}, 1, 1); }
    if (kind == 11) { for (size_t i = 0; i < n; ++i) { double a = g.u(0, 10), c = g.u(0, 10); add({(a + c) / 2, g.u(0, 10)}, std::fabs(a - c), 0); } }
    if (kind == 12) { for (size_t i = 0; i < n; ++i) { double h = g.logu(1e-6, 10); add({g.u(0, 1e-6), 0}, 0, h); } add({1e5, 1e5}, 1, 1); }
    if (kind == 13) { double H = std::getenv("CG_H") ? atof(std::getenv("CG_H")) : 4; for (size_t i = 0; i < n; ++i) { double a = g.u(0, 10), c = g.u(0, 10); add({(a + c) / 2, g.u(0, H)}, std::fabs(a - c), 0); } add({1e5, 1e5}, 1, 1); }
    if (kind == 4) for (size_t i = 0; i < n; ++i) add({g.u(-1, 1) * 1e8, g.u(-1, 1) * 1e-8}, g.logu(1e-6, 1e6), 0);
    if (kind == 6) { for (size_t i = 0; i < n; ++i) add({g.u(0, 1e-3), g.u(0, 1e-3)}, g.logu(1e-6, 1e-3), g.logu(1e-6, 1e-3)); add({1e4, 0}, 0, 0); add({-1e4, 7}, 0, 0); }
    if (kind == 1) { for (size_t i = 0; i < n; ++i) { double s = std::pow(10.0, -g.i(0, 6)); add({g.u(0, 1) * s, g.u(0, 1) * s}, s * 1e-3 * g.u(0, 1), s * 1e-3 * g.u(0, 1)); } add({1e5, 1e5}, 1, 1); }
    if (kind == 14) { for (size_t i = 0; i < n; ++i) add({g.u(0, 1e-3), g.u(0, 1e-3)}, g.logu(1e-6, 1e-3), g.logu(1e-6, 1e-3)); }
    if (kind == 15) { for (size_t i = 0; i < n; ++i) { double r = 0.01 * double(i + 1); add({0, 0}, 2 * r, 2 * r); } }
    if (kind == 16) { for (size_t i = 0; i < n; ++i) { double r = 0.01 * double(i / 2 + 1); add(i % 2 ? Vec2{0, r / 2} : Vec2{0, -r / 2}, 2 * r, r); } add({1e5, 1e5}, 1, 1); }
    if (kind == 9) for (size_t i = 0; i < n; ++i) add({g.u(0, 100), g.u(0, 100)}, g.u(0, 0.2), g.u(0, 0.2));
    auto t0 = std::chrono::steady_clock::now();
    cg::ClusterGrid grid(b, fill);
    if (!fill) for (auto& x : b) grid.add(x);
    double bt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    size_t cells = 0, entries = 0; for (auto& l : grid.levels_) { cells += l.cells.size(); for (auto& c : l.cells) entries += c.size(); }
    std::printf("kind %d n %zu: build %.3g s, levels %zu, cells %zu, entries %zu (%.1f per item)\n", kind, n, bt, grid.levels_.size(), cells, entries, double(entries) / n);
    { auto t1 = std::chrono::steady_clock::now(); cg::BoxGrid bg(b); double bt2 = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
      size_t e2 = 0; for (auto& c : bg.cells_) e2 += c.size(); std::printf("  BoxGrid (uniform, as main's step 7 / other users): build %.3g s, cells %zu, entries %zu (%.1f per item)\n", bt2, bg.cells_.size(), e2, double(e2) / n); }
    for (size_t k = 0; k < std::min<size_t>(grid.levels_.size(), 5); ++k) std::printf("  level %zu: %zux%zu cell %.3g extent %.3g x %.3g\n", k, grid.levels_[k].nx, grid.levels_[k].ny, grid.levels_[k].cell, grid.levels_[k].extent.width(), grid.levels_[k].extent.height());
}
