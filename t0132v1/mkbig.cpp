#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include <cstdio>
#include <ctime>
#include <random>
#include <span>
using namespace carvetto::document; using carvetto::toolpath::Move;
static double cpu() { timespec t; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t); return double(t.tv_sec) + double(t.tv_nsec) * 1e-9; }
int main(int, char** argv) {
    size_t n = std::stoul(argv[1]); std::mt19937 rng(5); std::uniform_real_distribution<double> u(-500, 500);
    std::vector<Move> m; m.reserve(n);
    double y = 0;
    while (m.size() < n) {
        switch (m.size() % 10) {
            case 0: m.push_back(Move::rapid({u(rng), y, 5})); break;
            case 1: m.push_back(Move::plunge({m.back().to.x, y, -1.5}, 300.0)); break;
            case 2: case 3: case 4: m.push_back(Move::feed({u(rng), y, -1.5})); break;
            case 5: m.push_back(Move::arcCW({u(rng), y + 1, -1.5}, {u(rng), y + 0.5}, 900.0)); break;
            case 6: m.push_back(Move::arcCCW({u(rng), y + 1, -1.5}, {u(rng), y + 0.5}, 900.0)); break;
            case 7: m.push_back(Move::dwell(m.back().to, 0.25)); break;
            case 8: m.push_back(Move::feed({m.back().to.x, y + 1, -1.5}, std::uniform_real_distribution<double>(100, 2000)(rng))); break;
            default: m.push_back(Move::retract({m.back().to.x, m.back().to.y, 5})); y += 0.1; break;
        }
    }
    Document doc; ToolpathRecord r; r.name = "big"; r.selection = VectorSelection::allVectors(); r.moves = shareMoves(std::move(m)); doc.insertToolpath(r);
    double t0 = cpu(); auto s = saveCarv(doc, std::filesystem::path(argv[2])); double t1 = cpu();
    auto l = loadCarv(std::filesystem::path(argv[2])); double t2 = cpu();
    std::printf("n=%zu save %s %.2fs, load %s %.2fs, equal=%d\n", n, s.ok() ? "ok" : s.error().message.c_str(), t1 - t0, l.ok() ? "ok" : l.error().message.c_str(), t2 - t1,
                l.ok() && *l.value().document.toolpaths()[0].moves == *r.moves);
}
