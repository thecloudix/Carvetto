// T-0132 verifier round 3: save CPU and save peak (extra RSS during saveCarv) for realistic big jobs; load CPU/peak.
#include <cstdio>
#include <cstring>
#include <fstream>
#include <malloc.h>
#include <random>
#include <span>
#include <sys/wait.h>
#include <unistd.h>
#include <ctime>
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
using namespace carvetto::document;
using carvetto::toolpath::Move;
using Bytes = std::vector<std::uint8_t>;
static long statusKb(const char* key) { std::ifstream f("/proc/self/status"); std::string l; while (std::getline(f, l)) if (l.rfind(key, 0) == 0) return std::stol(l.substr(std::strlen(key))); return -1; }
static double cpu() { timespec ts; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static void resetHwm() { malloc_trim(0); std::ofstream c("/proc/self/clear_refs"); c << "5"; }
static std::vector<Move> raster(std::size_t n, double step) {
    std::vector<Move> v; v.reserve(n); double y = 0; bool dir = true; double z = -1;
    v.push_back(Move::rapid({0, 0, 5})); v.push_back(Move::plunge({0, 0, z}, 300));
    while (v.size() < n) {
        for (int k = 0; k < 40 && v.size() < n; ++k) { double x = dir ? k * 7.5 + 0.0001 * k * k : 300 - k * 7.5; v.push_back(Move::feed({x, y + x * 0.2126, z})); }
        y += step; dir = !dir;
        if (v.size() % 1000 < 40) { v.push_back(Move::arcCW({dir ? 0 : 300, y + 1, z}, {dir ? 0.5 : 299.5, y + 0.5}, 1200)); }
        if (y > 300) { y = 0; z -= 0.5; v.push_back(Move::retract({0, 0, 5})); v.push_back(Move::rapid({0, 0, 5})); v.push_back(Move::plunge({0, 0, z}, 300)); }
    }
    v.resize(n);
    return v;
}
static std::vector<Move> noisy(std::size_t n, unsigned seed) {
    std::mt19937_64 g(seed); std::normal_distribution<double> d(0, 0.4); std::vector<Move> v; v.reserve(n); double x = 0, y = 0, z = -2;
    while (v.size() < n) { x += d(g); y += d(g); if (v.size() % 97 == 0) z = -2 - double(v.size() % 13) * 0.25; if (v.size() % 211 == 0) v.push_back(Move::arcCCW({x + 1, y, z}, {x + 0.5, y}, 900)); else v.push_back(Move::feed({x, y, z})); }
    return v;
}
template <class Build> static void measure(const char* label, Build build) {
    int fd[2]; if (pipe(fd)) return;
    pid_t pid = fork();
    if (pid == 0) {
        close(fd[0]); alarm(300);
        Document doc; build(doc); std::size_t total = 0; for (auto& t : doc.toolpaths()) total += t.moveCount();
        resetHwm(); long base = statusKb("VmRSS:");
        double t0 = cpu(); auto s = saveCarv(doc); double t1 = cpu(); long hwm = statusKb("VmHWM:");
        char buf[512]; int n;
        if (!s.ok()) n = std::snprintf(buf, sizeof buf, "%-16s moves %8zu SAVE REFUSED (%.2fs, +%.0f MiB): %.150s", label, total, t1 - t0, (hwm - base) / 1024.0, s.error().message.c_str());
        else {
            Bytes f = std::move(s.value());
            resetHwm(); long b2 = statusKb("VmRSS:"); double a = cpu(); auto r = loadCarv(std::span<const std::uint8_t>(f)); double b = cpu(); long h2 = statusKb("VmHWM:");
            bool eq = r.ok() && r.value().document.toolpaths() == doc.toolpaths();
            n = std::snprintf(buf, sizeof buf, "%-16s moves %8zu file %6.2f MB save cpu %5.2f s save peak +%6.1f MiB | load %s cpu %.2f s peak +%6.1f MiB eq %d", label, total, f.size() / 1e6, t1 - t0,
                              (hwm - base) / 1024.0, r.ok() ? "OK" : r.error().message.c_str(), b - a, (h2 - b2) / 1024.0, eq);
        }
        if (write(fd[1], buf, size_t(n)) < 0) {} _exit(0);
    }
    close(fd[1]); char buf[513] = {0}; ssize_t k = read(fd[0], buf, 512); (void)k; close(fd[0]); int st; waitpid(pid, &st, 0);
    if (WIFSIGNALED(st)) std::printf("%s: signal %d\n", label, WTERMSIG(st)); else std::printf("%s\n", buf);
    std::fflush(stdout);
}
int main(int argc, char** argv) {
    std::string only = argc > 1 ? argv[1] : "";
    auto rec = [](std::vector<Move> m) { ToolpathRecord r; r.name = "big"; r.tool.diameter = 6; r.tool.feedRate = 1000; r.tool.plungeRate = 300; r.tool.spindleSpeed = 18000; r.selection = VectorSelection::allVectors(); r.moves = shareMoves(std::move(m)); return r; };
    measure("raster 2M", [&](Document& d) { d.insertToolpath(rec(raster(2'000'000, 0.123))); });
    measure("noisy 2M", [&](Document& d) { d.insertToolpath(rec(noisy(2'000'000, 1))); });
    measure("noisy 4x500k", [&](Document& d) { for (unsigned k = 0; k < 4; ++k) d.insertToolpath(rec(noisy(500'000, k))); });
    measure("raster 8M", [&](Document& d) { d.insertToolpath(rec(raster(8'000'000, 0.05))); });
    measure("noisy 8M", [&](Document& d) { d.insertToolpath(rec(noisy(8'000'000, 3))); });
    measure("noisy 8M+1", [&](Document& d) { d.insertToolpath(rec(noisy(8'000'001, 3))); });
    (void)only;
}
