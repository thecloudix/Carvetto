// T-0132 verifier round 2: realistic 2M-move jobs (raster, noisy, split) save/load time, peak, exactness.
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
static std::vector<Move> raster(std::size_t n, double step) {
    std::vector<Move> v; v.reserve(n); double y = 0; bool dir = true; double z = -1;
    v.push_back(Move::rapid({0, 0, 5})); v.push_back(Move::plunge({0, 0, z}, 300));
    while (v.size() < n) {
        // a raster line broken into segments (as a real pocket at an angle would be)
        for (int k = 0; k < 40 && v.size() < n; ++k) { double x = dir ? k * 7.5 + 0.0001 * k * k : 300 - k * 7.5; v.push_back(Move::feed({x, y + x * 0.2126, z})); }
        y += step; dir = !dir;
        if (v.size() % 1000 < 40) { v.push_back(Move::arcCW({dir ? 0 : 300, y + 1, z}, {dir ? 0.5 : 299.5, y + 0.5}, 1200)); }
        if (y > 300) { y = 0; z -= 0.5; v.push_back(Move::retract({0, 0, 5})); v.push_back(Move::rapid({0, 0, 5})); v.push_back(Move::plunge({0, 0, z}, 300)); }
    }
    return v;
}
static std::vector<Move> noisy(std::size_t n, unsigned seed) {
    std::mt19937_64 g(seed); std::normal_distribution<double> d(0, 0.4); std::vector<Move> v; v.reserve(n); double x = 0, y = 0, z = -2;
    while (v.size() < n) { x += d(g); y += d(g); if (v.size() % 97 == 0) z = -2 - double(v.size() % 13) * 0.25; if (v.size() % 211 == 0) v.push_back(Move::arcCCW({x + 1, y, z}, {x + 0.5, y}, 900)); else v.push_back(Move::feed({x, y, z})); }
    return v;
}
static void measure(const char* label, Document& doc) {
    std::size_t total = 0; for (auto& t : doc.toolpaths()) total += t.moveCount();
    double t0 = cpu(); auto s = saveCarv(doc); double t1 = cpu();
    if (!s.ok()) { std::printf("%s: SAVE REFUSED %s\n", label, s.error().message.c_str()); return; }
    const Bytes& f = s.value();
    int fd[2]; if (pipe(fd)) return;
    pid_t pid = fork();
    if (pid == 0) {
        close(fd[0]); alarm(120); malloc_trim(0); { std::ofstream c("/proc/self/clear_refs"); c << "5"; }
        long base = statusKb("VmRSS:"); double a = cpu(); auto r = loadCarv(std::span<const std::uint8_t>(f)); double b = cpu(); long hwm = statusKb("VmHWM:");
        char buf[256]; int n = std::snprintf(buf, sizeof buf, "load %s cpu %.2f s peak %.1f MiB", r.ok() ? "OK" : r.error().message.c_str(), b - a, (hwm - base) / 1024.0);
        if (write(fd[1], buf, size_t(n)) < 0) {} _exit(0);
    }
    close(fd[1]); char buf[256] = {0}; if (read(fd[0], buf, 255) < 0) {} close(fd[0]); int st; waitpid(pid, &st, 0);
    auto r = loadCarv(std::span<const std::uint8_t>(f));
    bool eq = r.ok() && r.value().document.toolpaths() == doc.toolpaths();
    auto s2 = r.ok() ? saveCarv(r.value().document, r.value().extras) : s;
    std::printf("%-22s moves %zu file %.2f MB (%.2f B/move) save cpu %.2f s | %s | records equal %d resave identical %d\n", label, total, double(f.size()) / 1e6,
                double(f.size()) / double(total), t1 - t0, buf, eq, s2.ok() && s2.value() == f);
}
int main() {
    auto rec = [](std::vector<Move> m) { ToolpathRecord r; r.name = "big"; r.selection = VectorSelection::allVectors(); r.moves = shareMoves(std::move(m)); return r; };
    { Document d; d.insertToolpath(rec(raster(2'000'000, 0.123))); measure("raster 2M", d); }
    { Document d; d.insertToolpath(rec(noisy(2'000'000, 1))); measure("noisy 2M", d); }
    { Document d; for (unsigned k = 0; k < 4; ++k) d.insertToolpath(rec(noisy(500'000, k))); measure("noisy 4x500k", d); }
    { Document d; d.insertToolpath(rec(noisy(8'000'000, 3))); measure("noisy 8M", d); }
    { Document d; d.insertToolpath(rec(noisy(8'000'001, 3))); measure("noisy 8M+1", d); }
}
