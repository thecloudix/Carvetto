// Verifier fuzz for T-0132 caches: zip-level lies, flips, truncation, structural cache mutations.
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <span>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <ctime>
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
using namespace carvetto::document;
using Bytes = std::vector<std::uint8_t>;
static int hangs = 0, crashes = 0, oks = 0, errs = 0; static double worst = 0;
static void run(const Bytes& f, const char* what, int i) {
    int fd[2]; if (pipe(fd)) return;
    pid_t pid = fork();
    if (pid == 0) {
        close(fd[0]); alarm(10);
        timespec a, b; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &a);
        auto r = loadCarv(std::span<const std::uint8_t>(f));
        if (r.ok()) { auto s = saveCarv(r.value().document, r.value().extras); if (!s.ok()) { std::fprintf(stderr, "%s %d: loads, save fails: %s\n", what, i, s.error().message.c_str()); } else { auto l2 = loadCarv(std::span<const std::uint8_t>(s.value())); if (!l2.ok()) std::fprintf(stderr, "%s %d: resave does not load: %s\n", what, i, l2.error().message.c_str()); } }
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &b);
        double t = double(b.tv_sec - a.tv_sec) + double(b.tv_nsec - a.tv_nsec) * 1e-9;
        char buf[64]; int n = std::snprintf(buf, sizeof buf, "%d %f", r.ok() ? 1 : 0, t);
        if (write(fd[1], buf, size_t(n)) < 0) {}
        _exit(0);
    }
    close(fd[1]); char buf[64] = {0}; ssize_t n = read(fd[0], buf, 63); close(fd[0]);
    int st; waitpid(pid, &st, 0);
    if (WIFSIGNALED(st)) { if (WTERMSIG(st) == SIGALRM) { ++hangs; std::printf("HANG %s %d\n", what, i); } else { ++crashes; std::printf("CRASH sig %d %s %d\n", WTERMSIG(st), what, i); std::ofstream o(std::string("crash_") + what + std::to_string(i) + ".carv", std::ios::binary); o.write((const char*)f.data(), (std::streamsize)f.size()); } return; }
    if (n > 0) { int ok; double t; std::sscanf(buf, "%d %lf", &ok, &t); (ok ? oks : errs)++; if (t > worst) worst = t; }
}
static std::uint32_t rd32(const Bytes& b, size_t o) { return b[o] | (b[o+1] << 8) | (b[o+2] << 16) | (std::uint32_t(b[o+3]) << 24); }
static void wr32(Bytes& b, size_t o, std::uint32_t v) { for (int i = 0; i < 4; ++i) b[o+size_t(i)] = std::uint8_t(v >> (8*i)); }
int main(int argc, char** argv) {
    std::ifstream f(argv[1], std::ios::binary); Bytes base((std::istreambuf_iterator<char>(f)), {});
    int iters = std::atoi(argv[2]); std::mt19937 rng(unsigned(std::atoi(argv[3])));
    // locate central directory entries and local headers of caches
    struct E { size_t cd, lh; std::uint32_t csize, usize; std::string name; };
    std::vector<E> caches;
    for (size_t o = 0; o + 46 < base.size(); ++o) {
        if (rd32(base, o) == 0x02014b50) {
            std::uint16_t nl = std::uint16_t(base[o+28] | (base[o+29] << 8));
            std::string name(base.begin() + long(o) + 46, base.begin() + long(o) + 46 + nl);
            if (name.find(".bin") != std::string::npos) caches.push_back({o, rd32(base, o+42), rd32(base, o+20), rd32(base, o+24), name});
        }
    }
    std::printf("caches found: %zu\n", caches.size());
    for (int i = 0; i < iters; ++i) {
        Bytes b = base;
        int mode = i % 5;
        const E& e = caches[rng() % caches.size()];
        if (mode == 0) { for (int k = 0, n = 1 + int(rng() % 8); k < n; ++k) b[rng() % b.size()] ^= std::uint8_t(1u << (rng() % 8)); }
        else if (mode == 1) { size_t start = e.lh + 30 + e.name.size(); for (int k = 0, n = 1 + int(rng() % 4); k < n; ++k) b[start + rng() % e.csize] ^= std::uint8_t(1u << (rng() % 8)); }
        else if (mode == 2) { b.resize(rng() % b.size()); }
        else if (mode == 3) {
            std::uint32_t choices[] = {0, 1, 16, e.usize - 1, e.usize + 1, e.usize * 2, 0xFFFFFFFFu, e.usize / 2, e.csize, std::uint32_t(rng())};
            std::uint32_t u = choices[rng() % 10];
            int which = int(rng() % 4);
            if (which == 0 || which == 2) { wr32(b, e.cd + 24, u); wr32(b, e.lh + 22, u); }
            if (which == 1 || which == 2) { std::uint32_t c = choices[rng() % 10]; wr32(b, e.cd + 20, c); wr32(b, e.lh + 18, c); }
            if (which == 3) { wr32(b, e.cd + 16, std::uint32_t(rng())); }  // crc
        } else {
            // flip bytes in the job.json count digits region: replace a random digit in the file's "count"
            auto ms = detail::readZip(base).members;
            for (auto& m : ms) if (m.name == "job.json") {
                std::string j(m.data.begin(), m.data.end());
                size_t p = j.find("\"count\": "); int skip = int(rng() % 2); while (skip-- && p != std::string::npos) p = j.find("\"count\": ", p + 1);
                if (p != std::string::npos) { size_t q = p + 9; while (q < j.size() && isdigit((unsigned char)j[q])) ++q; long delta = long(rng() % 7) - 3; long v = std::stol(j.substr(p + 9, q - p - 9)) + delta; if (v < 0) v = 0; j.replace(p + 9, q - p - 9, std::to_string(v)); }
                m.data.assign(j.begin(), j.end());
            }
            b = detail::writeZip(ms).bytes;
        }
        run(b, mode == 0 ? "flip" : mode == 1 ? "cacheflip" : mode == 2 ? "trunc" : mode == 3 ? "sizes" : "count", i);
    }
    std::printf("ok=%d err=%d hang=%d crash=%d worst_cpu=%.3f s\n", oks, errs, hangs, crashes, worst);
}
