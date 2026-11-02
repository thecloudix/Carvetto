// T-0132 verifier round 2: structure-aware fuzz (member contents mutated, re-zipped with valid CRC)
// plus zip-level lies. Child per input with alarm(10). Checks load -> save -> load consistency.
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
static int hangs = 0, crashes = 0, oks = 0, errs = 0, resaveRefused = 0, incons = 0; static double worst = 0;
static void run(const Bytes& f, const char* what, int i) {
    int fd[2]; if (pipe(fd)) return;
    pid_t pid = fork();
    if (pid == 0) {
        close(fd[0]); alarm(20);
        timespec a, b; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &a);
        auto r = loadCarv(std::span<const std::uint8_t>(f));
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &b);
        int state = r.ok() ? 1 : 0;
        if (r.ok()) {
            auto s = saveCarv(r.value().document, r.value().extras);
            if (!s.ok()) { state = 2; std::fprintf(stderr, "[%s %d] loads, save refuses: %s\n", what, i, s.error().message.c_str()); }
            else { auto l2 = loadCarv(std::span<const std::uint8_t>(s.value()));
                if (!l2.ok()) { state = 3; std::fprintf(stderr, "[%s %d] RESAVE DOES NOT LOAD: %s\n", what, i, l2.error().message.c_str()); }
                else if (!(l2.value().document.toolpaths() == r.value().document.toolpaths())) { state = 3; std::fprintf(stderr, "[%s %d] RESAVE RECORDS DIFFER\n", what, i); } }
        }
        double t = double(b.tv_sec - a.tv_sec) + double(b.tv_nsec - a.tv_nsec) * 1e-9;
        char buf[64]; int n = std::snprintf(buf, sizeof buf, "%d %f", state, t);
        if (write(fd[1], buf, size_t(n)) < 0) {}
        _exit(0);
    }
    close(fd[1]); char buf[64] = {0}; ssize_t n = read(fd[0], buf, 63); close(fd[0]);
    int st; waitpid(pid, &st, 0);
    if (WIFSIGNALED(st) || (WIFEXITED(st) && WEXITSTATUS(st) != 0)) {
        bool hang = WIFSIGNALED(st) && WTERMSIG(st) == SIGALRM;
        (hang ? hangs : crashes)++;
        std::printf("%s %s %d (status %d)\n", hang ? "HANG" : "CRASH", what, i, st);
        std::ofstream o(std::string("crash2_") + what + std::to_string(i) + ".carv", std::ios::binary); o.write((const char*)f.data(), (std::streamsize)f.size());
        return;
    }
    if (n > 0) { int s; double t; std::sscanf(buf, "%d %lf", &s, &t); if (s == 1) ++oks; else if (s == 0) ++errs; else if (s == 2) ++resaveRefused; else ++incons; if (t > worst) worst = t; }
}
static const char* kVals[] = {"-1", "0", "1", "2", "1e308", "-1e308", "18446744073709551615", "18446744073709551616", "4611686018427387904",
                              "9999999", "8000001", "\"NaN\"", "\"Infinity\"", "\"x\"", "null", "true", "[]", "{}", "[1,2]", "1.5", "-0.0", "\"toolpaths/0001.bin\"", "\"toolpaths/9999.bin\""};
int main(int argc, char** argv) {
    int iters = std::atoi(argv[2]); std::mt19937 rng(unsigned(std::atoi(argv[3])));
    std::ifstream f(argv[1], std::ios::binary); Bytes base((std::istreambuf_iterator<char>(f)), {});
    auto ms0 = detail::readZip(base).members;
    std::vector<std::size_t> idx;  // non-manifest members
    for (std::size_t k = 0; k < ms0.size(); ++k) if (ms0[k].name != "manifest.json") idx.push_back(k);
    for (int i = 0; i < iters; ++i) {
        auto ms = ms0;
        auto& m = ms[idx[rng() % idx.size()]];
        int mode = int(rng() % 6);
        const char* what = "";
        bool json = m.name.size() > 5 && m.name.substr(m.name.size() - 5) == ".json";
        if (mode == 0) { what = "flip"; for (int k = 0, n = 1 + int(rng() % 4); k < n && !m.data.empty(); ++k) m.data[rng() % m.data.size()] ^= std::uint8_t(1u << (rng() % 8)); }
        else if (mode == 1 && json) {
            what = "token"; std::string s(m.data.begin(), m.data.end());
            // pick a random value start after ": " or "[" or ","
            for (int tries = 0; tries < 20; ++tries) {
                std::size_t p = rng() % s.size();
                std::size_t q = s.find_first_of(":[,", p); if (q == std::string::npos) continue;
                ++q; while (q < s.size() && s[q] == ' ') ++q;
                std::size_t e = q; int depth = 0; bool str = false;
                while (e < s.size()) { char c = s[e]; if (str) { if (c == '"') str = false; } else if (c == '"') str = true; else if (c == '[' || c == '{') ++depth; else if (c == ']' || c == '}') { if (depth == 0) break; --depth; } else if ((c == ',') && depth == 0) break; ++e; }
                s.replace(q, e - q, kVals[rng() % (sizeof kVals / sizeof *kVals)]); break;
            }
            m.data.assign(s.begin(), s.end());
        } else if (mode == 1) { what = "tag"; for (int k = 0, n = 1 + int(rng() % 4); k < n && m.data.size() > 16; ++k) m.data[16 + rng() % (m.data.size() - 16)] = std::uint8_t(rng()); }
        else if (mode == 2) { what = "trunc"; if (!m.data.empty()) m.data.resize(rng() % m.data.size()); }
        else if (mode == 3) { what = "dupe"; if (!m.data.empty()) { std::size_t a = rng() % m.data.size(), l = std::min<std::size_t>(rng() % 64, m.data.size() - a); m.data.insert(m.data.begin() + long(rng() % m.data.size()), m.data.begin() + long(a), m.data.begin() + long(a + l)); } }
        else if (mode == 4) { what = "drop"; ms.erase(ms.begin() + long(&m - ms.data())); }
        else { what = "zipflip"; Bytes b = detail::writeZip(ms).bytes; for (int k = 0, n = 1 + int(rng() % 6); k < n; ++k) b[rng() % b.size()] ^= std::uint8_t(1u << (rng() % 8)); run(b, what, i); continue; }
        auto z = detail::writeZip(ms); if (!z.ok) continue;
        if (argc > 4 && i == std::atoi(argv[4])) { std::ofstream o("repro.carv", std::ios::binary); o.write((const char*)z.bytes.data(), (std::streamsize)z.bytes.size()); }
        if (argc > 4) continue;
        run(z.bytes, what, i);
    }
    std::printf("ok=%d err=%d loads-but-resave-refused=%d INCONSISTENT=%d hang=%d crash=%d worst_cpu=%.3f s\n", oks, errs, resaveRefused, incons, hangs, crashes, worst);
}
