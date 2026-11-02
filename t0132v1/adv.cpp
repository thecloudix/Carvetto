// Verifier driver T-0132: adversarial .carv loads, peak RSS growth, timing.
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <malloc.h>
#include <random>
#include <span>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>
#include <ctime>

#include "carv_json_reader.hpp"
#include "carv_moves.hpp"
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"

using namespace carvetto;
using namespace carvetto::document;
using carvetto::toolpath::Move;
using Bytes = std::vector<std::uint8_t>;

static long statusKb(const char* key) {
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind(key, 0) == 0) return std::stol(line.substr(std::strlen(key)));
    }
    return -1;
}

static double cpuNow() {
    timespec ts; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Forks; child loads; reports result code/message, cpu time, peak growth (MiB).
static void probe(const std::string& label, const Bytes& file) {
    int pipefd[2]; if (pipe(pipefd) != 0) return;
    pid_t pid = fork();
    if (pid == 0) {
        close(pipefd[0]);
        alarm(60);
        malloc_trim(0);
        { std::ofstream c("/proc/self/clear_refs"); c << "5"; }
        long base = statusKb("VmRSS:");
        double t0 = cpuNow();
        auto r = loadCarv(std::span<const std::uint8_t>(file));
        double t1 = cpuNow();
        long hwm = statusKb("VmHWM:");
        char buf[2048];
        std::string msg = r.ok() ? ("OK toolpaths=" + std::to_string(r.value().document.toolpaths().size()) +
                                    " moves0=" + std::to_string(r.value().document.toolpaths().empty() ? 0 : r.value().document.toolpaths()[0].moveCount()))
                                 : ("ERR code=" + std::to_string(int(r.error().code)) + " " + r.error().message.substr(0, 200));
        std::snprintf(buf, sizeof buf, "%-28s file=%9zu B  cpu=%7.3f s  peak=%7.1f MiB  %s\n", label.c_str(), file.size(), t1 - t0,
                      (hwm - base) / 1024.0, msg.c_str());
        if (write(pipefd[1], buf, std::strlen(buf)) < 0) {}
        _exit(0);
    }
    close(pipefd[1]);
    std::string out; char b[4096]; ssize_t n;
    while ((n = read(pipefd[0], b, sizeof b)) > 0) out.append(b, size_t(n));
    close(pipefd[0]);
    int st = 0; waitpid(pid, &st, 0);
    if (WIFSIGNALED(st)) std::printf("%-28s CHILD SIGNAL %d\n", label.c_str(), WTERMSIG(st));
    else std::printf("%s", out.c_str());
    std::fflush(stdout);
}

static std::vector<detail::ZipMember> members(const Bytes& a) { auto r = detail::readZip(a); if (!r.ok) { std::cerr << r.error; std::abort(); } return r.members; }
static Bytes zip(const std::vector<detail::ZipMember>& ms) { auto r = detail::writeZip(ms); if (!r.ok) { std::cerr << r.error; std::abort(); } return r.bytes; }
static std::string text(const Bytes& b) { return {b.begin(), b.end()}; }
static Bytes bytes(const std::string& s) { return {s.begin(), s.end()}; }
static std::string replaceAll(std::string s, const std::string& a, const std::string& b) {
    size_t p = 0; while ((p = s.find(a, p)) != std::string::npos) { s.replace(p, a.size(), b); p += b.size(); } return s; }

static void putU64(Bytes& o, std::uint64_t v) { for (int i = 0; i < 8; ++i) o.push_back(std::uint8_t(v >> (8 * i))); }
static Bytes header(std::uint64_t count) { Bytes b{'C', 'V', 'T', 'P', 1, 0, 0, 0}; putU64(b, count); return b; }
static Bytes fillCache(std::uint64_t count, std::uint8_t tag) { Bytes b = header(count); b.insert(b.end(), count, tag); return b; }

// Base doc: n toolpaths with 1 move each; returns file.
static Bytes baseFile(int n, Move m = Move::rapid({0, 0, 5})) {
    Document doc;
    for (int i = 0; i < n; ++i) {
        ToolpathRecord r; r.name = "tp" + std::to_string(i); r.selection = VectorSelection::allVectors();
        r.moves = shareMoves({m}); doc.insertToolpath(r);
    }
    auto s = saveCarv(doc); if (!s.ok()) { std::cerr << s.error().message; std::abort(); }
    return s.value();
}

// Replace job count values (all "count": 1) and caches.
static Bytes withCaches(const Bytes& base, const std::vector<std::pair<std::uint64_t, Bytes>>& caches) {
    auto ms = members(base);
    std::string job;
    for (auto& m : ms) if (m.name == "job.json") job = text(m.data);
    std::vector<detail::ZipMember> out;
    size_t pos = 0; int idx = 0;
    while ((pos = job.find("\"count\": 1", pos)) != std::string::npos && idx < (int)caches.size()) {
        std::string rep = "\"count\": " + std::to_string(caches[idx].first);
        job.replace(pos, 10, rep); pos += rep.size(); ++idx;
    }
    for (auto& m : ms) {
        if (m.name == "job.json") out.push_back({m.name, bytes(job)});
        else if (m.name.rfind("toolpaths/", 0) == 0) {
            int k = std::stoi(m.name.substr(10, 4)) - 1;
            out.push_back({m.name, caches[k].second});
        } else out.push_back(m);
    }
    return zip(out);
}

int main(int argc, char** argv) {
    std::string only = argc > 1 ? argv[1] : "";
    auto want = [&](const char* k) { return only.empty() || only == k; };
    std::printf("sizeof(Move)=%zu\n", sizeof(Move));
    const Bytes b1 = baseFile(1);
    if (want("small")) {
        // Worst case: max count of 1-byte moves (rapid, same xyz).
        probe("8M x 1B rapid (1 tp)", withCaches(b1, {{8'000'000, fillCache(8'000'000, 0x70)}}));
        probe("8000001 (over budget)", withCaches(b1, {{8'000'001, fillCache(8'000'001, 0x70)}}));
        probe("4194305 x 1B (1 tp)", withCaches(b1, {{4194305, fillCache(4194305, 0x70)}}));
        probe("2M x 1B (1 tp)", withCaches(b1, {{2'000'000, fillCache(2'000'000, 0x70)}}));
        const Bytes b8 = baseFile(8);
        std::vector<std::pair<std::uint64_t, Bytes>> c8;
        for (int i = 0; i < 8; ++i) c8.push_back({1'000'000, fillCache(1'000'000, 0x70)});
        probe("8 tp x 1M x 1B", withCaches(b8, c8));
        // truncated at the end: count says 8M, last byte missing (size check then CRC/early end)
        Bytes t = fillCache(8'000'000, 0x70); t.pop_back();
        probe("8M minus 1 byte", withCaches(b1, {{8'000'000, t}}));
        // last move invalid kind
        Bytes k = fillCache(8'000'000, 0x70); k.back() = 0x77;
        probe("8M, last tag kind 7", withCaches(b1, {{8'000'000, k}}));
        // trailing data
        Bytes tr = fillCache(8'000'000, 0x70); tr.insert(tr.end(), 40, 0x70);
        probe("8M + 40 trailing", withCaches(b1, {{8'000'000, tr}}));
    }
    if (want("many")) {
        const Bytes b = baseFile(9998);
        probe("9998 tp x 1 move", b);
        std::vector<std::pair<std::uint64_t, Bytes>> c;
        for (int i = 0; i < 9998; ++i) c.push_back({800, fillCache(800, 0x70)});
        probe("9998 tp x 800 x 1B", withCaches(b, c));
    }
    if (want("tags")) {
        // every tag value as the first and as the second move
        int okc = 0, errc = 0;
        for (int tag = 0; tag < 256; ++tag) {
            Bytes c = header(2);
            c.push_back(0x09);  // feed with feed override
            for (int i = 0; i < 3; ++i) { double d = 1.5; std::uint64_t u; std::memcpy(&u, &d, 8); putU64(c, u); }
            { double d = 100; std::uint64_t u; std::memcpy(&u, &d, 8); putU64(c, u); }
            c.push_back(std::uint8_t(tag));
            for (int i = 0; i < 6; ++i) { double d = 2.5; std::uint64_t u; std::memcpy(&u, &d, 8); putU64(c, u); }
            // trim to exact expected length is unknown; try all lengths 0..48 extra
            for (int extra = 0; extra <= 48; ++extra) {
                Bytes cc(c.begin(), c.begin() + 16 + 33 + 1 + extra);
                auto f = withCaches(b1, {{2, cc}});
                auto r = loadCarv(std::span<const std::uint8_t>(f));
                if (r.ok()) {
                    ++okc;
                    // must re-save and the moves must be finite
                    auto s = saveCarv(r.value().document);
                    if (!s.ok()) std::printf("tag %d extra %d: loads but does not save: %s\n", tag, extra, s.error().message.c_str());
                } else ++errc;
            }
        }
        std::printf("tags: %d ok, %d err\n", okc, errc);
        // nan / inf bit patterns
        for (std::uint64_t bits : {0x7ff0000000000000ull, 0xfff0000000000000ull, 0x7ff8000000000001ull, 0x7ff0000000000001ull, 0x0000000000000001ull, 0x8000000000000000ull}) {
            Bytes c = header(1); c.push_back(0x00 | 0x30); putU64(c, bits);  // rapid, z only
            auto f = withCaches(b1, {{1, c}});
            auto r = loadCarv(std::span<const std::uint8_t>(f));
            std::printf("z bits %016llx: %s\n", (unsigned long long)bits, r.ok() ? "OK" : r.error().message.c_str());
            if (r.ok()) {
                auto s = saveCarv(r.value().document);
                auto r2 = loadCarv(std::span<const std::uint8_t>(s.value()));
                std::uint64_t b2; std::memcpy(&b2, &(*r2.value().document.toolpaths()[0].moves)[0].to.z, 8);
                std::printf("   round trip bits %016llx, file identical=%d\n", (unsigned long long)b2, int(s.value() == f));
            }
        }
    }
    if (want("lies")) {
        // count in job.json != header
        Bytes c = fillCache(10, 0x70);
        probe("job 11, header 10", withCaches(b1, {{11, c}}));
        probe("job 9, header 10", withCaches(b1, {{9, c}}));
        probe("job 0 header 0", withCaches(b1, {{0, header(0)}}));
        probe("count 0 cache 64B", withCaches(b1, {{0, [] { Bytes b = header(0); b.insert(b.end(), 48, 0x70); return b; }()}}));
        probe("count 1e19", withCaches(b1, {{10'000'000'000'000'000'000ull, c}}));
        // json count as float / negative
        {
            auto ms = members(b1);
            for (const char* rep : {"\"count\": 1.0", "\"count\": -1", "\"count\": 1e3", "\"count\": 18446744073709551616", "\"count\": \"1\"", "\"count\": 01"}) {
                std::vector<detail::ZipMember> o;
                for (auto& m : ms) o.push_back(m.name == "job.json" ? detail::ZipMember{m.name, bytes(replaceAll(text(m.data), "\"count\": 1", rep))} : m);
                probe(rep, zip(o));
            }
        }
    }
    return 0;
}
