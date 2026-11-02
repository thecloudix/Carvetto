// Combined worst case: job.json object flood just under the document budget + 8M-move cache.
#include <cstdio>
#include <cstring>
#include <fstream>
#include <malloc.h>
#include <span>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <ctime>
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
using namespace carvetto::document;
using Bytes = std::vector<std::uint8_t>;
static long statusKb(const char* key) { std::ifstream f("/proc/self/status"); std::string l; while (std::getline(f, l)) if (l.rfind(key, 0) == 0) return std::stol(l.substr(std::strlen(key))); return -1; }
static double cpuNow() { timespec ts; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
int main(int argc, char** argv) {
    long n = std::stol(argv[1]); unsigned long moves = std::stoul(argv[2]); int interim = argc > 3 ? std::atoi(argv[3]) : 0;
    std::ifstream f("b1.carv", std::ios::binary); Bytes b((std::istreambuf_iterator<char>(f)), {});
    auto ms = detail::readZip(b).members;
    std::vector<detail::ZipMember> out;
    for (auto& m : ms) {
        if (m.name == "job.json") {
            std::string j(m.data.begin(), m.data.end());
            std::string objs; objs.reserve(size_t(n) * 70);
            for (long i = 0; i < n; ++i) { objs += (i ? "," : ""); objs += "{\"contours\":[],\"id\":" + std::to_string(10 + i) + ",\"layer\":2,\"name\":\"\",\"params\":{\"type\":\"none\"},\"sheet\":1,\"type\":\"vector\"}"; }
            j.replace(j.find("\"objects\": []"), 13, "\"objects\": [" + objs + "]");
            j.replace(j.find("\"next_id\": 4"), 12, "\"next_id\": " + std::to_string(20 + n));
            j.replace(j.find("\"count\": 1"), 10, "\"count\": " + std::to_string(moves));
            out.push_back({m.name, Bytes(j.begin(), j.end())});
        } else if (m.name == "toolpaths/0001.bin") {
            Bytes c{'C','V','T','P',1,0,0,0}; for (int i = 0; i < 8; ++i) c.push_back(std::uint8_t(std::uint64_t(moves) >> (8*i)));
            c.insert(c.end(), moves, 0x70); out.push_back({m.name, c});
        } else out.push_back(m);
    }
    // interim members: each ~1 MiB JSON of ["rapid",0,0,0] moves (no ratio check below 1 MiB)
    for (int k = 0; k < interim; ++k) {
        std::string s = "{\"format\":\"carvetto-toolpath\",\"format_version\":1,\"operation\":\"pocket\",\"name\":\"x\",\"params\":{},\"safe_z\":5,\"tool\":{\"name\":\"t\",\"tool_number\":1,\"diameter\":3,\"feed_rate\":1,\"plunge_rate\":1,\"spindle_speed\":1},\"moves\":[";
        bool first = true; while (s.size() < (1u << 20) - 64) { s += first ? "" : ","; s += "[\"rapid\",0,0,0]"; first = false; }
        s += "]}";
        char name[32]; std::snprintf(name, sizeof name, "toolpaths/%04d.json", k + 1);
        out.push_back({name, Bytes(s.begin(), s.end())});
    }
    Bytes file = detail::writeZip(out).bytes;
    if (argc > 4) { std::ofstream o(argv[4], std::ios::binary); o.write((const char*)file.data(), (std::streamsize)file.size()); }
    pid_t pid = fork();
    if (pid == 0) {
        alarm(120); malloc_trim(0); { std::ofstream c("/proc/self/clear_refs"); c << "5"; }
        long base = statusKb("VmRSS:"); double t0 = cpuNow();
        auto r = loadCarv(std::span<const std::uint8_t>(file));
        double t1 = cpuNow(); long hwm = statusKb("VmHWM:");
        std::printf("objects=%ld moves=%lu interim=%d file=%zu B cpu=%.2f s peak=%.1f MiB %s\n", n, moves, interim, file.size(), t1 - t0, (hwm - base) / 1024.0,
                    r.ok() ? "OK" : r.error().message.substr(0, 160).c_str());
        std::fflush(stdout); _exit(0);
    }
    int st; waitpid(pid, &st, 0); if (WIFSIGNALED(st)) std::printf("signal %d\n", WTERMSIG(st));
}
