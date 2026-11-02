// T-0132 verifier round 3: interim over-reservation (fresh probe budget) peak per file byte under the per-byte move rule.
// Builds crafted files in memory (padded with a random stored-ish binary member so moves ~= file
// size), loads each in a forked child, prints peak growth (MiB) and B/file-byte.
#include <cstdio>
#include <cstring>
#include <fstream>
#include <malloc.h>
#include <random>
#include <span>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <ctime>
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
using namespace carvetto::document;
using carvetto::toolpath::Move;
using Bytes = std::vector<std::uint8_t>;
static long statusKb(const char* key) { std::ifstream f("/proc/self/status"); std::string l; while (std::getline(f, l)) if (l.rfind(key, 0) == 0) return std::stol(l.substr(std::strlen(key))); return -1; }
static double cpuNow() { timespec ts; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static void probe(const std::string& label, const Bytes& file) {
    pid_t pid = fork();
    if (pid == 0) {
        alarm(120); malloc_trim(0); { std::ofstream c("/proc/self/clear_refs"); c << "5"; }
        long base = statusKb("VmRSS:"); double t0 = cpuNow();
        auto r = loadCarv(std::span<const std::uint8_t>(file));
        double t1 = cpuNow(); long hwm = statusKb("VmHWM:");
        std::size_t mv = 0; if (r.ok()) for (auto& t : r.value().document.toolpaths()) mv += t.moveCount();
        double peak = double(hwm - base) * 1024.0;
        std::printf("%-34s file=%9zu B moves=%8zu cpu=%6.2f s peak=%7.1f MiB = %6.1f B/file-byte  %s\n", label.c_str(), file.size(), mv, t1 - t0,
                    peak / 1048576.0, peak / double(file.size()), r.ok() ? "OK" : r.error().message.substr(0, 150).c_str());
        std::fflush(stdout); _exit(0);
    }
    int st; waitpid(pid, &st, 0); if (WIFSIGNALED(st)) std::printf("%s: signal %d\n", label.c_str(), WTERMSIG(st));
}
static Bytes randomBytes(std::size_t n, unsigned seed) { std::mt19937_64 g(seed); Bytes b(n); for (auto& c : b) c = std::uint8_t(g()); return b; }
static std::vector<detail::ZipMember> baseMembers(int nRec) {
    Document doc;
    for (int i = 0; i < nRec; ++i) { ToolpathRecord r; r.name = "t"; r.selection = VectorSelection::allVectors(); r.moves = shareMoves({Move::rapid({0, 0, 5})}); doc.insertToolpath(r); }
    return detail::readZip(saveCarv(doc).value()).members;
}
static std::string interimText(std::size_t k, std::size_t junk) {
    std::string s = "{\"format\":\"carvetto-toolpath\",\"format_version\":1,\"operation\":\"pocket\",\"name\":\"x\",\"params\":{},\"safe_z\":5,\"tool\":{\"name\":\"t\",\"tool_number\":1,\"diameter\":3,\"feed_rate\":1,\"plunge_rate\":1,\"spindle_speed\":1},\"zjunk\":\"";
    std::mt19937_64 g(7); for (std::size_t i = 0; i < junk; ++i) s += char('a' + g() % 26);
    s += "\",\"moves\":[";
    s.reserve(s.size() + k * 16);
    for (std::size_t i = 0; i < k; ++i) { if (i) s += ','; s += "[\"rapid\",0,0,0]"; }
    s += "]}";
    return s;
}
// native: nRec records sharing `total` moves of 1 byte each; padded so file ~ total + slack
static Bytes nativeFile(int nRec, std::uint64_t total) {
    auto ms = baseMembers(nRec);
    std::uint64_t per = total / std::uint64_t(nRec);
    std::vector<detail::ZipMember> out;
    for (auto& m : ms) {
        if (m.name == "job.json") {
            std::string j(m.data.begin(), m.data.end()); std::size_t p = 0;
            while ((p = j.find("\"count\": 1", p)) != std::string::npos) { std::string r = "\"count\": " + std::to_string(per); j.replace(p, 10, r); p += r.size(); }
            out.push_back({m.name, Bytes(j.begin(), j.end())});
        } else if (m.name.rfind("toolpaths/", 0) == 0) {
            Bytes c{'C','V','T','P',1,0,0,0}; for (int i = 0; i < 8; ++i) c.push_back(std::uint8_t(per >> (8*i)));
            c.insert(c.end(), per, 0x70); out.push_back({m.name, c});
        } else out.push_back(m);
    }
    Bytes probeFile = detail::writeZip(out).bytes;
    std::uint64_t need = per * std::uint64_t(nRec);
    if (probeFile.size() < need) out.push_back({"zpad.bin", randomBytes(need - probeFile.size() + 200, 1)});
    return detail::writeZip(out).bytes;
}
static Bytes interimFile(std::size_t k, std::size_t junk) {
    auto ms = baseMembers(0);
    std::string s = interimText(k, junk);
    ms.push_back({"toolpaths/0001.json", Bytes(s.begin(), s.end())});
    Bytes probeFile = detail::writeZip(ms).bytes;
    if (probeFile.size() < k) ms.push_back({"zpad.bin", randomBytes(k - probeFile.size() + 200, 2)});
    return detail::writeZip(ms).bytes;
}

// native record with n1 one-byte moves + one interim member with n2 moves, padded to >= max(n1,n2) bytes.
static Bytes mixedFile(std::uint64_t n1, std::size_t n2, int interimMembers) {
    auto ms = baseMembers(1);
    std::vector<detail::ZipMember> out;
    for (auto& m : ms) {
        if (m.name == "job.json") {
            std::string j(m.data.begin(), m.data.end()); std::size_t p = j.find("\"count\": 1");
            j.replace(p, 10, "\"count\": " + std::to_string(n1)); out.push_back({m.name, Bytes(j.begin(), j.end())});
        } else if (m.name.rfind("toolpaths/", 0) == 0) {
            Bytes c{'C','V','T','P',1,0,0,0}; for (int i = 0; i < 8; ++i) c.push_back(std::uint8_t(n1 >> (8*i)));
            c.insert(c.end(), n1, 0x70); out.push_back({m.name, c});
        } else out.push_back(m);
    }
    std::string s = interimText(n2, n2 * 16 / 150);
    for (int k = 0; k < interimMembers; ++k) { char nm[32]; std::snprintf(nm, sizeof nm, "toolpaths/%04d.json", k + 2); out.push_back({nm, Bytes(s.begin(), s.end())}); }
    Bytes probeFile = detail::writeZip(out).bytes;
    std::uint64_t need = std::max<std::uint64_t>(n1, n2);
    if (probeFile.size() < need) out.push_back({"zpad.bin", randomBytes(need - probeFile.size() + 200, 3)});
    return detail::writeZip(out).bytes;
}
int main() {
    probe("native 3.9M alone", mixedFile(3900000, 0, 0));
    probe("native 3.9M + interim 3.9M", mixedFile(3900000, 3900000, 1));
    probe("native 1 + 2 interim x 3.9M", mixedFile(1, 3900000, 2));
    probe("native 1 + 1 interim x 3.9M", mixedFile(1, 3900000, 1));
    return 0;
}
