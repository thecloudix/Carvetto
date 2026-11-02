#include <cstdio>
#include <cstring>
#include <fstream>
#include <malloc.h>
#include <span>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <ctime>
#include "carvetto/document/carv_io.hpp"
using namespace carvetto::document;
static long kb(const char* key) { std::ifstream f("/proc/self/status"); std::string l; while (std::getline(f, l)) if (l.rfind(key, 0) == 0) return std::stol(l.substr(std::strlen(key))); return -1; }
int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        std::ifstream f(argv[i], std::ios::binary); std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(f)), {});
        pid_t pid = fork();
        if (pid == 0) { alarm(120); malloc_trim(0); { std::ofstream c("/proc/self/clear_refs"); c << "5"; } long base = kb("VmRSS:");
            timespec a, c2; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &a);
            auto r = loadCarv(std::span<const std::uint8_t>(b)); clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &c2);
            std::printf("%s: %zu B, %s, cpu %.2f s, peak growth %.1f MiB\n", argv[i], b.size(), r.ok() ? "OK" : r.error().message.c_str(), double(c2.tv_sec - a.tv_sec) + double(c2.tv_nsec - a.tv_nsec) * 1e-9, (kb("VmHWM:") - base) / 1024.0);
            std::fflush(stdout); _exit(0); }
        int st; waitpid(pid, &st, 0); if (WIFSIGNALED(st)) std::printf("%s: signal %d\n", argv[i], WTERMSIG(st));
    }
}
