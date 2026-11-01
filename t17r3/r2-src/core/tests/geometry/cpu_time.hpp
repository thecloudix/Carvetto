#pragma once

// Process CPU time for the offset timing tests (T-0017). std::clock() measures process CPU time
// on POSIX but wall-clock time since process start on MSVC, so a "CPU time" bound measured with
// it would be a wall-clock bound in Windows CI and could fail under load. On Windows this reads
// the process's user + kernel time with GetProcessTimes instead (100 ns units, updated per
// scheduler tick, ~16 ms).

#include <ctime>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace carvetto::test {

/// CPU time used by this process so far, in seconds (user + system).
inline double processCpuSeconds() {
#ifdef _WIN32
    FILETIME creationTime{};
    FILETIME exitTime{};
    FILETIME kernelTime{};
    FILETIME userTime{};
    if (GetProcessTimes(GetCurrentProcess(), &creationTime, &exitTime, &kernelTime, &userTime) == 0) {
        return 0.0;
    }
    auto seconds = [](const FILETIME& t) {
        ULARGE_INTEGER v{};
        v.LowPart = t.dwLowDateTime;
        v.HighPart = t.dwHighDateTime;
        return static_cast<double>(v.QuadPart) * 1e-7;
    };
    return seconds(kernelTime) + seconds(userTime);
#else
    return static_cast<double>(std::clock()) / static_cast<double>(CLOCKS_PER_SEC);
#endif
}

}  // namespace carvetto::test
