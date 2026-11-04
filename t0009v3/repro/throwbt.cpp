// Prints a backtrace at every C++ throw (link with -Wl,--wrap=__cxa_throw -rdynamic).
#include <execinfo.h>
#include <cstdio>
#include <cstdlib>
extern "C" void __real___cxa_throw(void*, void*, void (*)(void*));
extern "C" void __wrap___cxa_throw(void* e, void* t, void (*d)(void*)) {
    void* b[40]; int n = backtrace(b, 40); char** s = backtrace_symbols(b, n);
    for (int i = 0; i < n; ++i) std::fprintf(stderr, "BT %s\n", s[i]);
    __real___cxa_throw(e, t, d);
}
extern "C" [[noreturn]] void __real__ZSt20__throw_length_errorPKc(const char*);
extern "C" [[noreturn]] void __wrap__ZSt20__throw_length_errorPKc(const char* m) {
    void* b[40]; int n = backtrace(b, 40); char** s = backtrace_symbols(b, n);
    for (int i = 0; i < n; ++i) std::fprintf(stderr, "BT %s\n", s[i]);
    __real__ZSt20__throw_length_errorPKc(m);
}
