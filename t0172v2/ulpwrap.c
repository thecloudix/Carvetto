/* Link-time libm perturbation: -Wl,--wrap=sin,--wrap=cos,... Each result moved by a deterministic
   pseudo-random -K..K ulps per argument (ULP_K, ULP_SEED env; K = 0 -> unchanged). ULP_COUNT=1 prints call counts. */
#define _GNU_SOURCE
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static int g_k = -1; static uint64_t g_seed; static long g_calls[9];
static void init(void) { if (g_k >= 0) return; const char* k = getenv("ULP_K"); const char* s = getenv("ULP_SEED"); g_k = k ? atoi(k) : 0; g_seed = s ? strtoull(s, 0, 0) : 0; }
static void report(void) __attribute__((destructor));
static void report(void) { if (getenv("ULP_COUNT")) fprintf(stderr, "ulpwrap calls sin %ld cos %ld atan2 %ld hypot %ld acos %ld asin %ld tan %ld sincos %ld atan %ld (K=%d seed %llu)\n", g_calls[0], g_calls[1], g_calls[2], g_calls[3], g_calls[4], g_calls[5], g_calls[6], g_calls[7], g_calls[8], g_k, (unsigned long long)g_seed); }
static uint64_t mix(uint64_t x) { x ^= x >> 30; x *= 0xbf58476d1ce4e5b9ULL; x ^= x >> 27; x *= 0x94d049bb133111ebULL; x ^= x >> 31; return x; }
static double pert(double r, uint64_t key, int fn) {
    init(); g_calls[fn]++;
    if (g_k == 0 || !isfinite(r) || r == 0.0) return r;
    uint64_t h = mix(key ^ mix(g_seed * 0x9E3779B97F4A7C15ULL + (uint64_t)fn));
    int off = (int)(h % (uint64_t)(2 * g_k + 1)) - g_k;
    for (; off > 0; --off) r = nextafter(r, INFINITY);
    for (; off < 0; ++off) r = nextafter(r, -INFINITY);
    return r;
}
static uint64_t bits(double x) { uint64_t u; memcpy(&u, &x, 8); return u; }
double __real_sin(double); double __real_cos(double); double __real_atan2(double, double); double __real_hypot(double, double);
double __real_acos(double); double __real_asin(double); double __real_tan(double); double __real_atan(double);
double __wrap_sin(double x) { return pert(__real_sin(x), bits(x), 0); }
double __wrap_cos(double x) { return pert(__real_cos(x), bits(x), 1); }
double __wrap_atan2(double y, double x) { return pert(__real_atan2(y, x), bits(y) * 31 + mix(bits(x)), 2); }
double __wrap_hypot(double x, double y) { return pert(__real_hypot(x, y), bits(x) * 31 + mix(bits(y)), 3); }
double __wrap_acos(double x) { return pert(__real_acos(x), bits(x), 4); }
double __wrap_asin(double x) { return pert(__real_asin(x), bits(x), 5); }
double __wrap_tan(double x) { return pert(__real_tan(x), bits(x), 6); }
double __wrap_atan(double x) { return pert(__real_atan(x), bits(x), 8); }
void __wrap_sincos(double x, double* s, double* c) { init(); g_calls[7]++; *s = __wrap_sin(x); *c = __wrap_cos(x); }
