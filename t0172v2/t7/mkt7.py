#!/usr/bin/env python3
# Writes br7.cpp / mn7.cpp: branch offset.cpp with a steady-clock timer around putBackStrays, origin/main offset.cpp with one
# around its inline step-7 block (globals g_t7_br / g_t7_main printed at exit). Compile each with the Release offset.cpp command
# of build-verify2-rel / build-verify2-mainns (via eval: quoted -D), ar into copies of the libs, link perf2.cpp -> perf2_t7.
W='/home/chris/wt-T0172'
b=open(W+'/core/src/geometry/offset.cpp').read()
b=b.replace('        putBackStrays(result, resultKeys, strays, tol);','        { auto t0_ = std::chrono::steady_clock::now(); putBackStrays(result, resultKeys, strays, tol); g_t7_br += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count(); }',1)
b='#include <chrono>\n#include <cstdio>\ndouble g_t7_br = 0; static struct P7 { ~P7() { std::fprintf(stderr, "step7 branch total %.4f s\\n", g_t7_br); } } p7_;\n'+b
open('br7.cpp','w').write(b)
m=open(W+'/build-verify2-mainsrc/core/src/geometry/offset.cpp').read()
i=m.index('    if (!strays.empty()) {'); j=m.index('        result = std::move(merged);\n    }\n    return result;\n}',i); end=j+len('        result = std::move(merged);\n    }')
m=m[:i]+'    { auto t0_ = std::chrono::steady_clock::now();\n'+m[i:end]+'\n    g_t7_main += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count(); }'+m[end:]
m='#include <chrono>\n#include <cstdio>\ndouble g_t7_main = 0; static struct P7M { ~P7M() { std::fprintf(stderr, "step7 main total %.4f s\\n", g_t7_main); } } p7_;\n'+m
open('mn7.cpp','w').write(m)
