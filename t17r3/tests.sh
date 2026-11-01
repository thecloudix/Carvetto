#!/bin/bash
S=/home/claude/t17r3
cd /home/claude/wt-T-0017
nice ctest --test-dir build --output-on-failure > $S/ctest_dbg.log 2>&1
nice ctest --test-dir build -j8 --repeat until-fail:3 > $S/ctest_j8.log 2>&1
nice ctest --test-dir build-rel --output-on-failure > $S/ctest_rel.log 2>&1
B=build-rel
nice g++ -std=c++20 -O2 -DNDEBUG -DCARVETTO_OFFSET_TRIALS=96 core/tests/geometry/open_offset_chains_test.cpp -Icore/include -Icore/tests/geometry -Icore/tests -I$B/_deps/catch2-src/src -I$B/_deps/catch2-build/generated-includes $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/_deps/catch2-build/src/libCatch2Main.a $B/_deps/catch2-build/src/libCatch2.a -o $S/t96 && (cd $S && nice ./t96 > t96.log 2>&1; echo rc=$? >> t96.log)
echo done > $S/tests.done
