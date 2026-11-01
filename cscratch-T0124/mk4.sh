#!/bin/bash
c++ -std=c++20 -O2 -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I/home/claude/wt-T-0120/core/include -I/home/claude/vscratch/t0124v4/tools -I/home/claude/vscratch/t0124v2 -isystem /home/claude/wt-T-0120/build-release/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem /home/claude/wt-T-0120/build-release/_deps/nlohmann_json-src/include "$1" -o "$2" /home/claude/wt-T-0120/build-release/core/libcarvetto_core.a $(find /home/claude/wt-T-0120/build-release/_deps -name "libClipper2*.a") /home/claude/wt-T-0120/build-release/core/libcarvetto_miniz.a 2>&1 | grep error
exit ${PIPESTATUS[0]}
