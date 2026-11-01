#!/bin/bash
V=/home/claude/vscratch/t0017v3; W=/home/claude/wt-T-0017; T=$1; B=$V/b-$T
D="-DFETCHCONTENT_SOURCE_DIR_CATCH2=$V/deps/catch2-src -DFETCHCONTENT_SOURCE_DIR_CLIPPER2=$V/deps/clipper2-src -DFETCHCONTENT_SOURCE_DIR_MINIZ=$V/deps/miniz-src -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=$V/deps/nlohmann_json-src -DFETCHCONTENT_FULLY_DISCONNECTED=ON"
rm -rf $B; cmake -S $W -B $B -G Ninja -DCMAKE_BUILD_TYPE=$T -DCARVETTO_BUILD_APP=OFF $D > $V/cfg-$T.log 2>&1 && cmake --build $B -j2 > $V/build-$T.log 2>&1; echo "exit $?" >> $V/build-$T.log
