#!/bin/bash
W=/home/claude/wt-T-0121
D=$W/build-verify4/_deps
FC="-DFETCHCONTENT_SOURCE_DIR_CATCH2=$D/catch2-src -DFETCHCONTENT_SOURCE_DIR_CLIPPER2=$D/clipper2-src -DFETCHCONTENT_SOURCE_DIR_MINIZ=$D/miniz-src -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=$D/nlohmann_json-src"
cmake -S $W -B $W/build-verify5-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-Werror $FC > $W/build-verify5-rel.cfg.log 2>&1
cmake --build $W/build-verify5-rel -j2 > $W/build-verify5-rel.build.log 2>&1; echo "REL BUILD EXIT $?" >> $W/build-verify5-rel.build.log
cmake -S $W -B $W/build-verify5 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-Werror $FC > $W/build-verify5.cfg.log 2>&1
cmake --build $W/build-verify5 -j2 > $W/build-verify5.build.log 2>&1; echo "DBG BUILD EXIT $?" >> $W/build-verify5.build.log
