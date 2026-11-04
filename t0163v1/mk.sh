#!/bin/bash
# mk.sh src.cpp out {br|main|asan}  -- links against the T-0163 verifier's builds in /home/chris/wt-T0163
# br/main: Release (build-verify1-rel/{br,main}); asan: Debug ASan+UBSan branch build (build-verify1-asan, libs snapshotted to libsnap/);
# asano2: RelWithDebInfo ASan+UBSan branch build (build-verify1-asan-o2), used for the sweeps (the -O0 one is too slow)
W0=/home/chris/wt-T0163; R=$W0/build-verify1-rel; F="-O2"
if [ "$3" = main ]; then W=$R/mainsrc; B=$R/main;
elif [ "$3" = asan ]; then W=$W0; B=$W0/build-verify1-asan; F="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined";
elif [ "$3" = asano2 ]; then W=$W0; B=$W0/build-verify1-asan-o2; F="-O2 -g -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined";
else W=$W0; B=$R/br; fi
T=$(dirname $(readlink -f $0))
L=$B/core/libcarvetto_core.a; C=$B/_deps/clipper2-build/libClipper2.a; M=$B/core/libcarvetto_miniz.a
if [ "$3" = asan ]; then L=$B/libsnap/libcarvetto_core.a; C=$B/libsnap/libClipper2.a; M=$B/libsnap/libcarvetto_miniz.a; fi
c++ $EXTRA $F -std=c++20 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $L $C $M
