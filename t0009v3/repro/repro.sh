#!/bin/bash
# T-0009 round-3 verifier repros (branch 8b4e69e vs origin/main 0fa0699). Build the drivers first:
#   cd /home/chris/harness/t0009v3 && for x in x2 ptq ptchk tipcap gap mcover dbgrun; do ./mk.sh br $x.cpp $x; done; ./mk.sh br x2.cpp x4
D=/home/chris/wt-T0009/build-verify3-drv; cd $D
echo "== F1 v9 0xf0003fad Round Left d=3.753745644092505: arc about the tip of the sub-tol spike (spans 20/21), 145..81 deg, valid offset, branch up to 2.07 mm away, main covers"
./tipcap 0xf0003fad 0 0 20 145 81
echo "== F1 v9 0x30000054 Round Right d=4.688: hairpin with a micro-cluster at its tip (run A 103, B 108), collapsed corner arc about V, branch ~10-12 mm away, main covers"
./ptchk 0x30000054 0 1 103 -66 94
echo "== F1 v9 0x312003dd Round Left d=21.58 closed: 33 mm of offset missing, a valid point 13.6 mm from the branch output, main 1.6e-4"
./ptq 0x312003dd 0 0 -10003.171654950755 -29.95412398179927 -9985.667869745781 -24.22555833465961
echo "== F2 v9 0x35000108 Round Left d=11.35 closed: closed loop -> one open chain, 10.6 mm gap (sub-tol cluster at a hairpin, spans 13-17, net turn +185 deg vs main's -175)"
./gap 0x35000108 0 0; ./mcover v9 0x35000108 0 0
echo "== F3 crash: g2 0x32000123 (closed, 134 lines, 133 sub-tol), d=0.0411, Left, every join: std::length_error"
g++ -std=c++20 -O2 -I/home/chris/wt-T0009/core/include /home/chris/harness/t0009v3/repro/crash_repro.cpp /home/chris/wt-T0009/build-verify3-rel/core/libcarvetto_core.a /home/chris/wt-T0009/build-verify3-rel/_deps/clipper2-build/libClipper2.a -o crash_repro && ./crash_repro
echo "== F4 Sharp/Square join kept at a sub-tol span swallows valid offset: v9 0x313007af Sharp Left d=24.05 (corner 18->19, 1.06 mm line -> 4.47e-15 mm line)"
./ptq 0x313007af 1 0 -16.840201239003022 19.60592157528097
echo "   v9 0xb0300988 Sharp Left d=11.30 (corner 21->22 at a 0.001 mm span before a 137 deg near-reversal)"
./ptq 0xb0300988 1 0 15.2473187657562 -3.9359180763716743
echo "== F5 v9 0x31600b4f Round Left d=0.425 closed: 2.4 mm of span 46's offset missing (not at a run)"
./ptq 0x31600b4f 0 0 -10001.27500410687 -933.55003557966552
echo "== round-2 regressions B1/e0000119 (fixed): branch vs main"
./x2 mainone 0xf0003fad 1 1 0; ./x2 mainone 0xf0003fad 1 2 0; ./x2 mainone 0xe0000119 1 0 1
