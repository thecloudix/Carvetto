#!/bin/bash
# T-0009 round-4 verifier repros: branch task/T-0009-subtol-r4 eab77ce vs origin/main e2e5345 (namespaced cvmain).
# Build first:  cd /home/chris/harness/t0009v4 && for x in x2 ptq lprobe loopchk subnear; do ./mk.sh br $x.cpp $x; done
D=/home/chris/wt-T0009r4/build-verify4-drv; H=/home/chris/harness/t0009v4; cd $D
echo "== R1 closed loop opened (Round): v9 0x45000866 Left d=0.00602 (fresh cl seed; coords ~1e5): main closed 12-span loop, branch open 9-span chain, gap 1.64e-3 (also in the round-2 build)"
python3 $H/opengap.py v9 0x45000866 0 0
echo "== R2 adversarial 0x50400046 Round Left d=0.0306: collapsed cluster (box 3.998 tol) at a -55 deg right turn; the Round corner arc (0.03 mm) is missing, chain split in two; main has it (survives XK_NETTURN/XK_ARCSEED)"
./x2 advone 0x50400046 0 0 2>/dev/null | grep -E "^(OK|FLAG)" | cut -c1-330
echo "== R3 adversarial 0x50300046 Left d=0.00278 closed, every join: branch open 12.4 mm chain, gap 2.81e-3 at a cluster of box 4.002 tol; main closed (main's loop 1.27e-3 inside d)"
OUTS=1 ./x2 advone 0x50300046 0 0 2>/dev/null | grep -E "^(OK|FLAG)" | cut -c1-330
echo "== R4 v9all 0x3300004b Sharp Right d=0.143: 1.19e-3 of span 32's offset missing (branch 1.8e-3 away, main covers), output line along sub-tol span 29's direction; not at a cluster under any exemption variant"
XK_NETTURN=1 XK_ARCSEED=1 bash -c "echo 'v9 0x3300004b 1 1' | ./x2 re" | grep RV4 | cut -c1-260
echo "== R5 Sharp/Square spikes beyond d outside every corner region, main none: v9 0x312005e4 Sharp Left (11.6 mm), 0x316001a6 Square Left (44.2 mm at d=168), 0x31700456 Sharp Left (0.711)"
./x2 mainone 0x312005e4 1 1 0 | cut -c1-200; ./x2 mainone 0x316001a6 1 2 0 | cut -c1-200; ./x2 mainone 0x31700456 1 1 0 | cut -c1-200
echo "== R6 g2 0x3500080e Sharp Left d=0.444: branch empty, main (and branch Round/Square) a 0.46 mm inner loop; loop and interior inside mitre regions of sub-tol jog corners (decision (a))"
./loopchk g2 0x3500080e 1 0; ./loopchk g2 0x3500080e 0 0 | head -1
echo "== R7 adversarial 0x504000d5 Round Right d=72.69: collapsed cluster (box 3.9 tol) at a +93.7 deg turn, 28.6 mm of the corner arc missing (branch 14.3 mm away), output in three chains; main continuous (survives the variants)"
XK_NETTURN=1 XK_ARCSEED=1 ./x2 advone 0x504000d5 0 1 2>/dev/null | grep -E "^(OK|FLAG)" | cut -c1-300
echo "== R8 adversarial 0x50300183 Round Left d=0.0394 closed: net-turn loop cluster (+377 deg, box ~4.1 tol > the 2 tol net-turn limit): branch open, gap 0.0233 mm; main closed (main ~3e-3 inside d there)"
OUTS=1 ./x2 advone 0x50300183 0 0 2>/dev/null | grep -E "^(OK|FLAG)" | cut -c1-300
echo "== E1 exemption question (net-turn cluster): v9 0x31600a3d Round Left: reference checker REG 7.7e-3 (branch 8.7e-3 away); with XK_NETTURN=1 it is exempt"
echo 'v9 0x31600a3d 0 0' | ./x2 re | grep RV4 | cut -c1-200; echo 'v9 0x31600a3d 0 0' | XK_NETTURN=1 ./x2 re | grep RV4 | cut -c1-200
echo "== E2 (c)-side misses main covers, in strips < 1.1e-4 mm wide: v9 0x312009ad Square Left"
./x2 thick v9 0x312009ad 0.39308520029760291 -0.26651765297411323
