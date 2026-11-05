T-0158 round-3 verifier harness (branch task/T-0158-tip-start 5649b2b; origin/main 2df9aba; merge-tree clean). Extends t0158v2.
GCC 13 is not installed locally (conda install was not permitted): cross-compiler identity is GCC 15.2 plain x86-64 vs
-march=x86-64-v3 (FMA contraction), as in round 2.
Libraries (mkall.sh -> /home/chris/wt-T0158/build-verify3-rel): br/brv3 (branch), main/mainv3 (origin/main export), log/logv3
(branch + tfs_instr3.py: same decisions, TFS_OFF=1 = plain projection = main, TFS_LOG = one TFS2 line per call with every
threshold margin incl. mTurn = distance of a run junction's turn from sharpest - kTipTurnTie, pick, runLen, jmin = run junction
nearest the target, jPickMinusMin = how much farther the picked junction is). mk.sh builds harness programs per variant.
- st.cpp (round-3 T line: ... cutd mTurn pick runLen jmin jpm), gen.cpp/gen2.cpp (round 1/2 families), gen3.cpp (new: fpoly fine
  convex polylines, split = start tip split into two corners by a 1e-3..5e-2 mm edge, sharpnb = start vertex next to a sharper
  neighbour, rmrg2/vmrg2 merged regions end mill / V-bit), probe3.cpp (ngon: fine regular polygon start; tie: start vertex A
  with a near neighbour B, turn difference at kTipTurnTie (DELTA) or L sin(tB) = tol (TB, LFAC); join: where on a merged
  region's polygonal round join the start lands), edge.cpp (round 2: chamfer travel direction etc.).
- q3.sh: sweep chain (gen3 pert, gen1+gen2 pert, stored scn VARS=1 x {log,logoff,logv3,logv3off}, stored pert, random pages).
- pmove.py (absolute start moves under scale/frame perturbations per family), jpick.py (starts not at the run junction nearest
  the target), xcomp.py (plain vs v3 / branch vs main hash + start diffs), pcmp2.py/cmp2.py (round-2 analyses, paths updated),
  floodcmp.sh + floodagg.py + wsf.cpp (whole-scene flood oracle, branch vs main), mutants.sh (test quality: M0 full revert,
  M1 merged region at d without slack, M2 pick = first junction, M3 kTipTurnTie = 0, M4 slack 0).
- repro3/: knife-edge repros (all bit-identical plain vs v3, move under <= 1e-9 scaling or translation):
  tietA0.3_d0.scn (start vertex turn 0.3 rad, preceding neighbour 1.697 um away turning exactly 0.3 - 1e-3: start flips 0.907 mm
  between the two mitres under +-1 ulp scaling); sharpnb_tA0.1_tB0.2_L5um.scn (neighbour turning 0.2 rad at L = tol/sin(0.2):
  0.456 mm under 1e-10); equalnb_tA0.1_tB0.1_L10um.scn (equal turns, L = tol/sin(0.1) = 10 um: 0.310 mm under 1e-9);
  sharpnb_tA0.3_tB1.0_L1.19um.scn (2.09 mm, L just above tol).
- results/: NOTES.txt and sweep summaries.
