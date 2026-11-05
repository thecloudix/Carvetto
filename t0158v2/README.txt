T-0158 round-2 verifier harness (branch 5e7ce1d; origin/main 41649cb; merged tree 306e4c3 = branch merged onto 41649cb).
Libraries (mkall.sh -> /home/chris/wt-T0158/build-verify2-rel): br/brv3 (branch, plain / -march=x86-64-v3 = FMA contraction),
main/mainv3 (41649cb, core built with -ffp-contract=off since T-0153), log/logv3 (branch + tfs_instr.py: tieFreeStart logs one
TFS2 line per call with the rule that fired and the margin of every threshold; TFS_OFF=1 = plain projection = main), mlog/mlogv3
(merged tree + tfs_instr.py). mk.sh builds harness programs against each lib with -ffp-contract=off so scenes are bit-identical.
- st.cpp: per-call cut-start logger (T lines: tag call# rule nearCount runs du dpt dRule x y mArcC mArcR mArcS mCut mNear mGap zda cutd);
  modes scn (VARS=1: orig/vsout/bbc/opt), pert (21 transforms), random (seed N scale offset).
- gen.cpp (round-1 families) and gen2.cpp (sharp-corner families: mt mtc mref min mon mloop mtri mshort mflat marc mmrg rmrg mopen).
- sweep.sh (scn/pert over a list, configs log logoff logv3 logv3off mlogv3), rndsweep.sh, cmp2.py (change rate, explained-by-start,
  plain vs v3, merged vs branch, knife-edge margins), bdiff.py, pcmp2.py / prule0.py (perturbation), pmax.sh (max start move of the
  first call under scale perturbations, branch vs main), edge.cpp (chamfer travel direction, short neighbour edge at the tip, gap at
  the start vertex), dbg.cpp (one scene, raw TFS2 + first moves), wsf.cpp + floodcmp.sh + floodagg.py (whole-scene flood), worse.py,
  rnddump.py, queue.sh/queue2.sh (chained sweeps).
- repro/: mmrg_014/022/023 (V-bit sharp, two overlapping stars -> merged-region cut at d+chordal: mitre-tip start flips 2-3.6 mm under
  1+-1e-9 scale, branch == main), gapmitre_1e-4 (single star with a 1e-4 closing gap -> merged path; flips under ulp scale and
  differs GCC plain vs x86-64-v3 on the branch), shortpost_L0.01_turn2 (tip followed by a 0.01 mm edge turning 2 deg: Climb starts at
  the neighbour's shallow mitre, 4.2 mm from the tip's mitre; Conventional at the tip's mitre). repro_v1/: round-1 repros.
- results/: see NOTES.txt.
