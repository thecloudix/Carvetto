T-0160 verifier round 1 (branch task/T-0160-corner-sharpen, commit 3fcccae; main reference origin/main f7151a3 = merge base
429c26b + backlog only). Libraries: Release plain and -march=x86-64-v3 in /home/chris/wt-T0160/build-verify1-rel{,3} and
/home/chris/wt-T0160v1-main/build-verify1-rel{,3} (buildall.sh / buildtree.sh); Debug -Werror build + ctest in
/home/chris/wt-T0160/build-verify1 (results/debug_build.log, results/ctest.log). Binaries in /home/chris/scratch-t0160v1/bin
(build.sh <tree> <build dir> <src> <out> [-DHAS_CS]; not committed).
  lo.hpp      verifier's own leftover oracle (independent of the branch's v_carve_leftover.hpp): carved surface per move
              (level: closed form; inclined V/ball: golden-section of a convex function; inclined end mill: quadratic
              interval), uniform-grid index; exact segment clearance vs the ORIGINAL curves via acc.hpp (t0153v3).
  lc.cpp      per-case corner checks, sharpening on vs off: insertion-only (on == off + complete cuts at the floor,
              clearance + diagnostics identical), exact min clearance of every inserted feed (links included) vs rho + c/2,
              leftover on the floor (dist >= rho + c) in each convex corner's tent disc (grid + pattern-search refine)
              vs the ridge h(s/2). ~150 named cases (squares/rects/triangles/L/L-hole/stars/n-gons/islands and holes near
              corners/necks/curved D, concave-arc and Bezier corners/rounded (arc and polyline) and chamfered corners/
              hexagon threshold +-1e-5/far offsets/small chordal acute corners/30-degree wedge with a hole in the tent).
              LC_EXACT=1 lc <name> runs one case; LC_PT="x y" prints which moves bound the leftover at a point.
  vc3.cpp     t0164v2 vc3 + VC_FM=2 (force flat depth + clearance tools), VC_CS=0/1, VC_INS=1 (on == off + insertions,
              VC_INSV=1 prints the first difference, multiset check and the input); conesweep.sh (cone/end-mill oracle,
              families 0-23 with clearance tools), hashsweep.sh + cmp.py (main vs branch off/on, plain vs v3),
              advhash.sh (t0153v4 adv/adv2 x 5 configs incl. clearance tools).
  perf.cpp    t0164v2 perf + VPERF_CS; perfrun.sh AC2 interleaved main/branch and the sharpening cost with end mills.
  mutants.sh  test-quality mutants on a git-archive copy of the branch (corner + post tests only).
  repro/      order_run.cpp + order_fam7_607_22.txt (later regions' cuts reordered), far.cpp (cone crossing at far offsets).
  results/    NOTES.txt and raw outputs.
