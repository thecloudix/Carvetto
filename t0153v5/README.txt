T-0153 verifier round 5 (commit 144e957, N1 fix: detail::SkeletonCheck). Build any driver:
  ./build.sh <source tree> <release build dir> <src.cpp> <out>   (adds -I<tree>/core/src/toolpath for v_carve_check.hpp)
Sources: export 144e957 / 209499a with git archive; sabotaged copy = 144e957 + sabotage.patch (round-4 patch, applies with fuzz);
"saball" copy = sabotaged + allpoints.patch (SkeletonCheck checks every skeleton point instead of 8 + deepest + last).
  net.cpp        SkeletonCheck(all contours of a file).consistent(medialAxis(subset)) - direct fault injection like the
                 branch's test; prints sampled points (index, point, r, inside, distance) and how many points are bad.
  stadium.py     stadium frame (outer R+g, hole R) with islands = x-slices of the hole inset by 0.005.
  autoattack.py  places islands exactly over the points SkeletonCheck samples on the wrong skeleton (SAB=4: {outer},{hole};
                 SAB=2: {outer,hole} with the hole flipped). Defeats the 8-sample net (repro5/).
  vc3.cpp        round-3/4 oracle + VC_HASH=1 (FNV-1a of all moves + diagnostics per case), VC_HASHONLY=1 skips the oracle.
  hashsweep.sh   families 0..23 x 15 + 2 mixes x 20 = 400 cases, hash only (results/hash_*.txt: identical 209499a vs 144e957).
  runadv.sh      round-4 adversarial runner (adv/ from ../t0153v4).  perf.cpp  AC2 (round-3 driver).
  mut_disc_off.patch  MUT=1 disables the disc-distance half of pointOk (the new test then fails 1 assertion).
  repro5/        stad_isl_sab4_c0.05 (SAB=4, V90 D12 chordal 0.05: -0.73 mm), stad_isl_sab2_c0.05 (SAB=2: -0.47 mm),
                 stad_isl_sab4_c0.1 (chordal 0.1: -0.99 mm), stad_isl_sab2_c0.05_rot30 (-1.02 mm); all refused with allpoints;
                 stad_thin_R0.03 / stad_thin_R0.6_c1 (SAB=2 under the r <= 2c+d threshold: carved on the wrong side by design).
  results/       NOTES.txt (findings), perf*.txt, sabadv.txt (SAB 2/3/4 x adv/adv2/repro4), hash_*.txt, advhash_*.txt.
