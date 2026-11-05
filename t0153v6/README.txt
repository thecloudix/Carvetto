T-0153 verifier round 6 (commit bdc932c, N1 fix: SkeletonCheck checks every skeleton point; membership carried
along the skeleton; own rings not measured). Build drivers: ./build.sh <tree> <release build dir> <src.cpp> <out>.
Sources: bdc932c Release lib in /home/chris/wt-T0153/build-verify6-rel; sabotaged = bdc932c + sabotage.patch (r4).
  net.cpp        SkeletonCheck(all contours).consistent(medialAxis(subset), own) like v_carve (own = the subset);
                 OWNLIST=01 overrides own, OWN=0 -> own={} (144e957 semantics); prints STRUCT if a branch end is not
                 its node point / not listed in its node / node index out of range (MA contract the net relies on).
  fuzz6.cpp      property fuzz with SYNTHETIC skeletons (random graphs, steps straddling r-(2c+d), loops, isolated
                 nodes; radii = exact distance to the own contours): BYPASS = accepted but outside/disc past a wall.
                 BADEND=1 breaks the branch-end==node contract; DROP=1 claims non-own contours as own (oracle sanity).
  autoattack6.py full island / thin slivers / gaps of k x point spacing / rotated, end to end under SAB 0/2/3/4;
                 FLAG = carved outside beyond 2c+d.
  repro6/        sq_hole_diag.txt (silent hole drop passes net with own={0,1}), fullisl_* and full_c0.05_g0.0103_d0.005
                 (2c+d disc slack, identical on 144e957), fuzzgeo/ fuzz geometries.
  scripts/       background runners (debug build+ctest, hash sweeps, perf, golden regen, SAB matrix, fuzz, attack).
  results/       NOTES.txt, repros_sab.txt, net_repros.txt, hash_bdc932c.txt, advhash_bdc932c.txt, sabadv.txt,
                 perf.txt, fuzz6.txt, autoattack6.txt, fullisland.txt, struct.txt.
