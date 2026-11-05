T-0142 round-2 verifier harness (branch task/T-0142-boolean-cost @ 15848d4, base origin/main 6e537a1, core == 53ae278).
Build: builds.sh (Release -march=x86-64-v3 build-verify2-rel, Debug -Werror build-verify2). Main Release reused from
round 1 (wt-T0142/build-verify1-mainrel, core source identical to origin/main).
mk.sh src out {br|main}: link a driver against branch or main Release; mk_r1.sh: against the round-1 (35b8ca9) build.
Drivers (new):
  po2.cpp   B2: outset +0.05 of n holed squares, interleaved n/2n CPU, ring count + byte hash. Layouts row/grid/rand
            (separated), tcol (vertical column, boxes touching), tgrid (touching grid), hex (hex packing: boxes overlap
            across rows), comb (column in a comb ring's notches), plate, deep. runpo.sh {br|main} n reps layouts...
  nd.cpp    B2: nesting decision round-1 global scan vs round-2 per-batch scan (re-implemented from both commits) on
            adversarial integer scenes; also counts hole/with-ring pairs with containing boxes in different batches.
  dfl2.cpp  B1: t0142v1/dfl.cpp + Drill (2000 holes + Bezier, timed) + plain V-Carve at 1e-9, 5e-7,
            nextafter(1e-6,0), 1e-6, 2e-6, 1e-5, 1e-4; main vs branch.
  dfl3.cpp  B1: distance at the floor with chordal at min/1e-3/1 for every op (nested refusals would show).
  pkh.cpp   Pocket Offset (stepover > radius) on hex-packed holed squares, main vs branch.
  runa.sh / runb.sh: re-runs of round-1 bat/strip/pb and hn/lo on the round-2 build.
results/notes.txt: numbers and verdict.
