T-0142 round-3 verifier harness (branch task/T-0142-boolean-cost @ fb08fc9, base origin/main 2df9aba, core == 53ae278).
builds.sh: Release -march=x86-64-v3 build-verify3-rel, Debug -Werror build-verify3. Main Release reused from round 1
(wt-T0142/build-verify1-mainrel; build-verify1-mainsrc/core verified identical to 53ae278). mutbuilds.sh: mutants A (offset.cpp
back to round 2), B (edge index off), C (box hierarchy -> linear scan). mk.sh src out {br|main|mutA|mutB|mutC} (-ffp-contract=off as core).
Drivers:
  pip3.cpp  RingLocator<Path64> (indexed after the 2nd query) vs Clipper2 PointInPolygon: random/rectilinear/one-row/duplicate/
            combs x and y/spikes/all-but-one-on-row/back-tracking/wavy rings, 3..2000 vertices, coords up to MAX_COORD.
  nd3.cpp   everyHoleNested<Path64> vs plain all-pairs scan (whole scene and per hole): nd.cpp families + big combs with holes on
            teeth/vertices, shuffled order, concentric C rings, duplicates/zero-area, huge coords.
  po3.cpp   po2.cpp + tcolrev/tcolshuf/hexrev (holes far from their outers), cring (boxes hold, insides do not), ycomb (teeth along
            y), xcombrev; 5th arg 1 = time only everyHoleNested (attribution). runpo3.sh / runcmp.sh: br/main alternation.
  hn.cpp / lo.cpp (from t0142v1): nested-outset byte hashes br vs main; LOST/FAR/LEFT oracle. runb3.sh.
results/notes.txt: numbers and verdict.
