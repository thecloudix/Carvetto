T-0158 round-5 verifier harness (branch task/T-0158-tip-start 1156cad; origin/main 34db4d0; round 4 = 999a90e). Extends t0158v4
(st/gen/gen2/gen3/probe3/probe4/chk4/one4/wsf flood). GCC 15.2; cross-compiler stand-in = core + Clipper2 built with
-march=x86-64-v3 -ffp-contract=fast (br5fma / logfma / mainfma).
Libraries (/home/chris/wt-T0158/build-verify5-rel, mkall5.sh / mklog5.sh there): br5 (worktree), br5fma, main (origin/main export),
mainfma, r4 (999a90e export), log / logfma (1156cad export + tfs_instr5.py; TFS_OFF=1 = main, hash-checked == br5 / main).
mk.sh builds a harness program per variant (VARIANTS=...).
- tfs_instr5.py: instruments the round-5 tieFreeStart (decisions untouched); st T line K4 columns reused: kind (0 projection,
  1 arc-centre rule, 2 own-edge corner, 3 fallback round-join first point, 4 fallback equidistant junction / most even feet),
  back = number of corner places with a nearest span, mOnC / mEqui / mLop = margins to the on-circle band, the labelling band
  and the travel sign test, rj = met. margins5.py summarises them per sweep directory.
- probe5.cpp: split (exactly collinear vertex s before/after the tip: sliver of the tip's own edge offset, labelling and
  on-circle bands, OFF=1000/3000), short (neighbour B at L, turn tB incl. reflex = consumed offset), tiny (degenerate / sub-tol
  edge of length e and direction phi at the start vertex; LIM = mitre limit, 1 or 2 = squared off), bez (Bezier edges at the
  tip), scan (jump finder: arc in-edge radius R scanned, discontinuities bisected to 1 ulp, MRG=1 merged region).
- mvnear.cpp: the cut starting nearest the first vector's start vertex, its start and neighbouring points (S/UP/DN/OX env).
- rcheck.sh: round-2/3/4 repros under 18 transforms per variant + chk4; mitre_decode.py: decodes fpoly starts off the mitre
  (turn, mitre ratio vs the scene's mitre limit, chamfer corners).
- q5.sh (stored VARS plain/FMA x branch/main, gen3/gen12 pert, generated plain/FMA, stored pert, random), q5b.sh + floodcmp5.sh
  (whole-scene flood br / main / r4) + delta54.py (stored runs where round 5 differs from round 4 and main), mutants.sh (N0-N13).
- repro5/: sq_tiny_* (squared-off tip, mitre limit 2, tA 2.5 rad, V-bit 90 d 3: duplicate start vertex (e0) or a 1e-9 mm closing
  edge -> fallback, symmetric chamfer tie, start flips 4.33 mm between the chamfer's corners under 1 ulp / 1e-12 scaling; r4
  stable), mrgarc_* (merged region, arc in-edge at the tip, R bisected onto the arc-chord labelling band: start moves one chord,
  0.36 mm (end mill) / 1.67 mm (V-bit) under 1 ulp), selfx_spike_* (self-intersecting spike: start flips 2.05 mm under a (3,-7)
  translation between the corner path and the fallback; main stable).
- results/: NOTES.txt and summaries.
