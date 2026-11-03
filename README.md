# Verifier and coder harnesses (scratch, not part of the product)

Source-only snapshot of the test harnesses, oracles, repros and logs used by
verifiers and coders, taken 2026-11-01 when the director moved from the cloud
session to a local WSL machine. Builds are not included; rebuild against the
relevant worktree.

Original locations (scripts may hard-code these; adjust paths when reusing):
- t0124v2/  -> /home/claude/vscratch/t0124v2  (Profile round-2 verifier: conn.cpp oracle, repro/)
- t0124v4/  -> /home/claude/vscratch/t0124v4  (Profile round-3 verifier: tools/conn6.cpp, tools/flood.cpp, repro/on_corner.cpp, repro/sev_*.scn, cmp/)
- cscratch-T0124/ -> /home/claude/cscratch-T0124 (Profile coder sweep logs)
- t0017v2/  -> /tmp/.../scratchpad/t0017v2 (T-0017 round-2 verifier: vdrv4.cpp, gen5.inc, cs.inc, ac.inc, cls.py, repro_*.cpp)
- t0017v3/  -> /home/claude/vscratch/t0017v3 (T-0017 round-3 verifier, partial; killed by restart)
- t17r3/    -> /home/claude/t17r3 (T-0017 coder round 3)
- t0126/    -> /home/claude/vscratch/t0126 (Pocket guard coder: v3src, jobs, final2/ logs)
- t0124v5/: T-0124 On-guard verifier (2026-11-01, local WSL; PASS on 736f955). Paths were /tmp/claude-1000/.../scratchpad/h. Includes negallow/neg*.scn repros for the negative-allowance tab defect and wflood.py (windowed flood oracle).
- t0123v1/: T-0123 verifier (2026-11-01, local; FAIL on 3f3af16 only for rest-travel owner decision). pockcheck.cpp = pocket safety oracle (gouge/inside/one-level/coverage/retracts), profmulti.cpp, bi.cpp, perf.cpp, restretract.cpp; build with mk.sh (EXTRA=-DNEWAPI). Paths were /home/chris/wt-T0123/build-verify/drv.
- t0127c/: T-0127 coder harness (2026-11-01, local): conn7 (On/NEGA modes), nontab3 hashing, mkscn, floodd, perf2; results/ and scn/ repros. See t0127c/README.
- t0127v1/: T-0127 round-1 verifier (FAIL on 9c3680c: kept tab holds nothing, part free). wsf4.cpp/wsf6.cpp = whole-scene flood oracle (exact capsules); wsh.cpp+cmph.py hashing; wsc.cpp counts; repro scenes in dumpnew/ d4new/ cnt/. Paths were /home/chris/wt-T0127/build-verify/drv.
- t17r4/: T-0017 verifier of 0cd9abc (pert.cpp ulp-perturbation driver, repro_arcloop.cpp).
- t17r5/: T-0017 verifier of 328008b (pert2.cpp large-radius/tiny-arc loops, retry_case.inc, gap_case.inc, mutants offset.mA/B/C.cpp).
- t0123v2/: T-0123 delta verifier (PASS 524c7ff): pc2.cpp (pockcheck with rest==clusters), rr2.cpp, bi2/bi3.cpp (BoxIndex edge cases).
- t0122v1/: T-0122 CLI verifier (PASS 2d7e3fa): driver.cpp (CLI == in-process toolpaths, angle checks), driver2.cpp (storage identity, bad member names), gcheck.py (G-code safety checker), mkmal.py (24 malicious .carv variants), gen.py (DXF generator).
- t0133v1/: T-0133 post-safety verifier (FAIL aec8e83): drv.cpp text-injection sweep, num.cpp number sweep (diff branch vs main), arc.cpp (B1), rt.cpp (Grbl realtime bytes / LinuxCNC LOGOPEN), mk2.py (rename toolpaths in .carv).
- t0127c/round2/: T-0127 coder round-2 harness (free-piece checker, aggregator).
- t0133v2/: T-0133 round-2 verifier (FAIL c3b1be6, B4 adjacent text values): adv.cpp fuzzer with LinuxCNC/Grbl model, gap.cpp repros, mk3.py (sets tool snapshot name), e2e/custom.pp.
- t0133v3/: T-0133 round-3 verifier (PASS 88904b5): adv3.cpp AC5 fuzzer (REAL/GAP/authored), mut.cpp, edge.cpp, ff.cpp (\f/\v/\r template gap), mk.sh.
- t0127v2/: T-0127 round-2 verifier (FAIL bddd93f): wsf7.cpp/wsf7w.cpp oracle (loose-piece cross-check, ANYRUN, NOFLOOD), wsh2.cpp+cmph2.py, hole.cpp V-bit hole repro, inst/ mutants, repro scenes d1/ d/wsf7/.
- t0132v1/: T-0132 verifier (FAIL 0305fe4): adv.cpp small-file worst cases, combo.cpp, fuzz.cpp zip-level lies, many.cpp (B1 member-count repro), ac1.cpp, rt.cpp, peak.cpp (forked peak RSS), mkbig.cpp, dump.cpp.
- t0127c/round3/: T-0127 coder round-3 harness/results.
- t0134v1/: T-0134 verifier (FAIL d7c2232, quadratic params nesting memory): pa.cpp parser control chars, adv3b.cpp, tu.cpp TOOLS_USED fuzzer, mutrun.py, mkh.py (crafted tool/feed/dwell). Deep-nesting repro: python one-liner in the report (60k nested arrays).
- t0132v2/: T-0132 round-2 verifier (FAIL a01c130): consist.cpp save/load consistency probes (modes count/nomoves/interimextra/strings/dupsel/movebudget/cap/zero), peak2.cpp, fuzz2.cpp (load->save->load), big2.cpp, mig.cpp, combo.cpp, build.sh.
- t0127v3/: T-0127 round-3 verifier (FAIL 5ff8a09 on AC3 loose-piece under-reporting): wsf8.cpp/wsf8v.cpp flood oracle (CNT, TABS), inst/profile_mut.cpp (MUT_KEEP, FIX_BOX, FIX_MSG), mkh.cpp/mkabc.cpp generators, repro/ scenes.
- t0132v3/: T-0132 round-3 verifier (PASS 3817e44): adv3.cpp invariant cases, big3.cpp save/load cost, peak3.cpp, fuzz3.cpp, set3.cpp, rep3.cpp, nan3.cpp, consist.cpp, build.sh.
- t0134v3/: T-0134 round-3 verifier (PASS acebb3b): pp/ incl. new inverse-time posts (c_inv_inch_mul, c_inv_fi_everywhere, c_inv_neg, c_fi_normal_inch), zl.cpp inverse probe, time.sh params timing.
- t0009c/: T-0009 coder harness (feffe02): t9.cpp sub-tol fuzz driver, corrected vdrv4 checker copy (see ADR-0012 A2 for the four checker fixes).
- t0129v1/: T-0129 verifier (PASS 968fbc2): ac.cpp allowance-limit matrix, wsh.cpp (mode 4: allowances around -r), cmp.py, mk.sh.
- t0018v1/: T-0018 verifier (PASS 9fca907): inr.cpp inradius/coverage/orientation oracle, cmpdump.cpp, dust.cpp (grid dust rings), clipb.cpp, edge.cpp, ann.cpp, hd.py/hdp.py path deviation.
- t0135c/: T-0135 coder harness (7d00ce6): agg.py, sweep aggregates, scenes.
- t0136v1/: T-0136 verifier (PASS d701656): adv4.cpp limit/name/NaN probes, adv5.cpp names that grow when stored, mkbase.cpp fuzz bases; big3/peak3 patched with a valid tool.
- t0135v1/: T-0135 round-1 verifier (FAIL 7d00ce6): wsf8x.cpp + chk.py (area matching), wsf8p.cpp probe maps, adv.cpp adversarial generator, wsh3.cpp (FORCEIOPOS), gfit.cpp fitTabs differential, perf3.cpp open-cut cluster pages, scn/ repros.
- t0138v1/: T-0138 verifier (PASS fc80d44): dor.cpp near-collapse orientation oracle (large offsets), cmpd.py, ac2.cpp threshold cases, pk.cpp/pk2.cpp Pocket/Profile near collapse, cost.cpp/cost2.cpp, ab.sh.
- t0009v1/: T-0009 round-1 verifier (FAIL feffe02): v9.cpp independent sub-tol injector (modes st/one/poly/shrinkpoly/time), vchk.cpp checker, repro_wrap.cpp (Sharp full-circle wrap B1), big.cpp/zz.cpp/zzc.cpp cost probes (cubic), edge.cpp, logs/shrink_*.
- t0135v2/: T-0135 round-2 verifier (FAIL 42d38af): lnk.cpp shortcut repro, dl.cpp/dl2.cpp in-process mutant screens (NOLINK/NOADD/OLDORDER), mkmut.sh + mutant sources, wsf8y.cpp, fcand*.sh, fcmp.py, hcmp.py, gfit2.cpp fitTabs loss repro, cap.cpp, perf.sh, scn/.
- t0134v2/: T-0134 round-2 verifier (FAIL 05d3eac): adv.cpp + cmp.py AC3 differential, one.cpp, pp/ custom posts (c_inverse, c_inv_fi, ...), pj/ params stress inputs.
- t0150v1/: T-0150 drill verifier (PASS 2cb366c): centre.cpp centroid oracle, gen.cpp move checks + G-code for all posts, gcheck_drill.py (gcheck extended with the chip-breaking rule), ring.cpp/dbg.cpp (ring-with-bridge-on-+X repro), nonfin.cpp, nan.cpp, s0.cpp.
- t0141v1/: T-0141 verifier (PASS 6f77637): cmpb.cpp batched vs single call, hashd/hashn byte identity, grid31.cpp, degen.cpp, dustbf.cpp, perf.cpp (row/jit/col/grid/spiral/holes/plate/touch), pcw.cpp wide-pocket oracle + mvcmp.py, proftol*.cpp, bez.cpp (tiny-distance blow-up), profwide.cpp.
- t0151v1/: T-0151 quick engrave verifier (PASS bfbf877): eg.cpp Fill pocket oracle (-DNEWAPI), ol.cpp Outline oracle, gen.cpp + gcheck2.py (extended G-code checker, mut/ mutation inputs), edge.cpp, lv.cpp.
