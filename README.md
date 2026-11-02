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
