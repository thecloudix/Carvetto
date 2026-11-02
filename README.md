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
