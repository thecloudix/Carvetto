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
