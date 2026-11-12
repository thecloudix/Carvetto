# In-flight handoff (director wind-down, 2026-11-10) — local only, not pushed

State of `main`: green at the last full run (see git log). All task statuses are in docs/BACKLOG.md (pushed). Re-verify anything below that says "verifier output lost": verifiers are stateless and their work is redone.

## Open PRs (all squash-merge when CI is green; rerun failed jobs for known timing flakes)
| PR | Task | Branch | State |
|----|------|--------|-------|
| #120 | T-0239 Fluting | task/T-0239-fluting | verifier PASS; CI 3/4 green, linux-core pending |
| #122 | T-0242 Tiling | task/T-0242-tiling | verifier PASS; first CI failed (GCC 13 -Wmaybe-uninitialized in tiling.cpp, fixed in c7a6cb19 and pushed); CI re-running |
| #123 | T-0238 Straight inlay | task/T-0238-inlay | verifier PASS (round 2); CI 3/4 green, one pending |
| #124 | T-0226 Flaky-test hardening | task/T-0226-flaky-ratio | verifier PASS; CI failed linux-app ubuntu-26.04 on timing test "T-0142 round 3 ... near-linear" (another flake; see card T-0270): rerun the failed job, merge when green |

ADR numbers used so far on main: up to 0036 (keep-out), 0037 thread, 0038 trace, 0039 double-sided, 0040 inlay (PR #123), 0041 fluting (PR #120), 0042 prism (merged), 0043 tiling (PR #122), 0046 keepout UI. Collisions to resolve at merge: T-0260 and T-0255 WIP both use ADR-0047; T-0261 uses 0048; T-0257/T-0254 may add none. Renumber by merging main into the branch (no force-push; the classifier denies force pushes) and `git mv`.

## Coders (all stopped; WIP committed locally by the director, nothing pushed, nothing verified)
| Card | Branch | Worktree | State |
|------|--------|----------|-------|
| T-0260 Trace bitmap UI (+ core work cap/cancel) | task/T-0260-trace-ui | .claude/worktrees/T-0260-trace-ui | WIP commit 3ccec5d3; unknown build/test state; ADR-0047 to renumber |
| T-0255 Dimensions UI wiring | task/T-0255-dimensions-ui | .claude/worktrees/T-0255-dimensions-ui | WIP commit cbc16371; unknown build/test state; ADR-0047 (reflex angular sign) to renumber |
| T-0183 Refused-contour diagnostics | task/T-0183-refused-diagnostics | .claude/worktrees/T-0183-refused-diagnostics | WIP commit e756fbed; unknown build/test state |
| T-0261 Double-sided UI/output | task/T-0261-double-sided-ui | .claude/worktrees/T-0261-double-sided-ui | WIP commit 393beaf2; ADR-0048; unknown build/test state; may need .carv minor 1.6 decision |

Restart each by sending a coder to the worktree with "finish the card, build, test, report READY".

## Coders that reported READY, verifier running when stopped (verifier output lost; re-dispatch verifiers with the same scope)
| Card | Branch | Head | Scope notes |
|------|--------|------|-------------|
| T-0254 Vector texture dialog | task/T-0254-texture-dialog | 39b1eb63 | also check the "undoStack().count() fell by 3" oddity noted by the coder |
| T-0256 Keep-out zones UI | task/T-0256-keepout-ui | 92b71682 | ADR-0046; preview chains all toolpaths from home vs Save chains per file; visibility not an undo step/dirty flag |
| T-0257 Document variables UI | task/T-0257-variables-ui | 7ce91793 | expressions session-only (T-0271 follow-up) |
| T-0157 Profile precision | task/T-0157-profile-precision | ce424eb0 | re-scoped to documented 1e5 mm limit + pinning test + signedArea sin/cos reuse (claimed bit-identical) |
All four branches are pushed to origin. Merge conflicts expected in app/src shared files (main_window.*, app/CMakeLists.txt, self_test.cpp) when several UI branches land: merge main into each branch, never force-push.

## Pending owner decisions
1. T-0250 Photo V-carve: Phase 1 or Phase 2 (heightmap work)? Card is blocked until answered.
2. T-0240 Prism geometry (merged as PR #121, ADR-0042): inverted-pyramid recess (built) vs raised prism (surface at centre, facets fall toward the vector). Flipping needs the cut order inverted too.
3. Product readings flagged in ADRs for confirmation: inlay (ADR-0040: allowance from male only, stepped shoulder at top, no pocket corner option), double-sided "Zero off same side" (ADR-0039), keep-out rapid mapping and 2D-only routing (ADR-0036), thread milling presets/depth semantics (ADR-0037), chamfer width = full V footprint (ADR-0035).

## Housekeeping notes
- Stale worktrees under .claude/worktrees with uncommitted work from early attempts (agent-aa79d4947425df7de, agent-ad6138c6a98b4b1f5) were left alone (force removal was denied). Many old wt-* and /home/chris/harness worktrees exist outside the repo.
- /tmp is a 28 GB tmpfs that filled up from verifier checkouts; verifiers should build under /home with TMPDIR on /home.
- Do not use sleep/poll loops; use one bounded wait (timeout N bash -c 'until ...') per check.
- Ready cards not yet started (see BACKLOG.md): T-0166, T-0174, T-0177's remainder T-0252, T-0230, T-0252, T-0258, T-0259, T-0262..T-0271, T-0183 (in WIP), T-0265 (needs T-0238 merged), T-0225/T-0231/T-0227 nits, plus the Phase 1 gaps still unwritten as cards: none (all sixteen from the owner list are carded).
