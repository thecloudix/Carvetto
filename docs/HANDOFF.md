# Director hand-off — 2026-11-01

The director moved from a 2-core cloud session to Chris's local WSL machine (i9-9900KS, 16 threads, ~56 GB) so verification can run in parallel. This file is the state at hand-off. Read it once, then work from `docs/BACKLOG.md`, `docs/WORKFLOW.md` and `.claude/agents/*.md` as usual.

## How to run as director

Act as `.claude/agents/director.md`. Coders and verifiers are spawned with the Agent tool using the instructions in `.claude/agents/coder.md` / `verifier.md` (pass the file's content or tell the agent to read it first). Key rules recap:

- Fresh-context verifiers; never forward the coder's summary to a verifier. Up to 3 fix rounds per card, then the owner decides (re-scope / merge with follow-up / extra round).
- Merge only via PR with green CI on all three jobs (linux-core, linux-app, windows-app MSVC); squash-merge; one CHANGELOG line per task; clean rebuild after rebase; never `git add -A` blindly.
- Commits: author `Chris Wood <thecloudix@gmail.com>`, message ends with a `Co-Authored-By: Claude …` line. PR bodies end with "🤖 Generated with Claude Code".
- Agents kill only their own PIDs (no broad pkill/killall).
- MSVC: include every std header used; no `fopen`/`fscanf` (C4996 is an error) — use `std::ifstream`; no constexpr `std::fabs`; CPU-time tests use GetProcessTimes on Windows.
- Owner preferences: ask at real decision points (AskUserQuestion with a recommended option); report the overall % and Phase 1 % whenever a card merges; do not send updates when nothing changed; on "pause", let agents finish their current step, merge only what passed, commit WIP on branches, record status here and in the backlog.
- Parallelism: this machine can run ~3–4 agents with heavy sweeps at once (use `ctest -j8`, sweeps with 4–6 processes each). Avoid two tasks touching the same interface.

## Progress

About **19% overall** of Aspire parity, **Phase 1 ~26%**. Merging T-0124 → ~20% / ~30%. T-0017 and T-0126 are quality cards (< 1% each). T-0122 (CLI) is the next feature card after T-0124 merges.

## Harnesses

Verifier/coder harness sources, oracles, repros and logs are on the orphan branch **`verifier-harness`** (`git fetch origin verifier-harness && git worktree add ../harness verifier-harness`). Its README maps each folder to its old cloud path; scripts may hard-code `/home/claude/...` paths — adjust when reusing. Builds are not included.

## Cards in flight

### T-0124 Profile tabs (branch `task/T-0120-profile`, last verified c2f4709)
- Round 3 verifier: FAIL only for profiles **On** the vector (short tab on a sharp turn kept but severed). Inside/Outside, tab-free toolpaths, performance, B2 (V-bit slope) all PASS. Report: `verifier-harness:t0124v4/` (conn6.cpp, flood.cpp, repro/on_corner.cpp, repro/sev_*.scn, F9 = seed 41006 fam 5 scene 9).
- Owner decision: merge with a conservative On guard; exact fix is card T-0127.
- At hand-off the cloud Profile coder was implementing the guard (fix On classification at profile.cpp:968, keep On tabs only if plateau ends ≥ 2r + 2·allowance + max(0.25, 0.05·D) and connectivity holds, regression tests, ADR-0016 §14 correction, Inside/Outside byte-identical). **Check the branch for a commit after c2f4709.** If none appears (or it is incomplete), dispatch a local coder with exactly that brief.
- Then: a fresh verifier checks only the guard (no On tab severed across the 11 sweep configs of t0124v4, Inside/Outside unchanged vs c2f4709, full test bar, ADR text). On PASS: accept ADR-0016, squash T-0120/T-0124 into one commit, rebase on main (expect a CHANGELOG conflict — keep both lines, newest first), clean build + serial ctest, push, PR, 3 green CI checks, squash-merge; mark T-0120/T-0124 done, unblock T-0127 and T-0122.

### T-0017 Offset quality II (branch `task/T-0017-offset-quality-ii` @ ec0856e, round 3 of 3)
- Coder claims B-1 (Round cusp gaps → checked straight bridge) and B-2 (no snapped extension along another chain's carrier) fixed, sub-tolerance end spans dropped, docs corrected, 503/503 tests.
- The round-3 verifier was killed by a cloud restart before reporting. **Dispatch a fresh verifier.** Pass rule (owner 2026-10-31): **no worse than main** — findings identical to main are non-blocking and go to a follow-up card. Harness: `verifier-harness:t0017v2/` (vdrv4.cpp, gen5.inc, …) and `t17r3/`.
- On PASS: rebase on main, merge as above; then start T-0018, T-0019, T-0009, T-0016.

### T-0126 Pocket guard hardening (branch `task/T-0126-pocket-guard` @ f85d8f7, pushed)
- Coder's fix is complete: edge checks after pull-in, depth-based phantom test, "trimmed" report, Clipper sliver regression. Release ctest 496/496 and its scans passed on f85d8f7.
- Still to run on f85d8f7: Debug serial ctest, `ctest -j8 --repeat until-fail:3`, and the `v3src sweep3 8800100 50` sweep (harness `verifier-harness:t0126/`). Then a fresh verifier per the card.

## Queue after these

T-0122 CLI (after T-0124), T-0127, T-0018, T-0019, T-0009, T-0016 (after T-0017), T-0123 (API alignment), then card Batch B (drawing & editing) and Batch C (remaining toolpaths, simulation, output) — ask the owner before starting a new phase.

## Owner-facing status doc

The claude.ai Project doc `claude/carvetto-status.md` is only reachable from the cloud session; from now on keep status in this file and `docs/BACKLOG.md`.

## Update after hand-off — T-0124 On guard pushed (736f955)

The cloud Profile coder finished the On guard on `task/T-0120-profile` @ **736f955** (4 files, +289/−10; ~30 lines in profile.cpp): On decided by profile side (not `Cut::distance`) in tabSections/tabHolds/quick check; On tabs kept only if end tool positions stay ≥ 2r + 2|allowance| + max(0.25, 0.05·D) apart on every pass and connectivity holds, else dropped as "cut through"; on_corner/sev_* /F9 regression tests (fail on c2f4709); ADR-0016 §14 updated (counts are "per primary cut"). Test bar: Debug -Werror, serial 507/507, -j2 until-fail:3 507/507. Byte-identity vs c2f4709: tabs off 960/960, Inside/Outside with tabs 724/724, On differs 56/236 (intended).

Open point for the verifier: conn6 sweep (11 configs × 150, CELL 0.05) flagged 20 tabs; re-run at CELL 0.03 leaves 4 scenes (5 tabs) only with conn6's seam extension, 0 with NOSEAM=1 — A 82652 sc2, F 82701 sc6 (On), F 82701 sc49, G 82712 sc0. Coder believes these are seam artefacts in conn6. The fresh verifier must decide each with an independent raster (flood.cpp) — a genuinely severed kept tab is blocking. Coder logs were in /home/claude/cscratch-T0124/sw4/ on the cloud machine (not on the harness branch; scenes are reproducible from seeds).
