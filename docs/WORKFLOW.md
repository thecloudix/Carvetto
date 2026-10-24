# Development workflow: director, coders, verifiers

Carvetto is built by a small hierarchy of agents coordinated by one human, the **project owner** (Chris). The point of the structure is to let many independent tasks run at the same time while keeping quality gates and keeping product decisions with the owner.

```
                 ┌──────────────────┐
   decisions ───►│  Project owner   │◄─── escalations, status
                 └────────┬─────────┘
                          │ priorities, answers
                 ┌────────▼─────────┐
                 │     Director     │  plans, decomposes, dispatches, merges
                 └──┬──────┬─────┬──┘
          task      │      │     │        verdict
      ┌─────────────▼┐  ┌──▼────▼──────┐
      │ Coder agents │  │  Verifiers   │
      │ (N parallel) │  │ (fresh ctx)  │
      └──────┬───────┘  └──────▲───────┘
             │  branch ready    │
             └──────────────────┘   fail → back to the same coder with the report
```

## Roles

### Project owner
- Owns product direction: scope, priorities, UX decisions, licensing, anything a user would notice.
- Answers escalations from the director. Is **not** asked about implementation details the director or coders can settle from `docs/`.

### Director (one, long-lived; normally the main session)
1. Keeps `docs/BACKLOG.md` current: tasks with IDs, dependencies, acceptance criteria, status.
2. Picks the ready tasks (dependencies done) and dispatches each to a **coder agent** in its own git worktree/branch, in parallel, up to the concurrency limit (default 4).
3. When a coder reports a branch ready, dispatches a **verifier agent** with the task card and branch name only — never the coder's own summary of what it did.
4. On **PASS**: reviews the diff size and scope, merges to `main` (fast-forward or squash), marks the task done, unblocks dependents.
5. On **FAIL**: sends the verifier's report back to the *same* coder agent (its context is intact) with instructions to fix; re-verifies. After **3 failed rounds** the director stops the loop and either re-scopes the task or escalates.
6. **Escalates to the owner** (via a question, not a guess) when: a task needs a product decision; two tasks conflict in a way the plan doesn't resolve; a dependency choice has licensing or platform implications; a phase is finished and the next phase's order should be confirmed; or a task has failed verification three times.
7. Posts a short status summary to the owner at natural checkpoints (a batch merged, a phase done, something blocked). Not a running commentary.

### Coder agents (many, task-scoped)
- Receive one task card. Work only inside their worktree/branch. May read anything in the repo and `docs/`.
- Deliver: code + unit tests + changelog line (+ ADR if they made an architectural choice). Run the build and `ctest` locally before reporting.
- Must not: widen the task's scope, touch unrelated files, change public interfaces owned by another in-flight task (listed on the card), or make product decisions. If blocked on a decision, they report `BLOCKED: <question>` to the director instead of guessing.
- Follow `CONTRIBUTING.md` (clean-room rules, conventions).

### Verifier agents (one per verification, fresh context)
- Receive: the task card (acceptance criteria) and the branch. They have **not** seen the coder's reasoning, so they test the artifact, not the story.
- Do: check out the branch, configure/build with warnings-as-errors, run all tests, then independently exercise each acceptance criterion (write a throwaway test or a small driver if the criterion isn't already covered), review the diff for scope creep, clean-room violations, missing tests, and convention breaches.
- Report a verdict in the fixed format below. A verifier never fixes code.

## Task card format (in `docs/BACKLOG.md`)

```
### T-0123 Short imperative title                      [status: ready|in-progress|verifying|done|blocked]
Phase: 1   Module: core/toolpath   Depends on: T-0100, T-0104   Owner agent: —
Goal: one paragraph on what exists when this is done and why it matters.
Interfaces: public types/functions this task owns or may change. Others are read-only.
Acceptance criteria:
  1. Observable, testable statements. "Offsetting a 10×10 square by +1 yields a 12×12 square with 4 arcs of r=1."
  2. ...
Out of scope: things a coder might be tempted to do but must not.
Notes/refs: docs/reference/feature-inventory.md §3 "Offset Vectors"
```

## Verifier report format

```
VERDICT: PASS | FAIL
Task: T-0123   Branch: task/T-0123-offset-vectors   Commit: abc1234
Build: ok | failed (first error)
Tests: 142 passed, 0 failed | ...
Acceptance criteria:
  1. PASS — how it was checked
  2. FAIL — expected X, observed Y, repro steps
Review findings (blocking): ...
Review findings (non-blocking): ...
```

## Branch and merge rules
- Branch per task: `task/T-0123-short-slug`. One task, one branch.
- `main` is always green. Only the director merges.
- Squash-merge with the task ID in the commit title. Commits end with the attribution lines the session provides.
- CI (`.github/workflows/ci.yml`) builds core+tests on Linux and the full Qt app on Windows. A red CI on `main` is the director's top priority.

## Concurrency guidance
- Tasks in the same module that share headers should not run concurrently unless the card lists the interface as frozen.
- Prefer many small tasks (half a day of agent work) over few big ones; big tasks fail verification more and block more.
- When a verifier fails a task for reasons outside the task (flaky test, broken main), the director fixes `main` first.

## Escalation examples
- "Should the tool database be a single JSON file or a SQLite DB?" → director decides (implementation) and records an ADR.
- "Should Carvetto's native file be a zip-of-JSON, or should we aim to be a drop-in for someone's existing files?" → owner decides (product/compat).
- "Pocket raster angle default 0° or 45°?" → copy the reference inventory's behaviour; no escalation.
- "Phase 1 is complete; start Phase 2 (3D relief) or harden Phase 1 with an installer first?" → owner.
