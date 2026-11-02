# ADR-0004: Implement the `.pp` post-processor template grammar

Date: 2026-10-24 · Status: accepted (director; owner informed in PLAN §3.3)

## Decision
The `post` module interprets the widely used plain-text `.pp` grammar (global settings, `VAR` definitions with `[tag|A/C|prefix|format|multiplier]`, and `begin … end` blocks). Carvetto ships its own posts in `posts/` (Grbl, Mach3/4, LinuxCNC, Marlin, generic G-code) and does not redistribute third-party posts.

## Rationale
Hundreds of community posts already exist for CNC controllers in this format; supporting it means users keep their machine setup. The format is a data format, not code, and its grammar is publicly documented.

## Consequences
- Golden-file tests compare Carvetto's output for a fixed toolpath against expected G-code per shipped post.
- Features that depend on post support (dwell, helical arcs, laser power, cutter comp, rotary wrap) are gated on the post declaring the block/variable.
