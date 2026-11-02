# ADR-0020: Post output safety (text sanitising, number limits)

Date: 2026-11-01 · Status: proposed (T-0133)

## Context
The post interpreter (ADR-0011) wrote text tags (`[TOOLNAME]`, `[TOOLPATH_NAME]`, `[TP_FILENAME]`,
`[FILE_NOTES]` …) after applying only the post's `SUBSTITUTE` pairs. A newline or other control
character in a name therefore ended the comment line and started live G-code: a toolpath named
`"Evil\nG0 Z-50"` produced the block `G0 Z-50)`. Tool names reach the post from tool databases,
and toolpath names and file names from `.carv` files and CLI parameters (T-0122), so this is
reachable from untrusted input. Separately, numbers were formatted whatever their value: non-finite
coordinates came out as `Z-inf`/`Xinf`, and `1e300` as a 300-digit word.

## Decision
Both checks live in the interpreter's render path, which every `[tag]` of every block goes
through, so no post, block or tag can bypass them.

1. **Text values.** After `SUBSTITUTE` (so a post setting cannot undo it), every rendered text
   value is sanitised:
   - every control character, bytes 0x00–0x1F and 0x7F (LF, CR, tab, NUL, ESC, DEL …), is
     **replaced by a space**. A space keeps the words of a multi-line note apart and the line
     length unchanged; removal would glue words together. Either is safe.
   - inside a `( … )` comment that is still open where the value lands, `(` becomes `[` and `)`
     becomes `]`, whether or not the post has `SUBSTITUTE = "([)]"`. This is the mapping all
     shipped paren-comment posts already declare, so their output is unchanged; it stops a value
     from closing the comment early (the rest would become live code) or nesting one (an error on
     LinuxCNC and Grbl).
   - The comment context is read from the output line written so far, including the VAR prefix:
     `(` opens a comment, `)` closes it, `;` outside parentheses makes the rest of the line a
     comment. After `;` only control characters need replacing; `;`, `(` and `)` are kept.
   - Text placed outside any comment (live G-code) is only stripped of control characters, so it
     cannot start a new block. What it says is still written as G-code: putting a text tag there
     is the post author's choice. The shipped posts only place text tags inside comments; a test
     enforces this.
   - Bytes ≥ 0x80 (UTF-8) are passed through unchanged.
2. **Numbers.** Move data is checked before anything is computed from it (so a bad coordinate is
   refused even on an axis the post does not write, and never reaches the arc maths), and every
   numeric tag is checked when it is rendered:
   - Every value must be finite.
   - Values in internal units must satisfy |v| ≤ **1e6**: lengths and coordinates (mm, including
     arc centres, I/J, radius, home, safe Z, material extents and tool diameter), feeds (mm/min,
     F/FC/FP and move feed overrides), spindle speed (rpm), dwell (s) and angles (degrees, the
     rotary `[A]` and `[Angle]`). The bound is inclusive. 1e6 mm is 1 km, far beyond any machine,
     and small enough that every value prints with a handful of digits.
   - The inverse-time feed `[FI]` (1/min) may reach **1e9**, because very short moves make it
     legitimately large.
   - After unit conversion and the VAR multiplier, the written value must satisfy |v| ≤ **1e9**
     (Marlin's dwell in milliseconds is 1e9 at the 1e6 s limit). This catches a large multiplier.
   - The interpreter's own integer counters (`[N]`, `[T]`, `[TP]`, drill-bank `[DB]`) are not
     limited.
   - A VAR multiplier that is not finite (`inf`, `nan`, `1e400`) is a parse error.
   - A failure aborts the run with an `Error` and no output, e.g.
     `cannot write G-code: toolpath 'Profile', move 2: Z -inf is not a finite number` or
     `cannot write G-code in SPINDLE_ON of toolpath 'Drill': [S] 1e+300 rpm is outside the
     supported range -1e+06..1e+06 rpm`. Names in messages are sanitised the same way and cut to
     64 bytes. The `Error` line is the VAR's line when its multiplier caused the failure, else 0.

## Consequences
- The shipped goldens are byte-identical: fixture names have no control characters, and the
  paren mapping equals the shipped posts' `SUBSTITUTE`.
- Third-party posts without `SUBSTITUTE = "([)]"` now get `[ ]` for parentheses in names inside
  comments, where they used to get broken output.
- Signs are not checked (a negative feed or spindle speed within range is still written), and
  overly long names are not shortened; a controller with a line-length limit rejects such a line,
  which stops the program safely.
- ADR-0011 §10 (`SUBSTITUTE` applies to text tags only) still holds; this sanitising runs after it.
