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
   value is sanitised to printable ASCII:
   - every control character, bytes 0x00–0x1F and 0x7F (LF, CR, tab, NUL, ESC, DEL …), is
     **replaced by a space**. A space keeps the words of a multi-line note apart and the line
     length unchanged; removal would glue words together. Either is safe.
   - `~`, `!` and `?` become `_`, and no byte ≥ 0x80 is written (owner decision 2026-11-01:
     "ASCII-only, all posts"). Grbl acts on these as realtime commands anywhere in the stream,
     even inside comments: `~` cycle start/resume, `!` feed hold, `?` status, and bytes 0x80–0xFF
     feed/rapid/spindle overrides, safety door and coolant (UTF-8 `Ñ` is C3 91: feed +10 %). A
     well-formed UTF-8 Latin-1 letter (U+00C0–U+00FF) becomes its plain letter (`Ñ`→`N`,
     `é`→`e`); any other well-formed character (`×`, `Æ`, `ß`, `€`, emoji, CJK) becomes one `_`;
     each byte of a malformed sequence (stray continuation, truncated, overlong, 0xF5–0xFF)
     becomes `_`. Error messages quote names through the same function.
   - inside a `( … )` comment that is still open where the value lands, `(` becomes `[` and `)`
     becomes `]`, whether or not the post has `SUBSTITUTE = "([)]"`. This is the mapping all
     shipped paren-comment posts already declare, so their output is unchanged; it stops a value
     from closing the comment early (the rest would become live code) or nesting one (an error on
     LinuxCNC and Grbl).
   - The comment context is read from the output line written so far, including the VAR prefix:
     `(` opens a comment, `)` closes it, `;` outside parentheses makes the rest of the line a
     comment. After `;` only control characters need replacing; `;`, `(` and `)` are kept.
   - **Active comments** (owner decision 2026-11-01: "neutralise in sanitiser"). LinuxCNC runs
     comments such as `(LOGOPEN,file)` (truncates the file), `(LOGAPPEND,…)`, `(LOG,…)`,
     `(MSG,…)`, `(DEBUG,…)`, `(PRINT,…)`, `(ABORT,…)`, `(PROBEOPEN file)`, `(PROBECLOSE)`,
     `(LOGCLOSE)` and, with the Python plugin, `(py,…)`/`(pyrun,…)`; Mach runs `(MSG,…)`. The
     shipped posts put names right after `(`. So when a value lands at the start of a comment
     (nothing but whitespace written since `(` or `;`, the VAR prefix included), a comma that
     follows only letters, digits, `_` and spaces becomes `_` — any word, not only the known
     keywords, so keywords we do not know are covered too — and a value whose leading word
     (spaces dropped, case ignored) starts with one of the argument-less or space-separated
     keywords PROBEOPEN, PROBECLOSE, LOGCLOSE gets a `_` in front. `LOGOPEN,/home/cnc/mill.hal`
     is written `(LOGOPEN_/home/cnc/mill.hal)`. Values further into a comment (`(Tool 1: a,b)`)
     keep their commas. A post template that itself supplies the comma right after a value at
     comment start (`"([TOOLPATH_NAME],x)"`) is not covered; no shipped post does this.
   - Text placed outside any comment (live G-code) is only stripped of control characters, so it
     cannot start a new block. What it says is still written as G-code: putting a text tag there
     is the post author's choice. The shipped posts only place text tags inside comments; a test
     enforces this.
2. **Numbers.** Move data is checked before anything is computed from it (so a bad coordinate is
   refused even on an axis the post does not write, and no non-finite coordinate reaches the arc
   maths), and every numeric tag is checked when it is rendered:
   - Every value must be finite.
   - Values in internal units must satisfy |v| ≤ **1e6**: lengths and coordinates (mm, including
     arc centres, I/J, radius, home, safe Z, material extents and tool diameter), feeds (mm/min,
     F/FC/FP and move feed overrides), spindle speed (rpm), dwell (s) and angles (degrees, the
     rotary `[A]` and `[Angle]`). The bound is inclusive. 1e6 mm is 1 km, far beyond any machine,
     and small enough that every value prints with a handful of digits.
   - The inverse-time feed `[FI]` (1/min) may reach **1e9**, because very short moves make it
     legitimately large; the same limit applies to `[F]` on cutting moves under
     `INVERSE_TIME_MODE = YES`, where `[F]` writes `[FI]`.
   - After unit conversion and the VAR multiplier, the written value must satisfy |v| ≤ **1e9**
     (Marlin's dwell in milliseconds is 1e9 at the 1e6 s limit). This catches a large multiplier.
   - The interpreter's own integer counters (`[N]`, `[T]`, `[TP]`, drill-bank `[DB]`) are not
     limited.
   - **Job settings.** `PostContext::arcTolerance` must lie in **1e-6 … 1 mm** (finite, positive;
     the old silent fallback to 0.01 mm for ≤ 0 is gone). Below about 1e-13·r the chord angle
     `2·acos(1 − tol/r)` rounds to zero, and coarser than 1 mm is no longer an arc. A linearised
     arc is split into at most **100,000** chords; a count that comes out infinite is capped
     there and never collapses to a single chord. With a rotary wrap, `rotaryDiameter` must be
     finite, positive and ≤ 1e6 mm (an infinite diameter made every `[A]` 0). The job home must be
     finite and within ±1e6 mm.
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
- Names with accents or symbols lose them in the G-code comments (`Ñ` → `N`, `×` → `_`); the
  job file keeps them.
- Signs are not checked (a negative feed or spindle speed within range is still written), and
  overly long names are not shortened; a controller with a line-length limit rejects such a line,
  which stops the program safely.
- ADR-0011 §10 (`SUBSTITUTE` applies to text tags only) still holds; this sanitising runs after it.
