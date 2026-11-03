# ADR-0020: Post output safety (text sanitising, number limits)

Date: 2026-11-01 · Status: proposed (T-0133), amended 2026-11-02 (T-0134, T-0136), 2026-11-03 (T-0154)

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
   - `~`, `!` and `?` become `_`, and no byte ≥ 0x80 is written (owner decision 2026-11-02:
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
   - **Active comments** (owner decision 2026-11-02: "neutralise in sanitiser"). LinuxCNC runs
     comments such as `(LOGOPEN,file)` (truncates the file), `(LOGAPPEND,…)`, `(LOG,…)`,
     `(MSG,…)`, `(DEBUG,…)`, `(PRINT,…)`, `(ABORT,…)`, `(PROBEOPEN file)`, `(PROBECLOSE)`,
     `(LOGCLOSE)` and, with the Python plugin, `(py,…)`/`(pyrun,…)`; Mach runs `(MSG,…)`. The
     shipped posts put names right after `(`. The rule looks at the whole comment written so
     far, not at the value alone, because adjacent values can build a keyword together
     (`([TOOLNAME][TOOLPATH_NAME])` with "LOGOPEN" and ",/file"; "PROBE" and "OPEN /file"):
     - While everything written since the `(` or `;` (template text, VAR prefixes and earlier
       values alike) is only letters, digits, `_` and spaces, that text plus the value is judged
       as one word run, spaces dropped and case ignored.
     - Every comma in the value that continues the run becomes `_`. This covers any word, not
       only the known keywords, so keywords we do not know are covered too. A replaced comma is
       itself a word character, so the run (and the replacing) goes on through the value and
       into later values of the same comment.
     - When the value completes PROBEOPEN, PROBECLOSE or LOGCLOSE (the keywords that take no
       comma) at the start of the run, the value gets a `_` in front (`(PROBE_OPEN /file)`).
     - Once any other character has been written in the comment (`(Tool 1: a,b)`,
       `([TOOLNAME]-[TOOLPATH_NAME])`), no active comment can form there, and commas are kept.
     - `LOGOPEN,/home/cnc/mill.hal` is written `(LOGOPEN_/home/cnc/mill.hal)`.
     - **Gap (custom posts only):** template text written *after* a value cannot be judged
       when the value is written, and values already written are not changed. So template text
       that completes the active comment right after a value is not covered: a comma
       (`"([TOOLPATH_NAME],x)"` with the name "LOGOPEN"), or the rest of a keyword from a VAR
       prefix (prefix `OPEN` after a name "PROBE"). No shipped post has template text after a
       value other than `)`: a text tag is followed only by `)`, the end of the line or another
       text tag (template text *before* a value, as in `(Generated by [PRODUCT])` or
       `(Tool [T]: [TOOLNAME])`, is judged and safe). A test enforces this for every shipped post
       (T-0134). Template text that by itself forms an active comment is the post author's own
       text.
     - **Control characters in templates** (T-0134). A form feed, vertical tab or CR between `(`
       and a value would hide the run from this rule while LinuxCNC skips it as whitespace
       (`"(\f[TOOLPATH_NAME])"` with "MSG,hi"). The parser therefore refuses every control
       character other than tab in a `.pp` line (a CR LF line end is trimmed), naming the line
       and the character code; and the word-run check treats every control character like a
       space, as a second guard.
     - **`[TOOLS_USED]`** joins the tool names with `"; "` (T-0134), not `", "`: a comma after the
       first name at the start of a comment would continue its word run and be written as `_`
       (`(EndMill_ Drill)`). A `;` ends the run, so later names keep their commas.
     - Comments the template nests or leaves unbalanced (`((a)…`) are outside this rule. Nested
       comments are an error on LinuxCNC; Grbl and Mach end a comment at the first `)`, so text
       after it runs as G-code (corrected in T-0154; in a drag section such templates are refused,
       see below).
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
     limited by the 1e6 rule; the 1e9 written limit still applies.
   - **Job settings.** `PostContext::arcTolerance` must lie in **1e-6 … 1 mm** (finite, positive;
     the old silent fallback to 0.01 mm for ≤ 0 is gone). Below about 1e-13·r the chord angle
     `2·acos(1 − tol/r)` rounds to zero, and coarser than 1 mm is no longer an arc. A linearised
     arc is split into at most **100,000** chords; a count that comes out infinite is capped
     there and never collapses to a single chord. With a rotary wrap, `rotaryDiameter` must be
     finite, positive and ≤ 1e6 mm (an infinite diameter made every `[A]` 0). The job home must be
     finite and within ±1e6 mm.
   - A VAR multiplier that is not finite (`inf`, `nan`, `1e400`) is a parse error.
   - **Spindle speed above zero** (owner decision 2026-11-02, T-0134). While a toolpath is
     written, its tool's spindle speed must be a finite number greater than zero wherever `[S]`
     is written and before every `SPINDLE_ON` block (whether or not that block writes `[S]`).
     Otherwise the run fails naming the toolpath and the tool:
     `cannot write G-code in SPINDLE_ON of toolpath 'Drill': tool 'Drill (3 mm)' has spindle
     speed 0 rpm; it must be greater than zero (for a manual spindle, set any positive nominal
     speed)`. Manual-spindle users set any positive nominal speed; a post for a manual spindle
     can simply not write `[S]` (and have no `SPINDLE_ON`). A job with no toolpaths has no
     spindle speed to check; likewise the feed, spindle and dwell rules below check nothing in
     such a job (its header and footer cut nothing).
   - **Toolpaths that run with the spindle off** (owner decision 2026-11-03, T-0154). The rule
     above has exactly one exemption: a toolpath whose tool snapshot has `spindleOff` set. Only
     Quick Engrave sets it, for diamond-drag tools (whose snapshot then has spindle speed 0 and
     records `toolType` = `diamond_drag`); the native record allows it only on engrave records
     whose snapshot records that tool type (the loader, `saveCarv` and the toolpath commands
     refuse it elsewhere, ADR-0021 §2), and the interpreter refuses a spindle-off toolpath whose
     snapshot is not a diamond drag tool's as a second guard (`toolpath 'X' is marked to run with
     the spindle off, but its tool 'Y' is not a diamond drag tool`). `Document::insertToolpath`
     is an internal API without these checks; the guards are at the commands, save, load and the
     post, so Profile, Pocket, Drill and Quick Engrave with any other
     tool stay under the rule; a test posts each of them with spindle speed 0 through every
     shipped post and expects the refusal. For such a toolpath the interpreter:
     - needs the post's `SPINDLE_OFF_PAUSE` block, and that block must write an `M5` and an `M0`
       as template text outside comments (read with the tokeniser below); a post without it, or
       with an empty or M5-less or M0-less block, refuses the run before writing anything (`toolpath 'Drag' runs with the spindle off (tool 'X' must not turn), but
       the post has no SPINDLE_OFF_PAUSE block to stop the spindle and pause for the operator
       before it`). Jobs without such toolpaths never use the block.
     - writes HEADER (when it is the first toolpath), TOOLCHANGE or NEW_SEGMENT as usual but
       leaves out every line of those blocks that writes `[S]` and does nothing else: outside its
       comments only `M3`/`M4` words, `[S]` (after an optional literal `S`), `[N]` and spaces.
       A line that writes `[S]` together with other codes (`T1 M6 M3 S[S]`) is refused rather
       than dropped, so leaving a line out never loses a tool change. SPINDLE_ON is not written.
     - then writes `SPINDLE_OFF_PAUSE` right before the toolpath's first move, every time a
       spindle-off toolpath starts (also after another spindle-off toolpath with the same tool).
       The shipped posts write `M5`, safe Z, a comment saying the spindle stays off for the drag
       tool and asking the operator to check it is stopped or locked, `M0` (Marlin: `M0` with a
       message), and safe Z again (Z may have been re-zeroed during the pause), the same shape
       as their manual tool-change stop. On every shipped post the pause is `M0`, also on Mach3,
       Mach4 and LinuxCNC, whose tool change is `M6`: `M6` is a tool change that a controller
       may be set to ignore, `M0` is the program stop every one of them honours.
     - refuses `[S]` anywhere else while the toolpath is written (move blocks, the pauses), and,
       as a second guard, any written line whose G-code outside comments holds a spindle start
       or speed word, whatever the post's template text says. Words are read the way RS274/NGC
       controllers and Grbl read them: spaces and tabs are ignored everywhere, letters are
       case-insensitive, a value may have a sign or a leading `.`, and every letter followed by a
       number is a word wherever it stands (`M 3`, `M<TAB>03`, `m4`, `M+3`, `S 1000`, `S.5`,
       `G0M3`, `/M3`, `N10M3`, and `M[T]` with tool 3 are all refused). Letters without a value
       (Marlin's `M0 Drag tool 3 spindle off` message) are not words. The same tokeniser decides
       which `[S]` lines are spindle-only and whether the pause blocks write `M5`/`M0`.
     - treats a change between a spindle-off and a spinning toolpath as a tool change even when
       the tool number and name are equal, so the spindle restarts only through TOOLCHANGE +
       SPINDLE_ON (and its `[S]` > 0 check).
     - **Unlock pause** (owner decision 2026-11-03). Before that TOOLCHANGE, i.e. after the last
       spindle-off toolpath of a drag section and before a spinning toolpath can start the
       spindle, it writes the post's `SPINDLE_UNLOCK_PAUSE` block: once per drag section, never
       between consecutive spindle-off toolpaths. A job that ends with a drag section gets it
       before FOOTER (director decision 2026-11-03: the next program could otherwise start a
       spindle that is still locked). The block is
       written in the drag toolpath's context (`[T]`, `[TOOLNAME]` are the drag tool's) and is
       under the same spindle-word guards. It must write an `M5` and an `M0`, like `SPINDLE_OFF_PAUSE`. The shipped posts write `M5`, safe Z, a comment "Diamond
       drag tool N done: unlock the spindle before it starts again", `M0` (Marlin: `M0 Unlock
       spindle then resume`) and safe Z. A post without the block refuses every job with a
       spindle-off toolpath that needs it: a spinning toolpath after a spindle-off one
       (`toolpath 'Mill' starts the spindle after the spindle-off toolpath 'Drag', but the post
       has no SPINDLE_UNLOCK_PAUSE block …`) or a job ending with one (`the job ends with the
       spindle-off toolpath 'Drag', but …`).
     - Profile and Pocket refuse diamond-drag tools (`InvalidTool`, "… is a diamond drag tool,
       which must not turn; use Quick Engrave for diamond-drag tools"; Pocket skips just that
       tool), so a drag tool can no longer run in a spinning toolpath of those operations.
     - **Strict template rule** (T-0154 verifier round 3, director 2026-11-03). Every output line
       of a `.pp` block must be printable ASCII (bytes ≥ 0x80 are a parse error for all posts:
       Grbl drops or acts on them, so `M<NBSP>3` would run M3; control characters were already
       refused). While a spindle-off toolpath is written (its HEADER, TOOLCHANGE or NEW_SEGMENT,
       both pause blocks and its move blocks), every template line that is written must have:
       no `#` (RS274/NGC parameters, `M#1`); no M or S directly followed (after spaces) by a tag
       (`M[T]`); only plain comments, where a comment ends at the first `)` as on Grbl and Mach,
       and a `(` inside an open comment, a `)` without one or an unclosed comment is refused;
       and no M3/M4/S-number word anywhere in the template text, comments included (it is the
       post author's fixed text and has no reason to mention them there). The written line
       (with its values) is checked again: printable ASCII, plain comments, no `#` or `[` in the
       live G-code, and no spindle word in the live G-code; values inside comments are sanitised
       already, so a tool named "S12 drill" in a comment is fine. Text after `;` is a comment for
       these checks only on posts whose comment style is `;` (some block line starts with `;` and
       none holds a `(`, i.e. Marlin); on other posts it counts as G-code. For the pause-content
       requirement (both pause blocks must write `M5` and `M0`), lines starting with `/` (block
       delete) and text after `;` never count. O-words, `M98` subroutine calls and user M-code
       macros (LinuxCNC `M100`–`M199`, Mach macros) in a post are the post author's
       responsibility: Carvetto cannot see what they do.
   - **Toolpaths without moves** (T-0136). A toolpath with zero moves cuts nothing and is
     skipped entirely: it is not written, its tool is not in `[TOOLS_USED]`, it causes no tool
     change, and its tool's spindle speed, feeds and dwells are not checked. The header tool (the
     `[T]`, `[S]`, `[TOOLNAME]` … the header writes) is the tool of the **first toolpath with
     moves**, so its spindle speed is the one checked for a `SPINDLE_ON`/`[S]` in the header. A
     job whose toolpaths all have zero moves is posted exactly like a job with no toolpaths:
     header and footer with the default (empty) tool, nothing checked. On a post whose header
     starts the spindle that writes `M3 S0` (as for an empty job since T-0133), which runs
     nothing. Refusing such a job was the alternative; it was not chosen because an empty job is
     posted the same way and a recalculated toolpath can legitimately come out empty (all vectors
     too small for the tool). Records without computed moves (`moves` null) are refused earlier
     by carvetto-cli `post` (ADR-0021 §6); a computed record of zero moves reaches this rule.
   - **No rate printed as zero** (owner decision 2026-11-02, T-0134). A feed (`[F]`, `[FC]`,
     `[FP]`, `[FI]`, feed overrides), spindle speed or dwell that is above zero but that the
     post's number format rounds to zero (`0.001` mm/min as `F0`, a 0.004 s dwell as `P0.00`, a
     denormal) fails the run instead of writing the zero, e.g. `[F] 0.001 mm/min would be
     written as F0; it is above zero but too small for the post's 0-decimal format`. The check
     looks at the written digits, so it follows the post's decimals, units and multiplier.
   - **Feeds and dwells of zero or less** (owner decision 2026-11-02, T-0134). A feed rate,
     plunge rate or feed override of zero or below is refused wherever a feed tag writes it
     (`[F] feed 0 mm/min would be written as F0; feeds must be greater than zero`); a feed the
     post never writes (a drill toolpath's cutting feed) is not checked. Inverse time (`[FI]`, and
     `[F]` on cutting moves under `INVERSE_TIME_MODE`) is judged by the feed it is computed from:
     on a cutting move longer than the zero-length tolerance (1e-9 mm) that feed (tool feed,
     plunge rate or the move's feed override) must be above zero, and an inverse time that is
     written as zero is refused even when the quotient underflowed to exactly 0 (a denormal
     feed). Only a cutting move of zero length, whose inverse time is 0 by definition, and `[FI]`
     outside a cutting move are exempt. A dwell
     move of zero seconds or less is refused when the move is validated, before anything is
     written and even when the post drops dwells (`toolpath 'Drill', move 7: dwell 0 s must be
     greater than zero`); the CLI's toolpath-member decoder refuses the same values on load.
   - A failure aborts the run with an `Error` and no output, e.g.
     `cannot write G-code: toolpath 'Profile', move 2: Z -inf is not a finite number` or
     `cannot write G-code in SPINDLE_ON of toolpath 'Drill': [S] 1e+300 rpm is outside the
     supported range -1e+06..1e+06 rpm`. Names in messages are sanitised the same way and cut to
     64 bytes. The `Error` line is the VAR's line when a written value is out of range and that VAR has a
     multiplier other than 1, else 0.

## Consequences
- The shipped goldens are byte-identical: fixture names have no control characters, and the
  paren mapping equals the shipped posts' `SUBSTITUTE`.
- Third-party posts without `SUBSTITUTE = "([)]"` now get `[ ]` for parentheses in names inside
  comments, where they used to get broken output.
- Names with accents or symbols lose them in the G-code comments (`Ñ` → `N`, `×` → `_`); the
  job file keeps them.
- T-0154: the shipped posts gained `SPINDLE_OFF_PAUSE`; goldens of jobs without spindle-off
  toolpaths are byte-identical, and each post has a drag golden (`<post>_drag.nc`). A build from
  before T-0154 drops the record's `spindle_off` key and, seeing spindle speed 0, refuses to post
  the drag toolpath (it never spins the drag tool). The unlock pause covers a locked spindle
  before it restarts, also on a controller whose tool change involves no operator.
- Signs are not checked for coordinates and other values; since T-0134 spindle speed, feeds
  and dwells must be above zero (see above), and
  overly long names are not shortened; a controller with a line-length limit rejects such a line,
  which stops the program safely.
- ADR-0011 §10 (`SUBSTITUTE` applies to text tags only) still holds; this sanitising runs after it.
