# ADR-0020: Post output safety (text sanitising, number limits)

Date: 2026-11-01 · Status: proposed (T-0133), amended 2026-11-02 (T-0134, T-0136), 2026-11-03 (T-0154), 2026-11-04 (T-0161, T-0162)

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
   - **Line length** (T-0162). No line is written longer than **250 characters** (line ending
     excluded), on every post, in every block. LinuxCNC reads a program line with
     `fgets(raw_line, LINELEN, …)` (LINELEN 255) and refuses it with `NCE_COMMAND_TOO_LONG` when
     the read reaches 254 characters, line ending included, aborting the program; grblHAL keeps
     every character of a line, comments included, in a 257-byte buffer (`LINE_BUFFER_SIZE`, 256
     characters) and drops a longer line with `error:overflow` (classic Grbl and Marlin skip
     comment characters; see the 79-character drag-section bound below). 250 plus CR LF is 252,
     below both. Before T-0162 a ~300-character tool name in `(Tool [T]: [TOOLNAME])` aborted a
     LinuxCNC program.
     - A line holding a text tag that comes out longer is expanded again with every text value
       that lands in a comment (paren or `;`) cut to at most `cap` characters, where a cut value
       keeps its first `cap − 3` characters and ends in `...`, so the cut is visible. `cap` is the
       largest value that fits (one cap for all values of the line, so a short name next to a
       long one stays whole). The cut is applied to the sanitised value, whose every prefix is
       as safe as the whole (no `)` closes the comment, commas in an active-comment run are
       already `_`); later values on the line are judged against the cut text. Line numbers and
       modal values are restored before each expansion, so the written line advances them once.
     - A line that still does not fit is refused: template text longer than the limit, a long
       text value in live G-code (not a comment, so not cut), or so many text values that `...`
       for each is too long (`the line '…' would be 251 characters long; lines are limited to 250
       (LinuxCNC and grblHAL refuse longer lines)`).
     - The shipped posts' lines are far shorter (numbers are bounded above), so only names are
       ever cut: the goldens are unchanged, and a test posts names of 0 to 5000 bytes in every
       text tag through all eight posts.
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
       tool and asking the operator to check it is stopped or locked, a bare `M0` (Marlin too: its
       message is the `;` comment before it), and safe Z again (Z may have been re-zeroed during the pause), the same shape
       as their manual tool-change stop. On every shipped post the pause is `M0`, also on Mach3,
       Mach4 and LinuxCNC, whose tool change is `M6`: `M6` is a tool change that a controller
       may be set to ignore, `M0` is the program stop every one of them honours.
     - refuses `[S]` anywhere else while the toolpath is written (move blocks, the pauses), and
       checks every written line against the drag-section allowlist below.
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
       drag tool N done: unlock the spindle before it starts again", a bare `M0` and safe Z. A post without the block refuses every job with a
       spindle-off toolpath that needs it: a spinning toolpath after a spindle-off one
       (`toolpath 'Mill' starts the spindle after the spindle-off toolpath 'Drag', but the post
       has no SPINDLE_UNLOCK_PAUSE block …`) or a job ending with one (`the job ends with the
       spindle-off toolpath 'Drag', but …`).
     - Profile and Pocket refuse diamond-drag tools (`InvalidTool`, "… is a diamond drag tool,
       which must not turn; use Quick Engrave for diamond-drag tools"; Pocket skips just that
       tool), so a drag tool can no longer run in a spinning toolpath of those operations.
     - **Drag-section allowlist** (T-0154; owner decisions 2026-11-03: an allowlist, not a
       blocklist, refined in a final round). Every output line of a `.pp` block must be printable
       ASCII (bytes ≥ 0x80 are a parse error for all posts: Grbl drops or acts on them, so
       `M<NBSP>3` would run M3; control characters were already refused), and `LINE_ENDING` must
       be LF, CR LF or CR (`[10]`, `[13][10]`, `[13]`); anything else is a parse error for all
       posts, since the ending is written after every line (`" M3 S12000[13][10]"` would add a
       spindle start to each). While a spindle-off toolpath is written (its HEADER, TOOLCHANGE or
       NEW_SEGMENT, both pause blocks and its move blocks):
       - *Template text* of every written line: no `#`; no M or S followed (after spaces) by a
         tag (`M[T]`); only plain comments, where a comment ends at the FIRST `)` as on Grbl and
         Mach (a `(` inside an open comment, a `)` without one, or an unclosed comment is
         refused); no M3/M4/S-number word anywhere, comments included; and none of Marlin's
         emergency-parser codes `M108 M112 M410 M876` anywhere (spaces ignored, leading zeros
         allowed).
       - *The exact bytes written* for the line (template text, VAR prefixes, values and the line
         ending) are checked: the line ending as above; printable ASCII only; none of Grbl's
         realtime characters `~ ! ?`; parentheses as above. On `;`-style posts (some block line
         starts with `;` and none holds a `(`: Marlin) text after `;` is a comment and no `(` or
         `)` may appear before it (Marlin without `PAREN_COMMENTS` has no paren comments); on
         other posts `;` is not a comment for this check.
       - No tab anywhere in a line written in a drag section, template text included, on every post: Marlin does not treat
         a tab as a separator (`N0<TAB>M0` is an unknown command and the pause is skipped), and no
         shipped post writes tabs. On `;`-style posts no `N` word either: hosts such as OctoPrint
         and Printrun prefix their own `N…*checksum` and Marlin skips only one `N`, so
         `N57 N1 M0` would be an unknown command (the shipped Marlin post writes no line numbers).
       - *Live G-code* of the written line (outside comments): only words LETTER+VALUE separated
         by spaces, the letter upper case and directly followed by its value. Any other
         character refuses the line, including `/` anywhere (block delete has no place in a drag
         section; a VAR prefix `/N` is refused too), `#`, `[`, `*`, `=`, `%`, `$`, `@`, `^`, `;`
         on paren posts, lower case, a letter without a value and a space inside a word. Allowed
         words, which is exactly what the eight shipped posts write in drag sections:
         - `X Y Z I J F P`: a plain number with an optional sign (`-0.100`, `+5`, `400`), at most
           8 integer and 6 decimal digits;
         - `N`: plain digits, at most 9 (Mach3 numbers lines in steps of 10 up to `N999990`),
           with a value of at most 9,999,999 (T-0161): Grbl refuses a line number above
           `MAX_LINE_NUMBER` (10,000,000) and drops the line. A post does not say which
           controller it is for, so the cap holds on every post; no shipped post comes near it;
         - `T H`: plain digits, at most 5;
         - `G`: plain digits with an optional decimal part, at most 5 digits in all, whose value
           is one of `0 1 2 3 4 17 20 21 40 43 49 80 90 91 91.1 94` (leading zeros allowed:
           `G01`);
         - `M`: plain digits (at most 5, leading zeros allowed, no sign, no decimal point) with
           the value `0 1 2 5 6 30` (`M6` for the tool change into a drag toolpath on Mach and
           LinuxCNC);
         - a line with `M0` or `M1` is bare (owner decision 2026-11-03): an optional `N` word and
           the `M0`/`M1`, nothing else, plus at most one plain comment (a paren comment on paren
           posts, a `;` comment on `;`-style posts). There is no message text after `M0` on any
           post: Marlin's `FASTER_GCODE_PARSER` reads every upper-case letter of a message as a
           parameter and skips spaces (`M0 SPINDLE STOP 1` is `P1`, a 1 ms timeout instead of a
           pause; `M0 TOOL UP 3` is `P3`), and Grbl and LinuxCNC drop spaces (`M0 3` is `M03`).
           The shipped Marlin post writes its pause messages as a `;` comment on the line before
           the bare `M0` (`; Load tool 3 then resume`, then `M0`), so the LCD shows Marlin's
           default prompt.
         - in the two pause blocks the `M5` line is bare too (T-0161): an optional `N` word and the
           `M5` (leading zeros allowed), plus at most one plain comment. Grbl drops a line that
           errors as a whole, so `M5 M6` (unsupported word), `M5 G0 G1` (modal-group violation) or
           a line past its buffer would lose the spindle stop; on Marlin only the first command
           of a line runs, so `G0 Z5 M5` never stopped it. Elsewhere in a drag section an `M5`
           may share its line (the pause's bare `M5` follows anyway).
         - at most 79 characters of live G-code per line (outside comments, spaces and values
           included, line ending excluded; T-0161): Grbl's `LINE_BUFFER_SIZE` is 80 and a longer
           line is dropped whole with `error:overflow`. Grbl and Marlin (`MAX_CMD_SIZE` 96) do not
           buffer comments, so a long tool or toolpath name, which lands only in comments, never
           causes a refusal. A test posts drag arcs at +-9,876 mm with Mach3's highest line
           numbers through every shipped post within the bound.
         So `M/3`, `S/1000` (Grbl drops `/` anywhere), `M3.004`, `M3.00005`, `M2.99995`, `M3.5`
         (controllers round or take the integer part), `M259` (Grbl truncates to 8 bits),
         `M000000005`, `G00000091.1`, any S word, other letters (`D E R K A B C Q L O S`) and
         other G or M codes are refused, also when a tool name places them in live G-code.
         Custom posts that need more in a drag section are refused; the set can be widened case
         by case.
       - Values inside comments are sanitised already (above), so a tool named "S12 drill M3"
         or "Drag 3mm 108" in a comment is fine.
       - The pause blocks must write `M5` and `M0`. This is checked on the template text before
         anything is written (lines starting with `/` and text after `;` do not count) and again
         on the lines actually written (so `M0[T]` writing `M05` does not count as an M0); on
         `;`-style posts only the first word of a line counts (Marlin runs one command per line,
         so `M5 M0` on one line gives no M0).
       - **Order** (T-0162): in both pause blocks the first `M5` comes before every `M0` and `M1`.
         A block written `M0` then `M5` paused for the operator while the spindle could still
         turn; an `M1` (optional stop) is an operator pause too. Checked on the template text
         before anything is written (same word reading as above: lines starting with `/` and
         text after `;` do not count, so `/M5` then `M0` is refused) and again on the lines
         actually written; the run is refused with `… block pauses (M0 or M1) before it stops the
         spindle (M5); write the M5 first`. More pauses and stops after the first `M5` are fine.
         `SPINDLE_UNLOCK_PAUSE` follows the same rule although the spindle is already off at that
         point (coder's choice, reported to the director): the two blocks then have one rule, the
         shipped posts already write `M5` first in both, and the operator is never told to unlock
         a spindle before the program has stopped it.
       - Unused words on other lines (Grbl rejects `G0 P1` with an error) and the arguments a
         controller gives to allowed codes are the post author's responsibility. Such lines cannot
         start the spindle, but they are not harmless in general: Marlin with `PAREN_COMMENTS`
         reads `M0 P1` as a 1 ms timeout, which is why the `M0` line itself must be bare (and,
         since T-0161, the pause blocks' `M5` line).
       - Unverified: whether Mach3/Mach4 treat `;` as a comment. Carvetto treats `;` as a
         comment only on `;`-style posts; on the paren posts a `;` in a drag section is refused.
       - O-words, `M98` subroutine calls and user M-code macros (LinuxCNC `M100`–`M199`, Mach
         macros) are refused in drag sections by the allowlist; elsewhere they are the post
         author's responsibility: Carvetto cannot see what they do.
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
  and dwells must be above zero (see above). Since T-0162 overly long names are cut to keep each
  line within 250 characters (above); the job file keeps them whole.
- ADR-0011 §10 (`SUBSTITUTE` applies to text tags only) still holds; this sanitising runs after it.
