# ADR-0118: Compliance audit and the forbidden-name list

Date: 2026-11-17 · Status: proposed (T-0356)

## Context
The clean-room policy (CONTRIBUTING.md, PLAN section 7) bars third-party product and trademark names and
proprietary format internals from code, docs, UI strings, samples and the changelog. A checklist of those names
inside the repository would itself be a mention.

## Decision
1. `tools/check_compliance.py` (ctest `lint.compliance`, unit tests `lint.compliance_unit`, also run by CI through
   ctest) scans every text file and every path for forbidden tokens, and checks that each `FetchContent_Declare`
   has a row in `THIRD_PARTY_NOTICES.md`, that `LICENSE` exists, and that licence files the notices reference exist.
2. The forbidden list is kept as 20-hex-digit truncated SHA-256 digests of lower-case tokens, so the tool and
   its allowlist contain no forbidden word. Tokens are runs of letters and digits; two-word names are caught by
   joining adjacent capitalised tokens separated by one space. Reports give path, line and digest only. To add a
   name: `python3 tools/check_compliance.py --hash <word>` and paste the line into `FORBIDDEN`. Digests of short
   words are guessable; this is a mention-avoidance measure, not secrecy.
3. Factual compatibility claims and scope statements (for example the policy text naming the proprietary file
   extensions it forbids, or citing the public DXF specification) are allowed per file and per digest in
   `tools/compliance_allowlist.txt`; each entry needs a reason. Unit tests build their words from fragments.
4. Generic technique names (V-carve) and open controller/post formats are not on the list. The audit found no
   real violations on main, so no wording changes were needed.
