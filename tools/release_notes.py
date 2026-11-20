#!/usr/bin/env python3
"""Extract the release notes for a version tag from CHANGELOG.md (T-0361).

  python3 -I tools/release_notes.py v0.1.0 [--changelog CHANGELOG.md] [--output FILE]

Finds the `## X.Y.Z` heading (optionally followed by a space and a date or other text) and prints
everything up to the next `## ` heading, with surrounding blank lines trimmed. Exits 1 with a
message on stderr if the tag is not a version tag, the section is missing, or it is empty.
When the section is too long for a GitHub release body (limit 125000 characters; T-0362) a
condensed body is produced instead: a short header, a link to the full changelog for the tag, then
each entry cut to its card id and first sentence. Standard library only.
"""

import argparse
import re
import sys

REPO_URL = "https://github.com/thecloudix/Carvetto"
FULL_LIMIT = 100000      # above this the condensed form is used
HARD_LIMIT = 110000      # the condensed form never exceeds this
ENTRY_LIMIT = 300        # characters per condensed entry
ID_RE = re.compile(r"^- (T-\d{4}(?:[/-]T?-?\d{4})*:)\s*")
TAG_RE = re.compile(r"^v?(\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.\-+]*)?)$")


def version_of(tag):
    m = TAG_RE.match(tag.strip())
    if not m:
        raise ValueError("not a version tag: %r (expected vX.Y.Z)" % tag)
    return m.group(1)


def extract(text, version):
    """Return the notes of section `## <version>`; raise ValueError if absent or empty."""
    lines = text.replace("\r\n", "\n").split("\n")
    heading = re.compile(r"^##[ \t]+" + re.escape(version) + r"(?:[ \t].*)?$")
    start = None
    for i, line in enumerate(lines):
        if heading.match(line):
            start = i + 1
            break
    if start is None:
        raise ValueError("CHANGELOG.md has no '## %s' section (prerelease tags such as vX.Y.Z-rc1 "
                         "need their own '## X.Y.Z-rc1' section)" % version)
    end = len(lines)
    for j in range(start, len(lines)):
        if lines[j].startswith("## "):
            end = j
            break
    body = "\n".join(lines[start:end]).strip("\n")
    if not body.strip():
        raise ValueError("the '## %s' section is empty" % version)
    return body + "\n"


def shorten_line(line, limit=ENTRY_LIMIT):
    """Cut one entry to its card id plus the first sentence/clause, at a word boundary."""
    line = line.rstrip()
    m = ID_RE.match(line)
    head = "- " + m.group(1) + " " if m else ""
    rest = line[m.end():] if m else line
    cut = re.search(r"(?<=[.!?;])\s", rest)
    clause = rest[:cut.start()] if cut else rest
    clause = clause.rstrip(" ;,:—–")
    full = head + clause
    short = len(clause) < len(rest.rstrip())
    if len(full) <= limit:
        return full + (" …" if short and not clause.endswith(".") else "")
    room = limit - len(head) - 1
    clause = clause[:room]
    sp = clause.rfind(" ")
    if sp > 0:
        clause = clause[:sp]
    return head + clause.rstrip(" ;,:—–.") + "…"


def condense(notes, version, tag, full_limit=FULL_LIMIT, hard_limit=HARD_LIMIT):
    """Return `notes` unchanged when it fits, else a short body of at most `hard_limit` characters."""
    if len(notes) <= full_limit:
        return notes
    url = "%s/blob/%s/CHANGELOG.md" % (REPO_URL, tag)
    header = ("## %s\n\nThis is a condensed list of the changes (card id and first sentence). "
              "The full text of every entry is in the changelog for this tag: %s\n\n" % (version, url))
    lines = [shorten_line(ln) for ln in notes.split("\n") if ln.strip()]
    out = []
    used = len(header)
    for i, ln in enumerate(lines):
        trailer = "\n- … and %d more entries; see the full changelog linked above.\n" % (len(lines) - i)
        if used + len(ln) + 1 + len(trailer) > hard_limit:
            out.append(trailer.strip("\n"))
            break
        out.append(ln)
        used += len(ln) + 1
    return header + "\n".join(out) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("tag")
    ap.add_argument("--changelog", default="CHANGELOG.md")
    ap.add_argument("--output", help="write here instead of stdout")
    args = ap.parse_args(argv)
    try:
        version = version_of(args.tag)
        with open(args.changelog, encoding="utf-8") as f:
            notes = condense(extract(f.read(), version), version, args.tag.strip())
    except (ValueError, OSError) as e:
        print("release_notes: %s" % e, file=sys.stderr)
        return 1
    if args.output:
        with open(args.output, "w", encoding="utf-8", newline="\n") as f:
            f.write(notes)
    else:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stdout.write(notes)
    return 0


if __name__ == "__main__":
    sys.exit(main())
