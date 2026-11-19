#!/usr/bin/env python3
"""Extract the release notes for a version tag from CHANGELOG.md (T-0361).

  python3 -I tools/release_notes.py v0.1.0 [--changelog CHANGELOG.md] [--output FILE]

Finds the `## X.Y.Z` heading (optionally followed by a space and a date or other text) and prints
everything up to the next `## ` heading, with surrounding blank lines trimmed. Exits 1 with a
message on stderr if the tag is not a version tag, the section is missing, or it is empty.
Standard library only.
"""

import argparse
import re
import sys

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
        raise ValueError("CHANGELOG.md has no '## %s' section" % version)
    end = len(lines)
    for j in range(start, len(lines)):
        if lines[j].startswith("## "):
            end = j
            break
    body = "\n".join(lines[start:end]).strip("\n")
    if not body.strip():
        raise ValueError("the '## %s' section is empty" % version)
    return body + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("tag")
    ap.add_argument("--changelog", default="CHANGELOG.md")
    ap.add_argument("--output", help="write here instead of stdout")
    args = ap.parse_args(argv)
    try:
        version = version_of(args.tag)
        with open(args.changelog, encoding="utf-8") as f:
            notes = extract(f.read(), version)
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
