#!/usr/bin/env python3
"""Check and assemble Carvetto's changelog fragments (changelog.d/).

Every task card adds its changelog entry as its own file in changelog.d/ instead of editing
CHANGELOG.md, so open pull requests no longer conflict with each other on the top of the file.

Fragment rules (enforced by `check`):
  - Name: changelog.d/T-XXXX.md for one card, or T-XXXX-YYYY.md (more ids allowed: T-XXXX-YYYY-ZZZZ.md)
    for an entry that covers several cards, ids strictly ascending. Every id must have a card
    ("### T-XXXX ...") in docs/BACKLOG.md. A card appears in at most one fragment.
  - Content: exactly one non-empty line (a final newline is optional; LF or CRLF), starting with the
    bullet prefix that matches the name: "- T-XXXX: " or, for several cards, "- T-XXXX/T-YYYY: ",
    followed by text. Same style as the existing CHANGELOG.md lines.
  - Characters: UTF-8 without BOM. Typographic characters used today (an em dash, <=, the degree
    sign, ...) are fine; control characters (tab included), invisible format characters (zero-width,
    bidi overrides, BOM), line/paragraph separators, private-use and unassigned code points, and
    leading/trailing whitespace are not.
  - changelog.d/README.md and dotfiles are ignored; any other file is an error.

Subcommands:
  check                 validate all fragments and that CHANGELOG.md has a "## Unreleased" heading.
  check --base <ref>    additionally compare against the merge base of <ref> and HEAD (used by CI on
                        pull requests): the PR must not edit CHANGELOG.md's Unreleased section by hand.
                        Allowed: every line the PR adds to the section is the line of a fragment the
                        same PR deletes (that is what `assemble` produces), and every line it removes
                        from the section still appears elsewhere in CHANGELOG.md (moved to a release
                        section); the lines kept stay in the same order.
  assemble              insert all fragments under "## Unreleased" (above the existing entries),
                        newest first by card number (the highest id of a multi-card fragment, ties
                        broken by the full id list, then the name), then delete the fragments.
                        Deterministic; idempotent (a fragment line already in the section is not
                        inserted twice, and with no fragments nothing changes).

Exit codes: 0 ok, 1 validation error (messages on stderr), 2 usage error.

Examples:
    python3 tools/changelog.py check
    python3 tools/changelog.py check --base origin/main
    python3 tools/changelog.py assemble
"""

import argparse
import pathlib
import re
import subprocess
import sys
import unicodedata

REPO = pathlib.Path(__file__).resolve().parent.parent
FRAGMENT_DIR = "changelog.d"
CHANGELOG = "CHANGELOG.md"
BACKLOG = "docs/BACKLOG.md"
UNRELEASED = "## Unreleased"
IGNORED = {"README.md"}

NAME_RE = re.compile(r"^T-(\d{4}(?:-\d{4})*)\.md$")
CARD_RE = re.compile(r"^### (T-\d{4})\b", re.MULTILINE)
# Unicode categories refused in a fragment line: control, format (zero-width, bidi, BOM),
# surrogate, private use, unassigned, line and paragraph separators.
BAD_CATEGORIES = {"Cc", "Cf", "Cs", "Co", "Cn", "Zl", "Zp"}


class Fragment:
    """One valid fragment file: its name, card ids (ascending) and its single line."""

    def __init__(self, name, ids, line):
        self.name = name
        self.ids = ids
        self.line = line

    def sort_key(self):
        # Newest first: highest card number first, then the full id list, then the name.
        return (-max(self.ids), [-i for i in reversed(self.ids)], self.name)


def prefix_for(ids):
    return "- " + "/".join("T-%04d" % i for i in ids) + ": "


def validate_fragment(name, data, known_cards=None):
    """Validate one fragment given its file name and raw bytes.

    Returns (Fragment or None, list of error strings)."""
    where = FRAGMENT_DIR + "/" + name
    m = NAME_RE.match(name)
    if not m:
        return None, ["%s: bad file name; expected T-XXXX.md or T-XXXX-YYYY.md (4-digit card ids)" % where]
    ids = [int(s) for s in m.group(1).split("-")]
    errors = []
    if any(b <= a for a, b in zip(ids, ids[1:])):
        errors.append("%s: card ids in the name must be distinct and ascending" % where)
    if known_cards is not None:
        for i in ids:
            if "T-%04d" % i not in known_cards:
                errors.append("%s: no card T-%04d in %s" % (where, i, BACKLOG))
    if data.startswith(b"\xef\xbb\xbf"):
        return None, errors + ["%s: starts with a UTF-8 byte order mark; save without BOM" % where]
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as e:
        return None, errors + ["%s: not valid UTF-8 (byte %d)" % (where, e.start)]
    if text.endswith("\r\n"):
        text = text[:-2]
    elif text.endswith("\n"):
        text = text[:-1]
    if text.strip() == "":
        return None, errors + ["%s: empty; it must hold exactly one bullet line" % where]
    if "\n" in text or "\r" in text:
        return None, errors + ["%s: more than one line; a fragment is exactly one bullet line "
                               "(no blank lines before or after it)" % where]
    for col, ch in enumerate(text, start=1):
        cat = unicodedata.category(ch)
        if cat in BAD_CATEGORIES:
            errors.append("%s: column %d: character U+%04X (%s) is not allowed"
                          % (where, col, ord(ch), unicodedata.name(ch, "category " + cat)))
            break
    if text != text.strip():
        errors.append("%s: leading or trailing whitespace" % where)
    expected = prefix_for(ids)
    if not text.startswith(expected):
        errors.append("%s: the line must start with %r (matching the file name), got %r"
                      % (where, expected, text[: len(expected) + 8]))
    elif text[len(expected):].strip() == "":
        errors.append("%s: no text after %r" % (where, expected.rstrip()))
    if errors:
        return None, errors
    return Fragment(name, ids, text), []


def known_cards_from(backlog_text):
    return set(CARD_RE.findall(backlog_text))


def check_fragment_set(entries, backlog_text):
    """Validate a set of fragments given as {name: bytes}. Returns (fragments, errors)."""
    known = known_cards_from(backlog_text) if backlog_text is not None else None
    fragments, errors = [], []
    for name in sorted(entries):
        if name in IGNORED or name.startswith("."):
            continue
        frag, errs = validate_fragment(name, entries[name], known)
        errors += errs
        if frag is not None:
            fragments.append(frag)
    owner = {}
    for frag in fragments:
        for i in frag.ids:
            if i in owner:
                errors.append("%s/%s: card T-%04d already has the fragment %s (one fragment per card)"
                              % (FRAGMENT_DIR, frag.name, i, owner[i]))
            else:
                owner[i] = frag.name
    fragments.sort(key=Fragment.sort_key)
    return fragments, errors


def read_fragment_dir(root):
    d = root / FRAGMENT_DIR
    if not d.is_dir():
        return None
    out = {}
    for p in d.iterdir():
        if p.is_dir():
            if not p.name.startswith("."):
                out[p.name] = None
            continue
        out[p.name] = p.read_bytes()
    return out


def split_lines(text):
    """Split into lines without terminators; also report whether the text used CRLF."""
    crlf = "\r\n" in text
    lines = text.replace("\r\n", "\n").split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    return lines, crlf


def unreleased_bounds(lines):
    """Index of the '## Unreleased' heading and the end (exclusive) of its section, or None."""
    for i, line in enumerate(lines):
        if line.rstrip() == UNRELEASED:
            end = len(lines)
            for j in range(i + 1, len(lines)):
                if lines[j].startswith("## ") or lines[j].startswith("# "):
                    end = j
                    break
            return i, end
    return None


def section_entries(lines):
    """Non-blank lines of the Unreleased section (None if there is no such section)."""
    b = unreleased_bounds(lines)
    if b is None:
        return None
    return [ln for ln in lines[b[0] + 1:b[1]] if ln.strip() != ""]


def assemble_text(changelog_text, fragments):
    """Return CHANGELOG text with the fragment lines inserted (pure function)."""
    lines, crlf = split_lines(changelog_text)
    b = unreleased_bounds(lines)
    if b is None:
        raise ValueError("%s has no '%s' heading" % (CHANGELOG, UNRELEASED))
    head, end = b
    present = set(lines[head + 1:end])
    new = [f.line for f in fragments if f.line not in present]
    if new:
        first = head + 1
        while first < end and lines[first].strip() == "":
            first += 1
        insert = [""] + new
        if first == end:  # empty section: keep a blank line before the next heading
            insert += [""] if end < len(lines) else []
        lines = lines[:head + 1] + insert + lines[first:]
    nl = "\r\n" if crlf else "\n"
    return nl.join(lines) + nl


def cmd_check(root, base):
    errors = []
    entries = read_fragment_dir(root)
    if entries is None:
        errors.append("%s/ is missing (it holds README.md and the fragments)" % FRAGMENT_DIR)
        entries = {}
    for name, data in entries.items():
        if data is None:
            errors.append("%s/%s: unexpected directory" % (FRAGMENT_DIR, name))
    backlog = root / BACKLOG
    backlog_text = backlog.read_text(encoding="utf-8") if backlog.is_file() else None
    fragments, errs = check_fragment_set({k: v for k, v in entries.items() if v is not None},
                                         backlog_text)
    errors += errs
    changelog = root / CHANGELOG
    if not changelog.is_file():
        errors.append("%s is missing" % CHANGELOG)
    elif section_entries(split_lines(changelog.read_text(encoding="utf-8"))[0]) is None:
        errors.append("%s has no '%s' heading" % (CHANGELOG, UNRELEASED))
    if base is not None and not errors:
        errors += check_against_base(root, base)
    for e in errors:
        print("error: " + e, file=sys.stderr)
    if errors:
        print("changelog check failed (%d error%s); see changelog.d/README.md"
              % (len(errors), "" if len(errors) == 1 else "s"), file=sys.stderr)
        return 1
    print("changelog check ok: %d fragment%s%s" % (len(fragments), "" if len(fragments) == 1 else "s",
                                                   "" if base is None else ", no direct edits to the "
                                                   "Unreleased section"))
    return 0


def git(root, *args):
    r = subprocess.run(["git", *args], cwd=str(root), capture_output=True)
    if r.returncode != 0:
        raise RuntimeError("git %s failed: %s" % (" ".join(args), r.stderr.decode("utf-8", "replace").strip()))
    return r.stdout


def fragments_at(root, rev):
    out = git(root, "ls-tree", "--name-only", rev, FRAGMENT_DIR + "/").decode("utf-8")
    names = [ln.split("/", 1)[1] for ln in out.splitlines() if "/" in ln]
    return {n: git(root, "show", "%s:%s/%s" % (rev, FRAGMENT_DIR, n)) for n in names
            if n not in IGNORED and not n.startswith(".")}


def check_against_base(root, base):
    """Errors if HEAD edits the Unreleased section other than by assembling deleted fragments."""
    try:
        mb = git(root, "merge-base", base, "HEAD").decode("ascii").strip()
        base_text = git(root, "show", "%s:%s" % (mb, CHANGELOG)).decode("utf-8")
        base_frags = fragments_at(root, mb)
    except (RuntimeError, UnicodeDecodeError) as e:
        return ["cannot compare with %s: %s" % (base, e)]
    head_lines = split_lines((root / CHANGELOG).read_text(encoding="utf-8"))[0]
    base_lines = split_lines(base_text)[0]
    base_entries = section_entries(base_lines) or []
    head_entries = section_entries(head_lines) or []
    head_names = set((read_fragment_dir(root) or {}).keys())
    deleted = set()
    for name, data in base_frags.items():
        if name not in head_names:
            frag, _ = validate_fragment(name, data)
            if frag is not None:
                deleted.add(frag.line)
    hb = unreleased_bounds(head_lines)
    elsewhere = set(head_lines[:hb[0]] + head_lines[hb[1]:]) if hb else set(head_lines)
    errors = []
    base_set, head_set = set(base_entries), set(head_entries)
    for line in head_entries:
        if line not in base_set and line not in deleted:
            errors.append("%s: line added to '%s' by hand (write a fragment in %s/ instead): %s"
                          % (CHANGELOG, UNRELEASED, FRAGMENT_DIR, short(line)))
    for line in base_entries:
        if line not in head_set and line not in elsewhere:
            errors.append("%s: line removed or edited in '%s' (existing entries are not edited): %s"
                          % (CHANGELOG, UNRELEASED, short(line)))
    kept_head = [ln for ln in head_entries if ln in base_set]
    kept_base = [ln for ln in base_entries if ln in head_set]
    if kept_head != kept_base:
        errors.append("%s: existing '%s' lines were reordered or duplicated" % (CHANGELOG, UNRELEASED))
    return errors


def short(line):
    return line if len(line) <= 90 else line[:87] + "..."


def cmd_assemble(root):
    entries = read_fragment_dir(root) or {}
    backlog = root / BACKLOG
    backlog_text = backlog.read_text(encoding="utf-8") if backlog.is_file() else None
    fragments, errors = check_fragment_set({k: v for k, v in entries.items() if v is not None},
                                           backlog_text)
    changelog = root / CHANGELOG
    if not changelog.is_file():
        errors.append("%s is missing" % CHANGELOG)
    if errors:
        for e in errors:
            print("error: " + e, file=sys.stderr)
        print("nothing assembled; fix the fragments first", file=sys.stderr)
        return 1
    with open(changelog, "r", encoding="utf-8", newline="") as f:
        old = f.read()
    try:
        new = assemble_text(old, fragments)
    except ValueError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1
    if new != old:
        with open(changelog, "w", encoding="utf-8", newline="") as f:
            f.write(new)
    for frag in fragments:  # only after CHANGELOG.md is written
        (root / FRAGMENT_DIR / frag.name).unlink()
    print("assembled %d fragment%s into %s" % (len(fragments), "" if len(fragments) == 1 else "s",
                                               CHANGELOG))
    for frag in fragments:
        print("  " + frag.name)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog="See the module docstring (or changelog.d/README.md) for the rules.")
    parser.add_argument("--root", default=str(REPO), help="repository root (default: this checkout)")
    sub = parser.add_subparsers(dest="command", required=True)
    p_check = sub.add_parser("check", help="validate the fragments")
    p_check.add_argument("--base", metavar="REF",
                         help="also refuse hand edits to the Unreleased section since merge-base(REF, HEAD)")
    sub.add_parser("assemble", help="move the fragments into CHANGELOG.md and delete them")
    args = parser.parse_args(argv)
    root = pathlib.Path(args.root).resolve()
    if args.command == "check":
        return cmd_check(root, args.base)
    return cmd_assemble(root)


if __name__ == "__main__":
    sys.exit(main())
