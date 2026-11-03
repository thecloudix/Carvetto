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
  check                 validate all fragments, that docs/BACKLOG.md exists and that CHANGELOG.md is
                        UTF-8 text (no BOM, no NUL) with exactly one byte-exact "## Unreleased" line.
  check --base <ref>    additionally compare CHANGELOG.md, as raw bytes, with its version at the merge
                        base of <ref> and HEAD (CI on pull requests). HEAD must equal the merge base
                        byte for byte after exactly one of these changes, and nothing else:
                          1. none (an ordinary PR). It may add fragments; it may delete fragments only
                             if it also adds at least one (a rename or a merge of fragments);
                          2. assemble: the lines of all fragments the PR deletes, each once, in
                             assemble's order, inserted at the top of Unreleased exactly as `assemble`
                             does; the PR adds no fragment;
                          3. release (no fragment deleted): one "## X.Y.Z" or "## X.Y.Z - YYYY-MM-DD"
                             heading for a version not yet in the file, inserted at the top of a
                             non-empty Unreleased section followed by a blank line, so all former
                             Unreleased lines now belong to the release and Unreleased is empty.
                        Line endings count as bytes: converting the file's line endings is refused
                        too (the repository keeps CHANGELOG.md and changelog.d/ as LF, .gitattributes).
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
import os
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

NAME_RE = re.compile(r"^T-([0-9]{4}(?:-[0-9]{4})*)\.md$")
CARD_RE = re.compile(r"^### (T-[0-9]{4})(?![0-9])", re.MULTILINE)
# A release heading: '## X.Y.Z' or '## X.Y.Z - YYYY-MM-DD' (ASCII digits).
VERSION_RE = re.compile(r"^## ([0-9]+\.[0-9]+\.[0-9]+)( - [0-9]{4}-[0-9]{2}-[0-9]{2})?$")
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


def read_backlog(root):
    """docs/BACKLOG.md text (fragment ids are checked against its cards), or an error."""
    backlog = root / BACKLOG
    if not backlog.is_file():
        return "", ["%s is missing (fragment card ids are checked against it)" % BACKLOG]
    return backlog.read_text(encoding="utf-8"), []


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


def split_raw(text):
    """Split into lines that keep their terminators ("\\n" or "\\r\\n"; a lone "\\r" stays inside)."""
    return re.findall(r"[^\n]*\n|[^\n]+$", text)


def content(line):
    """A raw line without its terminator."""
    if line.endswith("\r\n"):
        return line[:-2]
    return line[:-1] if line.endswith("\n") else line


def is_blank(line):
    return content(line).strip() == ""


def is_heading(line):
    c = content(line)
    return c.startswith("## ") or c.startswith("# ")


def decode_changelog(data, what):
    """(text, None) or (None, error) for raw CHANGELOG.md bytes."""
    if data.startswith(b"\xef\xbb\xbf"):
        return None, "%s starts with a UTF-8 byte order mark" % what
    if b"\x00" in data:
        return None, "%s contains NUL bytes (binary?)" % what
    try:
        return data.decode("utf-8"), None
    except UnicodeDecodeError as e:
        return None, "%s is not valid UTF-8 (byte %d)" % (what, e.start)


def unreleased_section(lines):
    """(heading index, section end) of the one byte-exact '## Unreleased' heading; ValueError otherwise."""
    idx = [i for i, ln in enumerate(lines) if content(ln) == UNRELEASED]
    if len(idx) != 1:
        raise ValueError("%s must have exactly one '%s' heading (byte-exact), found %d"
                         % (CHANGELOG, UNRELEASED, len(idx)))
    h = idx[0]
    end = next((j for j in range(h + 1, len(lines)) if is_heading(lines[j])), len(lines))
    return h, end


def insert_below_unreleased(lines, new, release=False):
    """The lines with `new` (strings without terminators) inserted at the top of Unreleased.

    A pure insertion: no existing byte changes, except that a heading at the very end of the
    file without a newline gets one. Layout: '## Unreleased', one blank line, the new lines, then
    the former entries; in an empty section a blank line separates them from the next heading.
    With release=True (`new` is one version heading) a blank line always follows it, so the former
    entries become the release's section. The inserted lines use the heading's line ending."""
    h, end = unreleased_section(lines)
    lines = list(lines)
    if not lines[h].endswith("\n"):
        lines[h] += "\n"
    nl = "\r\n" if lines[h].endswith("\r\n") else "\n"
    first = next((j for j in range(h + 1, end) if not is_blank(lines[j])), None)
    if first is not None:
        k = first
    else:
        k = h + 2 if h + 1 < len(lines) and is_blank(lines[h + 1]) else h + 1
    ins = [nl] if k == h + 1 else []
    ins += [c + nl for c in new]
    if (release or first is None) and k < len(lines) and not is_blank(lines[k]):
        ins.append(nl)
    return lines[:k] + ins + lines[k:]


def section_entries(lines):
    h, end = unreleased_section(lines)
    return [content(ln) for ln in lines[h + 1:end] if not is_blank(ln)]


def assemble_text(changelog_text, fragments):
    """CHANGELOG text with the fragment lines inserted (pure; lines already there are skipped)."""
    lines = split_raw(changelog_text)
    present = set(section_entries(lines))
    new = [f.line for f in fragments if f.line not in present]
    return "".join(insert_below_unreleased(lines, new)) if new else changelog_text


def read_changelog(root):
    """(text, None) or (None, error) for the working tree's CHANGELOG.md, read as raw bytes."""
    path = root / CHANGELOG
    if not path.is_file():
        return None, "%s is missing" % CHANGELOG
    try:
        data = path.read_bytes()
    except OSError as e:
        return None, "cannot read %s: %s" % (CHANGELOG, e)
    return decode_changelog(data, CHANGELOG)


def cmd_check(root, base):
    errors = []
    entries = read_fragment_dir(root)
    if entries is None:
        errors.append("%s/ is missing (it holds README.md and the fragments)" % FRAGMENT_DIR)
        entries = {}
    for name, data in entries.items():
        if data is None:
            errors.append("%s/%s: unexpected directory" % (FRAGMENT_DIR, name))
    backlog_text, err = read_backlog(root)
    errors += err
    fragments, errs = check_fragment_set({k: v for k, v in entries.items() if v is not None},
                                         backlog_text)
    errors += errs
    text, err = read_changelog(root)
    if err:
        errors.append(err)
    else:
        try:
            unreleased_section(split_raw(text))
        except ValueError as e:
            errors.append(str(e))
    if base is not None and not errors:
        errors += check_against_base(root, base, text)
    for e in errors:
        print("error: " + e, file=sys.stderr)
    if errors:
        print("changelog check failed (%d error%s); see changelog.d/README.md"
              % (len(errors), "" if len(errors) == 1 else "s"), file=sys.stderr)
        return 1
    print("changelog check ok: %d fragment%s%s" % (len(fragments), "" if len(fragments) == 1 else "s",
                                                   "" if base is None else ", no hand edits to "
                                                   "CHANGELOG.md"))
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


def first_difference(a, b):
    """1-based line number and the HEAD line where two raw line lists first differ."""
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return i + 1, y
    n = min(len(a), len(b))
    return n + 1, (b[n] if n < len(b) else "<end of file>")


def release_candidate(base_lines, head_lines):
    """The version heading text HEAD inserted where a release PR puts it, or None."""
    probe = insert_below_unreleased(base_lines, ["\0"], release=True)
    k = next(i for i, ln in enumerate(probe) if content(ln) == "\0")
    return content(head_lines[k]) if k < len(head_lines) else None


def check_against_base(root, base, head_text):
    """Errors unless HEAD's CHANGELOG.md is the merge base's, byte for byte, after exactly one of:
      - nothing (any PR; deleting fragments then requires adding at least one, i.e. a rename/merge);
      - assemble: the lines of all fragments the PR deletes, each once, in assemble's order, at the
        top of Unreleased (see insert_below_unreleased); the PR adds no fragment;
      - release (no fragment deleted): one '## X.Y.Z' or '## X.Y.Z - YYYY-MM-DD' heading, for a
        version not yet in the file, inserted at the top of a non-empty Unreleased section, so all
        its former lines now belong to the release and Unreleased is empty.
    """
    try:
        mb = git(root, "merge-base", base, "HEAD").decode("ascii").strip()
        base_data = git(root, "show", "%s:%s" % (mb, CHANGELOG))
        base_frags = fragments_at(root, mb)
    except RuntimeError as e:
        return ["cannot compare with %s: %s" % (base, e)]
    base_text, err = decode_changelog(base_data, "%s at the merge base" % CHANGELOG)
    if err:
        return [err]
    head_entries = read_fragment_dir(root) or {}
    deleted, errors = [], []
    for name in sorted(base_frags):
        if name not in head_entries:
            frag, errs = validate_fragment(name, base_frags[name])
            if frag is None:
                errors.append("%s/%s was deleted but is not a valid fragment at the merge base: %s"
                              % (FRAGMENT_DIR, name, "; ".join(errs)))
            else:
                deleted.append(frag)
    if errors:
        return errors
    added = [n for n in head_entries if n not in base_frags and n not in IGNORED and not n.startswith(".")]
    deleted.sort(key=Fragment.sort_key)
    names = ", ".join(f.name for f in deleted)
    if head_text == base_text:
        if deleted and not added:
            return ["%s deleted without being assembled into %s (and no fragment added in its place); "
                    "keep it, or run tools/changelog.py assemble" % (names, CHANGELOG)]
        return []
    base_lines, head_lines = split_raw(base_text), split_raw(head_text)
    try:
        unreleased_section(base_lines)
    except ValueError as e:
        return ["at the merge base: %s" % e]
    if deleted:
        if added:
            return ["%s changed together with adding %s: an assemble PR only assembles and deletes "
                    "fragments; add new fragments in another PR" % (CHANGELOG, ", ".join(sorted(added)))]
        present = set(section_entries(base_lines))
        dup = [f.name for f in deleted if f.line in present]
        if dup:
            return ["the line of %s is already in '%s' at the merge base; delete the fragment without "
                    "touching %s" % (", ".join(dup), UNRELEASED, CHANGELOG)]
        expected = insert_below_unreleased(base_lines, [f.line for f in deleted])
        if head_lines == expected:
            return []
        n, line = first_difference(expected, head_lines)
        return ["%s is not what tools/changelog.py assemble makes of the fragments this PR deletes (%s); "
                "first difference at line %d: %s. Never edit %s by hand"
                % (CHANGELOG, names, n, short(content(line)), CHANGELOG)]
    version = release_candidate(base_lines, head_lines)
    m = VERSION_RE.match(version or "")
    if m:
        known = {mm.group(1) for mm in (VERSION_RE.match(content(ln)) for ln in base_lines) if mm}
        if m.group(1) in known:
            return ["%s: version %s is already in the file" % (CHANGELOG, m.group(1))]
        if not section_entries(base_lines):
            return ["%s: release %s with an empty '%s' section" % (CHANGELOG, m.group(1), UNRELEASED)]
        expected = insert_below_unreleased(base_lines, [version], release=True)
        if head_lines == expected:
            return []
    else:
        expected = base_lines
    n, line = first_difference(expected, head_lines)
    return ["%s edited by hand (first difference at line %d: %s). Only two changes are accepted: "
            "tools/changelog.py assemble together with deleting the fragments, or a release that "
            "inserts one '## X.Y.Z' heading (optionally ' - YYYY-MM-DD') right below '%s'; "
            "write a fragment in %s/ instead" % (CHANGELOG, n, short(content(line)), UNRELEASED, FRAGMENT_DIR)]


def short(line):
    return line if len(line) <= 90 else line[:87] + "..."


def cmd_assemble(root):
    entries = read_fragment_dir(root) or {}
    errors = []
    backlog_text, err = read_backlog(root)
    errors += err
    fragments, errs = check_fragment_set({k: v for k, v in entries.items() if v is not None},
                                         backlog_text)
    errors += errs
    changelog = root / CHANGELOG
    old, err = read_changelog(root)
    if err:
        errors.append(err)
    if errors:
        for e in errors:
            print("error: " + e, file=sys.stderr)
        print("nothing assembled; fix the fragments first", file=sys.stderr)
        return 1
    try:
        new = assemble_text(old, fragments)
    except ValueError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1
    try:
        if new != old:
            if not os.access(str(changelog), os.W_OK):
                raise OSError("%s is not writable" % changelog)
            tmp = changelog.with_name(CHANGELOG + ".tmp")
            try:
                with open(tmp, "wb") as f:
                    f.write(new.encode("utf-8"))
                os.replace(str(tmp), str(changelog))  # atomic: never a half-written CHANGELOG.md
            finally:
                if tmp.exists():
                    tmp.unlink()
        for frag in fragments:  # only after CHANGELOG.md is written
            (root / FRAGMENT_DIR / frag.name).unlink()
    except OSError as e:
        print("error: %s" % e, file=sys.stderr)
        print("assemble did not finish; run it again after fixing this (it is idempotent)", file=sys.stderr)
        return 1
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
    for stream in (sys.stdout, sys.stderr):  # e.g. a cp1252 Windows console: never UnicodeEncodeError
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="backslashreplace")
    root = pathlib.Path(args.root).resolve()
    if args.command == "check":
        return cmd_check(root, args.base)
    return cmd_assemble(root)


if __name__ == "__main__":
    sys.exit(main())
