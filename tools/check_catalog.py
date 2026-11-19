#!/usr/bin/env python3
"""Lint: the translation catalog lists every user-visible source string (T-0351).

Every QCoreApplication::translate("Context", "text") and QT_TRANSLATE_NOOP("Context", "text") with
literal arguments in app/src must appear in app/translations/carvetto_<lang>.ts under that context
(run the `update_translations` target after adding or changing a string). Self-test files are
skipped: they use made-up strings on purpose. Usage: check_catalog.py --root DIR
"""
import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

STRING = r'"(?:[^"\\]|\\.)*"'
CALL = re.compile(r'(?:\bQCoreApplication::translate|\bQT_TRANSLATE_NOOP)\(\s*"(\w+)"\s*,\s*((?:%s\s*)+)' % STRING)
PIECE = re.compile(STRING)
ESCAPES = {"n": "\n", "t": "\t", '"': '"', "\\": "\\", "'": "'"}


def unescape(piece):
    body = piece[1:-1]
    return re.sub(r"\\(.)", lambda m: ESCAPES.get(m.group(1), m.group(1)), body)


def sources(root):
    found = set()
    for path in sorted((root / "app" / "src").glob("*.cpp")):
        if path.name.endswith("self_test.cpp"):
            continue
        text = re.sub(r"(?m)//[^\n]*$", "", path.read_text(encoding="utf-8", errors="replace"))
        for m in CALL.finditer(text):
            found.add((m.group(1), "".join(unescape(p) for p in PIECE.findall(m.group(2)))))
    return found


def catalog(path):
    entries = set()
    for ctx in ET.parse(path).getroot().iter("context"):
        name = ctx.findtext("name")
        for msg in ctx.iter("message"):
            entries.add((name, msg.findtext("source") or ""))
    return entries


def missing(root):
    out = {}
    for ts in sorted((root / "app" / "translations").glob("*.ts")):
        have = catalog(ts)
        out[ts.name] = sorted(sources(root) - have)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    a = ap.parse_args(argv)
    bad = 0
    for name, items in missing(Path(a.root)).items():
        for ctx, text in items:
            print("%s: [%s] %r is not in the catalog; run the update_translations target" % (name, ctx, text),
                  file=sys.stderr)
        bad += len(items)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
