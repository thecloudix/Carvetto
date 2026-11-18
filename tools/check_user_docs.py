#!/usr/bin/env python3
"""Check the user documentation under docs/user (T-0355).

1. Every relative Markdown link resolves to an existing file.
2. docs/user/README.md links every other page of the guide.
3. Every menu path written in bold, like **File > Import DXF...**, matches the menus in app/src: the first
   segment is a top-level menu (menuBar()->addMenu) and every segment is a label in the sources.
   Ampersand mnemonics are ignored. A trailing "(rotary)" style suffix is part of the label.
Usage: check_user_docs.py --root DIR
"""
import argparse
import re
import sys
from pathlib import Path

LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")
MENU_PATH = re.compile(r"\*\*([A-Za-z][^*]*? > [^*]+?)\*\*")
STRING = re.compile(r'(?:QStringLiteral|translate)\((?:[^"()]*"[^"]*",\s*)?"((?:[^"\\]|\\.)*)"')
TOP = re.compile(r'menuBar\(\)->addMenu\((?:[^"]*"[^"]*",\s*)?(?:QStringLiteral\()?"((?:[^"\\]|\\.)*)"')


def strip_mnemonic(s):
    return s.replace("&&", "\0").replace("&", "").replace("\0", "&")


def source_labels(root):
    labels, tops = set(), set()
    for p in sorted((root / "app" / "src").glob("*.cpp")):
        text = p.read_text(encoding="utf-8", errors="replace")
        for m in STRING.finditer(text):
            labels.add(strip_mnemonic(m.group(1)))
        for m in TOP.finditer(text):
            tops.add(strip_mnemonic(m.group(1)))
    return labels, tops


def check(root):
    errors = []
    docs = root / "docs" / "user"
    pages = sorted(docs.glob("*.md"))
    if not pages:
        return ["docs/user has no pages"]
    labels, tops = source_labels(root)
    index = docs / "README.md"
    linked = set()
    for page in pages:
        text = page.read_text(encoding="utf-8")
        rel = page.relative_to(root).as_posix()
        for m in LINK.finditer(text):
            target = m.group(1)
            if re.match(r"[a-z]+:", target) or target.startswith("#"):
                continue
            path = (page.parent / target.split("#", 1)[0]).resolve()
            if not path.is_file():
                errors.append("%s: broken link %s" % (rel, target))
            elif page == index:
                linked.add(path)
        for m in MENU_PATH.finditer(text):
            segs = [s.strip() for s in m.group(1).split(" > ")]
            if segs[0] not in tops:
                errors.append("%s: unknown top-level menu in '%s'" % (rel, m.group(1)))
            for seg in segs[1:]:
                if seg not in labels:
                    errors.append("%s: unknown menu entry '%s' in '%s'" % (rel, seg, m.group(1)))
    for page in pages:
        if page != index and page.resolve() not in linked:
            errors.append("README.md does not link %s" % page.name)
    return errors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    args = ap.parse_args()
    errors = check(Path(args.root).resolve())
    for e in errors:
        print(e)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
