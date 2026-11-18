#!/usr/bin/env python3
"""Lint: user-visible string literals in app/src must go through tr() (T-0350).

A literal "bypasses tr()" when a UI call that shows text (addMenu, addAction, setWindowTitle, setText,
setToolTip, QLabel(...), ...) is given QStringLiteral("...") or a bare "..." containing a letter, on a
line with no tr( / translate( / QT_TR_NOOP. Existing code is grandfathered by tools/tr_literals_baseline.txt
(one `path count` per line): a file may never have more offending lines than its baseline, and files
without an entry must have none. The sweep T-0351 drives the baseline to empty.
Usage: check_tr_literals.py --root DIR [--print-baseline]
"""
import argparse
import re
import sys
from pathlib import Path

CALLS = (r"addMenu|addAction|addSection|setWindowTitle|setText|setToolTip|setStatusTip|setWhatsThis|"
         r"setPlaceholderText|setTitle|setLabelText|addRow|setHeaderLabels|setHorizontalHeaderLabels|"
         r"QLabel|QPushButton|QCheckBox|QRadioButton|QGroupBox|QAction|addTab|QMessageBox::\w+|"
         r"getOpenFileName|getSaveFileName|getExistingDirectory|showMessage")
LITERAL = r'(?:QStringLiteral\(\s*"[^"]*[A-Za-z][^"]*"\s*\)|QLatin1String\(\s*"[^"]*[A-Za-z][^"]*"\s*\))'
OFFENDING = re.compile(r"\b(?:%s)\s*\([^;]*?%s" % (CALLS, LITERAL))
SAFE = re.compile(r"\btr\(|\btranslate\(|QT_TR_NOOP|QT_TRANSLATE_NOOP|lint:\s*no-tr")


def offending_lines(path):
    n = 0
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        code = line.split("//", 1)[0]
        if OFFENDING.search(code) and not SAFE.search(line):
            n += 1
    return n


def scan(root):
    counts = {}
    for p in sorted((root / "app" / "src").glob("*.cpp")):
        if p.name.endswith("self_test.cpp"):
            continue
        n = offending_lines(p)
        if n:
            counts[p.relative_to(root).as_posix()] = n
    return counts


def load_baseline(path):
    base = {}
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                name, num = line.rsplit(" ", 1)
                base[name] = int(num)
    return base


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--print-baseline", action="store_true")
    a = ap.parse_args(argv)
    root = Path(a.root)
    counts = scan(root)
    if a.print_baseline:
        for k, v in counts.items():
            print(k, v)
        return 0
    base = load_baseline(root / "tools" / "tr_literals_baseline.txt")
    bad = [(k, v, base.get(k, 0)) for k, v in counts.items() if v > base.get(k, 0)]
    for k, v, b in bad:
        print("%s: %d user-visible literal(s) bypass tr() (baseline %d): wrap them in "
              "QCoreApplication::translate(\"Context\", \"text\")" % (k, v, b), file=sys.stderr)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
