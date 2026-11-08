#!/usr/bin/env python3
"""Windows-safe names lint (T-0182).

<windows.h> defines macros (far, near, small, interface, IN, OUT, ERROR, min, max, ...) and GDI
declarations (Polyline, Rectangle, Ellipse, Arc, ...). We cannot compile MSVC locally, so this checks
C++ sources under core/, app/ and tools/ for tokens that break there. Only files that can see
<windows.h> are checked: those including it (directly, or via cpu_time.hpp / cpu_scaling.hpp) and
headers under tests/ (shared helpers included by them).

  python3 -I tools/check_windows_names.py [--root DIR] [--allowlist FILE] [paths...]

Escape hatch: `// lint-allow-windows-name` on the line, or a "path:line" / "path" entry in the
allowlist file (tools/windows_names_allowlist.txt). Exit status 1 when violations are found.
"""
import os
import re
import sys

MACRO_NAMES = {"far", "near", "small", "interface", "IN", "OUT", "ERROR"}
GDI_NAMES = {"Polyline", "Polygon", "Rectangle", "Ellipse", "Arc", "Pie", "Chord", "Point", "Rect",
             "Region", "Path", "Pen", "Brush", "Font", "Bitmap", "Icon", "Cursor", "Window", "Menu",
             "Dialog", "Message", "Event"}
WIN_INCLUDE = re.compile(r'#\s*include\s*[<"][^>"]*(cpu_time\.hpp|cpu_scaling\.hpp|windows\.h)[>"]', re.I)
ALLOW = "lint-allow-windows-name"
EXTS = (".cpp", ".hpp", ".h", ".cc", ".cxx", ".hxx", ".inl")
SKIP_DIRS = {".git", "build", "_deps", "third_party", "thirdparty", "node_modules", "fixtures", "golden"}


def strip(text):
    """Replace comments and string/char literals by spaces (newlines kept). Returns (code, allow_lines)."""
    out = []
    allow = set()
    i, n, line = 0, len(text), 1
    raw_re = re.compile(r'R"([^(\s]*)\(')
    while i < n:
        c = text[i]
        if c == "/" and text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            if ALLOW in text[i:j]:
                allow.add(line)
            out.append(" " * (j - i))
            i = j
        elif c == "/" and text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            seg = text[i:j]
            if ALLOW in seg:
                allow.add(line)
            out.append(re.sub(r"[^\n]", " ", seg))
            line += seg.count("\n")
            i = j
        elif c == "R" and text.startswith('R"', i) and raw_re.match(text, i) and (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_")):
            m = raw_re.match(text, i)
            end = text.find(")" + m.group(1) + '"', m.end())
            j = n if end < 0 else end + len(m.group(1)) + 2
            seg = text[i:j]
            out.append(re.sub(r"[^\n]", " ", seg))
            line += seg.count("\n")
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            # digit separator 1'000: a quote right after a digit is not a char literal
            if c == "'" and i > 0 and text[i - 1].isalnum() and not text[i - 1].isalpha():
                out.append(c)
                i += 1
                continue
            out.append(" " * (j - i))
            i = j
        else:
            if c == "\n":
                line += 1
            out.append(c)
            i += 1
    return "".join(out), allow


TOKEN = re.compile(r"[A-Za-z_][A-Za-z_0-9]*")


_HELPER = {}
ROOT = os.getcwd()


def _helper_defines(name):
    if "t" not in _HELPER:
        try:
            with open(os.path.join(ROOT, "core/tests/geometry/cpu_time.hpp"), encoding="utf-8") as fh:
                _HELPER["t"] = fh.read()
        except OSError:
            _HELPER["t"] = ""
    return re.search(r"#\s*define\s+" + name + r"\b", _HELPER["t"]) is not None


def scan(path, text):
    code, allow = strip(text)
    # Where <windows.h> can be in effect: files that include it (directly, or via cpu_time.hpp /
    # cpu_scaling.hpp) and shared headers under tests/ (included by such files). By convention those
    # includes come last, so other files are unaffected.
    shared_test_header = path.endswith((".hpp", ".h")) and "/tests/" in "/" + path
    inc = [l for l in text.splitlines() if l.lstrip().startswith("#")]
    direct_win = any(re.search(r'#\s*include\s*[<"]windows\.h[>"]', l, re.I) for l in inc)
    via_helper = any(WIN_INCLUDE.search(l) for l in inc) and not direct_win
    # cpu_time.hpp defines WIN32_LEAN_AND_MEAN (no `small`) and NOGDI (no Polyline & co) before
    # <windows.h>; a direct include has neither.
    helper_safe_small = _helper_defines("WIN32_LEAN_AND_MEAN")
    helper_safe_gdi = _helper_defines("NOGDI")
    win_any = direct_win or via_helper or shared_test_header
    check_small = direct_win or ((via_helper or shared_test_header) and not helper_safe_small)
    check_gdi = direct_win or ((via_helper or shared_test_header) and not helper_safe_gdi)
    hits = []
    for ln, l in enumerate(code.split("\n"), 1):
        if ln in allow:
            continue
        for m in TOKEN.finditer(l):
            tok = m.group(0)
            before = l[:m.start()].rstrip()
            after = l[m.end():].lstrip()
            qualified = before.endswith("::") or before.endswith(".") or before.endswith("->")
            if tok.startswith("#") or before.endswith("#"):
                continue
            if not win_any:
                continue
            if tok in MACRO_NAMES and not qualified and (tok != "small" or check_small):
                hits.append((ln, tok, "Windows macro"))
            elif tok in ("min", "max") and after.startswith("(") and not qualified:
                if before.endswith("(") and after.startswith(")"):
                    continue
                if re.search(r"\(\s*$", before) and False:
                    continue
                hits.append((ln, tok, "min/max macro call; write std::" + tok + " or (std::" + tok + ")"))
            elif check_gdi and tok in GDI_NAMES and not qualified:
                hits.append((ln, tok, "GDI name unqualified in a file that pulls in <windows.h>"))
    return hits


def iter_files(root, paths):
    for p in paths:
        full = os.path.join(root, p)
        if os.path.isfile(full):
            yield p
            continue
        for d, dirs, files in os.walk(full):
            dirs[:] = sorted(x for x in dirs if x not in SKIP_DIRS and not x.startswith("build") and not x.startswith("."))
            for f in sorted(files):
                if f.endswith(EXTS):
                    yield os.path.relpath(os.path.join(d, f), root).replace(os.sep, "/")


def load_allow(path):
    entries = set()
    if path and os.path.isfile(path):
        with open(path, encoding="utf-8") as fh:
            for l in fh:
                l = l.split("#", 1)[0].strip()
                if l:
                    entries.add(l)
    return entries


def main(argv):
    root = os.getcwd()
    allow_file = None
    paths = []
    it = iter(argv)
    for a in it:
        if a == "--root":
            root = next(it)
        elif a == "--allowlist":
            allow_file = next(it)
        else:
            paths.append(a)
    if allow_file is None:
        allow_file = os.path.join(root, "tools", "windows_names_allowlist.txt")
    if not paths:
        paths = [p for p in ("core", "app", "tools") if os.path.isdir(os.path.join(root, p))]
    global ROOT
    ROOT = root
    allowed = load_allow(allow_file)
    bad = 0
    for rel in iter_files(root, paths):
        with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        for ln, tok, why in scan(rel, text):
            if rel in allowed or "%s:%d" % (rel, ln) in allowed:
                continue
            print("%s:%d: '%s': %s" % (rel, ln, tok, why))
            bad += 1
    if bad:
        print("%d Windows-name violation(s); see CONTRIBUTING.md 'Windows-safe names'" % bad)
        return 1
    print("windows names ok")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
