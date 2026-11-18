#!/usr/bin/env python3
"""Compliance audit (T-0356): clean-room scan and licence/dependency inventory checks.

  python3 -I tools/check_compliance.py [--root DIR] [--allowlist FILE]
  python3 -I tools/check_compliance.py --hash WORD      # print the entry to add to FORBIDDEN

Checks (exit status 1 when any fails):
  1. No file name or text file under the repo contains a forbidden third-party product or
     proprietary-format token (CONTRIBUTING.md clean-room policy, docs/PLAN.md section 7).
     Factual compatibility claims are allowlisted in tools/compliance_allowlist.txt as
     "path<TAB>token-hash-or-*<TAB>reason" lines; every entry needs a reason.
  2. Every FetchContent_Declare in a CMakeLists.txt has a row in THIRD_PARTY_NOTICES.md.
  3. LICENSE exists and each bundled font/data licence file referenced in the notices exists.

The forbidden list is stored as truncated SHA-256 digests of lower-case tokens (letters and digits;
adjacent capitalised token pairs separated by one space are also joined, so two-word names are caught), so that this file does not itself
mention any of the names (ADR-0118). Hits are reported by path, line and hash, never the word.
"""
import argparse
import hashlib
import os
import re
import subprocess
import sys

FORBIDDEN = {
    "1a48a417c938287ae0cb",
    "716c618208fce71c1da0",
    "bfe0a8205d1917c9f109",
    "89bf545cb7f2cbdbff9f",
    "30231f09636cf73fb5f3",
    "d1193d3ec96c64bafe4c",
    "959ff14564cfae3c4895",
    "cb43a9848f1fa47c4167",
    "56381cf8b1d5bbe77620",
    "da7d404340102e978d87",
    "69a11ff2a38f377b3e43",
    "c690c6c8148b27b184ac",
    "320d597682ed208cdaa5",
    "e6d162f60c9a3ea80af3",
    "8ab353529558609496f0",
    "2472a6a8a5b3dc8dff82",
    "648f4c306f130e0e48d6",
    "6dc5134a14b1cac34ed6",
    "d6c33dbbb6bd75f28f32",
    "7f1f6de9e6f99e16e416",
    "93f39fc83bd973a77914",
    "c81291d2961580309b79",
    "ef5ae9bfa6c07dd0741b",
    "773aa1e91538d3792cdf",
    "9b90a9496047ba136fab",
}
SKIP_DIRS = {".git", ".claude", ".fc-src", "node_modules", "__pycache__"}
SKIP_PREFIXES = ("build",)
TOKEN = re.compile(r"[A-Za-z0-9]+")
# Notice rows may name a dependency differently from its FetchContent name.
DEP_ALIASES = {"nlohmann_json": "nlohmann/json"}


def digest(word):
    return hashlib.sha256(word.lower().encode("utf-8")).hexdigest()[:20]


def hits_in(text):
    """Yield (column, hash) for every forbidden token or joined adjacent token pair."""
    toks = [(m.start(), m.group(), m.end()) for m in TOKEN.finditer(text)]
    for i, (pos, tok, end) in enumerate(toks):
        h = digest(tok)
        if h in FORBIDDEN:
            yield pos, h
        if i + 1 < len(toks) and text[end:toks[i + 1][0]] == " " and not tok.islower() and not toks[i + 1][1].islower():
            h2 = digest(tok + toks[i + 1][1])
            if h2 in FORBIDDEN:
                yield pos, h2


def load_allowlist(path):
    entries = {}
    if path and os.path.isfile(path):
        with open(path, encoding="utf-8") as fh:
            for n, line in enumerate(fh, 1):
                line = line.rstrip("\n")
                if not line.strip() or line.startswith("#"):
                    continue
                parts = line.split("\t")
                if len(parts) < 3 or not parts[2].strip():
                    raise SystemExit("%s:%d: allowlist entry needs path<TAB>hash-or-*<TAB>reason" % (path, n))
                entries[(parts[0], parts[1])] = parts[2]
    return entries


def git_files(root):
    """Tracked plus untracked-but-not-ignored files, or None when root is not a git checkout."""
    if not os.path.exists(os.path.join(root, ".git")):
        return None
    try:
        out = subprocess.run(["git", "-C", root, "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
                             capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    return sorted(set(x for x in out.decode("utf-8", errors="replace").split("\0") if x))


def walk(root):
    """Our own files only: git's file list in a checkout, else a directory walk skipping fetched/build dirs."""
    listed = git_files(root)
    if listed is not None:
        for rel in listed:
            full = os.path.join(root, rel)
            if os.path.isfile(full):
                yield rel, full
        return
    for d, dirs, files in os.walk(root):
        dirs[:] = sorted(x for x in dirs if x not in SKIP_DIRS and not x.startswith(SKIP_PREFIXES))
        for f in sorted(files):
            full = os.path.join(d, f)
            yield os.path.relpath(full, root).replace(os.sep, "/"), full


def scan(root, allow):
    problems = []

    def report(rel, line, h, what):
        if (rel, h) in allow or (rel, "*") in allow:
            return
        problems.append("%s:%d: forbidden %s (hash %s)" % (rel, line, what, h))

    for rel, full in walk(root):
        for _, h in hits_in(rel):
            report(rel, 0, h, "name in path")
        try:
            with open(full, "rb") as fh:
                raw = fh.read()
        except OSError:
            continue
        if b"\0" in raw:
            continue
        text = raw.decode("utf-8", errors="replace")
        for n, line in enumerate(text.splitlines(), 1):
            for _, h in hits_in(line):
                report(rel, n, h, "token")
    return problems


def check_dependencies(root):
    problems = []
    notices_path = os.path.join(root, "THIRD_PARTY_NOTICES.md")
    if not os.path.isfile(os.path.join(root, "LICENSE")):
        problems.append("LICENSE: missing")
    if not os.path.isfile(notices_path):
        return problems + ["THIRD_PARTY_NOTICES.md: missing"]
    with open(notices_path, encoding="utf-8") as fh:
        raw_notices = fh.read()
    notices = raw_notices.lower()
    for rel, full in walk(root):
        if os.path.basename(rel) != "CMakeLists.txt":
            continue
        with open(full, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        for m in re.finditer(r"FetchContent_Declare\s*\(\s*([A-Za-z0-9_.+-]+)", text):
            name = m.group(1)
            key = DEP_ALIASES.get(name, name).lower()
            if key not in notices:
                problems.append("%s: dependency '%s' has no row in THIRD_PARTY_NOTICES.md" % (rel, name))
    for m in re.finditer(r"`((?:data|core)/[^`\s]*(?:LICENSE|OFL|COPYING)[^`\s]*)`", raw_notices, re.I):
        if not os.path.isfile(os.path.join(root, m.group(1))):
            problems.append("THIRD_PARTY_NOTICES.md: referenced licence file '%s' is missing" % m.group(1))
    return problems


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    ap.add_argument("--allowlist", default=None)
    ap.add_argument("--hash", default=None)
    a = ap.parse_args(argv)
    if a.hash:
        print('    "%s",' % digest(a.hash))
        return 0
    root = os.path.abspath(a.root)
    allow_path = a.allowlist or os.path.join(root, "tools", "compliance_allowlist.txt")
    problems = scan(root, load_allowlist(allow_path)) + check_dependencies(root)
    for p in problems:
        print(p)
    if problems:
        print("compliance audit: %d problem(s)" % len(problems))
        return 1
    print("compliance audit: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
