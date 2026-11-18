#!/usr/bin/env python3
"""Unit tests for tools/check_compliance.py. Forbidden words are assembled from fragments at run time
so this file does not itself mention them."""
import os
import subprocess
import sys
import tempfile
import unittest

TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "check_compliance.py")
sys.path.insert(0, os.path.dirname(TOOL))
import check_compliance  # noqa: E402

WORD = "Vec" + "tric"
FMT = "cr" + "v3d"
CMAKE_OK = "FetchContent_Declare(\n  lua\n  URL x\n)\n"


def run(files, allow=None):
    with tempfile.TemporaryDirectory() as root:
        base = {"LICENSE": "GPL\n", "THIRD_PARTY_NOTICES.md": "| Lua | 5.4 | MIT |\n",
                "core/CMakeLists.txt": CMAKE_OK}
        base.update(files)
        for rel, text in base.items():
            full = os.path.join(root, rel)
            os.makedirs(os.path.dirname(full), exist_ok=True)
            with open(full, "w", encoding="utf-8") as fh:
                fh.write(text)
        args = [sys.executable, "-I", TOOL, "--root", root]
        if allow is not None:
            path = os.path.join(root, "allow.txt")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(allow)
            args += ["--allowlist", path]
        r = subprocess.run(args, capture_output=True, text=True)
        return r.returncode, r.stdout


class Compliance(unittest.TestCase):
    def test_clean_tree_passes(self):
        rc, out = run({"docs/a.md": "A V-carve toolpath cuts a groove.\n"})
        self.assertEqual(rc, 0, out)

    def test_injected_name_fails_without_echoing_it(self):
        rc, out = run({"docs/a.md": "ok\nlike %s does\n" % WORD})
        self.assertEqual(rc, 1)
        self.assertIn("docs/a.md:2", out)
        self.assertNotIn(WORD.lower(), out.lower())

    def test_case_insensitive_and_in_code(self):
        rc, _ = run({"core/src/x.cpp": "// %s\n" % WORD.upper()})
        self.assertEqual(rc, 1)

    def test_format_extension_and_path(self):
        rc, _ = run({"docs/a.md": "reads `.%s`\n" % FMT})
        self.assertEqual(rc, 1)
        rc, out = run({"samples/%s_job.txt" % WORD: "x\n"})
        self.assertEqual(rc, 1)
        self.assertIn("name in path", out)

    def test_two_word_name(self):
        rc, _ = run({"docs/a.md": "Carb" + "ide Cre" + "ate\n"})
        self.assertEqual(rc, 1)
        rc, _ = run({"docs/a.md": ("Carb" + "ide Cre" + "ate\n").lower()})
        self.assertEqual(rc, 0)

    def test_allowlist_requires_reason_and_matches(self):
        h = check_compliance.digest(FMT)
        rc, out = run({"docs/a.md": "`.%s`\n" % FMT}, allow="docs/a.md\t%s\tpolicy statement\n" % h)
        self.assertEqual(rc, 0, out)
        rc, out = run({"docs/a.md": "`.%s`\n" % FMT}, allow="docs/a.md\t%s\t\n" % h)
        self.assertNotEqual(rc, 0)
        rc, _ = run({"docs/b.md": "`.%s`\n" % FMT}, allow="docs/a.md\t%s\tpolicy\n" % h)
        self.assertEqual(rc, 1)

    def test_binary_files_skipped(self):
        # text files only are scanned; a NUL byte marks binary
        rc, out = run({"data/x.bin": "\0%s\n" % WORD})
        self.assertEqual(rc, 0, out)

    def test_undocumented_dependency_fails(self):
        rc, out = run({"core/CMakeLists.txt": CMAKE_OK + "FetchContent_Declare(\n  zlibfoo\n)\n"})
        self.assertEqual(rc, 1)
        self.assertIn("zlibfoo", out)

    def test_missing_licence_file_fails(self):
        rc, out = run({"THIRD_PARTY_NOTICES.md": "| Lua |\n`data/fonts/FOO-LICENSE.txt`\n"})
        self.assertEqual(rc, 1)
        self.assertIn("FOO-LICENSE", out)

    def test_hash_mode(self):
        r = subprocess.run([sys.executable, "-I", TOOL, "--hash", "Abc"], capture_output=True, text=True)
        self.assertIn(check_compliance.digest("abc"), r.stdout)

    def test_real_repo_forbidden_list_has_no_plain_words(self):
        with open(TOOL, encoding="utf-8") as fh:
            text = fh.read()
        self.assertEqual(list(check_compliance.hits_in(text)), [])


if __name__ == "__main__":
    unittest.main()
