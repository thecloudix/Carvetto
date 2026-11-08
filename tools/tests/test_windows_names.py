#!/usr/bin/env python3
"""Unit tests for tools/check_windows_names.py (fixtures are written to a temp dir)."""
import os
import subprocess
import sys
import tempfile
import unittest

TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "check_windows_names.py")

HELPER_SAFE = "#define NOMINMAX\n#define WIN32_LEAN_AND_MEAN\n#define NOGDI\n#include <windows.h>\n"
HELPER_UNSAFE = "#define NOMINMAX\n#include <windows.h>\n"


def run(files, allow=None):
    with tempfile.TemporaryDirectory() as root:
        files = dict(files)
        files.setdefault("core/tests/geometry/cpu_time.hpp", HELPER_SAFE)
        for rel, text in files.items():
            full = os.path.join(root, rel)
            os.makedirs(os.path.dirname(full), exist_ok=True)
            with open(full, "w", encoding="utf-8") as fh:
                fh.write(text)
        args = [sys.executable, "-I", TOOL, "--root", root]
        if allow is not None:
            path = os.path.join(root, "allow.txt")
            with open(path, "w") as fh:
                fh.write(allow)
            args += ["--allowlist", path]
        r = subprocess.run(args, capture_output=True, text=True)
        return r.returncode, r.stdout


WIN = '#include "cpu_time.hpp"\n'


class WindowsNames(unittest.TestCase):
    def bad(self, body, files=None):
        rc, out = run({"core/tests/a_test.cpp": WIN + body, **(files or {})})
        self.assertEqual(rc, 1, out)
        return out

    def good(self, body, files=None):
        rc, out = run({"core/tests/a_test.cpp": WIN + body, **(files or {})})
        self.assertEqual(rc, 0, out)

    def test_macros_caught(self):
        for name in ("far", "near", "interface", "IN", "OUT", "ERROR"):
            out = self.bad("int f() { int %s = 1; return %s; }\n" % (name, name))
            self.assertIn("a_test.cpp:2", out)
            self.assertIn("'%s'" % name, out)

    def test_min_max_calls(self):
        self.bad("int f(int a, int b) { return max(a, b); }\n")
        self.bad("int f(int a, int b) { return min(a, b); }\n")
        self.good("#include <algorithm>\nint f(int a, int b) { return std::max(a, b) + (std::min)(a, b); }\n")
        self.good("int f(Box b) { return b.max(1) + p->min(2) + ns::max(1); }\n")
        self.good("int f(int a) { return max_value(a); }\n")

    def test_gdi_names_unqualified(self):
        files = {"core/tests/geometry/cpu_time.hpp": HELPER_UNSAFE}
        out = self.bad("void f() { Polyline p; }\n", files)
        self.assertIn("'Polyline'", out)
        self.bad("void f() { Rectangle(); }\n", files)
        self.good("void f() { geometry::Polyline p; x.Rect = 1; }\n", files)

    def test_gdi_safe_with_nogdi_helper(self):
        self.good("void f() { Polyline p; }\n")

    def test_small_only_when_not_lean(self):
        self.good("int small = 1;\n")
        self.bad("int small = 1;\n", {"core/tests/geometry/cpu_time.hpp": HELPER_UNSAFE})

    def test_comments_and_strings_ignored(self):
        self.good('// far near\n/* small\n far */ const char* s = "near far IN OUT";\nchar c = \'x\';\n'
                  'auto r = R"(far near)";\nint n = 1\'000;\n')

    def test_escape_comment(self):
        self.good("int far = 1;  // lint-allow-windows-name\n")

    def test_allowlist(self):
        rc, _ = run({"core/tests/a_test.cpp": WIN + "int far = 1;\n"}, allow="core/tests/a_test.cpp:2\n")
        self.assertEqual(rc, 0)
        rc, _ = run({"core/tests/a_test.cpp": WIN + "int far = 1;\n"}, allow="core/tests/a_test.cpp\n")
        self.assertEqual(rc, 0)
        rc, _ = run({"core/tests/a_test.cpp": WIN + "int far = 1;\n"}, allow="# nothing\n")
        self.assertEqual(rc, 1)

    def test_files_that_cannot_see_windows_h_are_not_checked(self):
        rc, _ = run({"core/src/x.cpp": "int far = 1; int near = 2;\n"})
        self.assertEqual(rc, 0)

    def test_shared_test_headers_are_checked(self):
        rc, out = run({"core/tests/util.hpp": "inline int f() { auto far = 1; return far; }\n"})
        self.assertEqual(rc, 1, out)
        self.assertIn("util.hpp:1", out)

    def test_direct_windows_h(self):
        rc, _ = run({"app/x.cpp": "#include <windows.h>\nint near = 1;\n"})
        self.assertEqual(rc, 1)


if __name__ == "__main__":
    unittest.main()
