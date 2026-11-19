#!/usr/bin/env python3
"""Unit tests for tools/release_notes.py (T-0361). Standard library only.

Run: python3 -I tools/tests/test_release_notes.py   (ctest runs it as tools.release_notes_unit)
"""

import os
import pathlib
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import release_notes  # noqa: E402

CHANGELOG = ("# Changelog\n\n## Unreleased\n\n- T-0002: next\n\n"
             "## 0.1.0\n\n- T-0001: first — with ≤ 0.5 mm\n- T-0003: third\n\n"
             "## 0.0.9 - 2026-01-01\n\n- T-0000: older\n\n## 0.1.0-rc1\n\n- rc\n")


class ReleaseNotesTest(unittest.TestCase):
    def test_version_of(self):
        self.assertEqual(release_notes.version_of("v0.1.0"), "0.1.0")
        self.assertEqual(release_notes.version_of("1.2.3"), "1.2.3")
        self.assertEqual(release_notes.version_of("v1.2.3-rc1"), "1.2.3-rc1")
        for bad in ("v1", "latest", "v1.2", "vv1.2.3", ""):
            with self.assertRaises(ValueError):
                release_notes.version_of(bad)

    def test_extracts_section_only(self):
        notes = release_notes.extract(CHANGELOG, "0.1.0")
        self.assertEqual(notes, "- T-0001: first — with ≤ 0.5 mm\n- T-0003: third\n")

    def test_heading_with_date_and_prefix_versions_not_confused(self):
        self.assertEqual(release_notes.extract(CHANGELOG, "0.0.9"), "- T-0000: older\n")
        self.assertEqual(release_notes.extract(CHANGELOG, "0.1.0-rc1"), "- rc\n")

    def test_last_section_and_crlf(self):
        text = "## 1.0.0\r\n\r\n- a\r\n- b\r\n"
        self.assertEqual(release_notes.extract(text, "1.0.0"), "- a\n- b\n")

    def test_missing_and_empty(self):
        with self.assertRaises(ValueError):
            release_notes.extract(CHANGELOG, "9.9.9")
        with self.assertRaises(ValueError):
            release_notes.extract("## 1.0.0\n\n## 0.9.0\n- x\n", "1.0.0")

    def test_main_fake_tag(self):
        with tempfile.TemporaryDirectory() as d:
            cl = os.path.join(d, "CHANGELOG.md")
            out = os.path.join(d, "notes.md")
            with open(cl, "w", encoding="utf-8", newline="\n") as f:
                f.write(CHANGELOG)
            self.assertEqual(release_notes.main(["v0.1.0", "--changelog", cl, "--output", out]), 0)
            with open(out, encoding="utf-8", newline="") as f:
                self.assertEqual(f.read(), "- T-0001: first — with ≤ 0.5 mm\n- T-0003: third\n")
            self.assertEqual(release_notes.main(["v7.7.7", "--changelog", cl, "--output", out]), 1)
            self.assertEqual(release_notes.main(["nonsense", "--changelog", cl]), 1)
            self.assertEqual(release_notes.main(["v0.1.0", "--changelog", cl + ".none"]), 1)


class ReleaseWorkflowTest(unittest.TestCase):
    """Lint of the release job in ci.yml: only the intended packages may be published."""

    def release_job(self):
        root = pathlib.Path(__file__).resolve().parent.parent.parent
        text = (root / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
        i = text.index("\n  release:\n")
        return text[i:]

    def test_downloads_are_explicit(self):
        job = self.release_job()
        self.assertNotIn("pattern:", job)
        self.assertNotIn("merge-multiple", job)
        names = sorted(__import__("re").findall(r"^\s+name: (carvetto-[\w-]+)\s*$", job, __import__("re").M))
        self.assertEqual(names, ["carvetto-linux-packages", "carvetto-windows-installer"])

    def test_upload_is_explicit_list(self):
        job = self.release_job()
        self.assertNotIn("release-assets/*", job)
        upload = [ln for ln in job.splitlines() if "gh release upload" in ln]
        self.assertEqual(len(upload), 1)
        self.assertEqual(upload[0].split()[4:-1],
                         ["Carvetto-*-win64.exe", "carvetto_*_amd64.deb", "carvetto-*.x86_64.rpm",
                          "Carvetto-*-x86_64.AppImage", "SHA256SUMS"])

    def test_needs_all_test_jobs(self):
        job = self.release_job()
        for j in ("linux-core", "linux-app", "windows-app", "windows-installer", "linux-packages"):
            self.assertIn(j, job.split("runs-on:")[0])


if __name__ == "__main__":
    unittest.main()
