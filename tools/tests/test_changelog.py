#!/usr/bin/env python3
"""Unit tests for tools/changelog.py (T-0146). Standard library only (unittest, no pytest).

Run: python3 tools/tests/test_changelog.py   (ctest runs it as tools.changelog_unit)
"""

import contextlib
import io
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True  # keep the source tree clean (no tools/__pycache__)
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import changelog  # noqa: E402  (tools/changelog.py)

BACKLOG = "".join("### T-%04d Card %d   [status: ready]\nGoal: x\n\n" % (i, i)
                  for i in (9, 17, 120, 124, 146, 150, 151))

CHANGELOG = ("# Changelog\n\n## Unreleased\n\n"
             "- T-0017: older entry — with ≤ 0.5 mm and 45°\n"
             "- T-0009: oldest entry\n")


def run_main(*args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = changelog.main(list(args))
    return code, out.getvalue(), err.getvalue()


class Repo:
    """A throwaway repository root with CHANGELOG.md, docs/BACKLOG.md and changelog.d/."""

    def __init__(self, changelog_text=CHANGELOG):
        self.dir = pathlib.Path(tempfile.mkdtemp(prefix="carvetto-changelog-"))
        (self.dir / "docs").mkdir()
        (self.dir / "docs" / "BACKLOG.md").write_text(BACKLOG, encoding="utf-8")
        (self.dir / "changelog.d").mkdir()
        (self.dir / "changelog.d" / "README.md").write_text("# not a fragment\n", encoding="utf-8")
        self.write_changelog(changelog_text)

    def write_changelog(self, text):
        with open(self.dir / "CHANGELOG.md", "w", encoding="utf-8", newline="") as f:
            f.write(text)

    def changelog(self):
        with open(self.dir / "CHANGELOG.md", "r", encoding="utf-8", newline="") as f:
            return f.read()

    def frag(self, name, content):
        data = content if isinstance(content, bytes) else content.encode("utf-8")
        (self.dir / "changelog.d" / name).write_bytes(data)

    def names(self):
        return sorted(p.name for p in (self.dir / "changelog.d").iterdir())

    def check(self, *extra):
        return run_main("--root", str(self.dir), "check", *extra)

    def assemble(self):
        return run_main("--root", str(self.dir), "assemble")

    def cleanup(self):
        shutil.rmtree(self.dir, ignore_errors=True)


class RepoTest(unittest.TestCase):
    def setUp(self):
        self.repo = Repo()

    def tearDown(self):
        self.repo.cleanup()


class CheckValid(RepoTest):
    def test_no_fragments(self):
        code, out, _ = self.repo.check()
        self.assertEqual(code, 0)
        self.assertIn("0 fragments", out)

    def test_single_card(self):
        self.repo.frag("T-0146.md", "- T-0146: changelog fragments — one line ≤ 1 per card\n")
        self.assertEqual(self.repo.check()[0], 0)

    def test_multi_card(self):
        self.repo.frag("T-0120-0124.md", "- T-0120/T-0124: profile toolpath\n")
        self.assertEqual(self.repo.check()[0], 0)

    def test_no_final_newline_and_crlf(self):
        self.repo.frag("T-0150.md", "- T-0150: drilling")
        self.repo.frag("T-0151.md", "- T-0151: engraving\r\n")
        self.assertEqual(self.repo.check()[0], 0)

    def test_readme_and_dotfiles_ignored(self):
        self.repo.frag(".gitkeep", "")
        self.assertEqual(self.repo.check()[0], 0)


class CheckInvalid(RepoTest):
    def assert_fails(self, name, content, needle):
        self.repo.frag(name, content)
        code, _, err = self.repo.check()
        self.assertEqual(code, 1, err)
        self.assertIn(needle, err)
        self.assertIn("changelog check failed", err)

    def test_bad_names(self):
        for name in ("t-0146.md", "T-146.md", "T-0146.txt", "T-0146-.md", "T-0146_0150.md", "notes.md"):
            with self.subTest(name=name):
                repo = Repo()
                try:
                    repo.frag(name, "- T-0146: x\n")
                    code, _, err = repo.check()
                    self.assertEqual(code, 1)
                    self.assertIn("bad file name", err)
                finally:
                    repo.cleanup()

    def test_ids_not_ascending(self):
        self.assert_fails("T-0124-0120.md", "- T-0124/T-0120: x\n", "distinct and ascending")

    def test_repeated_id(self):
        self.assert_fails("T-0120-0120.md", "- T-0120/T-0120: x\n", "distinct and ascending")

    def test_unknown_card(self):
        self.assert_fails("T-0999.md", "- T-0999: x\n", "no card T-0999")

    def test_empty(self):
        self.assert_fails("T-0146.md", "", "empty")

    def test_blank_line_only(self):
        self.assert_fails("T-0146.md", "\n\n", "empty")

    def test_two_lines(self):
        self.assert_fails("T-0146.md", "- T-0146: one\n- T-0146: two\n", "more than one line")

    def test_trailing_blank_line(self):
        self.assert_fails("T-0146.md", "- T-0146: one\n\n", "more than one line")

    def test_not_a_bullet(self):
        self.assert_fails("T-0146.md", "T-0146: missing bullet\n", "must start with '- T-0146: '")

    def test_prefix_does_not_match_name(self):
        self.assert_fails("T-0146.md", "- T-0150: other card\n", "must start with '- T-0146: '")

    def test_multi_card_prefix(self):
        self.assert_fails("T-0120-0124.md", "- T-0120: only one id\n", "'- T-0120/T-0124: '")

    def test_heading_instead_of_bullet(self):
        self.assert_fails("T-0146.md", "## T-0146: heading\n", "must start with")

    def test_no_text(self):
        self.assert_fails("T-0146.md", "- T-0146:  \n", "trailing whitespace")
        repo = Repo()
        try:
            repo.frag("T-0146.md", "- T-0146:")
            code, _, err = repo.check()
            self.assertEqual(code, 1)
        finally:
            repo.cleanup()

    def test_tab(self):
        self.assert_fails("T-0146.md", "- T-0146: a\tb\n", "U+0009")

    def test_zero_width_and_bidi(self):
        self.assert_fails("T-0146.md", "- T-0146: a​b\n", "U+200B")
        repo = Repo()
        try:
            repo.frag("T-0146.md", "- T-0146: a‮b\n")
            self.assertEqual(repo.check()[0], 1)
        finally:
            repo.cleanup()

    def test_bom(self):
        self.assert_fails("T-0146.md", b"\xef\xbb\xbf- T-0146: x\n", "byte order mark")

    def test_invalid_utf8(self):
        self.assert_fails("T-0146.md", b"- T-0146: caf\xe9\n", "not valid UTF-8")

    def test_bare_cr(self):
        self.assert_fails("T-0146.md", "- T-0146: a\rb\n", "more than one line")

    def test_leading_space(self):
        self.assert_fails("T-0146.md", " - T-0146: x\n", "leading or trailing whitespace")

    def test_card_in_two_fragments(self):
        self.repo.frag("T-0120.md", "- T-0120: a\n")
        self.assert_fails("T-0120-0124.md", "- T-0120/T-0124: b\n", "one fragment per card")

    def test_subdirectory(self):
        (self.repo.dir / "changelog.d" / "sub").mkdir()
        code, _, err = self.repo.check()
        self.assertEqual(code, 1)
        self.assertIn("unexpected directory", err)

    def test_missing_unreleased_heading(self):
        self.repo.write_changelog("# Changelog\n\n## 0.1.0\n\n- T-0009: x\n")
        code, _, err = self.repo.check()
        self.assertEqual(code, 1)
        self.assertIn("no '## Unreleased' heading", err)

    def test_missing_fragment_dir(self):
        shutil.rmtree(self.repo.dir / "changelog.d")
        code, _, err = self.repo.check()
        self.assertEqual(code, 1)
        self.assertIn("changelog.d/ is missing", err)

    def test_all_errors_reported(self):
        self.repo.frag("T-0146.md", "")
        self.repo.frag("bad.md", "x")
        code, _, err = self.repo.check()
        self.assertEqual(code, 1)
        self.assertIn("2 errors", err)


FRAGS = {
    "T-0150.md": "- T-0150: drilling\n",
    "T-0009.md": "- T-0009: subtol spans\n",
    "T-0120-0124.md": "- T-0120/T-0124: profile\n",
    "T-0151.md": "- T-0151: engraving\n",
}


class Assemble(RepoTest):
    def test_order_newest_first_and_fragments_deleted(self):
        self.repo.write_changelog("# Changelog\n\n## Unreleased\n\n- T-0017: older entry\n")
        for name, line in FRAGS.items():
            self.repo.frag(name, line)
        code, out, err = self.repo.assemble()
        self.assertEqual(code, 0, err)
        self.assertEqual(self.repo.changelog(),
                         "# Changelog\n\n## Unreleased\n\n"
                         "- T-0151: engraving\n"
                         "- T-0150: drilling\n"
                         "- T-0120/T-0124: profile\n"
                         "- T-0009: subtol spans\n"
                         "- T-0017: older entry\n")
        self.assertEqual(self.repo.names(), ["README.md"])
        self.assertIn("assembled 4 fragments", out)
        self.assertEqual(self.repo.check()[0], 0)

    def test_multi_card_sorts_by_highest_id(self):
        self.repo.write_changelog("# Changelog\n\n## Unreleased\n")
        self.repo.frag("T-0120-0151.md", "- T-0120/T-0151: both\n")
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        self.repo.frag("T-0124.md", "- T-0124: profile\n")
        self.assertEqual(self.repo.assemble()[0], 0)
        self.assertEqual(self.repo.changelog(),
                         "# Changelog\n\n## Unreleased\n\n"
                         "- T-0120/T-0151: both\n- T-0150: drilling\n- T-0124: profile\n")

    def test_idempotent(self):
        for name, line in FRAGS.items():
            self.repo.frag(name, line)
        self.assertEqual(self.repo.assemble()[0], 0)
        once = self.repo.changelog()
        code, out, _ = self.repo.assemble()
        self.assertEqual(code, 0)
        self.assertIn("assembled 0 fragments", out)
        self.assertEqual(self.repo.changelog(), once)

    def test_interrupted_assemble_does_not_duplicate(self):
        for name, line in FRAGS.items():
            self.repo.frag(name, line)
        self.assertEqual(self.repo.assemble()[0], 0)
        once = self.repo.changelog()
        # As if the fragments had survived (e.g. the deletion was not committed): re-run.
        for name, line in FRAGS.items():
            self.repo.frag(name, line)
        self.assertEqual(self.repo.assemble()[0], 0)
        self.assertEqual(self.repo.changelog(), once)
        self.assertEqual(self.repo.names(), ["README.md"])

    def test_deterministic_regardless_of_creation_order(self):
        results = []
        for order in (sorted(FRAGS), sorted(FRAGS, reverse=True)):
            repo = Repo()
            try:
                for name in order:
                    repo.frag(name, FRAGS[name])
                self.assertEqual(repo.assemble()[0], 0)
                results.append(repo.changelog())
            finally:
                repo.cleanup()
        self.assertEqual(results[0], results[1])

    def test_empty_section_before_next_heading(self):
        self.repo.write_changelog("# Changelog\n\n## Unreleased\n\n## 0.1.0\n\n- T-0009: released\n")
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        self.assertEqual(self.repo.assemble()[0], 0)
        self.assertEqual(self.repo.changelog(),
                         "# Changelog\n\n## Unreleased\n\n- T-0150: drilling\n\n"
                         "## 0.1.0\n\n- T-0009: released\n")

    def test_crlf_changelog_kept_crlf(self):
        self.repo.write_changelog(CHANGELOG.replace("\n", "\r\n"))
        self.repo.frag("T-0150.md", "- T-0150: drilling\r\n")
        self.assertEqual(self.repo.assemble()[0], 0)
        self.assertEqual(self.repo.changelog(),
                         CHANGELOG.replace("## Unreleased\n\n", "## Unreleased\n\n- T-0150: drilling\n")
                         .replace("\n", "\r\n"))

    def test_invalid_fragment_changes_nothing(self):
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        self.repo.frag("T-0151.md", "- T-0151: two\n- lines\n")
        code, _, err = self.repo.assemble()
        self.assertEqual(code, 1)
        self.assertIn("nothing assembled", err)
        self.assertEqual(self.repo.changelog(), CHANGELOG)
        self.assertEqual(self.repo.names(), ["README.md", "T-0150.md", "T-0151.md"])

    def test_no_unreleased_heading_changes_nothing(self):
        text = "# Changelog\n\n- T-0009: x\n"
        self.repo.write_changelog(text)
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        self.assertEqual(self.repo.assemble()[0], 1)
        self.assertEqual(self.repo.changelog(), text)
        self.assertEqual(self.repo.names(), ["README.md", "T-0150.md"])


@unittest.skipIf(shutil.which("git") is None, "git not available")
class CheckBase(RepoTest):
    """check --base REF: hand edits to the Unreleased section are refused in a PR."""

    def git(self, *args):
        subprocess.run(["git", *args], cwd=str(self.repo.dir), check=True, capture_output=True)

    def setUp(self):
        super().setUp()
        self.git("init", "-q", "-b", "main")
        self.git("config", "user.email", "test@example.invalid")
        self.git("config", "user.name", "test")
        self.git("config", "core.autocrlf", "false")
        self.git("config", "commit.gpgsign", "false")
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        self.commit("base")
        self.git("checkout", "-q", "-b", "pr")

    def commit(self, message):
        self.git("add", "-A")
        self.git("commit", "-q", "--allow-empty", "-m", message)

    def check_base(self):
        return self.repo.check("--base", "main")

    def test_fragment_only_pr_passes(self):
        self.repo.frag("T-0151.md", "- T-0151: engraving\n")
        self.commit("pr")
        code, out, err = self.check_base()
        self.assertEqual(code, 0, err)
        self.assertIn("no direct edits", out)

    def test_direct_edit_fails(self):
        self.repo.write_changelog(CHANGELOG.replace("## Unreleased\n\n", "## Unreleased\n\n- T-0151: by hand\n"))
        self.commit("pr")
        code, _, err = self.check_base()
        self.assertEqual(code, 1)
        self.assertIn("added to '## Unreleased' by hand", err)

    def test_uncommitted_direct_edit_fails(self):
        self.repo.write_changelog(CHANGELOG + "- T-0151: by hand\n")
        self.assertEqual(self.check_base()[0], 1)

    def test_editing_existing_line_fails(self):
        self.repo.write_changelog(CHANGELOG.replace("oldest entry", "oldest entry, reworded"))
        self.commit("pr")
        code, _, err = self.check_base()
        self.assertEqual(code, 1)
        self.assertIn("removed or edited", err)

    def test_reordering_fails(self):
        self.repo.write_changelog("# Changelog\n\n## Unreleased\n\n- T-0009: oldest entry\n"
                                  "- T-0017: older entry — with ≤ 0.5 mm and 45°\n")
        self.commit("pr")
        code, _, err = self.check_base()
        self.assertEqual(code, 1)
        self.assertIn("reordered", err)

    def test_assemble_pr_passes(self):
        self.assertEqual(self.repo.assemble()[0], 0)
        self.commit("assemble")
        code, _, err = self.check_base()
        self.assertEqual(code, 0, err)

    def test_assemble_pr_with_extra_hand_line_fails(self):
        self.assertEqual(self.repo.assemble()[0], 0)
        self.repo.write_changelog(self.repo.changelog().replace("## Unreleased\n\n",
                                                                "## Unreleased\n\n- T-0151: sneaked in\n"))
        self.commit("assemble")
        code, _, err = self.check_base()
        self.assertEqual(code, 1)
        self.assertIn("sneaked in", err)

    def test_release_move_passes(self):
        released = CHANGELOG.replace("## Unreleased\n\n", "## Unreleased\n\n## 0.1.0\n\n")
        self.repo.write_changelog(released)
        self.commit("release")
        code, _, err = self.check_base()
        self.assertEqual(code, 0, err)

    def test_base_moved_on_is_not_blamed(self):
        # main gets an assembled entry after the PR branched; the PR itself changes nothing there.
        self.git("checkout", "-q", "main")
        self.assertEqual(self.repo.assemble()[0], 0)
        self.commit("assemble on main")
        self.git("checkout", "-q", "pr")
        self.repo.frag("T-0151.md", "- T-0151: engraving\n")
        self.commit("pr")
        self.assertEqual(self.check_base()[0], 0)

    def test_unknown_ref(self):
        code, _, err = self.repo.check("--base", "no-such-ref")
        self.assertEqual(code, 1)
        self.assertIn("cannot compare with no-such-ref", err)


if __name__ == "__main__":
    unittest.main(verbosity=1)
