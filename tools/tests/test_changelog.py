#!/usr/bin/env python3
"""Unit tests for tools/changelog.py (T-0146). Standard library only (unittest, no pytest).

Run: python3 tools/tests/test_changelog.py   (ctest runs it as tools.changelog_unit)
"""

import contextlib
import io
import os
import pathlib
import shutil
import stat
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
        self.assertIn("exactly one '## Unreleased' heading (byte-exact), found 0", err)

    def test_two_unreleased_headings(self):
        self.repo.write_changelog(CHANGELOG + "\n## Unreleased\n\n- T-0150: x\n")
        code, _, err = self.repo.check()
        self.assertEqual(code, 1)
        self.assertIn("found 2", err)

    def test_missing_backlog(self):
        (self.repo.dir / "docs" / "BACKLOG.md").unlink()
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        code, _, err = self.repo.check()
        self.assertEqual(code, 1)
        self.assertIn("docs/BACKLOG.md is missing", err)
        code, _, err = self.repo.assemble()
        self.assertEqual(code, 1)
        self.assertEqual(self.repo.names(), ["README.md", "T-0150.md"])

    def test_non_ascii_digits(self):
        # Arabic-Indic digits are Unicode decimal digits but not card ids.
        self.assert_fails("T-\u0660\u0661\u0664\u0666.md", "- T-0146: x\n", "bad file name")

    def test_fragment_line_already_in_changelog(self):
        # CHANGELOG fixture holds "- T-0017: older entry — with ≤ 0.5 mm and 45°".
        self.assert_fails("T-0017.md", "- T-0017: older entry — with ≤ 0.5 mm and 45°\n",
                          "T-0017.md: this line is already in CHANGELOG.md")

    @unittest.skipIf(not hasattr(os, "symlink"), "no symlinks")
    def test_symlinks_refused(self):
        try:
            os.symlink("README.md", str(self.repo.dir / "changelog.d" / "T-0150.md"))
        except OSError:
            self.skipTest("cannot create symlinks here")
        code, _, err = self.repo.check()
        self.assertEqual(code, 1)
        self.assertIn("T-0150.md: must be a regular file, not a symlink", err)
        (self.repo.dir / "changelog.d" / "T-0150.md").unlink()
        real = self.repo.dir / "docs" / "cl.md"
        real.write_text(CHANGELOG, encoding="utf-8")
        (self.repo.dir / "CHANGELOG.md").unlink()
        os.symlink(os.path.join("docs", "cl.md"), str(self.repo.dir / "CHANGELOG.md"))
        code, _, err = self.repo.check()
        self.assertEqual(code, 1)
        self.assertIn("CHANGELOG.md must be a regular file, not a symlink", err)

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

    @unittest.skipIf(hasattr(os, "geteuid") and os.geteuid() == 0, "root ignores file permissions")
    def test_read_only_changelog_clean_error(self):
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        path = self.repo.dir / "CHANGELOG.md"
        path.chmod(stat.S_IREAD)
        try:
            code, _, err = self.repo.assemble()
        finally:
            path.chmod(stat.S_IREAD | stat.S_IWRITE)
        self.assertEqual(code, 1)
        self.assertIn("not writable", err)
        self.assertNotIn("Traceback", err)
        self.assertEqual(self.repo.changelog(), CHANGELOG)
        self.assertEqual(self.repo.names(), ["README.md", "T-0150.md"])
        self.assertFalse((self.repo.dir / "CHANGELOG.md.tmp").exists())

    def test_no_unreleased_heading_changes_nothing(self):
        text = "# Changelog\n\n- T-0009: x\n"
        self.repo.write_changelog(text)
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        self.assertEqual(self.repo.assemble()[0], 1)
        self.assertEqual(self.repo.changelog(), text)
        self.assertEqual(self.repo.names(), ["README.md", "T-0150.md"])


@unittest.skipIf(shutil.which("git") is None, "git not available")
class CheckBase(RepoTest):
    """check --base REF: hand edits to CHANGELOG.md are refused in a PR."""

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
        self.assertIn("no hand edits", out)

    def test_direct_edit_fails(self):
        self.repo.write_changelog(CHANGELOG.replace("## Unreleased\n\n", "## Unreleased\n\n- T-0151: by hand\n"))
        self.commit("pr")
        code, _, err = self.check_base()
        self.assertEqual(code, 1)
        self.assertIn("edited by hand (first difference at line 5: - T-0151: by hand)", err)

    def test_uncommitted_edit_is_not_judged_but_noted(self):
        # check --base reads committed blobs (what CI sees); the working tree only gets a note.
        self.repo.write_changelog(CHANGELOG + "- T-0151: by hand\n")
        code, _, err = self.check_base()
        self.assertEqual(code, 0, err)
        self.assertIn("note: uncommitted changes", err)
        self.commit("now committed")
        self.assertEqual(self.check_base()[0], 1)

    def test_editing_existing_line_fails(self):
        self.repo.write_changelog(CHANGELOG.replace("oldest entry", "oldest entry, reworded"))
        self.commit("pr")
        code, _, err = self.check_base()
        self.assertEqual(code, 1)
        self.assertIn("edited by hand", err)

    def test_reordering_fails(self):
        self.repo.write_changelog("# Changelog\n\n## Unreleased\n\n- T-0009: oldest entry\n"
                                  "- T-0017: older entry — with ≤ 0.5 mm and 45°\n")
        self.commit("pr")
        code, _, err = self.check_base()
        self.assertEqual(code, 1)
        self.assertIn("first difference at line 5: - T-0009: oldest entry", err)

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

    def test_release_move_with_entries_under_version_passes(self):
        self.repo.write_changelog("# Changelog\n\n## Unreleased\n\n## 0.1.0\n\n"
                                  "- T-0017: older entry — with ≤ 0.5 mm and 45°\n- T-0009: oldest entry\n")
        self.commit("release")
        self.assertEqual(self.check_base()[0], 0)

    # Bypasses found by the T-0146 round-1 verifier: the whole file is compared, not one section.
    def assert_base_fails(self, text, needle):
        self.repo.write_changelog(text)
        self.commit("pr")
        code, _, err = self.check_base()
        self.assertEqual(code, 1, err)
        self.assertIn(needle, err)

    def test_fake_heading_inside_unreleased_fails(self):
        for heading in ("# x", "## anything"):
            with self.subTest(heading=heading):
                self.assert_base_fails(
                    CHANGELOG.replace("## Unreleased\n\n", "## Unreleased\n\n%s\n\n- T-0151: hand\n" % heading),
                    "edited by hand (first difference at line 5: %s)" % heading)

    def test_h1_heading_alone_fails(self):
        self.assert_base_fails(CHANGELOG.replace("## Unreleased\n\n", "## Unreleased\n\n# x\n\n"),
                               "edited by hand (first difference at line 5: # x)")

    def test_line_above_unreleased_fails(self):
        self.assert_base_fails(CHANGELOG.replace("## Unreleased", "- T-0151: on top\n\n## Unreleased"),
                               "- T-0151: on top")

    def test_second_unreleased_section_fails(self):
        self.assert_base_fails(CHANGELOG + "\n## Unreleased\n\n- T-0151: hand\n", "'## Unreleased'")

    def test_line_added_to_release_section_fails(self):
        base = CHANGELOG + "\n## 0.1.0\n\n- T-0009: released\n"
        self.repo.write_changelog(base)
        self.commit("release on main")
        self.git("branch", "-f", "main")
        self.assert_base_fails(base + "- T-0151: hand in release\n", "- T-0151: hand in release")

    def test_line_edited_in_release_section_fails(self):
        base = CHANGELOG + "\n## 0.1.0\n\n- T-0124: released\n"
        self.repo.write_changelog(base)
        self.commit("release on main")
        self.git("branch", "-f", "main")
        self.assert_base_fails(base.replace("- T-0124: released", "- T-0124: released, reworded"),
                               "edited by hand")

    def test_fragment_line_assembled_twice_fails(self):
        self.assertEqual(self.repo.assemble()[0], 0)
        self.assert_base_fails(self.repo.changelog() + "- T-0150: drilling\n", "first difference at line 8: - T-0150: drilling")

    def test_assembled_line_outside_unreleased_fails(self):
        (self.repo.dir / "changelog.d" / "T-0150.md").unlink()
        self.assert_base_fails(CHANGELOG.replace("# Changelog\n", "# Changelog\n\n- T-0150: drilling\n"),
                               "is not what tools/changelog.py assemble makes")

    def test_duplicated_existing_line_fails(self):
        self.assert_base_fails(CHANGELOG + "- T-0009: oldest entry\n", "- T-0009: oldest entry")

    def test_unknown_ref(self):
        code, _, err = self.repo.check("--base", "no-such-ref")
        self.assertEqual(code, 1)
        self.assertIn("cannot compare with no-such-ref", err)


HAND_MSG = "CHANGELOG.md edited by hand"
BASE2 = ("# Changelog\n\n## Unreleased\n\n- T-0017: older entry\n- T-0009: oldest entry\n\n"
         "## 0.1.0\n\n- T-0124: released two\n- T-0120: released\n")


@unittest.skipIf(shutil.which("git") is None, "git not available")
class CheckBaseExact(RepoTest):
    """The exact-structure rule (T-0146 round 2): HEAD = base after none / assemble / release."""

    git = CheckBase.git
    commit = CheckBase.commit
    check_base = CheckBase.check_base

    def setUp(self):
        super().setUp()
        self.git("init", "-q", "-b", "main")
        for k, v in (("user.email", "test@example.invalid"), ("user.name", "test"),
                     ("core.autocrlf", "false"), ("commit.gpgsign", "false")):
            self.git("config", k, v)
        self.repo.write_changelog(BASE2)
        self.repo.frag("T-0150.md", "- T-0150: drilling\n")
        self.repo.frag("T-0151.md", "- T-0151: engrave\n")
        self.commit("base")
        self.git("checkout", "-q", "-b", "pr")

    def write_raw(self, data):
        (self.repo.dir / "CHANGELOG.md").write_bytes(data)

    def rm_frag(self, name):
        (self.repo.dir / "changelog.d" / name).unlink()

    def expect(self, code, needle=None):
        self.commit("pr")
        got, out, err = self.check_base()
        self.assertEqual(got, code, out + err)
        self.assertNotIn("Traceback", err)
        if needle is not None:
            self.assertIn(needle, err if code else out)

    HAND = "edited by hand"
    # (name, transformation of BASE2, expected needle); every one must fail.
    EDITS = [
        ("nbsp_unreleased_heading", lambda t: t.replace("## 0.1.0", "## Unreleased \n\n- T-0146: hand\n\n## 0.1.0"), HAND),
        ("cyrillic_heading_alone", lambda t: t.replace("## 0.1.0", "## Unrеleased\n\n## 0.1.0"), HAND),
        ("cyrillic_heading_capturing", lambda t: t.replace("## Unreleased\n", "## Unreleased\n\n## Unrеleased\n"), HAND),
        ("crlf_one_line", lambda t: t.replace("- T-0009: oldest entry\n", "- T-0009: oldest entry\r\n"), HAND),
        ("crlf_heading", lambda t: t.replace("## Unreleased\n", "## Unreleased\r\n"), HAND),
        ("lone_cr", lambda t: t.replace("- T-0017: older entry\n", "- T-0017: older entry\r- T-0146: hand\n"), HAND),
        ("whole_file_crlf", lambda t: t.replace("\n", "\r\n"), HAND),
        ("fake_version", lambda t: t.replace("## Unreleased\n", "## Unreleased\n\n## Hello world, hand text\n"), HAND),
        ("entry_as_heading", lambda t: t.replace("## 0.1.0", "## - T-0146: hand entry as heading\n\n## 0.1.0"), HAND),
        ("version_below_with_moved_line", lambda t: t.replace("- T-0017: older entry\n", "") + "\n## 0.0.9\n\n- T-0017: older entry\n", HAND),
        ("released_line_into_unreleased", lambda t: t.replace("- T-0120: released\n", "").replace("- T-0017", "- T-0120: released\n- T-0017"), HAND),
        ("unreleased_line_into_release", lambda t: t.replace("- T-0017: older entry\n", "").replace("- T-0120: released", "- T-0017: older entry\n- T-0120: released"), HAND),
        ("reorder_release", lambda t: t.replace("- T-0124: released two\n- T-0120: released\n", "- T-0120: released\n- T-0124: released two\n"), HAND),
        ("duplicate_version", lambda t: t.replace("## Unreleased\n\n", "## Unreleased\n\n## 0.1.0\n\n"), "version 0.1.0 is already in the file"),
        ("version_with_trailing_text", lambda t: t.replace("## Unreleased\n\n", "## Unreleased\n\n## 0.2.0 hand\n\n"), HAND),
        ("version_bad_date", lambda t: t.replace("## Unreleased\n\n", "## Unreleased\n\n## 0.2.0 - 2026-11\n\n"), HAND),
        ("version_not_at_top", lambda t: t.replace("- T-0009", "## 0.2.0\n\n- T-0009"), HAND),
        ("duplicated_line", lambda t: t + "- T-0120: released\n", HAND),
        ("blank_lines_appended", lambda t: t + "\n\n\n", HAND),
        ("trailing_space_on_heading", lambda t: t.replace("## Unreleased\n", "## Unreleased  \n"), "byte-exact"),
        ("line_above_unreleased", lambda t: t.replace("## Unreleased", "- T-0146: on top\n\n## Unreleased"), HAND),
    ]

    def test_hand_edits_fail(self):
        for name, edit, needle in self.EDITS:
            with self.subTest(name=name):
                self.git("checkout", "-q", "-f", "pr")
                self.git("reset", "-q", "--hard", "main")
                self.write_raw(edit(BASE2).encode("utf-8"))
                self.expect(1, needle)

    def test_binary_and_invalid_utf8_changelog(self):
        for name, data, needle in (("nul", b"# Changelog\n\x00\x01\x02\n", "contains NUL bytes"),
                                   ("invalid_utf8", BASE2.encode() + b"\xff\xfe", "is not valid UTF-8"),
                                   ("bom", b"\xef\xbb\xbf" + BASE2.encode(), "starts with a UTF-8 byte order mark")):
            with self.subTest(name=name):
                self.git("reset", "-q", "--hard", "main")
                self.write_raw(data)
                self.expect(1, "error: CHANGELOG.md " + needle)
                code, _, err = self.repo.check()  # plain check too
                self.assertEqual(code, 1)
                self.assertNotIn("Traceback", err)
                code, _, err = self.repo.assemble()
                self.assertEqual(code, 1)
                self.assertNotIn("Traceback", err)

    def test_invalid_utf8_at_merge_base(self):
        self.git("checkout", "-q", "main")
        self.write_raw(BASE2.encode() + b"\xff")
        self.commit("bad base")
        self.git("checkout", "-q", "pr")
        self.git("reset", "-q", "--hard", "main")
        self.write_raw(BASE2.encode())
        self.expect(1, "at the merge base is not valid UTF-8")

    def test_altered_fragment_line_fails(self):
        self.rm_frag("T-0150.md")
        self.write_raw(BASE2.replace("## Unreleased\n\n", "## Unreleased\n\n- T-0150: drilling!\n").encode())
        self.expect(1, "is not what tools/changelog.py assemble makes")

    def test_other_fragments_line_fails(self):
        self.rm_frag("T-0150.md")
        self.write_raw(BASE2.replace("## Unreleased\n\n", "## Unreleased\n\n- T-0151: engrave\n").encode())
        self.expect(1, "T-0151.md: this line is already in CHANGELOG.md")

    def test_partial_assemble_of_deleted_fragments_fails(self):
        self.rm_frag("T-0150.md")
        self.rm_frag("T-0151.md")
        self.write_raw(BASE2.replace("## Unreleased\n\n", "## Unreleased\n\n- T-0151: engrave\n").encode())
        self.expect(1, "(T-0151.md, T-0150.md)")

    def test_assemble_in_wrong_order_fails(self):
        self.rm_frag("T-0150.md")
        self.rm_frag("T-0151.md")
        self.write_raw(BASE2.replace("## Unreleased\n\n",
                                     "## Unreleased\n\n- T-0150: drilling\n- T-0151: engrave\n").encode())
        self.expect(1, "is not what tools/changelog.py assemble makes")

    def test_delete_fragment_without_assembling_fails(self):
        self.rm_frag("T-0150.md")
        self.expect(1, "T-0150.md deleted without being assembled")

    def test_rename_fragment_passes(self):
        self.rm_frag("T-0150.md")
        self.repo.frag("T-0146.md", "- T-0146: drilling\n")
        self.expect(0, "no hand edits")

    def test_merge_two_fragments_passes(self):
        self.rm_frag("T-0150.md")
        self.rm_frag("T-0151.md")
        self.repo.frag("T-0150-0151.md", "- T-0150/T-0151: drilling and engraving\n")
        self.expect(0)

    def test_rename_with_unmatched_content_fails(self):
        self.rm_frag("T-0150.md")
        self.repo.frag("T-0146.md", "- T-0150: drilling\n")
        self.expect(1, "must start with '- T-0146: '")

    def test_rename_plus_assembled_old_line_fails(self):
        self.rm_frag("T-0150.md")
        self.repo.frag("T-0146.md", "- T-0146: drilling\n")
        self.write_raw(BASE2.replace("## Unreleased\n\n", "## Unreleased\n\n- T-0150: drilling\n").encode())
        self.expect(1, "an assemble PR only assembles and deletes fragments")

    def test_assemble_passes(self):
        self.assertEqual(self.repo.assemble()[0], 0)
        self.assertEqual(self.repo.changelog(), BASE2.replace(
            "## Unreleased\n\n", "## Unreleased\n\n- T-0151: engrave\n- T-0150: drilling\n"))
        self.expect(0)

    def test_assemble_into_empty_unreleased_passes(self):
        empty = "# Changelog\n\n## Unreleased\n\n## 0.1.0\n\n- T-0120: released\n"
        self.git("checkout", "-q", "main")
        self.write_raw(empty.encode())
        self.commit("empty unreleased")
        self.git("checkout", "-q", "pr")
        self.git("reset", "-q", "--hard", "main")
        self.assertEqual(self.repo.assemble()[0], 0)
        self.assertEqual(self.repo.changelog(), empty.replace(
            "## Unreleased\n\n", "## Unreleased\n\n- T-0151: engrave\n- T-0150: drilling\n\n"))
        self.expect(0)

    def test_release_passes(self):
        for heading in ("## 0.2.0", "## 0.2.0 - 2026-11-03", "## 10.20.30"):
            with self.subTest(heading=heading):
                self.git("reset", "-q", "--hard", "main")
                self.write_raw(BASE2.replace("## Unreleased\n\n", "## Unreleased\n\n%s\n\n" % heading).encode())
                self.expect(0)

    def test_release_written_as_heading_then_blank_passes(self):
        # sed 's/^## Unreleased/&\n\n## 0.2.0/' gives the same bytes as inserting below the blank line.
        self.write_raw(BASE2.replace("## Unreleased\n", "## Unreleased\n\n## 0.2.0\n").encode())
        self.expect(0)

    def test_release_of_empty_unreleased_fails(self):
        empty = "# Changelog\n\n## Unreleased\n\n## 0.1.0\n\n- T-0120: released\n"
        self.git("checkout", "-q", "main")
        self.write_raw(empty.encode())
        self.commit("empty unreleased")
        self.git("checkout", "-q", "pr")
        self.git("reset", "-q", "--hard", "main")
        self.write_raw(empty.replace("## Unreleased\n\n", "## Unreleased\n\n## 0.2.0\n\n").encode())
        self.expect(1, "empty '## Unreleased' section")

    def test_release_with_fragment_deletion_fails(self):
        self.rm_frag("T-0150.md")
        self.write_raw(BASE2.replace("## Unreleased\n\n", "## Unreleased\n\n## 0.2.0\n\n").encode())
        self.expect(1, "is not what tools/changelog.py assemble makes")

    # Round 3: duplicate lines, committed blobs and file modes, release versions.
    def dup_fragment_on_main(self):
        """The stuck state: main holds a fragment whose line is already in CHANGELOG.md."""
        self.git("checkout", "-q", "main")
        self.repo.frag("T-0017.md", "- T-0017: older entry\n")
        self.commit("legacy duplicate fragment")
        self.git("checkout", "-q", "pr")
        self.git("reset", "-q", "--hard", "main")

    def test_adding_fragment_with_existing_line_fails(self):
        self.repo.frag("T-0017.md", "- T-0017: older entry\n")
        self.expect(1, "T-0017.md: this line is already in CHANGELOG.md")

    def test_duplicate_fragment_cleared_by_deleting_it(self):
        self.dup_fragment_on_main()
        self.rm_frag("T-0017.md")
        self.expect(0)

    def test_duplicate_fragment_cleared_by_assemble(self):
        self.dup_fragment_on_main()
        code, out, _ = self.repo.assemble()
        self.assertEqual(code, 0)
        self.assertIn("T-0017.md  (line already in CHANGELOG.md: deleted, not inserted again)", out)
        self.assertEqual(self.repo.names(), ["README.md"])
        self.assertEqual(self.repo.changelog().count("- T-0017: older entry\n"), 1)
        self.expect(0)

    def test_duplicate_fragment_deleted_with_hand_edit_fails(self):
        self.dup_fragment_on_main()
        self.rm_frag("T-0017.md")
        self.write_raw((BASE2 + "- T-0146: hand\n").encode())
        self.expect(1, HAND_MSG)

    def test_reads_committed_blobs_not_working_tree(self):
        # The committed CHANGELOG.md has a hand edit; the working tree shows the base bytes.
        self.write_raw((BASE2 + "- T-0146: hand\n").encode())
        self.commit("hand edit")
        self.write_raw(BASE2.encode())
        code, _, err = self.check_base()
        self.assertEqual(code, 1, err)
        self.assertIn(HAND_MSG, err)

    def test_working_tree_encoding_trick_fails(self):
        # .gitattributes working-tree-encoding makes the checkout look like the base while the
        # committed blob is mojibake: the blob is what counts.
        mojibake = BASE2.replace("## 0.1.0", "## 0.1.0 \u2014 x").encode("utf-8").decode("latin-1").encode("utf-8")
        self.git("checkout", "-q", "main")
        self.write_raw(BASE2.replace("## 0.1.0", "## 0.1.0 \u2014 x").encode())
        self.commit("dash")
        self.git("checkout", "-q", "pr")
        self.git("reset", "-q", "--hard", "main")
        sha = subprocess.run(["git", "hash-object", "-w", "--stdin", "--no-filters"], cwd=str(self.repo.dir),
                             input=mojibake, capture_output=True, check=True).stdout.decode().strip()
        self.git("update-index", "--cacheinfo", "100644,%s,CHANGELOG.md" % sha)
        self.git("commit", "-q", "-m", "mojibake")
        self.git("checkout", "-q", "--", "CHANGELOG.md")
        code, _, err = self.check_base()
        self.assertEqual(code, 1, err)
        self.assertIn(HAND_MSG, err)

    def blob_with_mode(self, path, data, mode):
        sha = subprocess.run(["git", "hash-object", "-w", "--stdin"], cwd=str(self.repo.dir),
                             input=data, capture_output=True, check=True).stdout.decode().strip()
        self.git("update-index", "--add", "--cacheinfo", "%s,%s,%s" % (mode, sha, path))
        self.git("commit", "-q", "-m", "mode " + mode)

    def test_symlink_changelog_blob_fails(self):
        self.blob_with_mode("CHANGELOG.md", b"docs/BACKLOG.md", "120000")
        # The working tree keeps a regular file (plain check passes); the committed blob is the link.
        code, _, err = self.check_base()
        self.assertEqual(code, 1, err)
        self.assertIn("CHANGELOG.md at HEAD must be a regular file (mode 100644), found mode 120000", err)

    def test_symlink_fragment_blob_fails(self):
        self.blob_with_mode("changelog.d/T-0146.md", b"T-0150.md", "120000")
        # The working tree keeps a regular file (plain check passes); the committed blob is the link.
        code, _, err = self.check_base()
        self.assertEqual(code, 1, err)
        self.assertIn("mode 100644), found mode 120000", err)

    def test_executable_fragment_blob_fails(self):
        self.blob_with_mode("changelog.d/T-0146.md", b"- T-0146: x\n", "100755")
        code, _, err = self.check_base()
        self.assertEqual(code, 1, err)
        self.assertIn("changelog.d/T-0146.md at HEAD must be a regular file (mode 100644), found mode 100755", err)

    def test_release_version_rules(self):
        cases = [("## 00.1.0", 1, "without leading zeros"), ("## 0.01.0", 1, "without leading zeros"),
                 ("## 0.0.1", 1, "not greater than the latest release 0.1.0"),
                 ("## 0.1.0 - 2026-12-01", 1, "already in the file"),
                 ("## 0.2.0 - 2026-99-99", 1, "not a valid date"), ("## 0.2.0 - 2026-02-30", 1, "not a valid date"),
                 ("## 0.2.0 - 2028-02-29", 0, None), ("## 0.1.1", 0, None), ("## 1.0.0", 0, None),
                 ("## 0.10.0", 0, None)]
        for heading, code, needle in cases:
            with self.subTest(heading=heading):
                self.git("reset", "-q", "--hard", "main")
                self.write_raw(BASE2.replace("## Unreleased\n\n", "## Unreleased\n\n%s\n\n" % heading).encode())
                self.expect(code, needle)

    def test_release_compares_versions_numerically(self):
        self.git("checkout", "-q", "main")
        self.write_raw(BASE2.replace("## 0.1.0", "## 0.9.0").encode())
        self.commit("0.9.0")
        self.git("checkout", "-q", "pr")
        self.git("reset", "-q", "--hard", "main")
        self.write_raw(BASE2.replace("## 0.1.0", "## 0.9.0").replace("## Unreleased\n\n", "## Unreleased\n\n## 0.10.0\n\n").encode())
        self.expect(0)

    def ci_merge_commit(self):
        """Like actions/checkout on a PR: main has moved on, HEAD is the merge of the PR into main."""
        self.commit("pr")
        self.git("checkout", "-q", "main")
        self.repo.frag("T-0146.md", "- T-0146: landed on main meanwhile\n")
        self.commit("other PR merged")
        self.git("checkout", "-q", "--detach", "main")
        self.git("merge", "-q", "--no-ff", "-m", "merge pr", "pr")

    def test_assemble_on_ci_merge_commit_passes(self):
        self.assertEqual(self.repo.assemble()[0], 0)
        self.ci_merge_commit()
        code, out, err = self.check_base()
        self.assertEqual(code, 0, out + err)

    def test_release_on_ci_merge_commit_passes(self):
        self.write_raw(BASE2.replace("## Unreleased\n\n", "## Unreleased\n\n## 0.2.0 - 2026-11-03\n\n").encode())
        self.ci_merge_commit()
        code, out, err = self.check_base()
        self.assertEqual(code, 0, out + err)

    def test_hand_edit_on_ci_merge_commit_fails(self):
        self.write_raw((BASE2 + "- T-0146: hand\n").encode())
        self.ci_merge_commit()
        self.assertEqual(self.check_base()[0], 1)


if __name__ == "__main__":
    unittest.main(verbosity=1)
