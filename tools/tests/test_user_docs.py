#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("chk", Path(__file__).resolve().parent.parent / "check_user_docs.py")
chk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chk)

SRC = '''menuBar()->addMenu(QCoreApplication::translate("MainWindow", "&File"));
file->addAction(QStringLiteral("&New Job..."));
'''


def run(pages):
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        (root / "app" / "src").mkdir(parents=True)
        (root / "docs" / "user").mkdir(parents=True)
        (root / "app" / "src" / "m.cpp").write_text(SRC, encoding="utf-8")
        for name, text in pages.items():
            (root / "docs" / "user" / name).write_text(text, encoding="utf-8")
        return chk.check(root)


class UserDocs(unittest.TestCase):
    def test_good(self):
        self.assertEqual(run({"README.md": "[q](q.md) **File > New Job...**\n", "q.md": "x\n"}), [])

    def test_unknown_menu_entry(self):
        self.assertTrue(run({"README.md": "**File > Nope**\n"}))

    def test_unknown_top_menu(self):
        self.assertTrue(run({"README.md": "**Nope > New Job...**\n"}))

    def test_broken_link(self):
        self.assertTrue(run({"README.md": "[x](missing.md)\n"}))

    def test_unlinked_page(self):
        self.assertTrue(run({"README.md": "hi\n", "q.md": "x\n"}))

    def test_real_docs_pass(self):
        self.assertEqual(chk.check(Path(__file__).resolve().parents[2]), [])


if __name__ == "__main__":
    unittest.main()
