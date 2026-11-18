#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("chk", Path(__file__).resolve().parent.parent / "check_tr_literals.py")
chk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chk)


def run(files, baseline=None):
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        (root / "app" / "src").mkdir(parents=True)
        (root / "tools").mkdir()
        for name, text in files.items():
            (root / "app" / "src" / name).write_text(text, encoding="utf-8")
        if baseline is not None:
            (root / "tools" / "tr_literals_baseline.txt").write_text(baseline, encoding="utf-8")
        return chk.main(["--root", d])


class TrLiterals(unittest.TestCase):
    def test_literal_menu_is_flagged(self):
        self.assertEqual(run({"a.cpp": 'x->addMenu(QStringLiteral("&File"));\n'}), 1)

    def test_translate_is_accepted(self):
        self.assertEqual(run({"a.cpp": 'x->addMenu(QCoreApplication::translate("W", "&File"));\n'}), 0)

    def test_object_names_and_data_are_ignored(self):
        self.assertEqual(run({"a.cpp": 'w->setObjectName(QStringLiteral("optionsDialog"));\n'}), 0)

    def test_opt_out_comment_and_comments(self):
        self.assertEqual(run({"a.cpp": 'x->setText(QStringLiteral("Deutsch"));  // lint: no-tr\n'
                                       '// x->setText(QStringLiteral("Hello"));\n'}), 0)

    def test_baseline_grandfathers_but_does_not_grow(self):
        src = 'a->setText(QStringLiteral("One"));\nb->setText(QStringLiteral("Two"));\n'
        self.assertEqual(run({"a.cpp": src}, "app/src/a.cpp 2\n"), 0)
        self.assertEqual(run({"a.cpp": src}, "app/src/a.cpp 1\n"), 1)

    def test_self_tests_are_exempt(self):
        self.assertEqual(run({"a_self_test.cpp": 'x->setText(QStringLiteral("Hello"));\n'}), 0)


if __name__ == "__main__":
    unittest.main()
