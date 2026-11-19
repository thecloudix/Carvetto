#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("cat", Path(__file__).resolve().parent.parent / "check_catalog.py")
cat = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cat)

TS = """<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de" sourcelanguage="en">
<context><name>W</name>
<message><source>&amp;File</source><translation type="unfinished"></translation></message>
<message><source>Two
lines "quoted"</source><translation type="unfinished"></translation></message>
</context>
</TS>
"""


def run(source, ts=TS):
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        (root / "app" / "src").mkdir(parents=True)
        (root / "app" / "translations").mkdir()
        for name, text in source.items():
            (root / "app" / "src" / name).write_text(text, encoding="utf-8")
        (root / "app" / "translations" / "carvetto_de.ts").write_text(ts, encoding="utf-8")
        return cat.main(["--root", d])


class Catalog(unittest.TestCase):
    def test_listed_strings_pass(self):
        self.assertEqual(run({"a.cpp": 'x(QCoreApplication::translate("W", "&File"));\n'}), 0)

    def test_noop_and_multi_literal_and_escapes(self):
        src = 'const char* t = QT_TRANSLATE_NOOP("W", "Two\\n"\n    "lines \\"quoted\\"");\n'
        self.assertEqual(run({"a.cpp": src}), 0)

    def test_missing_string_is_reported(self):
        self.assertEqual(run({"a.cpp": 'x(QCoreApplication::translate("W", "Brand new"));\n'}), 1)

    def test_wrong_context_is_reported(self):
        self.assertEqual(run({"a.cpp": 'x(QCoreApplication::translate("Other", "&File"));\n'}), 1)

    def test_self_tests_and_comments_are_skipped(self):
        self.assertEqual(run({"a_self_test.cpp": 'x(QCoreApplication::translate("W", "Nope"));\n',
                              "b.cpp": '// QCoreApplication::translate("W", "Nope")\n'}), 0)

    def test_the_shipped_catalog_is_current(self):
        root = Path(__file__).resolve().parent.parent.parent
        self.assertEqual(cat.main(["--root", str(root)]), 0)


if __name__ == "__main__":
    unittest.main()
