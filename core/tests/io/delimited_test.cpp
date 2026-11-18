#include <catch2/catch_test_macros.hpp>

#include <string>

#include "carvetto/io/delimited.hpp"

using namespace carvetto::io;

TEST_CASE("delimited: plain records, CRLF, blank lines and BOM", "[io][delimited]") {
    const DelimitedTable t = parseDelimited("\xEF\xBB\xBFname,qty\r\nAnna,3\r\n\r\nBob,4\n");
    REQUIRE(t.ok());
    REQUIRE(t.records.size() == 3);
    CHECK(t.columns == 2);
    CHECK(t.records[0].fields == std::vector<std::string>{"name", "qty"});
    CHECK(t.records[1].line == 2);
    CHECK(t.records[2].line == 4);
    CHECK(t.records[2].fields[0] == "Bob");
}

TEST_CASE("delimited: quoting, embedded separators, quotes and line breaks", "[io][delimited]") {
    const DelimitedTable t = parseDelimited("a,b\n\"x,y\",\"say \"\"hi\"\"\"\n\"two\nlines\",z\nlast,\n");
    REQUIRE(t.ok());
    REQUIRE(t.records.size() == 4);
    CHECK(t.records[1].fields[0] == "x,y");
    CHECK(t.records[1].fields[1] == "say \"hi\"");
    CHECK(t.records[2].fields[0] == "two\nlines");
    CHECK(t.records[2].line == 3);
    CHECK(t.records[3].line == 5);  // the quoted line break was counted
    CHECK(t.records[3].fields[1].empty());
}

TEST_CASE("delimited: separators", "[io][delimited]") {
    DelimitedOptions tab;
    tab.separator = '\t';
    CHECK(parseDelimited("a\tb\n1\t2", tab).records[1].fields[1] == "2");
    DelimitedOptions semi;
    semi.separator = ';';
    CHECK(parseDelimited("a;b\n1,5;2", semi).records[1].fields[0] == "1,5");
    DelimitedOptions bad;
    bad.separator = '"';
    CHECK_FALSE(parseDelimited("a", bad).fatal.empty());
}

TEST_CASE("delimited: malformed records are reported with line numbers and skipped", "[io][delimited]") {
    const DelimitedTable t = parseDelimited("a,b\n1,2\n3\n4,5,6\n\"open,1\n7,8\n");
    REQUIRE(t.diagnostics.size() >= 3);
    CHECK(t.diagnostics[0].line == 3);
    CHECK(t.diagnostics[1].line == 4);
    CHECK(t.diagnostics[2].line == 5);  // unterminated quote swallows the rest
    CHECK(t.records.size() == 2);
    CHECK_FALSE(t.ok());
}

TEST_CASE("delimited: stray quotes and control characters", "[io][delimited]") {
    const DelimitedTable t = parseDelimited("a,b\nab\"c,1\n\"q\"x,2\nok\x01,3\nfine,4\n");
    REQUIRE(t.diagnostics.size() == 3);
    CHECK(t.diagnostics[0].line == 2);
    CHECK(t.diagnostics[1].line == 3);
    CHECK(t.diagnostics[2].line == 4);
    REQUIRE(t.records.size() == 2);
    CHECK(t.records[1].fields[0] == "fine");
}

TEST_CASE("delimited: caps", "[io][delimited]") {
    CHECK_FALSE(parseDelimited(std::string(kDelimitedMaxBytes + 1, 'a')).fatal.empty());
    std::string many;
    for (std::size_t i = 0; i < kDelimitedMaxRecords + 1; ++i) {
        many += "x\n";
    }
    CHECK_FALSE(parseDelimited(many).fatal.empty());
    const DelimitedTable longField = parseDelimited("a\n" + std::string(kDelimitedMaxFieldBytes + 10, 'z') + "\nb\n");
    REQUIRE(longField.diagnostics.size() == 1);
    CHECK(longField.diagnostics[0].line == 2);
    CHECK(longField.records.size() == 2);
    std::string wide;
    for (int i = 0; i < 300; ++i) {
        wide += "c,";
    }
    CHECK_FALSE(parseDelimited(wide + "c\n").diagnostics.empty());
    std::string bad;
    for (int i = 0; i < 500; ++i) {
        bad += "a,b\n1\n";
    }
    const DelimitedTable t = parseDelimited(bad);
    CHECK(t.diagnostics.size() == kDelimitedMaxDiagnostics);
    CHECK(t.suppressed > 0);
}

TEST_CASE("delimited: empty and header-only input", "[io][delimited]") {
    CHECK(parseDelimited("").records.empty());
    CHECK(parseDelimited("\n\n").records.empty());
    CHECK(parseDelimited("only,header").records.size() == 1);
}
