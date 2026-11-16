#include "carvetto/io/parts_import.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <string>

using namespace carvetto::io;

namespace {

const char* kMapping =
    "units = mm\n"
    "column.length = Length\n"
    "column.width = Width\n"
    "column.thickness = Thickness\n"
    "column.material = Material\n"
    "column.label = Name\n"
    "column.quantity = Qty\n"
    "column.holes = Holes\n"
    "column.dados = Dados\n";

PartsMapping mapping(const std::string& text = kMapping) {
    const MappingResult r = parsePartsMapping(text);
    REQUIRE(r.ok);
    return r.mapping;
}

bool hasText(const std::vector<PartsDiagnostic>& d, const std::string& needle) {
    return std::any_of(d.begin(), d.end(), [&](const PartsDiagnostic& x) { return x.text.find(needle) != std::string::npos; });
}

const char* kHeader = "Name,Length,Width,Thickness,Material,Qty,Holes,Dados\n";

}  // namespace

TEST_CASE("csv: quotes, embedded newlines, doubled quotes, BOM, CRLF and CR", "[parts][csv]") {
    const std::string text = "\xEF\xBB\xBF" "a,b,c\r\n\"x,1\",\"he said \"\"hi\"\"\",\"two\nlines\"\rlast,,\n";
    const CsvResult r = parseCsv(text);
    REQUIRE(r.ok);
    REQUIRE(r.table.rows.size() == 3);
    CHECK(r.table.rows[0][0] == "a");  // BOM gone
    CHECK(r.table.rows[1][0] == "x,1");
    CHECK(r.table.rows[1][1] == "he said \"hi\"");
    CHECK(r.table.rows[1][2] == "two\nlines");
    CHECK(r.table.rows[2] == std::vector<std::string>{"last", "", ""});
    CHECK(r.table.lines == std::vector<std::size_t>{1, 2, 3});
}

TEST_CASE("csv: blank records are skipped but still counted in line numbers", "[parts][csv]") {
    const CsvResult r = parseCsv("a,b\n\n\nc,d\n");
    REQUIRE(r.ok);
    REQUIRE(r.table.rows.size() == 2);
    CHECK(r.table.lines[1] == 4);
}

TEST_CASE("csv: delimiter detection", "[parts][csv]") {
    CHECK(parseCsv("a;b;c\n1;2;3\n").table.delimiter == ';');
    CHECK(parseCsv("a\tb\tc\n1\t2\t3\n").table.delimiter == '\t');
    CHECK(parseCsv("a,b,c\n").table.delimiter == ',');
    CHECK(parseCsv("single\nvalue\n").table.delimiter == ',');
    // A quoted delimiter does not count.
    CHECK(parseCsv("\"a;b;c\",d\n").table.delimiter == ',');
    // Forced delimiter wins.
    const CsvResult forced = parseCsv("a;b,c\n", ',');
    CHECK(forced.table.rows[0] == std::vector<std::string>{"a;b", "c"});
    // Mixed: the most frequent is used and a warning is given.
    const CsvResult mixed = parseCsv("a,b,c;d\n1,2,3;4\n");
    CHECK(mixed.table.delimiter == ',');
    CHECK(hasText(mixed.diagnostics, "more than one delimiter"));
    // Tie: comma before semicolon before tab.
    CHECK(parseCsv("a,b;c\n").table.delimiter == ',');
    CHECK(parseCsv("a;b\tc\n").table.delimiter == ';');
}

TEST_CASE("csv: hostile input is refused with a diagnostic, never a crash", "[parts][csv]") {
    SECTION("unterminated quote") {
        const CsvResult r = parseCsv("a,b\n\"never closed,1\n2,3\n");
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "not closed"));
    }
    SECTION("NUL byte") {
        const CsvResult r = parseCsv(std::string("a,b\n1,\0 2\n", 10));
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "NUL"));
    }
    SECTION("huge cell, quoted and not") {
        const std::string big(kMaxCsvFieldBytes + 1, 'x');
        CHECK_FALSE(parseCsv("a,b\n" + big + ",1\n").ok);
        CHECK_FALSE(parseCsv("a,b\n\"" + big + "\",1\n").ok);
        CHECK(parseCsv("a,b\n" + std::string(kMaxCsvFieldBytes, 'x') + ",1\n").ok);
    }
    SECTION("too many columns") {
        std::string row;
        for (std::size_t i = 0; i <= kMaxCsvColumns; ++i) row += (i ? "," : "") + std::string("c");
        CHECK_FALSE(parseCsv(row + "\n").ok);
        row.erase(row.size() - 2);
        CHECK(parseCsv(row + "\n").ok);
    }
    SECTION("too many records") {
        std::string text;
        for (std::size_t i = 0; i < kMaxCsvRecords + 1; ++i) text += "a,b\n";
        CHECK_FALSE(parseCsv(text).ok);
        text.erase(text.size() - 4);
        CHECK(parseCsv(text).ok);
    }
    SECTION("file too large") {
        const std::string text(kMaxPartsFileBytes + 1, 'a');
        const CsvResult r = parseCsv(text);
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "larger than"));
    }
    SECTION("text after a closing quote is kept with a warning") {
        const CsvResult r = parseCsv("\"ab\"cd,e\n");
        REQUIRE(r.ok);
        CHECK(r.table.rows[0][0] == "abcd");
        CHECK(hasText(r.diagnostics, "after a closing quote"));
    }
    SECTION("empty input is an empty table") {
        const CsvResult r = parseCsv("");
        CHECK(r.ok);
        CHECK(r.table.rows.empty());
    }
}

TEST_CASE("mapping: a valid file", "[parts][mapping]") {
    const PartsMapping m = mapping(std::string("# a comment\n\n") + kMapping +
                                   "default.material = Birch ply\n"
                                   "layer.panels = Parts\nlayer.holes = Drill\nlayer.dados = Grooves\n"
                                   "sheet.length_mm = 2800\nsheet.width_mm = 2070\n"
                                   "nest.gap_mm = 8\nnest.margin_mm = 12\nnest.rotate = yes\n"
                                   "delimiter = semicolon\nheader = yes\n");
    CHECK(m.units == PartsUnits::Millimetres);
    CHECK(m.length == ColumnRef{ColumnRef::Kind::Name, "Length", 0});
    CHECK(m.defaultMaterial == "Birch ply");
    CHECK(m.panelLayer == "Parts");
    CHECK(m.holeLayer == "Drill");
    CHECK(m.dadoLayer == "Grooves");
    CHECK(m.sheetLengthMm == 2800.0);
    CHECK(m.sheetWidthMm == 2070.0);
    CHECK(m.gapMm == 8.0);
    CHECK(m.marginMm == 12.0);
    CHECK(m.rotate);
    CHECK(m.delimiter == ';');
}

TEST_CASE("mapping: keys are case-insensitive, CRLF and BOM tolerated, index columns", "[parts][mapping]") {
    const MappingResult r = parsePartsMapping("\xEF\xBB\xBF" "UNITS = Inch\r\nColumn.Length = #1\r\ncolumn.width=#2\r\ndefault.thickness_mm = 18\r\nheader = no\r\n");
    REQUIRE(r.ok);
    CHECK(r.mapping.units == PartsUnits::Inches);
    CHECK(r.mapping.length == ColumnRef{ColumnRef::Kind::Index, "", 1});
    CHECK(r.mapping.width.index == 2);
    CHECK_FALSE(r.mapping.header);
    CHECK(*r.mapping.defaultThicknessMm == 18.0);
}

TEST_CASE("mapping: diagnostics name the line and the problem", "[parts][mapping]") {
    auto errors = [](const std::string& extra) {
        return parsePartsMapping(std::string(kMapping) + extra);
    };
    SECTION("unknown key") {
        const MappingResult r = errors("colour = red\n");
        CHECK_FALSE(r.ok);
        REQUIRE_FALSE(r.diagnostics.empty());
        CHECK(r.diagnostics.front().line == 10);
        CHECK(r.diagnostics.front().source == "mapping");
        CHECK(hasText(r.diagnostics, "unknown key `colour`"));
    }
    SECTION("missing equals") {
        const MappingResult r = errors("just words\n");
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "expected `key = value`"));
    }
    SECTION("repeated key") {
        const MappingResult r = errors("units = inch\n");
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "more than once"));
    }
    SECTION("bad values") {
        CHECK(hasText(errors("nest.gap_mm = lots\n").diagnostics, "`nest.gap_mm` must be a number"));
        CHECK(hasText(errors("nest.gap_mm = -1\n").diagnostics, "`nest.gap_mm` must be a number"));
        CHECK(hasText(errors("sheet.length_mm = 5\n").diagnostics, "`sheet.length_mm`"));
        CHECK(hasText(errors("sheet.length_mm = 100001\n").diagnostics, "`sheet.length_mm`"));
        CHECK(errors("sheet.length_mm = 100000\n").ok);
        CHECK(errors("sheet.length_mm = 10\n").ok);
        CHECK(hasText(errors("nest.rotate = maybe\n").diagnostics, "yes or no"));
        CHECK(hasText(errors("header = maybe\n").diagnostics, "yes or no"));
        CHECK(hasText(errors("delimiter = pipe\n").diagnostics, "auto, comma, semicolon or tab"));
        CHECK(hasText(parsePartsMapping("units = cubits\ncolumn.length=a\ncolumn.width=b\ndefault.thickness_mm=1\n").diagnostics,
                      "must be mm or inch"));
        CHECK(hasText(errors("column.units = #0\n").diagnostics, "column index"));
        CHECK(hasText(errors("column.units = #65\n").diagnostics, "column index"));
        CHECK(hasText(errors("column.units = \n").diagnostics, "needs a header name"));
        CHECK(hasText(errors("default.thickness_mm = 0\n").diagnostics, "default.thickness_mm"));
        CHECK(hasText(errors("layer.panels = \n").diagnostics, "layer name"));
    }
    SECTION("same column for two roles") {
        const MappingResult r = parsePartsMapping(
            "units = mm\ncolumn.length = Size\ncolumn.width = size\ncolumn.thickness = #3\ncolumn.holes = #3\n");
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "mapped to both length and width"));
        CHECK(hasText(r.diagnostics, "mapped to both thickness and holes"));
    }
    SECTION("required keys") {
        const MappingResult r = parsePartsMapping("");
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "`column.length` is required"));
        CHECK(hasText(r.diagnostics, "`column.width` is required"));
        CHECK(hasText(r.diagnostics, "column.thickness"));
        CHECK(hasText(r.diagnostics, "units must be explicit"));
    }
    SECTION("units may come from a column instead") {
        CHECK(parsePartsMapping("column.length=a\ncolumn.width=b\ndefault.thickness_mm=18\ncolumn.units=U\n").ok);
    }
    SECTION("no header needs index columns") {
        const MappingResult r = parsePartsMapping("units=mm\nheader=no\ncolumn.length=#1\ncolumn.width=Wide\ndefault.thickness_mm=18\n");
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "use #index"));
    }
    SECTION("layers must differ") {
        const MappingResult r = errors("layer.holes = Panels\n");
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "different names"));
    }
    SECTION("limits") {
        std::string big(kMaxMappingBytes + 1, '#');
        CHECK_FALSE(parsePartsMapping(big).ok);
        std::string lines;
        for (std::size_t i = 0; i < kMaxMappingLines + 1; ++i) lines += "#\n";
        const MappingResult r = parsePartsMapping(lines);
        CHECK_FALSE(r.ok);
        CHECK(hasText(r.diagnostics, "more than 512 lines"));
        const MappingResult longLine = parsePartsMapping(std::string(kMaxMappingLineBytes + 1, 'k') + " = v\n");
        CHECK(hasText(longLine.diagnostics, "line longer than"));
        CHECK_FALSE(parsePartsMapping(std::string("units=mm\0\n", 10)).ok);
    }
}

TEST_CASE("parts: a full row with every column", "[parts][read]") {
    const std::string csv = std::string(kHeader) +
                            "Side,720,560,18,Oak ply,2,100:50:8|620:50:8:10,L:100:18:9|W:20:5:3\n";
    const PartsList l = readPartsText(csv, mapping());
    REQUIRE_FALSE(l.hasErrors());
    REQUIRE(l.parts.size() == 1);
    const Part& p = l.parts[0];
    CHECK(p.label == "Side");
    CHECK(p.material == "Oak ply");
    CHECK(p.lengthMm == 720.0);
    CHECK(p.widthMm == 560.0);
    CHECK(p.thicknessMm == 18.0);
    CHECK(p.quantity == 2);
    CHECK(p.line == 2);
    CHECK(l.instances == 2);
    REQUIRE(p.holes.size() == 2);
    CHECK(p.holes[0] == PartHole{100, 50, 8, 0});
    CHECK(p.holes[1] == PartHole{620, 50, 8, 10});
    REQUIRE(p.dados.size() == 2);
    CHECK(p.dados[0] == PartDado{true, 100, 18, 9});
    CHECK(p.dados[1] == PartDado{false, 20, 5, 3});
}

TEST_CASE("parts: inches via the mapping and via a units column", "[parts][read]") {
    const PartsMapping inch = mapping("units = inch\ncolumn.length=L\ncolumn.width=W\ncolumn.thickness=T\ncolumn.holes=H\n");
    const PartsList a = readPartsText("L,W,T,H\n10,5.5,0.75,1:2:0.25\n", inch);
    REQUIRE_FALSE(a.hasErrors());
    CHECK(a.parts[0].lengthMm == 254.0);
    CHECK(std::abs(a.parts[0].widthMm - 139.7) < 1e-9);
    CHECK(std::abs(a.parts[0].thicknessMm - 19.05) < 1e-9);
    CHECK(std::abs(a.parts[0].holes[0].xMm - 25.4) < 1e-9);
    CHECK(std::abs(a.parts[0].holes[0].diameterMm - 6.35) < 1e-9);

    const PartsMapping col = mapping("column.length=L\ncolumn.width=W\ncolumn.thickness=T\ncolumn.units=U\n");
    const PartsList b = readPartsText("L,W,T,U\n100,50,18,mm\n10,5,1,IN\n100,50,18,\n", col);
    CHECK(b.hasErrors());  // the third row has no units and the mapping sets none
    REQUIRE(b.parts.size() == 2);
    CHECK(b.parts[0].lengthMm == 100.0);
    CHECK(b.parts[1].lengthMm == 254.0);
    CHECK(hasText(b.diagnostics, "no units and the mapping sets none"));
    CHECK(hasText(readPartsText("L,W,T,U\n100,50,18,cubit\n", col).diagnostics, "unknown units"));

    // A row's units cell overrides the mapping's setting.
    const PartsMapping both = mapping("units=mm\ncolumn.length=L\ncolumn.width=W\ncolumn.thickness=T\ncolumn.units=U\n");
    const PartsList c = readPartsText("L,W,T,U\n10,5,1,inches\n10,5,1,\n", both);
    REQUIRE(c.parts.size() == 2);
    CHECK(c.parts[0].lengthMm == 254.0);
    CHECK(c.parts[1].lengthMm == 10.0);
}

TEST_CASE("parts: defaults, labels and header matching", "[parts][read]") {
    const PartsMapping m = mapping("units=mm\ncolumn.length= LENGTH \ncolumn.width=width\ndefault.thickness_mm=16\ndefault.material=MDF\ncolumn.label=Name\ncolumn.material=Mat\n");
    const PartsList l = readPartsText("length,Width ,Name,Mat\n500,300,,\n400,200,Shelf,Pine\n", m);
    REQUIRE_FALSE(l.hasErrors());
    REQUIRE(l.parts.size() == 2);
    CHECK(l.parts[0].thicknessMm == 16.0);
    CHECK(l.parts[0].material == "MDF");
    CHECK(l.parts[0].label == "Part 2");
    CHECK(l.parts[0].quantity == 1);
    CHECK(l.parts[1].label == "Shelf");
    CHECK(l.parts[1].material == "Pine");
}

TEST_CASE("parts: column resolution diagnostics", "[parts][read]") {
    SECTION("unknown header lists the headers") {
        const PartsMapping m = mapping("units=mm\ncolumn.length=Len\ncolumn.width=Width\ndefault.thickness_mm=18\n");
        const PartsList l = readPartsText("Length,Width\n1,2\n", m);
        CHECK(l.hasErrors());
        CHECK(l.parts.empty());
        CHECK(hasText(l.diagnostics, "column.length = \"Len\" matches no header"));
        CHECK(hasText(l.diagnostics, "\"Length\", \"Width\""));
    }
    SECTION("ambiguous header") {
        const PartsList l = readPartsText("Length,Width,Length\n1,2,3\n", mapping("units=mm\ncolumn.length=Length\ncolumn.width=Width\ndefault.thickness_mm=18\n"));
        CHECK(hasText(l.diagnostics, "matches 2 headers"));
    }
    SECTION("index beyond the header") {
        const PartsList l = readPartsText("a,b\n1,2\n", mapping("units=mm\ncolumn.length=#1\ncolumn.width=#5\ndefault.thickness_mm=18\n"));
        CHECK(hasText(l.diagnostics, "column.width is #5 but the header has 2 columns"));
    }
    SECTION("empty and header-only") {
        const PartsMapping m = mapping();
        CHECK(hasText(readPartsText("", m).diagnostics, "empty"));
        CHECK(hasText(readPartsText(kHeader, m).diagnostics, "no rows"));
    }
    SECTION("header = no uses the first record as data") {
        const PartsList l = readPartsText("100,50\n", mapping("units=mm\nheader=no\ncolumn.length=#1\ncolumn.width=#2\ndefault.thickness_mm=18\n"));
        REQUIRE(l.parts.size() == 1);
        CHECK(l.parts[0].lengthMm == 100.0);
        CHECK(l.parts[0].line == 1);
    }
    SECTION("extra cells warn, missing cells are empty") {
        const PartsMapping m = mapping("units=mm\ncolumn.length=L\ncolumn.width=W\ncolumn.thickness=T\ncolumn.label=N\n");
        const PartsList l = readPartsText("L,W,T,N\n100,50,18,x,extra\n100,50,18\n", m);
        CHECK_FALSE(l.hasErrors());
        CHECK(hasText(l.diagnostics, "more cells than the header"));
        CHECK(l.parts.size() == 2);
        CHECK(l.parts[1].label == "Part 3");
    }
}

TEST_CASE("parts: bad rows are skipped and reported by line", "[parts][read]") {
    const PartsMapping m = mapping();
    const std::string csv = std::string(kHeader) +
                            "ok,100,50,18,A,1,,\n"
                            "bad length,abc,50,18,A,1,,\n"
                            "ok2,100,50,18,A,1,,\n";
    const PartsList l = readPartsText(csv, m);
    CHECK(l.hasErrors());
    CHECK(l.rowsSkipped == 1);
    REQUIRE(l.parts.size() == 2);
    CHECK(l.parts[1].label == "ok2");
    bool found = false;
    for (const auto& d : l.diagnostics) {
        if (d.line == 3 && d.text.find("length \"abc\" is not a number") != std::string::npos) found = true;
    }
    CHECK(found);
}

TEST_CASE("parts: numbers are strict (formula-looking and exotic forms are refused)", "[parts][read]") {
    const PartsMapping m = mapping();
    auto row = [&](const std::string& length) {
        return readPartsText(std::string(kHeader) + "n," + length + ",50,18,A,1,,\n", m);
    };
    CHECK_FALSE(row("100").hasErrors());
    CHECK_FALSE(row(" 100.5 ").hasErrors());
    CHECK(row(".5e1").hasErrors());  // exponent refused
    CHECK(row("=1+1").hasErrors());
    CHECK(row("+100").hasErrors());
    CHECK(row("1e3").hasErrors());
    CHECK(row("0x10").hasErrors());
    CHECK(row("inf").hasErrors());
    CHECK(row("nan").hasErrors());
    CHECK(row("1.2.3").hasErrors());
    CHECK(row("-").hasErrors());
    CHECK(row("").hasErrors());
    CHECK(row("\"1,5\"").hasErrors());       // quoted: decimal comma is not accepted
    CHECK(row(std::string(40, '1')).hasErrors());
    CHECK(row("-100").hasErrors());
    CHECK(row("0.9999").hasErrors());
    CHECK_FALSE(row("1").hasErrors());
    CHECK_FALSE(row("10000").hasErrors());
    CHECK(row("10000.01").hasErrors());
}

TEST_CASE("parts: text cells are inert and cleaned", "[parts][read]") {
    const PartsMapping m = mapping();
    const std::string csv = std::string(kHeader) +
                            "\"=HYPERLINK(\"\"x\"\")\",100,50,18,@evil,1,,\n"
                            "\"line1\nline2\x01\",100,50,18,A,1,,\n" +
                            "\"" + std::string(500, 'z') + "\",100,50,18,A,1,,\n";
    const PartsList l = readPartsText(csv, m);
    REQUIRE_FALSE(l.hasErrors());
    REQUIRE(l.parts.size() == 3);
    CHECK(l.parts[0].label == "=HYPERLINK(\"x\")");
    CHECK(hasText(l.diagnostics, "starts with '='"));
    CHECK(l.parts[0].material == "@evil");
    CHECK(l.parts[1].label == "line1 line2");  // control characters and the trailing one trimmed
    CHECK(l.parts[2].label.size() == kMaxLabelBytes);
    CHECK(hasText(l.diagnostics, "cleaned of control characters or shortened"));
}

TEST_CASE("parts: label truncation never splits a UTF-8 sequence", "[parts][read]") {
    std::string label(kMaxLabelBytes - 1, 'a');
    label += "\xC3\xA9";  // e-acute straddles the limit
    const PartsList l = readPartsText(std::string(kHeader) + label + ",100,50,18,A,1,,\n", mapping());
    REQUIRE(l.parts.size() == 1);
    CHECK(l.parts[0].label == std::string(kMaxLabelBytes - 1, 'a'));
}

TEST_CASE("parts: quantity", "[parts][read]") {
    const PartsMapping m = mapping();
    auto q = [&](const std::string& v) { return readPartsText(std::string(kHeader) + "n,100,50,18,A," + v + ",,\n", m); };
    CHECK(q("3").parts[0].quantity == 3);
    CHECK(q("").parts[0].quantity == 1);
    CHECK(q("1000").parts[0].quantity == 1000);
    CHECK(q("0").hasErrors());
    CHECK(q("1001").hasErrors());
    CHECK(q("2.5").hasErrors());
    CHECK(q("-1").hasErrors());
    CHECK(q("abc").hasErrors());
    CHECK(q("1234567890").hasErrors());
}

TEST_CASE("parts: total instances are bounded", "[parts][read]") {
    std::string csv = kHeader;
    for (int i = 0; i < 6; ++i) csv += "n,100,50,18,A,1000,,\n";
    const PartsList l = readPartsText(csv, mapping());
    CHECK(l.hasErrors());
    CHECK(l.instances == 5000);
    CHECK(l.parts.size() == 5);
    CHECK(hasText(l.diagnostics, "add up to more than 5000"));
}

TEST_CASE("parts: row count is bounded", "[parts][read]") {
    std::string csv = kHeader;
    for (std::size_t i = 0; i < kMaxPartRows + 1; ++i) csv += "n,100,50,18,A,,,\n";
    const PartsList l = readPartsText(csv, mapping());
    CHECK(l.hasErrors());
    CHECK(l.parts.empty());
    CHECK(hasText(l.diagnostics, "more than 10000 rows"));
}

TEST_CASE("parts: holes", "[parts][read]") {
    const PartsMapping m = mapping();
    auto h = [&](const std::string& cell) {
        return readPartsText(std::string(kHeader) + "n,100,50,18,A,1,\"" + cell + "\",\n", m);
    };
    CHECK(h("10:10:5").parts[0].holes.size() == 1);
    CHECK(h("10:10:5|").parts[0].holes.size() == 1);  // trailing separator
    CHECK(h("10:10:5 | 20:20:5:9").parts[0].holes[1].depthMm == 9.0);
    CHECK(h("10:10:5:18").parts[0].holes[0].depthMm == 0.0);  // full thickness = through
    CHECK(h("10:10:5:").parts[0].holes[0].depthMm == 0.0);
    CHECK(h("10:10").hasErrors());
    CHECK(h("10:10:5:6:7").hasErrors());
    CHECK(h("a:10:5").hasErrors());
    CHECK(h("10:10:0").hasErrors());
    CHECK(h("10:10:-3").hasErrors());
    CHECK(h("10:10:5:-1").hasErrors());
    CHECK(h("10:10:5:18.5").hasErrors());
    CHECK(hasText(h("10:10:5:18.5").diagnostics, "depth must be 0 (through) up to the thickness"));
    // Edge fit: centre 5 with d 10 touches the edge and is allowed; 4.99 is not.
    CHECK_FALSE(h("5:5:10").hasErrors());
    CHECK(h("4.99:5:10").hasErrors());
    CHECK(h("5:4.99:10").hasErrors());
    CHECK_FALSE(h("95:45:10").hasErrors());
    CHECK(h("95.01:45:10").hasErrors());
    CHECK(h("95:45.01:10").hasErrors());
    CHECK(hasText(h("200:10:5").diagnostics, "does not lie inside the panel"));
    // Limit: 200 holes pass, 201 fail.
    std::string many;
    for (int i = 0; i < 200; ++i) many += (i ? "|" : "") + std::string("10:10:2");
    CHECK_FALSE(h(many).hasErrors());
    CHECK(h(many + "|10:10:2").hasErrors());
    CHECK(hasText(h(many + "|10:10:2").diagnostics, "more than 200 holes"));
}

TEST_CASE("parts: dados", "[parts][read]") {
    const PartsMapping m = mapping();
    auto d = [&](const std::string& cell) {
        return readPartsText(std::string(kHeader) + "n,100,50,18,A,1,,\"" + cell + "\"\n", m);
    };
    CHECK(d("L:10:10:9").parts[0].dados[0] == PartDado{true, 10, 10, 9});
    CHECK(d("w:10:10:9").parts[0].dados[0] == PartDado{false, 10, 10, 9});
    CHECK(d("L:10:10:9|W:0:100:17.9").parts[0].dados.size() == 2);
    CHECK(d("X:10:10:9").hasErrors());
    CHECK(d("L:10:10").hasErrors());
    CHECK(d("L:10:10:9:1").hasErrors());
    CHECK(d("L:a:10:9").hasErrors());
    // Extent: along L the groove spans the width (50), along W the length (100).
    CHECK_FALSE(d("L:40:10:9").hasErrors());
    CHECK(d("L:40.01:10:9").hasErrors());
    CHECK_FALSE(d("W:90:10:9").hasErrors());
    CHECK(d("W:90.01:10:9").hasErrors());
    CHECK(d("L:-1:10:9").hasErrors());
    CHECK(d("L:0:0:9").hasErrors());
    CHECK_FALSE(d("L:0:10:9").hasErrors());
    // Depth strictly between 0 and the thickness.
    CHECK(d("L:10:10:0").hasErrors());
    CHECK(d("L:10:10:18").hasErrors());
    CHECK(hasText(d("L:10:10:18").diagnostics, "less than the thickness"));
    CHECK_FALSE(d("L:10:10:17.99").hasErrors());
    std::string many;
    for (int i = 0; i < 32; ++i) many += (i ? "|" : "") + std::string("L:0:1:1");
    CHECK_FALSE(d(many).hasErrors());
    CHECK(hasText(d(many + "|L:0:1:1").diagnostics, "more than 32 dados"));
}

TEST_CASE("parts: diagnostics are capped but errors still counted", "[parts][read]") {
    std::string csv = kHeader;
    for (int i = 0; i < 500; ++i) csv += "n,abc,50,18,A,1,,\n";
    const PartsList l = readPartsText(csv, mapping());
    CHECK(l.errorCount == 500);
    CHECK(l.diagnostics.size() == kMaxDiagnostics + 1);
    CHECK(hasText(l.diagnostics, "further messages not shown"));
}

TEST_CASE("readBoundedFile checks the size before reading", "[parts][files]") {
    std::string err;
    CHECK_FALSE(readBoundedFile("/nonexistent/definitely/missing.csv", 100, &err));
    CHECK_FALSE(err.empty());
}

// --- Nesting ----------------------------------------------------------------------------------

namespace {

struct Rect {
    double x0, y0, x1, y1;
};

Rect placed(const NestItem& it, const NestPlacement& p) {
    const double w = p.rotated ? it.widthMm : it.lengthMm;
    const double h = p.rotated ? it.lengthMm : it.widthMm;
    return {p.xMm, p.yMm, p.xMm + w, p.yMm + h};
}

void checkLayout(const std::vector<NestItem>& items, const NestOptions& o, const NestResult& r) {
    std::vector<int> seen(items.size(), 0);
    for (const NestPlacement& p : r.placements) {
        REQUIRE(p.item < items.size());
        ++seen[p.item];
        const Rect a = placed(items[p.item], p);
        CHECK(a.x0 >= o.marginMm - 1e-9);
        CHECK(a.y0 >= o.marginMm - 1e-9);
        CHECK(a.x1 <= o.sheetLengthMm - o.marginMm + 1e-9);
        CHECK(a.y1 <= o.sheetWidthMm - o.marginMm + 1e-9);
        CHECK(p.sheet < r.sheetCount);
        for (const NestPlacement& q : r.placements) {
            if (&q == &p || q.sheet != p.sheet || q.item <= p.item) continue;
            const Rect b = placed(items[q.item], q);
            const bool apart = a.x1 + o.gapMm <= b.x0 + 1e-9 || b.x1 + o.gapMm <= a.x0 + 1e-9 ||
                               a.y1 + o.gapMm <= b.y0 + 1e-9 || b.y1 + o.gapMm <= a.y0 + 1e-9;
            CHECK(apart);
        }
    }
    for (std::size_t u : r.unplaced) ++seen[u];
    for (int s : seen) CHECK(s == 1);
}

}  // namespace

TEST_CASE("nest: one item sits at the margin corner", "[parts][nest]") {
    NestOptions o;
    const NestResult r = nestRectangles({{600, 300}}, o);
    REQUIRE(r.placements.size() == 1);
    CHECK(r.placements[0].xMm == 10.0);
    CHECK(r.placements[0].yMm == 10.0);
    CHECK(r.placements[0].sheet == 0);
    CHECK_FALSE(r.placements[0].rotated);
    CHECK(r.sheetCount == 1);
    CHECK(r.unplaced.empty());
}

TEST_CASE("nest: shelf rows, gaps and order", "[parts][nest]") {
    NestOptions o;
    o.sheetLengthMm = 1000;
    o.sheetWidthMm = 600;
    o.marginMm = 10;
    o.gapMm = 5;
    // Three 400 x 200 items: two fit across (400+5+400 = 805 <= 980), the third starts a shelf.
    const std::vector<NestItem> items{{400, 200}, {400, 200}, {400, 200}};
    const NestResult r = nestRectangles(items, o);
    REQUIRE(r.placements.size() == 3);
    CHECK(r.placements[0].xMm == 10);
    CHECK(r.placements[1].xMm == 415);  // 10 + 400 + gap 5
    CHECK(r.placements[1].yMm == 10);
    CHECK(r.placements[2].xMm == 10);
    CHECK(r.placements[2].yMm == 215);  // 10 + 200 + gap 5
    CHECK(r.sheetCount == 1);
    checkLayout(items, o, r);
}

TEST_CASE("nest: exact fits are accepted, an epsilon over is not", "[parts][nest]") {
    NestOptions o;
    o.sheetLengthMm = 120;
    o.sheetWidthMm = 120;
    o.marginMm = 10;
    o.gapMm = 0;
    CHECK(nestRectangles({{100, 100}}, o).placements.size() == 1);
    CHECK(nestRectangles({{100.01, 100}}, o).unplaced.size() == 1);
    CHECK(nestRectangles({{100, 100.01}}, o).unplaced.size() == 1);
    // Two halves with a gap that exactly fills the width.
    o.gapMm = 4;
    CHECK(nestRectangles({{48, 100}, {48, 100}}, o).sheetCount == 1);
    CHECK(nestRectangles({{48.01, 100}, {48, 100}}, o).sheetCount == 2);
    // Same in the other direction: shelves stack with the gap.
    CHECK(nestRectangles({{100, 48}, {100, 48}}, o).sheetCount == 1);
    CHECK(nestRectangles({{100, 48.01}, {100, 48}}, o).sheetCount == 2);
}

TEST_CASE("nest: tall items go first, short ones fill the shelf", "[parts][nest]") {
    NestOptions o;
    o.sheetLengthMm = 1000;
    o.sheetWidthMm = 1000;
    o.marginMm = 0;
    o.gapMm = 0;
    const std::vector<NestItem> items{{100, 50}, {100, 300}, {100, 50}};
    const NestResult r = nestRectangles(items, o);
    REQUIRE(r.placements.size() == 3);
    CHECK(r.placements[0].item == 1);  // the tall item opens the first shelf
    CHECK(r.placements[1].item == 0);
    CHECK(r.placements[1].yMm == 0);   // same shelf (height 300 >= 50)
    CHECK(r.placements[1].xMm == 100);
    CHECK(r.placements[2].xMm == 200);
}

TEST_CASE("nest: overflow opens new sheets, up to the limit", "[parts][nest]") {
    NestOptions o;
    o.sheetLengthMm = 200;
    o.sheetWidthMm = 200;
    o.marginMm = 0;
    o.gapMm = 0;
    const std::vector<NestItem> items(5, NestItem{150, 150});  // one per sheet
    NestResult r = nestRectangles(items, o);
    CHECK(r.sheetCount == 5);
    CHECK(r.unplaced.empty());
    checkLayout(items, o, r);
    o.maxSheets = 3;
    r = nestRectangles(items, o);
    CHECK(r.sheetCount == 3);
    CHECK(r.placements.size() == 3);
    CHECK(r.unplaced.size() == 2);
    checkLayout(items, o, r);
}

TEST_CASE("nest: a new shelf may go on an earlier sheet with room", "[parts][nest]") {
    NestOptions o;
    o.sheetLengthMm = 100;
    o.sheetWidthMm = 100;
    o.marginMm = 0;
    o.gapMm = 0;
    // Tall pair fills sheet 0's shelf (h 60); the 100-wide item needs a new shelf: 60+30 fits sheet 0.
    const std::vector<NestItem> items{{100, 60}, {100, 30}, {100, 60}};
    const NestResult r = nestRectangles(items, o);
    CHECK(r.sheetCount == 2);
    // Sorted: 60, 60, 30. Shelf 1 (sheet 0, y 0), shelf 2 (sheet 1 because 60+60 > 100), then the 30 goes on sheet 0 y 60.
    REQUIRE(r.placements.size() == 3);
    CHECK(r.placements[2].item == 1);
    CHECK(r.placements[2].sheet == 0);
    CHECK(r.placements[2].yMm == 60);
}

TEST_CASE("nest: rotation only when allowed", "[parts][nest]") {
    NestOptions o;
    o.sheetLengthMm = 500;
    o.sheetWidthMm = 200;
    o.marginMm = 0;
    o.gapMm = 0;
    const std::vector<NestItem> tall{{150, 400}};  // too tall unless turned
    CHECK(nestRectangles(tall, o).unplaced.size() == 1);
    o.allowRotate = true;
    NestResult r = nestRectangles(tall, o);
    REQUIRE(r.placements.size() == 1);
    CHECK(r.placements[0].rotated);
    checkLayout(tall, o, r);
    // Both fit: the lower shelf wins (long side along X); equal sides stay unrotated.
    o.sheetWidthMm = 500;
    r = nestRectangles({{400, 150}}, o);
    CHECK_FALSE(r.placements[0].rotated);
    r = nestRectangles({{150, 400}}, o);
    CHECK(r.placements[0].rotated);
    r = nestRectangles({{200, 200}}, o);
    CHECK_FALSE(r.placements[0].rotated);
    // Fits only unrotated although rotation is allowed.
    o.sheetLengthMm = 500;
    o.sheetWidthMm = 100;
    r = nestRectangles({{400, 90}}, o);
    CHECK_FALSE(r.placements[0].rotated);
}

TEST_CASE("nest: degenerate input is unplaced, not an error", "[parts][nest]") {
    NestOptions o;
    const std::vector<NestItem> items{{0, 10}, {10, -1}, {std::nan(""), 10}, {10, 10}};
    const NestResult r = nestRectangles(items, o);
    CHECK(r.placements.size() == 1);
    CHECK(r.unplaced == std::vector<std::size_t>{0, 1, 2});
    o.marginMm = 2000;  // no usable area
    CHECK(nestRectangles({{10, 10}}, o).unplaced.size() == 1);
    o = NestOptions{};
    o.maxSheets = 0;
    CHECK(nestRectangles({{10, 10}}, o).unplaced.size() == 1);
    o = NestOptions{};
    o.gapMm = -1;
    CHECK(nestRectangles({{10, 10}}, o).unplaced.size() == 1);
}

TEST_CASE("nest: deterministic and work is bounded by shelves", "[parts][nest]") {
    NestOptions o;
    std::vector<NestItem> items;
    for (int i = 0; i < 2000; ++i) {
        items.push_back({100.0 + (i * 37) % 400, 50.0 + (i * 91) % 300});
    }
    const NestResult a = nestRectangles(items, o);
    const NestResult b = nestRectangles(items, o);
    CHECK(a.placements.size() == b.placements.size());
    for (std::size_t i = 0; i < a.placements.size(); ++i) {
        CHECK(a.placements[i].item == b.placements[i].item);
        CHECK(a.placements[i].xMm == b.placements[i].xMm);
        CHECK(a.placements[i].yMm == b.placements[i].yMm);
    }
    CHECK(a.unplaced.empty());
    checkLayout(items, o, a);
    // steps counts shelf/sheet probes: at most items * (shelves + sheets), here far below items^2.
    CHECK(a.steps < static_cast<std::uint64_t>(items.size()) * 2000u);
    CHECK(a.steps >= items.size());
}
