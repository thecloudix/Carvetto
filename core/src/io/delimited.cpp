#include "carvetto/io/delimited.hpp"

namespace carvetto::io {

namespace {

bool separatorOk(char c) {
    if (c == '\t') {
        return true;
    }
    return c >= 0x20 && c < 0x7F && c != '"';
}

bool badControl(char c) {
    const unsigned char u = static_cast<unsigned char>(c);
    return (u < 0x20 && c != '\t' && c != '\n' && c != '\r') || u == 0x7F;
}

struct Reader {
    std::string_view text;
    std::size_t pos{0};
    std::size_t line{1};
    char sep{','};

    bool atEnd() const { return pos >= text.size(); }
};

void report(DelimitedTable& table, std::size_t line, std::string message) {
    if (table.diagnostics.size() < kDelimitedMaxDiagnostics) {
        table.diagnostics.push_back({line, std::move(message)});
    } else {
        ++table.suppressed;
    }
}

/// Reads one record. Returns false when the record is malformed (the message is set); the reader
/// is then past the record.
bool readRecord(Reader& r, std::vector<std::string>& fields, std::string& problem) {
    fields.clear();
    problem.clear();
    std::string field;
    bool inQuotes = false;
    bool quotedField = false;
    bool fieldStarted = false;
    bool bad = false;
    const auto endField = [&]() {
        if (fields.size() >= kDelimitedMaxColumns && problem.empty()) {
            problem = "more than 256 columns";
            bad = true;
        }
        if (fields.size() < kDelimitedMaxColumns) {
            fields.push_back(std::move(field));
        }
        field.clear();
        quotedField = false;
        fieldStarted = false;
    };
    const auto fail = [&](const char* msg) {
        if (problem.empty()) {
            problem = msg;
        }
        bad = true;
    };
    while (!r.atEnd()) {
        const char c = r.text[r.pos];
        if (inQuotes) {
            ++r.pos;
            if (c == '"') {
                if (!r.atEnd() && r.text[r.pos] == '"') {
                    field.push_back('"');
                    ++r.pos;
                } else {
                    inQuotes = false;
                }
            } else {
                if (c == '\n') {
                    ++r.line;
                } else if (c == '\r') {
                    if (!r.atEnd() && r.text[r.pos] == '\n') {
                        ++r.pos;
                    }
                    ++r.line;
                    field.push_back('\n');
                    continue;
                } else if (badControl(c)) {
                    fail("control character in the data");
                }
                field.push_back(c);
            }
        } else if (c == '\n' || c == '\r') {
            ++r.pos;
            if (c == '\r' && !r.atEnd() && r.text[r.pos] == '\n') {
                ++r.pos;
            }
            ++r.line;
            endField();
            return !bad;
        } else if (c == r.sep) {
            ++r.pos;
            endField();
            fieldStarted = true;  // a separator always starts another field
            continue;
        } else if (c == '"') {
            ++r.pos;
            if (fieldStarted && !quotedField && !field.empty()) {
                fail("a quote in the middle of an unquoted field");
                field.push_back(c);
            } else if (quotedField) {
                fail("text after a closing quote");
            } else {
                inQuotes = true;
                quotedField = true;
            }
        } else {
            ++r.pos;
            if (quotedField) {
                fail("text after a closing quote");
            } else if (badControl(c)) {
                fail("control character in the data");
            }
            field.push_back(c);
        }
        fieldStarted = true;
        if (field.size() > kDelimitedMaxFieldBytes) {
            fail("a field is longer than 4096 bytes");
            field.resize(kDelimitedMaxFieldBytes);
        }
    }
    if (inQuotes) {
        fail("unterminated quoted field");
    }
    endField();
    return !bad;
}

}  // namespace

DelimitedTable parseDelimited(std::string_view text, const DelimitedOptions& options) {
    DelimitedTable table;
    if (!separatorOk(options.separator) || options.separator == '\n' || options.separator == '\r') {
        table.fatal = "The separator must be a printable character other than a quote.";
        return table;
    }
    if (text.size() > kDelimitedMaxBytes) {
        table.fatal = "The file is larger than 4 MiB.";
        return table;
    }
    Reader r{text, 0, 1, options.separator};
    if (text.size() >= 3 && text.substr(0, 3) == "\xEF\xBB\xBF") {
        r.pos = 3;
    }
    std::vector<std::string> fields;
    std::string problem;
    std::size_t seen = 0;
    while (!r.atEnd()) {
        const std::size_t startLine = r.line;
        const std::size_t start = r.pos;
        const bool ok = readRecord(r, fields, problem);
        // Blank line: nothing but a line end.
        if (ok && fields.size() == 1 && fields.front().empty() && r.pos - start <= 2 &&
            (text[start] == '\n' || text[start] == '\r')) {
            continue;
        }
        if (++seen > kDelimitedMaxRecords) {
            table.fatal = "The file has more than 20000 records.";
            table.records.clear();
            return table;
        }
        if (!ok) {
            report(table, startLine, problem);
            continue;
        }
        if (table.records.empty() && table.columns == 0) {
            table.columns = fields.size();
        } else if (options.requireUniformColumns && fields.size() != table.columns) {
            report(table, startLine,
                   "expected " + std::to_string(table.columns) + " fields but found " +
                       std::to_string(fields.size()));
            continue;
        }
        table.records.push_back({startLine, fields});
    }
    return table;
}

}  // namespace carvetto::io
