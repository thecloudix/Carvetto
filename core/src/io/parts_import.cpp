#include "carvetto/io/parts_import.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <set>
#include <system_error>

namespace carvetto::io {

namespace {

constexpr const char* kMapping = "mapping";
constexpr const char* kParts = "parts";
constexpr double kEps = 1e-9;

bool isControl(unsigned char c) { return c < 0x20 || c == 0x7F; }

std::string trim(std::string_view s) {
    std::size_t a = 0;
    std::size_t b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) --b;
    return std::string(s.substr(a, b - a));
}

std::string lower(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return s;
}

/// Strict decimal: optional '-', digits with at most one '.', at least one digit, at most 32
/// characters. No sign '+', exponent, hex, inf or nan (those are how formulas and tricks look).
std::optional<double> parseNumber(std::string_view text) {
    const std::string s = trim(text);
    if (s.empty() || s.size() > 32) return std::nullopt;
    std::size_t i = 0;
    if (s[0] == '-') i = 1;
    std::size_t digits = 0;
    std::size_t dots = 0;
    for (std::size_t k = i; k < s.size(); ++k) {
        if (s[k] >= '0' && s[k] <= '9') {
            ++digits;
        } else if (s[k] == '.') {
            ++dots;
        } else {
            return std::nullopt;
        }
    }
    if (digits == 0 || dots > 1) return std::nullopt;
    // from_chars wants a digit first: normalise ".5" and "-.5".
    std::string norm = s;
    if (norm[i] == '.') norm.insert(i, "0");
    double v = 0.0;
    const char* first = norm.data();
    const char* last = norm.data() + norm.size();
    if (norm[0] == '-') ++first;  // from_chars accepts '-' itself; keep the sign separately for clarity
    const auto r = std::from_chars(first, last, v);
    if (r.ec != std::errc() || r.ptr != last || !std::isfinite(v)) return std::nullopt;
    return norm[0] == '-' ? -v : v;
}

std::optional<std::size_t> parseCount(std::string_view text) {
    const std::string s = trim(text);
    if (s.empty() || s.size() > 9) return std::nullopt;
    std::size_t v = 0;
    for (char c : s) {
        if (c < '0' || c > '9') return std::nullopt;
        v = v * 10 + static_cast<std::size_t>(c - '0');
    }
    return v;
}

/// Replaces control characters by spaces, trims, and cuts to `maxBytes` on a UTF-8 boundary.
std::string sanitise(std::string_view in, std::size_t maxBytes, bool* changed) {
    std::string s;
    s.reserve(std::min(in.size(), maxBytes + 4));
    bool ch = false;
    for (char c : in) {
        if (isControl(static_cast<unsigned char>(c))) {
            s.push_back(' ');
            ch = true;
        } else {
            s.push_back(c);
        }
    }
    s = trim(s);
    if (s.size() > maxBytes) {
        std::size_t cut = maxBytes;
        while (cut > 0 && (static_cast<unsigned char>(s[cut]) & 0xC0u) == 0x80u) --cut;
        s.resize(cut);
        ch = true;
    }
    if (changed) *changed = ch;
    return s;
}

bool validName(const std::string& s) {
    if (s.empty() || s.size() > 64) return false;
    for (char c : s) {
        if (isControl(static_cast<unsigned char>(c))) return false;
    }
    return trim(s) == s;
}

}  // namespace

void DiagnosticSink::add(PartsDiagnostic::Severity severity, const char* source, std::size_t line,
                         std::string text) {
    if (severity == PartsDiagnostic::Severity::Error) ++errors;
    if (items.size() >= kMaxDiagnostics) {
        ++dropped;
        return;
    }
    items.push_back(PartsDiagnostic{severity, source, line, std::move(text)});
}

// --- CSV --------------------------------------------------------------------------------------------

namespace {

struct RecordReader {
    std::string_view text;
    std::size_t pos{0};
    std::string error;
    bool garbageAfterQuote{false};

    bool atEnd() const { return pos >= text.size(); }

    /// Reads one record into `cells`. False on a limit/format error (`error` set).
    bool read(char delim, std::vector<std::string>& cells) {
        cells.clear();
        garbageAfterQuote = false;
        const std::size_t n = text.size();
        for (;;) {
            std::string field;
            if (pos < n && text[pos] == '"') {
                ++pos;
                bool closed = false;
                while (pos < n) {
                    const char c = text[pos];
                    if (c == '"') {
                        if (pos + 1 < n && text[pos + 1] == '"') {
                            field.push_back('"');
                            pos += 2;
                        } else {
                            ++pos;
                            closed = true;
                            break;
                        }
                    } else {
                        field.push_back(c);
                        ++pos;
                    }
                    if (field.size() > kMaxCsvFieldBytes) {
                        error = "a cell is longer than " + std::to_string(kMaxCsvFieldBytes) + " bytes";
                        return false;
                    }
                }
                if (!closed) {
                    error = "a quoted cell is not closed";
                    return false;
                }
                while (pos < n && text[pos] != delim && text[pos] != '\n' && text[pos] != '\r') {
                    field.push_back(text[pos]);
                    garbageAfterQuote = true;
                    ++pos;
                    if (field.size() > kMaxCsvFieldBytes) {
                        error = "a cell is longer than " + std::to_string(kMaxCsvFieldBytes) + " bytes";
                        return false;
                    }
                }
            } else {
                while (pos < n && text[pos] != delim && text[pos] != '\n' && text[pos] != '\r') {
                    field.push_back(text[pos]);
                    ++pos;
                    if (field.size() > kMaxCsvFieldBytes) {
                        error = "a cell is longer than " + std::to_string(kMaxCsvFieldBytes) + " bytes";
                        return false;
                    }
                }
            }
            cells.push_back(std::move(field));
            if (cells.size() > kMaxCsvColumns) {
                error = "a record has more than " + std::to_string(kMaxCsvColumns) + " cells";
                return false;
            }
            if (pos < n && text[pos] == delim) {
                ++pos;
                continue;
            }
            if (pos < n && text[pos] == '\r') {
                ++pos;
                if (pos < n && text[pos] == '\n') ++pos;
            } else if (pos < n && text[pos] == '\n') {
                ++pos;
            }
            return true;
        }
    }
};

}  // namespace

CsvResult parseCsv(std::string_view text, char delimiter) {
    CsvResult result;
    DiagnosticSink sink;
    auto finish = [&](bool ok) {
        result.ok = ok;
        result.diagnostics = std::move(sink.items);
        return result;
    };
    if (text.size() > kMaxPartsFileBytes) {
        sink.error(kParts, 0, "the parts list is larger than " + std::to_string(kMaxPartsFileBytes / (1024 * 1024)) + " MiB");
        return finish(false);
    }
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF && static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF) {
        text.remove_prefix(3);
    }
    if (std::memchr(text.data(), 0, text.size()) != nullptr) {
        sink.error(kParts, 0, "the parts list contains NUL bytes: it is not a text file");
        return finish(false);
    }

    char delim = delimiter;
    if (delim == 0) {
        const char candidates[3] = {',', ';', '\t'};
        std::size_t best = 1;
        delim = ',';
        int kinds = 0;
        for (char cand : candidates) {
            RecordReader probe{text, 0, {}, false};
            std::vector<std::string> cells;
            if (probe.read(cand, cells) && cells.size() > 1) {
                ++kinds;
                if (cells.size() > best) {
                    best = cells.size();
                    delim = cand;
                }
            }
        }
        if (kinds > 1) {
            sink.warning(kParts, 1,
                         std::string("the first record uses more than one delimiter; using ") +
                             (delim == '\t' ? "tab" : (delim == ';' ? "semicolon" : "comma")) +
                             " (set `delimiter` in the mapping to choose)");
        }
    }
    result.table.delimiter = delim;

    RecordReader rd{text, 0, {}, false};
    std::size_t record = 0;
    std::vector<std::string> cells;
    while (!rd.atEnd()) {
        ++record;
        if (!rd.read(delim, cells)) {
            sink.error(kParts, record, rd.error);
            return finish(false);
        }
        if (rd.garbageAfterQuote) {
            sink.warning(kParts, record, "text after a closing quote was kept as part of the cell");
        }
        const bool blank = cells.size() == 1 && cells[0].empty();
        if (blank) continue;
        if (result.table.rows.size() >= kMaxCsvRecords) {
            sink.error(kParts, record, "more than " + std::to_string(kMaxCsvRecords) + " records");
            return finish(false);
        }
        result.table.rows.push_back(cells);
        result.table.lines.push_back(record);
    }
    return finish(true);
}

// --- Mapping ----------------------------------------------------------------------------------------

namespace {

std::optional<bool> parseBool(const std::string& v) {
    const std::string s = lower(v);
    if (s == "yes" || s == "true" || s == "1") return true;
    if (s == "no" || s == "false" || s == "0") return false;
    return std::nullopt;
}

}  // namespace

MappingResult parsePartsMapping(std::string_view text) {
    MappingResult result;
    DiagnosticSink sink;
    PartsMapping& m = result.mapping;
    auto finish = [&]() {
        result.ok = sink.errors == 0;
        result.diagnostics = std::move(sink.items);
        return result;
    };
    if (text.size() > kMaxMappingBytes) {
        sink.error(kMapping, 0, "the mapping file is larger than " + std::to_string(kMaxMappingBytes / 1024) + " KiB");
        return finish();
    }
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF && static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF) {
        text.remove_prefix(3);
    }
    if (std::memchr(text.data(), 0, text.size()) != nullptr) {
        sink.error(kMapping, 0, "the mapping file contains NUL bytes: it is not a text file");
        return finish();
    }

    std::set<std::string> seen;
    std::size_t lineNo = 0;
    std::size_t pos = 0;
    bool unitsSet = false;
    while (pos <= text.size()) {
        std::size_t eol = text.find('\n', pos);
        if (eol == std::string_view::npos) eol = text.size();
        const std::string_view raw = text.substr(pos, eol - pos);
        pos = eol + 1;
        ++lineNo;
        if (lineNo > kMaxMappingLines) {
            sink.error(kMapping, lineNo, "more than " + std::to_string(kMaxMappingLines) + " lines");
            return finish();
        }
        if (raw.size() > kMaxMappingLineBytes) {
            sink.error(kMapping, lineNo, "line longer than " + std::to_string(kMaxMappingLineBytes) + " bytes");
            continue;
        }
        const std::string line = trim(raw);
        if (line.empty() || line[0] == '#') continue;
        if (line.find('\r') != std::string::npos) {
            sink.error(kMapping, lineNo,
                       "CR-only line endings are not supported (CRLF and LF are): save the file with LF or CRLF line endings");
            continue;
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) {
            sink.error(kMapping, lineNo, "expected `key = value`");
            continue;
        }
        const std::string key = lower(trim(std::string_view(line).substr(0, eq)));
        const std::string value = trim(std::string_view(line).substr(eq + 1));
        if (!seen.insert(key).second) {
            sink.error(kMapping, lineNo, "`" + key + "` is set more than once");
            continue;
        }

        auto column = [&](ColumnRef& out) {
            if (value.empty()) {
                sink.error(kMapping, lineNo, "`" + key + "` needs a header name or #index");
                return;
            }
            if (value[0] == '#') {
                const auto n = parseCount(std::string_view(value).substr(1));
                if (!n || *n < 1 || *n > kMaxCsvColumns) {
                    sink.error(kMapping, lineNo,
                               "`" + key + "`: the column index must be a whole number 1 to " +
                                   std::to_string(kMaxCsvColumns) + " after '#'");
                    return;
                }
                out.kind = ColumnRef::Kind::Index;
                out.index = *n;
                return;
            }
            if (value.size() > 128) {
                sink.error(kMapping, lineNo, "`" + key + "`: the header name is longer than 128 bytes");
                return;
            }
            for (char c : value) {
                if (isControl(static_cast<unsigned char>(c))) {
                    sink.error(kMapping, lineNo, "`" + key + "`: control character in the header name");
                    return;
                }
            }
            out.kind = ColumnRef::Kind::Name;
            out.name = value;
        };
        auto number = [&](double& out, double lo, double hi) {
            const auto v = parseNumber(value);
            if (!v || *v < lo || *v > hi) {
                sink.error(kMapping, lineNo,
                           "`" + key + "` must be a number from " + std::to_string(static_cast<long long>(lo)) + " to " +
                               std::to_string(static_cast<long long>(hi)) + " (millimetres)");
                return false;
            }
            out = *v;
            return true;
        };

        if (key == "units") {
            const std::string v = lower(value);
            if (v == "mm" || v == "millimetres" || v == "millimeters") {
                m.units = PartsUnits::Millimetres;
                unitsSet = true;
            } else if (v == "in" || v == "inch" || v == "inches") {
                m.units = PartsUnits::Inches;
                unitsSet = true;
            } else {
                sink.error(kMapping, lineNo, "`units` must be mm or inch");
            }
        } else if (key == "delimiter") {
            const std::string v = lower(value);
            if (v == "auto") {
                m.delimiter = 0;
            } else if (v == "comma") {
                m.delimiter = ',';
            } else if (v == "semicolon") {
                m.delimiter = ';';
            } else if (v == "tab") {
                m.delimiter = '\t';
            } else {
                sink.error(kMapping, lineNo, "`delimiter` must be auto, comma, semicolon or tab");
            }
        } else if (key == "header") {
            const auto b = parseBool(value);
            if (!b) {
                sink.error(kMapping, lineNo, "`header` must be yes or no");
            } else {
                m.header = *b;
            }
        } else if (key == "column.length") {
            column(m.length);
        } else if (key == "column.width") {
            column(m.width);
        } else if (key == "column.thickness") {
            column(m.thickness);
        } else if (key == "column.material") {
            column(m.material);
        } else if (key == "column.label") {
            column(m.label);
        } else if (key == "column.quantity") {
            column(m.quantity);
        } else if (key == "column.units") {
            column(m.unitsColumn);
        } else if (key == "column.holes") {
            column(m.holes);
        } else if (key == "column.dados") {
            column(m.dados);
        } else if (key == "default.thickness_mm") {
            double v = 0;
            if (number(v, kMinPartDimensionMm, kMaxPartDimensionMm)) m.defaultThicknessMm = v;
        } else if (key == "default.material") {
            bool changed = false;
            m.defaultMaterial = sanitise(value, kMaxLabelBytes, &changed);
            if (changed) sink.warning(kMapping, lineNo, "`default.material` was cleaned of control characters or shortened");
        } else if (key == "layer.panels" || key == "layer.holes" || key == "layer.dados") {
            if (!validName(value)) {
                sink.error(kMapping, lineNo, "`" + key + "` must be a layer name of 1 to 64 characters without control characters");
            } else if (key == "layer.panels") {
                m.panelLayer = value;
            } else if (key == "layer.holes") {
                m.holeLayer = value;
            } else {
                m.dadoLayer = value;
            }
        } else if (key == "sheet.length_mm") {
            number(m.sheetLengthMm, 10.0, kMaxSheetDimensionMm);
        } else if (key == "sheet.width_mm") {
            number(m.sheetWidthMm, 10.0, kMaxSheetDimensionMm);
        } else if (key == "nest.gap_mm") {
            number(m.gapMm, 0.0, 1000.0);
        } else if (key == "nest.margin_mm") {
            number(m.marginMm, 0.0, 1000.0);
        } else if (key == "nest.rotate") {
            const auto b = parseBool(value);
            if (!b) {
                sink.error(kMapping, lineNo, "`nest.rotate` must be yes or no");
            } else {
                m.rotate = *b;
            }
        } else {
            sink.error(kMapping, lineNo, "unknown key `" + key + "`");
        }
    }

    // Whole-file checks.
    if (!m.length.isSet()) sink.error(kMapping, 0, "`column.length` is required");
    if (!m.width.isSet()) sink.error(kMapping, 0, "`column.width` is required");
    if (!m.thickness.isSet() && !m.defaultThicknessMm) {
        sink.error(kMapping, 0, "set `column.thickness` or `default.thickness_mm`");
    }
    if (!unitsSet && !m.unitsColumn.isSet()) {
        sink.error(kMapping, 0, "units must be explicit: set `units = mm` or `units = inch`, or give `column.units`");
    }
    if (!m.header) {
        const std::pair<const char*, const ColumnRef*> roles[] = {
            {"column.length", &m.length},     {"column.width", &m.width},       {"column.thickness", &m.thickness},
            {"column.material", &m.material}, {"column.label", &m.label},       {"column.quantity", &m.quantity},
            {"column.units", &m.unitsColumn}, {"column.holes", &m.holes},       {"column.dados", &m.dados}};
        for (const auto& r : roles) {
            if (r.second->kind == ColumnRef::Kind::Name) {
                sink.error(kMapping, 0, std::string("`") + r.first + "` names a header but `header = no`: use #index");
            }
        }
    }
    {
        const std::pair<const char*, const ColumnRef*> roles[] = {
            {"length", &m.length},     {"width", &m.width},           {"thickness", &m.thickness},
            {"material", &m.material}, {"label", &m.label},           {"quantity", &m.quantity},
            {"units", &m.unitsColumn}, {"holes", &m.holes},           {"dados", &m.dados}};
        for (std::size_t i = 0; i < std::size(roles); ++i) {
            for (std::size_t j = i + 1; j < std::size(roles); ++j) {
                const ColumnRef& a = *roles[i].second;
                const ColumnRef& b = *roles[j].second;
                if (a.isSet() && a.kind == b.kind &&
                    (a.kind == ColumnRef::Kind::Index ? a.index == b.index : lower(a.name) == lower(b.name))) {
                    sink.error(kMapping, 0,
                               std::string("the same column is mapped to both ") + roles[i].first + " and " + roles[j].first);
                }
            }
        }
    }
    if (m.panelLayer == m.holeLayer || m.panelLayer == m.dadoLayer || m.holeLayer == m.dadoLayer) {
        sink.error(kMapping, 0, "the panel, hole and dado layers must have different names");
    }
    return finish();
}

// --- Parts ------------------------------------------------------------------------------------------

namespace {

std::vector<std::string_view> split(std::string_view s, char sep) {
    std::vector<std::string_view> out;
    std::size_t a = 0;
    for (;;) {
        const std::size_t b = s.find(sep, a);
        if (b == std::string_view::npos) {
            out.push_back(s.substr(a));
            return out;
        }
        out.push_back(s.substr(a, b - a));
        a = b + 1;
    }
}

struct Resolved {
    std::optional<std::size_t> idx[9];  // length, width, thickness, material, label, quantity, units, holes, dados
};

}  // namespace

PartsList readParts(const CsvTable& table, const PartsMapping& mapping) {
    PartsList out;
    DiagnosticSink sink;
    auto finish = [&]() {
        if (sink.dropped > 0) {
            out.diagnostics = std::move(sink.items);
            out.diagnostics.push_back(PartsDiagnostic{PartsDiagnostic::Severity::Warning, kParts, 0,
                                                      std::to_string(sink.dropped) + " further messages not shown"});
        } else {
            out.diagnostics = std::move(sink.items);
        }
        out.errorCount = sink.errors;
        return out;
    };

    if (table.rows.empty()) {
        sink.error(kParts, 0, "the parts list is empty");
        return finish();
    }
    const std::size_t firstData = mapping.header ? 1 : 0;
    if (table.rows.size() <= firstData) {
        sink.error(kParts, 0, "the parts list has a header but no rows");
        return finish();
    }
    if (table.rows.size() - firstData > kMaxPartRows) {
        sink.error(kParts, 0, "more than " + std::to_string(kMaxPartRows) + " rows");
        return finish();
    }

    const ColumnRef* refs[9] = {&mapping.length,   &mapping.width,       &mapping.thickness,
                                &mapping.material, &mapping.label,       &mapping.quantity,
                                &mapping.unitsColumn, &mapping.holes,    &mapping.dados};
    const char* roleNames[9] = {"length", "width", "thickness", "material", "label", "quantity", "units", "holes", "dados"};
    Resolved res;
    bool columnsOk = true;
    for (std::size_t r = 0; r < 9; ++r) {
        const ColumnRef& ref = *refs[r];
        if (!ref.isSet()) continue;
        if (ref.kind == ColumnRef::Kind::Index) {
            if (mapping.header && ref.index > table.rows[0].size()) {
                sink.error(kParts, table.lines[0],
                           std::string("column.") + roleNames[r] + " is #" + std::to_string(ref.index) + " but the header has " +
                               std::to_string(table.rows[0].size()) + " columns");
                columnsOk = false;
            } else {
                res.idx[r] = ref.index - 1;
            }
            continue;
        }
        const std::string want = lower(trim(ref.name));
        std::size_t found = 0;
        std::size_t at = 0;
        for (std::size_t c = 0; c < table.rows[0].size(); ++c) {
            if (lower(trim(table.rows[0][c])) == want) {
                ++found;
                at = c;
            }
        }
        if (found == 0) {
            std::string have;
            for (std::size_t c = 0; c < table.rows[0].size() && have.size() < 200; ++c) {
                if (c) have += ", ";
                have += "\"" + sanitise(table.rows[0][c], 40, nullptr) + "\"";
            }
            sink.error(kParts, table.lines[0],
                       std::string("column.") + roleNames[r] + " = \"" + sanitise(ref.name, 40, nullptr) +
                           "\" matches no header; the headers are: " + have);
            columnsOk = false;
        } else if (found > 1) {
            sink.error(kParts, table.lines[0],
                       std::string("column.") + roleNames[r] + " = \"" + sanitise(ref.name, 40, nullptr) +
                           "\" matches " + std::to_string(found) + " headers: use #index");
            columnsOk = false;
        } else {
            res.idx[r] = at;
        }
    }
    if (!columnsOk) return finish();

    for (std::size_t ri = firstData; ri < table.rows.size(); ++ri) {
        const std::vector<std::string>& row = table.rows[ri];
        const std::size_t line = table.lines[ri];
        const std::size_t errorsBefore = sink.errors;
        if (mapping.header && row.size() > table.rows[0].size()) {
            sink.warning(kParts, line, "the row has more cells than the header; the extra cells are ignored");
        }
        auto cell = [&](std::size_t role) -> std::string_view {
            if (!res.idx[role] || *res.idx[role] >= row.size()) return {};
            return row[*res.idx[role]];
        };

        Part part;
        part.line = line;

        // Units: the row's units cell, else the mapping's.
        double factor = 0.0;
        {
            const std::string u = lower(trim(cell(6)));
            if (!u.empty()) {
                if (u == "mm" || u == "millimetres" || u == "millimeters") {
                    factor = 1.0;
                } else if (u == "in" || u == "inch" || u == "inches") {
                    factor = kInchMm;
                } else {
                    sink.error(kParts, line, "unknown units \"" + sanitise(u, 20, nullptr) + "\" (use mm or inch)");
                }
            } else if (mapping.units == PartsUnits::Millimetres) {
                factor = 1.0;
            } else if (mapping.units == PartsUnits::Inches) {
                factor = kInchMm;
            } else {
                sink.error(kParts, line, "the row has no units and the mapping sets none");
            }
        }

        auto dimension = [&](std::size_t role, const char* what, double* dst) {
            const auto v = parseNumber(cell(role));
            if (!v) {
                sink.error(kParts, line, std::string(what) + " \"" + sanitise(cell(role), 30, nullptr) + "\" is not a number");
                return;
            }
            if (factor == 0.0) return;
            const double mm = *v * factor;
            if (mm < kMinPartDimensionMm || mm > kMaxPartDimensionMm) {
                sink.error(kParts, line,
                           std::string(what) + " must be " + std::to_string(static_cast<int>(kMinPartDimensionMm)) + " to " +
                               std::to_string(static_cast<int>(kMaxPartDimensionMm)) + " mm");
                return;
            }
            *dst = mm;
        };
        dimension(0, "length", &part.lengthMm);
        dimension(1, "width", &part.widthMm);
        if (!trim(cell(2)).empty() || !mapping.defaultThicknessMm) {
            dimension(2, "thickness", &part.thicknessMm);
        } else {
            part.thicknessMm = *mapping.defaultThicknessMm;
        }

        // Quantity.
        if (!trim(cell(5)).empty()) {
            const auto q = parseCount(cell(5));
            if (!q || *q < 1 || *q > kMaxQuantityPerRow) {
                sink.error(kParts, line, "quantity must be a whole number 1 to " + std::to_string(kMaxQuantityPerRow));
            } else {
                part.quantity = *q;
            }
        }

        // Text cells: inert data.
        {
            bool changed = false;
            part.label = sanitise(cell(4), kMaxLabelBytes, &changed);
            if (part.label.empty()) {
                part.label = "Part " + std::to_string(line);
            } else if (changed) {
                sink.warning(kParts, line, "the label was cleaned of control characters or shortened");
            }
            const char first = part.label[0];
            if (first == '=' || first == '+' || first == '-' || first == '@') {
                sink.warning(kParts, line, "the label starts with '" + std::string(1, first) + "'; it is kept as plain text, never evaluated");
            }
            part.material = sanitise(cell(3), kMaxLabelBytes, &changed);
            if (part.material.empty()) part.material = mapping.defaultMaterial;
            else if (changed) sink.warning(kParts, line, "the material was cleaned of control characters or shortened");
        }

        const bool geometryOk = sink.errors == errorsBefore && factor != 0.0;
        // Holes: x:y:diameter[:depth] separated by '|'.
        if (geometryOk && !trim(cell(7)).empty()) {
            std::size_t itemNo = 0;
            for (std::string_view item : split(cell(7), '|')) {
                if (trim(item).empty()) continue;
                ++itemNo;
                if (itemNo > kMaxHolesPerPart) {
                    sink.error(kParts, line, "more than " + std::to_string(kMaxHolesPerPart) + " holes");
                    break;
                }
                const auto f = split(item, ':');
                if (f.size() < 3 || f.size() > 4) {
                    sink.error(kParts, line, "hole " + std::to_string(itemNo) + ": expected x:y:diameter[:depth]");
                    continue;
                }
                const auto x = parseNumber(f[0]);
                const auto y = parseNumber(f[1]);
                const auto d = parseNumber(f[2]);
                std::optional<double> depth = 0.0;
                if (f.size() == 4 && !trim(f[3]).empty()) depth = parseNumber(f[3]);
                if (!x || !y || !d || !depth) {
                    sink.error(kParts, line, "hole " + std::to_string(itemNo) + ": a value is not a number");
                    continue;
                }
                PartHole h{*x * factor, *y * factor, *d * factor, *depth * factor};
                const double r = h.diameterMm * 0.5;
                if (h.diameterMm <= 0.0) {
                    sink.error(kParts, line, "hole " + std::to_string(itemNo) + ": the diameter must be positive");
                } else if (h.xMm - r < -kEps || h.xMm + r > part.lengthMm + kEps || h.yMm - r < -kEps ||
                           h.yMm + r > part.widthMm + kEps) {
                    sink.error(kParts, line, "hole " + std::to_string(itemNo) + " does not lie inside the panel");
                } else if (h.depthMm < 0.0 || h.depthMm > part.thicknessMm + kEps) {
                    sink.error(kParts, line,
                               "hole " + std::to_string(itemNo) + ": the depth must be 0 (through) up to the thickness");
                } else {
                    if (h.depthMm >= part.thicknessMm - kEps) h.depthMm = 0.0;
                    part.holes.push_back(h);
                }
            }
        }
        // Dados: L|W:pos:width:depth separated by '|'.
        if (geometryOk && !trim(cell(8)).empty()) {
            std::size_t itemNo = 0;
            for (std::string_view item : split(cell(8), '|')) {
                if (trim(item).empty()) continue;
                ++itemNo;
                if (itemNo > kMaxDadosPerPart) {
                    sink.error(kParts, line, "more than " + std::to_string(kMaxDadosPerPart) + " dados");
                    break;
                }
                const auto f = split(item, ':');
                if (f.size() != 4) {
                    sink.error(kParts, line, "dado " + std::to_string(itemNo) + ": expected L:position:width:depth or W:...");
                    continue;
                }
                const std::string o = lower(trim(f[0]));
                if (o != "l" && o != "w") {
                    sink.error(kParts, line, "dado " + std::to_string(itemNo) + ": the direction must be L or W");
                    continue;
                }
                const auto p = parseNumber(f[1]);
                const auto w = parseNumber(f[2]);
                const auto d = parseNumber(f[3]);
                if (!p || !w || !d) {
                    sink.error(kParts, line, "dado " + std::to_string(itemNo) + ": a value is not a number");
                    continue;
                }
                PartDado dd{o == "l", *p * factor, *w * factor, *d * factor};
                const double extent = dd.alongLength ? part.widthMm : part.lengthMm;
                if (dd.widthMm <= 0.0 || dd.posMm < 0.0 || dd.posMm + dd.widthMm > extent + kEps) {
                    sink.error(kParts, line, "dado " + std::to_string(itemNo) + " does not lie inside the panel");
                } else if (dd.depthMm <= 0.0 || dd.depthMm >= part.thicknessMm - kEps) {
                    sink.error(kParts, line,
                               "dado " + std::to_string(itemNo) + ": the depth must be more than 0 and less than the thickness");
                } else {
                    part.dados.push_back(dd);
                }
            }
        }

        if (sink.errors != errorsBefore) {
            ++out.rowsSkipped;
            continue;
        }
        if (out.instances + part.quantity > kMaxPartInstances) {
            sink.error(kParts, line, "the quantities add up to more than " + std::to_string(kMaxPartInstances) + " panels");
            ++out.rowsSkipped;
            break;
        }
        out.instances += part.quantity;
        out.parts.push_back(std::move(part));
    }
    return finish();
}

PartsList readPartsText(std::string_view csv, const PartsMapping& mapping) {
    CsvResult c = parseCsv(csv, mapping.delimiter);
    if (!c.ok) {
        PartsList out;
        out.diagnostics = std::move(c.diagnostics);
        for (const auto& d : out.diagnostics) {
            if (d.severity == PartsDiagnostic::Severity::Error) ++out.errorCount;
        }
        return out;
    }
    PartsList out = readParts(c.table, mapping);
    out.diagnostics.insert(out.diagnostics.begin(), c.diagnostics.begin(), c.diagnostics.end());
    return out;
}

std::optional<std::string> readBoundedFile(const std::string& path, std::size_t maxBytes, std::string* error) {
    auto fail = [&](std::string text) -> std::optional<std::string> {
        if (error) *error = std::move(text);
        return std::nullopt;
    };
    std::error_code ec;
    const std::filesystem::path p(path);
    if (!std::filesystem::is_regular_file(p, ec) || ec) return fail("not a readable file: " + path);
    const auto size = std::filesystem::file_size(p, ec);
    if (ec) return fail("cannot read the size of " + path);
    if (size > maxBytes) return fail(path + " is larger than " + std::to_string(maxBytes) + " bytes");
    std::ifstream in(p, std::ios::binary);
    if (!in) return fail("cannot open " + path);
    std::string data(static_cast<std::size_t>(size), '\0');
    in.read(data.data(), static_cast<std::streamsize>(size));
    if (in.gcount() != static_cast<std::streamsize>(size)) return fail("cannot read " + path);
    return data;
}

// --- Nesting ----------------------------------------------------------------------------------------

NestResult nestRectangles(const std::vector<NestItem>& items, const NestOptions& options) {
    NestResult result;
    const double usableW = options.sheetLengthMm - 2.0 * options.marginMm;
    const double usableH = options.sheetWidthMm - 2.0 * options.marginMm;
    const bool valid = std::isfinite(usableW) && std::isfinite(usableH) && usableW > 0.0 && usableH > 0.0 &&
                       options.gapMm >= 0.0 && std::isfinite(options.gapMm) && options.maxSheets > 0;

    struct Oriented {
        std::size_t item{0};
        double w{0};
        double h{0};
        bool rotated{false};
    };
    std::vector<Oriented> order;
    order.reserve(items.size());
    for (std::size_t i = 0; i < items.size(); ++i) {
        const NestItem& it = items[i];
        const bool sane = valid && std::isfinite(it.lengthMm) && std::isfinite(it.widthMm) && it.lengthMm > 0.0 &&
                          it.widthMm > 0.0;
        const bool plainFits = sane && it.lengthMm <= usableW + kEps && it.widthMm <= usableH + kEps;
        const bool turnFits = sane && options.allowRotate && it.widthMm <= usableW + kEps && it.lengthMm <= usableH + kEps;
        if (!plainFits && !turnFits) {
            result.unplaced.push_back(i);
            continue;
        }
        Oriented o;
        o.item = i;
        if (plainFits && (!turnFits || it.widthMm <= it.lengthMm)) {
            o.w = it.lengthMm;
            o.h = it.widthMm;
        } else {
            o.w = it.widthMm;
            o.h = it.lengthMm;
            o.rotated = true;
        }
        order.push_back(o);
    }
    std::stable_sort(order.begin(), order.end(), [](const Oriented& a, const Oriented& b) {
        if (a.h != b.h) return a.h > b.h;
        if (a.w != b.w) return a.w > b.w;
        return a.item < b.item;
    });

    struct Shelf {
        std::size_t sheet;
        double y;
        double h;
        double xUsed;
    };
    std::vector<Shelf> shelves;
    std::vector<double> sheetYUsed;
    for (const Oriented& o : order) {
        bool placed = false;
        for (Shelf& s : shelves) {
            ++result.steps;
            if (o.h <= s.h + kEps && s.xUsed + o.w <= usableW + kEps) {
                result.placements.push_back({o.item, s.sheet, options.marginMm + s.xUsed, s.y, o.rotated});
                s.xUsed += o.w + options.gapMm;
                placed = true;
                break;
            }
        }
        if (placed) continue;
        std::size_t sheet = sheetYUsed.size();
        for (std::size_t s = 0; s < sheetYUsed.size(); ++s) {
            ++result.steps;
            if (sheetYUsed[s] + o.h <= usableH + kEps) {
                sheet = s;
                break;
            }
        }
        if (sheet == sheetYUsed.size()) {
            if (sheetYUsed.size() >= options.maxSheets) {
                result.unplaced.push_back(o.item);
                continue;
            }
            sheetYUsed.push_back(0.0);
        }
        Shelf s{sheet, options.marginMm + sheetYUsed[sheet], o.h, o.w + options.gapMm};
        sheetYUsed[sheet] += o.h + options.gapMm;
        shelves.push_back(s);
        result.placements.push_back({o.item, sheet, options.marginMm, s.y, o.rotated});
    }
    result.sheetCount = sheetYUsed.size();
    std::sort(result.unplaced.begin(), result.unplaced.end());
    return result;
}

}  // namespace carvetto::io
