#include "carvetto/viewmodel/data_merge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <unordered_set>
#include <utility>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/io/delimited.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/viewmodel/array_copies.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Transform2;
using geometry::Vec2;

namespace {

bool finiteIn(double v, double limit) { return std::isfinite(v) && std::fabs(v) <= limit; }

bool nameChar(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
}

bool validPlaceholderName(const std::string& name) {
    return !name.empty() && name.size() <= 64 && document::DocumentVariables::isValidName(name);
}

std::string lower(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return s;
}

/// Splits `text` into literal pieces and placeholders: calls `onText(piece)` / `onVar(name)`.
template <typename OnText, typename OnVar>
void scanPlaceholders(const std::string& text, OnText onText, OnVar onVar) {
    std::string literal;
    std::size_t i = 0;
    while (i < text.size()) {
        if (text[i] == '!' && i + 1 < text.size() && text[i + 1] == '!') {
            std::size_t j = i + 2;
            while (j < text.size() && nameChar(text[j])) {
                ++j;
            }
            if (j > i + 2 && j + 1 < text.size() + 0 && text[j] == '!' && text[j + 1] == '!' &&
                validPlaceholderName(text.substr(i + 2, j - i - 2))) {
                onText(literal);
                literal.clear();
                onVar(text.substr(i + 2, j - i - 2));
                i = j + 2;
                continue;
            }
        }
        literal.push_back(text[i]);
        ++i;
    }
    onText(literal);
}

// --- Template -----------------------------------------------------------------------------------

struct Entry {
    Object proto;  ///< Id and parent cleared.
    int parent{-1};
    bool placeholders{false};
};

struct Template {
    std::vector<Entry> entries;  ///< Pre-order: a group precedes its children.
    std::vector<ObjectId> roots;
    Box2 bounds;
    std::size_t spans{0};
    bool badGeometry{false};
    std::size_t unitCost() const { return entries.size() + spans; }
};

bool layerUsable(const Document& doc, document::LayerId id) {
    const document::Layer* layer = doc.findLayer(id);
    return layer != nullptr && layer->visible && !layer->locked;
}

bool chainUsable(const Document& doc, const Object& obj) {
    const Object* cur = &obj;
    for (std::size_t guard = 0; guard <= doc.objectCount(); ++guard) {
        if (!layerUsable(doc, cur->layer)) {
            return false;
        }
        if (!cur->parent.has_value()) {
            return true;
        }
        cur = doc.findObject(*cur->parent);
        if (cur == nullptr) {
            return false;
        }
    }
    return false;
}

bool ancestorIn(const Document& doc, const Object& obj, const std::unordered_set<ObjectId>& set) {
    const Object* cur = &obj;
    for (std::size_t guard = 0; guard <= doc.objectCount(); ++guard) {
        if (!cur->parent.has_value()) {
            return false;
        }
        if (set.contains(*cur->parent)) {
            return true;
        }
        cur = doc.findObject(*cur->parent);
        if (cur == nullptr) {
            return false;
        }
    }
    return false;
}

bool hasPlaceholder(const std::string& text) {
    bool found = false;
    scanPlaceholders(
        text, [](const std::string&) {}, [&](const std::string&) { found = true; });
    return found;
}

bool addTree(const Document& doc, const Object& obj, int parent, Template& t, std::size_t depth) {
    if (depth > doc.objectCount()) {
        return false;
    }
    Entry entry;
    entry.proto = obj;
    entry.proto.id = {};
    entry.proto.parent.reset();
    entry.parent = parent;
    if (const document::GroupObject* group = obj.asGroup(); group != nullptr) {
        const std::size_t first = t.entries.size();
        const int self = static_cast<int>(first);
        entry.proto.data = document::GroupObject{};
        t.entries.push_back(std::move(entry));
        bool any = false;
        for (ObjectId child : group->children) {
            const Object* c = doc.findObject(child);
            if (c != nullptr && layerUsable(doc, c->layer)) {
                any = addTree(doc, *c, self, t, depth + 1) || any;
            }
        }
        if (!any) {
            t.entries.resize(first);
        }
        return any;
    }
    if (const document::VectorObject* vec = obj.asVector(); vec != nullptr) {
        for (const geometry::Contour& c : vec->contours) {
            t.spans += c.spans.size();
        }
        const Box2 b = vec->bounds();
        t.badGeometry = t.badGeometry ||
                        (!b.isEmpty() && !(finiteIn(b.min.x, 1e9) && finiteIn(b.min.y, 1e9) &&
                                           finiteIn(b.max.x, 1e9) && finiteIn(b.max.y, 1e9)));
        t.bounds.expand(b);
        if (const auto* text = std::get_if<document::TextParams>(&vec->params)) {
            entry.placeholders = hasPlaceholder(text->text);
        }
    } else if (const document::BitmapObject* bmp = obj.asBitmap(); bmp != nullptr) {
        const Box2 b = bmp->bounds();
        t.badGeometry = t.badGeometry || !(finiteIn(b.min.x, 1e9) && finiteIn(b.min.y, 1e9) &&
                                           finiteIn(b.max.x, 1e9) && finiteIn(b.max.y, 1e9));
        t.bounds.expand(b);
    }
    t.entries.push_back(std::move(entry));
    return true;
}

Template collectTemplate(const Document& doc) {
    Template t;
    const std::unordered_set<ObjectId> selected(doc.selection().ids().begin(), doc.selection().ids().end());
    for (ObjectId id : doc.selection().ids()) {
        const Object* obj = doc.findObject(id);
        if (obj == nullptr || obj->sheet != doc.activeSheet() || !chainUsable(doc, *obj) ||
            ancestorIn(doc, *obj, selected)) {
            continue;
        }
        if (addTree(doc, *obj, -1, t, 0)) {
            t.roots.push_back(id);
        }
    }
    return t;
}

// --- Plates -------------------------------------------------------------------------------------

struct Planned {
    Object obj;
    int parent{-1};
    bool top{false};
};

struct Plate {
    std::size_t line{0};
    std::size_t sheet{0};  ///< Index into the sheets used: 0 is the active sheet.
    std::vector<Planned> objects;
};

document::CreationParams translatedParams(const document::CreationParams& params, const Transform2& m, Vec2 delta) {
    return std::visit(
        [&](const auto& p) -> document::CreationParams {
            using T = std::decay_t<decltype(p)>;
            T q = p;
            if constexpr (std::is_same_v<T, document::NoParams>) {
                return q;
            } else if constexpr (std::is_same_v<T, document::TextParams>) {
                q.placement = m * q.placement;
                if (q.guideId.has_value()) {
                    q.guideId.reset();  // the guide snapshot travels with the text; the id would dangle
                }
                return q;
            } else {
                q.centreMm = Vec2{p.centreMm.x + delta.x, p.centreMm.y + delta.y};
                return q;
            }
        },
        params);
}

std::string substitute(const std::string& text, const std::map<std::string, std::string>& values) {
    std::string out;
    scanPlaceholders(
        text, [&](const std::string& piece) { out += piece; },
        [&](const std::string& name) {
            const auto it = values.find(name);
            if (it != values.end()) {
                out += it->second;
            }
        });
    return out;
}

struct Context {
    const Document& doc;
    const DataMergeOptions& options;
    std::map<std::string, text::Font> fonts;
    std::string fontError;

    const text::Font* font(const std::string& key) {
        const auto it = fonts.find(key);
        if (it != fonts.end()) {
            return &it->second;
        }
        const document::FontAsset* asset = doc.findFont(key);
        if (asset == nullptr) {
            fontError = "A template text uses a font that is not embedded in the document.";
            return nullptr;
        }
        text::FontLoad load = text::Font::load(asset->bytes);
        if (!load.ok()) {
            fontError = "A template text uses a font that cannot be read.";
            return nullptr;
        }
        return &fonts.emplace(key, std::move(load.font)).first->second;
    }
};

/// Instantiates the template at `delta`; false (with `problem`) when a text cannot be laid out.
bool instantiate(Context& ctx, const Template& t, Vec2 delta, const std::map<std::string, std::string>& values,
                 Plate& plate, std::string& problem, std::string& warning) {
    const Transform2 m = Transform2::translation(delta);
    const int base = 0;
    for (const Entry& e : t.entries) {
        Planned p;
        p.obj = e.proto;
        if (document::VectorObject* vec = p.obj.asVector(); vec != nullptr) {
            vec->params = translatedParams(vec->params, m, delta);
            if (e.placeholders) {
                auto* text = std::get_if<document::TextParams>(&vec->params);
                text->text = substitute(text->text, values);
                const text::Font* font = ctx.font(text->fontKey);
                if (font == nullptr) {
                    problem = ctx.fontError;
                    return false;
                }
                document::TextBuild built = document::buildTextContours(*font, *text);
                if (!built.ok()) {
                    problem = "the text \"" + text->text.substr(0, 40) + "\" cannot be laid out: " + built.message;
                    return false;
                }
                if (built.overflow) {
                    warning = "the text \"" + text->text.substr(0, 40) + "\" does not fit its box";
                } else if (built.missingGlyphs) {
                    warning = "the font has no glyph for part of \"" + text->text.substr(0, 40) + "\"";
                }
                vec->contours = std::move(built.contours);
            } else {
                vec->transform(m);
            }
        } else if (document::BitmapObject* bmp = p.obj.asBitmap(); bmp != nullptr) {
            bmp->transform(m);
        } else if (document::DimensionObject* dim = p.obj.asDimension(); dim != nullptr) {
            dim->transform(m);
        }
        if (e.parent >= 0) {
            p.parent = base + e.parent;
        } else {
            p.top = true;
        }
        plate.objects.push_back(std::move(p));
    }
    return true;
}

DataMergeResult failure(std::string message) {
    DataMergeResult r;
    r.message = std::move(message);
    return r;
}

struct Plan {
    DataMergeResult result;  ///< Filled up to the counts; `ok` set when `plates` is usable.
    Template tmpl;
    std::vector<Plate> plates;
};

std::string describe(const std::vector<MergeDiagnostic>& diagnostics, std::size_t extra) {
    std::string msg = std::to_string(diagnostics.size() + extra) + " problem(s) in the data";
    if (!diagnostics.empty()) {
        msg += "; first at line " + std::to_string(diagnostics.front().line) + ": " + diagnostics.front().message;
    }
    msg += ".";
    return msg;
}

Plan plan(const Document& doc, std::string_view csvText, const DataMergeOptions& o) {
    Plan out;
    if (std::string bad = validateDataMergeOptions(o); !bad.empty()) {
        out.result = failure(std::move(bad));
        return out;
    }
    out.tmpl = collectTemplate(doc);
    const Template& t = out.tmpl;
    if (t.roots.empty() || t.bounds.isEmpty()) {
        out.result = failure("Select the template objects first (hidden and locked layers are skipped).");
        return out;
    }
    if (t.badGeometry) {
        out.result = failure("The template has coordinates that are not finite or beyond 1e9 mm.");
        return out;
    }

    io::DelimitedOptions csvOptions;
    csvOptions.separator = o.separator;
    io::DelimitedTable table = io::parseDelimited(csvText, csvOptions);
    if (!table.fatal.empty()) {
        out.result = failure(table.fatal);
        return out;
    }
    std::vector<MergeDiagnostic> bad;
    for (const io::DelimitedDiagnostic& d : table.diagnostics) {
        bad.push_back({d.line, d.message});
    }
    std::size_t badRows = bad.size() + table.suppressed;
    std::size_t first = 0;
    if (o.firstRowNames) {
        if (table.records.empty() || (!bad.empty() && bad.front().line < table.records.front().line)) {
            out.result = failure("The header row is missing or malformed.");
            out.result.diagnostics = bad;
            return out;
        }
        first = 1;
    }
    if (table.records.size() <= first) {
        out.result = failure("The data has no rows to merge.");
        out.result.diagnostics = bad;
        return out;
    }

    // Placeholders must all be bound.
    std::map<std::string, std::size_t> columnOf;
    for (const VariableAssignment& a : o.assignments) {
        if (a.column < 0 || static_cast<std::size_t>(a.column) >= table.columns) {
            out.result = failure("Variable " + a.name + " is assigned to column " + std::to_string(a.column + 1) +
                                 ", but the data has " + std::to_string(table.columns) + " column(s).");
            return out;
        }
        columnOf[a.name] = static_cast<std::size_t>(a.column);
    }
    std::set<std::string> counterNames;
    for (const CounterSpec& c : o.counters) {
        counterNames.insert(c.name);
    }
    for (const std::string& name : templatePlaceholders(doc)) {
        if (!columnOf.contains(name) && !counterNames.contains(name)) {
            out.result = failure("The template uses !!" + name + "!! but no column or counter is assigned to it.");
            return out;
        }
    }

    // Layout capacity.
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    if (sheet == nullptr) {
        out.result = failure("There is no active sheet.");
        return out;
    }
    const Box2 material = sheet->materialBoundsMm();
    const double pw = t.bounds.width();
    const double ph = t.bounds.height();
    const double availW = material.width() - o.marginLeftMm - o.marginRightMm;
    const double availH = material.height() - o.marginTopMm - o.marginBottomMm;
    constexpr double kEps = 1e-9;
    if (pw > availW + kEps || ph > availH + kEps) {
        out.result = failure("A plate (" + std::to_string(pw) + " x " + std::to_string(ph) +
                             " mm) does not fit inside the sheet margins.");
        return out;
    }
    std::size_t cols = 1;
    std::size_t rows = 1;
    if (o.layout == MergeLayout::Grid) {
        cols = static_cast<std::size_t>(std::floor((availW + o.spacingMm) / std::max(pw + o.spacingMm, kEps) + kEps));
        rows = static_cast<std::size_t>(std::floor((availH + o.spacingMm) / std::max(ph + o.spacingMm, kEps) + kEps));
        cols = std::max<std::size_t>(cols, 1);
        rows = std::max<std::size_t>(rows, 1);
    }
    const std::size_t perSheet = cols * rows;

    // Rows to plates.
    const std::size_t dataRows = table.records.size() - first;
    if (dataRows > kMaxMergePlates) {
        out.result = failure("The data has " + std::to_string(dataRows) + " rows; at most " +
                             std::to_string(kMaxMergePlates) + " plates are made at once.");
        return out;
    }
    if (copyWorkExceeded(static_cast<double>(dataRows), t.unitCost())) {
        out.result = failure("The request is too large (plates times template objects and spans over 500000).");
        return out;
    }

    Context ctx{doc, o, {}, {}};
    std::vector<MergeDiagnostic> warnings;
    for (std::size_t r = first; r < table.records.size(); ++r) {
        const io::DelimitedRecord& record = table.records[r];
        const std::size_t index = out.plates.size();
        std::map<std::string, std::string> values;
        for (const auto& [name, col] : columnOf) {
            values[name] = record.fields[col];
        }
        for (const CounterSpec& c : o.counters) {
            values[c.name] = formatCounter(c, index);
        }
        Plate plate;
        plate.line = record.line;
        plate.sheet = o.layout == MergeLayout::Grid ? index / perSheet : index;
        const std::size_t slot = o.layout == MergeLayout::Grid ? index % perSheet : 0;
        const std::size_t col = slot % cols;
        const std::size_t row = slot / cols;
        const double x = material.min.x + o.marginLeftMm + static_cast<double>(col) * (pw + o.spacingMm);
        const double yTop = material.max.y - o.marginTopMm - static_cast<double>(row) * (ph + o.spacingMm);
        const Vec2 delta{x - t.bounds.min.x, (yTop - ph) - t.bounds.min.y};
        std::string problem;
        std::string warning;
        if (!instantiate(ctx, t, delta, values, plate, problem, warning)) {
            if (!ctx.fontError.empty()) {
                out.result = failure(ctx.fontError);
                return out;
            }
            bad.push_back({record.line, problem});
            ++badRows;
            continue;
        }
        if (!warning.empty()) {
            warnings.push_back({record.line, warning});
        }
        out.plates.push_back(std::move(plate));
    }
    std::sort(bad.begin(), bad.end(),
              [](const MergeDiagnostic& a, const MergeDiagnostic& b) { return a.line < b.line; });
    out.result.diagnostics = bad;
    out.result.warnings = std::move(warnings);
    if (badRows > 0 && !o.skipBadRows) {
        out.result.message = describe(bad, badRows - bad.size());
        out.plates.clear();
        return out;
    }
    if (out.plates.empty()) {
        out.result.message = "No row produced a plate.";
        return out;
    }
    // With skipped rows the plates were numbered by good rows only; counters and cells already
    // follow that numbering because `index` is the count of good plates so far.
    out.result.skippedRows = badRows;
    out.result.plates = out.plates.size();
    out.result.platesPerSheet = perSheet;
    out.result.sheetsUsed = out.plates.back().sheet + 1;
    out.result.ok = true;
    return out;
}

}  // namespace

std::string formatCounter(const CounterSpec& spec, std::size_t plateIndex) {
    const std::int64_t value = spec.start + static_cast<std::int64_t>(plateIndex) * spec.step;
    const bool negative = value < 0;
    std::string digits = std::to_string(negative ? -value : value);
    const std::size_t width = static_cast<std::size_t>(std::clamp(spec.width, 1, kMaxCounterWidth));
    if (digits.size() < width) {
        digits.insert(0, width - digits.size(), '0');
    }
    return spec.prefix + (negative ? "-" : "") + digits + spec.suffix;
}

std::string validateDataMergeOptions(const DataMergeOptions& o) {
    if (o.separator == '"' || o.separator == '\n' || o.separator == '\r' ||
        (static_cast<unsigned char>(o.separator) < 0x20 && o.separator != '\t') ||
        static_cast<unsigned char>(o.separator) >= 0x7F) {
        return "The separator must be a printable character other than a quote.";
    }
    std::set<std::string> names;
    for (const VariableAssignment& a : o.assignments) {
        if (!validPlaceholderName(a.name)) {
            return "\"" + a.name + "\" is not a valid variable name (letters, digits and _, starting with a letter).";
        }
        if (!names.insert(a.name).second) {
            return "The variable " + a.name + " is assigned twice.";
        }
    }
    for (const CounterSpec& c : o.counters) {
        if (!validPlaceholderName(c.name)) {
            return "\"" + c.name + "\" is not a valid counter name (letters, digits and _, starting with a letter).";
        }
        if (!names.insert(c.name).second) {
            return "The name " + c.name + " is used twice.";
        }
        constexpr std::int64_t kLimit = 1'000'000'000'000LL;
        if (c.start < -kLimit || c.start > kLimit || c.step < -kLimit || c.step > kLimit) {
            return "The counter " + c.name + " start and step must be within 1e12.";
        }
        if (c.width < 1 || c.width > kMaxCounterWidth) {
            return "The counter " + c.name + " width must be between 1 and 18.";
        }
        if (c.prefix.size() > 256 || c.suffix.size() > 256) {
            return "The counter " + c.name + " prefix and suffix are limited to 256 bytes.";
        }
    }
    for (double v : {o.marginLeftMm, o.marginRightMm, o.marginTopMm, o.marginBottomMm, o.spacingMm}) {
        if (!finiteIn(v, kMaxMergeLengthMm) || v < 0.0) {
            return "Margins and spacing must be between 0 and 1e6 mm.";
        }
    }
    return {};
}

bool canDataMerge(const Document& doc) { return !collectTemplate(doc).roots.empty(); }

std::vector<std::string> templatePlaceholders(const Document& doc) {
    const Template t = collectTemplate(doc);
    std::set<std::string> names;
    for (const Entry& e : t.entries) {
        const document::VectorObject* vec = e.proto.asVector();
        if (vec == nullptr) {
            continue;
        }
        if (const auto* text = std::get_if<document::TextParams>(&vec->params)) {
            scanPlaceholders(
                text->text, [](const std::string&) {}, [&](const std::string& n) { names.insert(n); });
        }
    }
    return {names.begin(), names.end()};
}

std::vector<std::string> csvHeaders(std::string_view csvText, char separator) {
    io::DelimitedOptions options;
    options.separator = separator;
    const io::DelimitedTable table = io::parseDelimited(csvText, options);
    if (!table.fatal.empty() || table.records.empty()) {
        return {};
    }
    return table.records.front().fields;
}

std::vector<VariableAssignment> autoAssign(const std::vector<std::string>& placeholders,
                                           const std::vector<std::string>& headers) {
    std::vector<VariableAssignment> out;
    for (const std::string& name : placeholders) {
        for (std::size_t i = 0; i < headers.size(); ++i) {
            if (lower(headers[i]) == lower(name)) {
                out.push_back({name, static_cast<int>(i)});
                break;
            }
        }
    }
    return out;
}

DataMergeResult previewDataMerge(const Document& doc, std::string_view csvText, const DataMergeOptions& options) {
    Plan p = plan(doc, csvText, options);
    return std::move(p.result);
}

DataMergeResult applyDataMerge(Document& doc, std::string_view csvText, const DataMergeOptions& options) {
    Plan p = plan(doc, csvText, options);
    DataMergeResult result = std::move(p.result);
    if (!result.ok) {
        return result;
    }
    std::uint64_t ids = (options.newLayer ? 1U : 0U) + (result.sheetsUsed - 1);
    for (const Plate& plate : p.plates) {
        ids += plate.objects.size();
    }
    if (!doc.idGenerator().canAllocate(ids)) {
        result.ok = false;
        result.message = "The document has no free object ids left.";
        return result;
    }
    const document::Sheet base = *doc.findSheet(doc.activeSheet());

    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Plate production");
    document::LayerId layer;
    if (options.newLayer) {
        document::Layer fresh;
        fresh.name = "Plates";
        auto cmd = std::make_unique<document::AddLayerCommand>(std::move(fresh), false);
        const document::AddLayerCommand* raw = cmd.get();
        if (doc.execute(std::move(cmd))) {
            layer = raw->id();
        }
    }
    std::vector<document::SheetId> sheets{doc.activeSheet()};
    for (std::size_t s = 1; s < result.sheetsUsed; ++s) {
        document::Sheet fresh = base;
        fresh.id = {};
        fresh.name = base.name + " (" + std::to_string(s + 1) + ")";
        fresh.partner = {};
        auto cmd = std::make_unique<document::AddSheetCommand>(std::move(fresh), false);
        const document::AddSheetCommand* raw = cmd.get();
        sheets.push_back(doc.execute(std::move(cmd)) ? raw->id() : doc.activeSheet());
    }
    for (Plate& plate : p.plates) {
        std::vector<ObjectId> created(plate.objects.size());
        for (std::size_t i = 0; i < plate.objects.size(); ++i) {
            Planned& planned = plate.objects[i];
            Object obj = std::move(planned.obj);
            if (options.newLayer) {
                obj.layer = layer;
            }
            if (planned.parent >= 0) {
                obj.parent = created[static_cast<std::size_t>(planned.parent)];
            }
            obj.sheet = sheets[plate.sheet];
            auto cmd = std::make_unique<document::AddObjectCommand>(std::move(obj));
            const document::AddObjectCommand* raw = cmd.get();
            if (doc.execute(std::move(cmd))) {
                created[i] = raw->id();
            }
            if (planned.top && created[i]) {
                result.created.push_back(created[i]);
            }
        }
    }
    if (options.removeTemplate) {
        doc.execute(std::make_unique<document::RemoveObjectsCommand>(p.tmpl.roots));
    }
    doc.undoStack().endMacro();
    doc.selection().assign(result.created);
    return result;
}

}  // namespace carvetto::viewmodel
