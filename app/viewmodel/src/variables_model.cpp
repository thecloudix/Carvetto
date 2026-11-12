#include "carvetto/viewmodel/variables_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <memory>
#include <system_error>

#include "carvetto/document/commands.hpp"
#include "carvetto/viewmodel/calculation.hpp"

namespace carvetto::viewmodel {

namespace {

constexpr double kMaxVariable = 1e12;

VariableEdit refused(std::string message) {
    VariableEdit e;
    e.message = std::move(message);
    return e;
}

/// A variable's value text: a plain number, or a calculation.
bool parseValue(const std::string& text, double* out, std::string* error) {
    std::string t = text;
    const auto notSpace = [](char c) { return c != ' ' && c != '\t'; };
    t.erase(t.begin(), std::find_if(t.begin(), t.end(), notSpace));
    t.erase(std::find_if(t.rbegin(), t.rend(), notSpace).base(), t.end());
    if (t.empty()) {
        *error = "enter a value";
        return false;
    }
    double v = 0.0;
    if (isExpressionText(t)) {
        const Calculation calc = calculate(t, document::Units::Millimetres);
        if (!calc.ok) {
            *error = calc.error;
            return false;
        }
        v = calc.value;
    } else {
        // Same grammar as an import line: a plain decimal number.
        document::DocumentVariables probe;
        const auto res = probe.importText("v = " + t);
        const auto got = probe.get("v");
        if (res.imported != 1 || !got.has_value()) {
            *error = "not a number (start with = to calculate)";
            return false;
        }
        v = *got;
    }
    if (!std::isfinite(v) || std::fabs(v) > kMaxVariable) {
        *error = "the value is too large";
        return false;
    }
    *out = v;
    return true;
}

}  // namespace

VariablesModel::VariablesModel(document::Document* doc, ToolpathSession* session) : doc_(doc), session_(session) {}

std::vector<VariableRow> VariablesModel::rows() const {
    std::vector<VariableRow> out;
    if (doc_ == nullptr) {
        return out;
    }
    if (session_ != nullptr) {
        session_->refreshReferences();
    }
    const document::DocumentVariables& vars = doc_->variables();
    for (const auto& [name, value] : vars.all()) {
        out.push_back(VariableRow{name, value, numberText(value), vars.referenceCount(name)});
    }
    return out;
}

std::vector<std::string> VariablesModel::toolpathsUsing(const std::string& name) const {
    std::vector<std::string> out;
    if (doc_ == nullptr || session_ == nullptr) {
        return out;
    }
    for (const document::ToolpathRecord& rec : doc_->toolpaths()) {
        for (const auto& [field, text] : session_->expressionsOf(rec.id)) {
            const Calculation calc = calculate(text, doc_->job().displayUnits);
            if (std::find(calc.usedVariables.begin(), calc.usedVariables.end(), name) != calc.usedVariables.end()) {
                out.push_back(rec.name);
                break;
            }
        }
    }
    return out;
}

template <class Body>
VariableEdit VariablesModel::edit(const char* text, Body&& body) {
    document::UndoStack& undo = doc_->undoStack();
    undo.breakCoalescing();
    const std::size_t before = undo.index();
    undo.beginMacro(text);
    VariableEdit result = body();
    BatchOutcome batch;
    if (result.ok && session_ != nullptr) {
        batch = session_->recalculateBound();
        result.recalculated = batch.recalculated;
        result.failed = batch.failed;
    }
    undo.endMacro();
    undo.breakCoalescing();
    if (result.ok && undo.index() == before) {
        result.message = "no change";  // nothing was different
    }
    if (result.ok && batch.failed != 0) {
        result.message += " (warning: " + batch.message + ")";
    }
    return result;
}

VariableEdit VariablesModel::set(const std::string& name, const std::string& valueText) {
    if (doc_ == nullptr) {
        return refused("there is no document");
    }
    if (!document::DocumentVariables::isValidName(name)) {
        return refused("a name starts with a letter and continues with letters, digits or _");
    }
    double value = 0.0;
    std::string error;
    if (!parseValue(valueText, &value, &error)) {
        return refused(error);
    }
    const bool existed = doc_->variables().contains(name);
    return edit("Set variable", [&]() {
        doc_->execute(std::make_unique<document::SetDocumentVariableCommand>(name, value));
        VariableEdit r;
        r.ok = true;
        r.message = (existed ? "changed " : "created ") + name;
        return r;
    });
}

VariableEdit VariablesModel::remove(const std::string& name) {
    if (doc_ == nullptr) {
        return refused("there is no document");
    }
    if (session_ != nullptr) {
        session_->refreshReferences();
    }
    if (!doc_->variables().contains(name)) {
        return refused("there is no variable '" + name + "'");
    }
    if (doc_->variables().referenceCount(name) != 0) {
        const std::vector<std::string> users = toolpathsUsing(name);
        std::string message = "'" + name + "' is used by ";
        if (users.empty()) {
            message += "an open toolpath form; close it first";
        } else {
            for (std::size_t i = 0; i < users.size() && i < 3; ++i) {
                message += (i == 0 ? "" : ", ") + users[i];
            }
            if (users.size() > 3) {
                message += " and " + std::to_string(users.size() - 3) + " more";
            }
        }
        return refused(message);
    }
    return edit("Remove variable", [&]() {
        VariableEdit r;
        r.ok = doc_->execute(std::make_unique<document::RemoveDocumentVariableCommand>(name));
        r.message = r.ok ? "deleted " + name : "could not delete " + name;
        return r;
    });
}

VariableEdit VariablesModel::rename(const std::string& name, const std::string& newName) {
    if (doc_ == nullptr) {
        return refused("there is no document");
    }
    if (session_ != nullptr) {
        session_->refreshReferences();
    }
    const auto value = doc_->variables().get(name);
    if (!value.has_value()) {
        return refused("there is no variable '" + name + "'");
    }
    if (!document::DocumentVariables::isValidName(newName)) {
        return refused("a name starts with a letter and continues with letters, digits or _");
    }
    if (newName == name) {
        return refused("the name is unchanged");
    }
    if (doc_->variables().contains(newName)) {
        return refused("'" + newName + "' already exists");
    }
    if (doc_->variables().referenceCount(name) != 0) {
        return refused("'" + name + "' is used by a toolpath; it cannot be renamed");
    }
    return edit("Rename variable", [&]() {
        doc_->execute(std::make_unique<document::SetDocumentVariableCommand>(newName, *value));
        doc_->undoStack().breakCoalescing();
        doc_->execute(std::make_unique<document::RemoveDocumentVariableCommand>(name));
        VariableEdit r;
        r.ok = true;
        r.message = "renamed " + name + " to " + newName;
        return r;
    });
}

VariableEdit VariablesModel::importText(const std::string& text) {
    if (doc_ == nullptr) {
        return refused("there is no document");
    }
    if (text.size() > kMaxImportBytes) {
        return refused("the file is too large");
    }
    document::DocumentVariables incoming;
    const document::DocumentVariables::ImportResult parsed = incoming.importText(text);
    if (parsed.imported == 0) {
        VariableEdit r = refused(parsed.rejectedLines.empty() ? "the file holds no variables"
                                                                : "no line could be read (expected Name = value)");
        r.rejectedLines = parsed.rejectedLines;
        return r;
    }
    VariableEdit r = edit("Import variables", [&]() {
        for (const auto& [name, value] : incoming.all()) {
            doc_->undoStack().breakCoalescing();  // one command per variable inside the macro
            doc_->execute(std::make_unique<document::SetDocumentVariableCommand>(name, value));
        }
        VariableEdit ok;
        ok.ok = true;
        return ok;
    });
    r.imported = parsed.imported;
    r.rejectedLines = parsed.rejectedLines;
    if (r.ok) {
        r.message = "imported " + std::to_string(parsed.imported) + " variable(s)" +
                    (parsed.rejectedLines.empty()
                         ? std::string()
                         : ", " + std::to_string(parsed.rejectedLines.size()) + " line(s) skipped") +
                    (r.failed != 0 ? r.message : std::string());
    }
    return r;
}

VariableEdit VariablesModel::importFile(const std::filesystem::path& path) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        return refused("not a readable file");
    }
    const std::uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec || size > kMaxImportBytes) {
        return refused("the file is too large");
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return refused("the file could not be opened");
    }
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return importText(text);
}

std::string VariablesModel::exportText() const {
    return doc_ == nullptr ? std::string() : doc_->variables().toText();
}

VariableEdit VariablesModel::exportFile(const std::filesystem::path& path) const {
    if (doc_ == nullptr) {
        return refused("there is no document");
    }
    const std::string text = exportText();
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return refused("the file could not be written");
    }
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.flush();
    if (!out) {
        return refused("the file could not be written");
    }
    VariableEdit r;
    r.ok = true;
    r.message = "exported " + std::to_string(doc_->variables().size()) + " variable(s)";
    return r;
}

}  // namespace carvetto::viewmodel
