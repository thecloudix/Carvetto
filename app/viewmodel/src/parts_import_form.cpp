#include "carvetto/viewmodel/parts_import_form.hpp"

#include <cmath>
#include <cstdio>
#include <map>

#include "carvetto/io/parts_build.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"

namespace carvetto::viewmodel {

using document::ToolpathOperation;

void PartsImportForm::setMappingText(std::string text) {
    mappingText_ = std::move(text);
    loadError_.clear();
    parsed_ = false;
}

void PartsImportForm::setPartsText(std::string text) {
    partsText_ = std::move(text);
    loadError_.clear();
    parsed_ = false;
}

std::string PartsImportForm::loadMappingFile(const std::string& path) {
    std::string err;
    auto text = io::readBoundedFile(path, io::kMaxMappingBytes, &err);
    if (!text) return err;
    setMappingText(std::move(*text));
    return {};
}

std::string PartsImportForm::loadPartsFile(const std::string& path) {
    std::string err;
    auto text = io::readBoundedFile(path, io::kMaxPartsFileBytes, &err);
    if (!text) return err;
    setPartsText(std::move(*text));
    return {};
}

void PartsImportForm::setTool(ToolpathOperation operation, document::ToolSource tool) {
    switch (operation) {
        case ToolpathOperation::Profile: profileTool_ = std::move(tool); break;
        case ToolpathOperation::Drill: drillTool_ = std::move(tool); break;
        case ToolpathOperation::Pocket: pocketTool_ = std::move(tool); break;
        default: break;
    }
}

void PartsImportForm::refreshCanApply() {
    preview_.canApply = preview_.mappingOk && !list_.parts.empty() && (!list_.hasErrors() || allowPartial_);
}

void PartsImportForm::parse() {
    preview_ = PartsImportPreview{};
    list_ = io::PartsList{};
    mapping_ = io::PartsMapping{};
    io::MappingResult m = io::parsePartsMapping(mappingText_);
    preview_.mappingOk = m.ok;
    preview_.diagnostics = m.diagnostics;
    if (m.ok) {
        mapping_ = m.mapping;
        list_ = io::readPartsText(partsText_, mapping_);
        preview_.diagnostics.insert(preview_.diagnostics.end(), list_.diagnostics.begin(), list_.diagnostics.end());
        preview_.partsRead = !list_.parts.empty() || !list_.hasErrors();
        preview_.parts = list_.parts.size();
        preview_.instances = list_.instances;
        preview_.rowsSkipped = list_.rowsSkipped;
    }
    for (const auto& d : preview_.diagnostics) {
        if (d.severity == io::PartsDiagnostic::Severity::Error) {
            ++preview_.errors;
        } else {
            ++preview_.warnings;
        }
    }
    parsed_ = true;
    refreshCanApply();
}

const PartsImportPreview& PartsImportForm::preview() {
    if (!parsed_) parse();
    refreshCanApply();
    return preview_;
}

namespace {

long long micro(double mm) { return std::llround(mm * 1000.0); }

std::string num(double v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.4f", v);
    return buf;
}

}  // namespace

PartsImportOutcome PartsImportForm::apply(document::Document& doc, ToolpathSession* session) {
    PartsImportOutcome out;
    if (!parsed_) parse();
    refreshCanApply();
    out.diagnostics = preview_.diagnostics;
    if (!preview_.mappingOk) {
        out.message = "The mapping file has errors; nothing was imported.";
        return out;
    }
    if (list_.parts.empty()) {
        out.message = "The parts list has no usable rows; nothing was imported.";
        return out;
    }
    if (!preview_.canApply) {
        out.message = "The parts list has errors; fix them or allow importing the valid rows.";
        return out;
    }

    document::UndoStack& undo = doc.undoStack();
    undo.beginMacro("Import parts");
    io::BuildResult built = io::buildPanels(doc, list_, mapping_);
    out.diagnostics.insert(out.diagnostics.end(), built.diagnostics.begin(), built.diagnostics.end());
    if (!built.ok) {
        undo.endMacro();
        out.message = "The panels could not be built; nothing was imported.";
        return out;
    }
    out.panels = built.placed;
    out.sheets = built.sheets.size();

    if (createToolpaths_ && session != nullptr) {
        const tooling::ToolDatabase& db = session->toolDatabase();
        auto note = [&](const std::string& text) {
            out.diagnostics.push_back({io::PartsDiagnostic::Severity::Warning, "toolpath", 0, text});
        };
        auto make = [&](ToolpathOperation op, const document::ToolSource& tool, const char* what, const std::string& name,
                        double depthMm, std::vector<document::ObjectId> objects) {
            if (tool.toolId.empty()) {
                note(std::string("no ") + what + " tool chosen: no " + what + " toolpath made");
                return;
            }
            ToolpathForm form(op, document::Units::Millimetres);
            auto r = form.setTool(db, tool);
            if (!r.ok) {
                note(std::string(what) + " tool: " + r.message);
                return;
            }
            r = form.setField("cutDepth", num(depthMm));
            if (!r.ok) {
                note(std::string(what) + " depth: " + r.message);
                return;
            }
            form.setName(name);
            form.setSelector(VectorSelector::fromObjects(std::move(objects)));
            const ToolpathOutcome o = session->add(form.definition());
            if (!o.ok) {
                note(name + ": " + o.message);
                return;
            }
            out.toolpaths.insert(out.toolpaths.end(), o.ids.begin(), o.ids.end());
        };

        for (const io::BuiltSheet& bs : built.sheets) {
            doc.setActiveSheet(bs.sheet);
            session->invalidate();
            const std::string sheetName = doc.findSheet(bs.sheet) ? doc.findSheet(bs.sheet)->name : std::string("Sheet");
            // Dados first (they weaken nothing yet), holes next, the outline last so the panel is
            // still held while it is machined.
            std::map<long long, std::vector<document::ObjectId>> dadoDepths;
            std::map<long long, double> dadoDepthMm;
            for (const io::BuiltFeature& f : bs.dados) {
                dadoDepths[micro(f.depthMm)].push_back(f.object);
                dadoDepthMm[micro(f.depthMm)] = f.depthMm;
            }
            for (auto& [key, ids] : dadoDepths) {
                make(ToolpathOperation::Pocket, pocketTool_, "dado", sheetName + " dados " + num(dadoDepthMm[key]) + " mm",
                     dadoDepthMm[key], ids);
            }
            std::map<long long, std::vector<document::ObjectId>> holeDepths;
            std::map<long long, double> holeDepthMm;
            for (const io::BuiltFeature& f : bs.holes) {
                const double d = f.depthMm > 0.0 ? f.depthMm : bs.thicknessMm;
                holeDepths[micro(d)].push_back(f.object);
                holeDepthMm[micro(d)] = d;
            }
            for (auto& [key, ids] : holeDepths) {
                make(ToolpathOperation::Drill, drillTool_, "drilling", sheetName + " holes " + num(holeDepthMm[key]) + " mm",
                     holeDepthMm[key], ids);
            }
            make(ToolpathOperation::Profile, profileTool_, "profile", sheetName + " outlines", bs.thicknessMm, bs.panels);
        }
        if (!built.sheets.empty()) doc.setActiveSheet(built.sheets.front().sheet);
        session->invalidate();
    }
    undo.endMacro();
    out.ok = true;
    out.message = "Imported " + std::to_string(out.panels) + " panels onto " + std::to_string(out.sheets) + " sheets with " +
                  std::to_string(out.toolpaths.size()) + " toolpaths.";
    return out;
}

}  // namespace carvetto::viewmodel
