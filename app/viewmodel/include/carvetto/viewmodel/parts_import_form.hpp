#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/io/parts_import.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

// The Import Parts form (T-0329, ADR-0110): a parts list (CSV) and a mapping file in, nested panels
// and Profile / Drilling / Pocket toolpaths out, as ONE undo step. Qt-free; the app's dialog (file
// pickers, diagnostics list, tool combos) is a follow-up that drives this object.
//
//   form.setMappingText(...) / loadMappingFile(path); form.setPartsText(...) / loadPartsFile(path)
//   form.setTool(ToolpathOperation::Profile, source) ... (optional: no tool = no toolpath of that kind)
//   const auto& pv = form.preview();     // diagnostics, counts, whether apply() would proceed
//   form.apply(doc, &session)            // one undo step: panels, nesting, toolpaths
//
// Files are read through `io::readBoundedFile` (size checked before reading).

namespace carvetto::viewmodel {

struct PartsImportPreview {
    bool mappingOk{false};
    /// The parts list could be read (rows with errors are counted in `rowsSkipped`).
    bool partsRead{false};
    std::size_t parts{0};       ///< Rows that passed.
    std::size_t instances{0};   ///< Sum of the quantities.
    std::size_t rowsSkipped{0};
    std::size_t errors{0};
    std::size_t warnings{0};
    /// Mapping, then parts-list diagnostics.
    std::vector<io::PartsDiagnostic> diagnostics;
    /// True when `apply` would build something: the mapping is valid, at least one row passed, and
    /// there are no errors (or `allowPartial` is set and the passing rows exist).
    bool canApply{false};
};

struct PartsImportOutcome {
    bool ok{false};
    std::string message;
    std::vector<io::PartsDiagnostic> diagnostics;
    std::size_t panels{0};
    std::size_t sheets{0};
    std::vector<document::ToolpathId> toolpaths;
};

class PartsImportForm {
   public:
    void setMappingText(std::string text);
    void setPartsText(std::string text);
    /// Empty string = loaded; else the reason (too large, unreadable).
    std::string loadMappingFile(const std::string& path);
    std::string loadPartsFile(const std::string& path);

    /// Tool for Profile, Drill or Pocket (any other operation is ignored). An empty tool id = none.
    void setTool(document::ToolpathOperation operation, document::ToolSource tool);
    /// Default true.
    void setCreateToolpaths(bool on) { createToolpaths_ = on; }
    /// Import the rows without errors although some rows have them (default false).
    void setAllowPartial(bool on) { allowPartial_ = on; }
    bool createToolpaths() const { return createToolpaths_; }
    bool allowPartial() const { return allowPartial_; }

    /// Parses both texts (cached until a setter changes them).
    const PartsImportPreview& preview();

    /// Builds the panels in `doc` and, with a session and tools, the toolpaths: one undo step. The
    /// document is untouched when `ok` is false (a failed toolpath is reported and the import stands;
    /// only an exhausted identifier space can leave a partial import, as one undoable step).
    PartsImportOutcome apply(document::Document& doc, ToolpathSession* session);

   private:
    void parse();
    void refreshCanApply();

    std::string mappingText_;
    std::string partsText_;
    std::string loadError_;
    document::ToolSource profileTool_, drillTool_, pocketTool_;
    bool createToolpaths_{true};
    bool allowPartial_{false};
    bool parsed_{false};
    PartsImportPreview preview_;
    io::PartsMapping mapping_;
    io::PartsList list_;
};

}  // namespace carvetto::viewmodel
