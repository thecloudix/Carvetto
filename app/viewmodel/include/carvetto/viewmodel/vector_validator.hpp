#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/validator.hpp"

/// Vector Validator (T-0249): runs `geometry/validator.hpp` over the vectors of a document, selects
/// offenders and repairs what can be repaired as one undo step. Qt-free.
namespace carvetto::viewmodel {

enum class ValidatorScope {
    Selection,   ///< The selected vectors (groups expand to their vectors).
    AllVisible,  ///< Every vector on the active sheet whose layer is visible and unlocked.
};

struct ValidatorOptions {
    ValidatorScope scope{ValidatorScope::AllVisible};
    /// V-carving mode: crossings inside text are expected (glyph outlines overlap), so text objects
    /// are not checked for self-intersection.
    bool vCarvingMode{false};
    geometry::validator::Options geometry{};
    geometry::validator::FixFlags fixes{};
};

/// Selection when something is selected, otherwise every visible vector.
ValidatorScope defaultValidatorScope(const document::Document& doc);

struct ValidatorReport {
    geometry::validator::Report report;
    /// `report.issues[i].object` indexes this list (and `otherObject`).
    std::vector<document::ObjectId> objects;

    std::size_t count(geometry::validator::IssueKind k) const { return report.count(k); }
    bool clean() const { return report.total() == 0 && report.skipped == 0 && !report.truncated; }
    /// Objects with at least one issue (of `kind` when given), each once, in document order.
    std::vector<document::ObjectId> offenders() const;
    std::vector<document::ObjectId> offenders(geometry::validator::IssueKind kind) const;
    /// Multi-line human-readable summary (counts per kind, skipped/truncated notes).
    std::string summary() const;
};

ValidatorReport validateVectors(const document::Document& doc, const ValidatorOptions& options = {});

/// Selects the offenders of the report (of one kind when given). Returns how many objects were
/// selected; the selection is left alone when none.
std::size_t selectOffenders(document::Document& doc, const ValidatorReport& report);
std::size_t selectOffenders(document::Document& doc, const ValidatorReport& report, geometry::validator::IssueKind kind);

/// Edit > Select All Duplicate Vectors: validates the scope and selects the objects holding a
/// repeated contour (the later copies).
std::size_t selectDuplicateVectors(document::Document& doc, const ValidatorOptions& options = {});

struct ValidatorFixResult {
    bool ok{false};
    std::string message;
    geometry::validator::FixSummary fixed;
    std::size_t objectsChanged{0};
    std::size_t objectsRemoved{0};  ///< Objects left without contours are deleted.
};

/// Repairs the scope as one undo step (nothing is recorded when nothing changed).
ValidatorFixResult fixVectors(document::Document& doc, const ValidatorOptions& options = {});

}  // namespace carvetto::viewmodel
