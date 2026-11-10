#pragma once

#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

// Double-sided job operations of the UI (T-0261, ADR-0039, ADR-0048): Copy/Move to Other Side for
// the selected objects and for toolpaths, and switching a job back to single-sided. Qt-free,
// stateless; each call is ONE undo step and is validated first (a refusal leaves the document and
// history alone). Geometry goes across mirrored about the centre line of the destination sheet's
// material (`document::flipPoint`), the same mapping the multi-sided view draws.

namespace carvetto::viewmodel {

struct SideCopyResult {
    EditResult edit;
    /// Objects created on the other side (for a move: the moved objects), top-level ones only.
    std::vector<document::ObjectId> objects;
    /// Toolpath records created on the other side.
    std::vector<document::ToolpathId> toolpaths;
};

/// Copies (or, with `move`, moves) the selected objects to the other side of the active sheet,
/// mirrored. Locked layers are respected: refused when any selected object is on a locked layer.
/// Refused unless the job is double-sided and the active sheet has a partner.
SideCopyResult copySelectionToOtherSide(document::Document& doc, bool move);

/// Copies the toolpaths `ids` to the other side of their sheet (the active sheet for toolpaths
/// without a link): the vectors they machine are copied mirrored, and each toolpath becomes a new
/// record on the other sheet over the copies, its moves mirrored (arcs swap direction), named
/// "<name> (<side>)". With `move` the original records are deleted afterwards (the original
/// vectors stay on their side).
SideCopyResult copyToolpathsToOtherSide(document::Document& doc, const std::vector<document::ToolpathId>& ids,
                                        bool move);

/// Switches a double-sided job back to single-sided (one undo step): the job type becomes single
/// and the sheets are unpaired. The back sheet is kept as an ordinary sheet with its drawing;
/// delete it afterwards if it is not wanted. Refused for jobs that are not double-sided.
EditResult makeSingleSided(document::Document& doc);

/// Job Setup (one undo step): applies `job` (from the form) and keeps the sheets in step with its
/// type: becoming double-sided pairs a back sheet (`makeDoubleSided`), leaving it unpairs
/// (`makeSingleSided`). A double-sided job keeps its flip direction and "zero off same side" from
/// `job`. Refused (document untouched) when the job cannot be made double-sided.
EditResult applyJobSetup(document::Document& doc, const document::Job& job);

/// Keys 1 and 2: activates the sheet of `side` of the active sheet's pair (no-op when it is the
/// active one). Refused without a partner.
EditResult activateSide(document::Document& doc, document::Side side);

}  // namespace carvetto::viewmodel
