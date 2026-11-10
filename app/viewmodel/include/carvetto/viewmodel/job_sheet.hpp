#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/viewmodel/time_readout.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

// Job sheet (T-0244, feature-inventory "Create Job Sheet" and "Time estimation"): a report of the
// job (material setup), the tools used, a summary of the time estimate and a list of every
// toolpath with its feed, plunge and spindle values, plus a layout thumbnail (inline SVG: the
// material block and the bounds of each included toolpath).
//
// Qt-free and pure: `makeJobSheet` copies the numbers out of the document and the session's
// `JobSummary` (so the times are exactly `toolpath::estimateTime`'s), the two renderers are pure
// functions of the sheet. Output is deterministic (no dates, no locale, fixed number formats).
// Every user-supplied string (job title, toolpath and tool names) is HTML-escaped in the HTML and
// has control characters replaced by spaces in the text form, so a hostile name cannot add
// markup or forge lines. Export format: HTML and plain text (no PDF).

namespace carvetto::viewmodel {

struct JobSheetToolpath {
    std::string name;
    std::string operation;
    std::string toolName;
    int toolNumber{0};
    double toolDiameterMm{0.0};
    double feedRateMmPerMin{0.0};
    double plungeRateMmPerMin{0.0};
    double spindleRpm{0.0};
    std::size_t moves{0};
    /// Enabled and computed: in the totals.
    bool included{false};
    /// Seconds by move kind (as the job summary).
    toolpath::TimeEstimate time;
    geometry::Box2 bounds;
};

struct JobSheetTool {
    std::string name;
    int number{0};
    double diameterMm{0.0};
    /// Names of the included toolpaths using the tool, in job order.
    std::vector<std::string> toolpaths;
};

struct JobSheet {
    std::string title;
    // Material setup.
    std::string jobType;
    double widthMm{0.0};
    double heightMm{0.0};
    double thicknessMm{0.0};
    std::string zZero;
    std::string datum;
    bool inches{false};
    geometry::Box2 material;
    // Summary.
    TimeReadout readout;
    JobSummary summary;
    std::vector<JobSheetTool> tools;
    std::vector<JobSheetToolpath> toolpaths;
};

/// The sheet of `doc` with `summary` (from `ToolpathSession::summary()` on the same document).
/// `title` is the job's name (empty = "Untitled job").
JobSheet makeJobSheet(const document::Document& doc, const JobSummary& summary, const std::string& title = {});

/// HTML-escapes &, <, >, " and ' and replaces control characters by spaces.
std::string escapeHtml(const std::string& text);

/// A complete, self-contained HTML5 document (no scripts, no external resources).
std::string renderJobSheetHtml(const JobSheet& sheet);
/// Plain text, UTF-8, LF line ends.
std::string renderJobSheetText(const JobSheet& sheet);

}  // namespace carvetto::viewmodel
