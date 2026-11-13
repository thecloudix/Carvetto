#pragma once

#include <cstddef>
#include <filesystem>
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
//
// Bounded size: every user string is clipped to `kJobSheetMaxNameBytes` bytes (cut at a UTF-8
// boundary, "..." appended); the toolpath list shows at most `kJobSheetMaxToolpathRows` rows, the
// tool table `kJobSheetMaxToolRows` rows, each tool lists at most `kJobSheetMaxToolpathsPerTool`
// toolpath names and at most `kJobSheetMaxWarnings` warnings are shown; what is left out is
// replaced by an "N more" notice. The layout thumbnail draws at most `kJobSheetMaxToolpathRows`
// rectangles and says so when toolpaths are left out. `makeJobSheet` itself stores only the capped
// rows (with the real counts), so the sheet and the output are bounded independently of the job
// size: the measured worst case is about 740 KB of HTML (every string at its limit).

namespace carvetto::viewmodel {

inline constexpr std::size_t kJobSheetMaxNameBytes = 80;
inline constexpr std::size_t kJobSheetMaxToolpathRows = 500;
inline constexpr std::size_t kJobSheetMaxToolRows = 100;
inline constexpr std::size_t kJobSheetMaxToolpathsPerTool = 20;
inline constexpr std::size_t kJobSheetMaxWarnings = 20;

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
    /// How many included toolpaths use the tool (`toolpaths` is capped; 0 = as stored).
    std::size_t toolpathsTotal{0};
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
    // `makeJobSheet` stores at most the capped number of rows; these give the real counts
    // (0 = as stored) so the "N more" notices stay right.
    std::size_t toolsTotal{0};
    std::size_t toolpathsTotal{0};
    /// Included toolpaths with bounds that were dropped from `toolpaths` and so are not in the thumbnail.
    std::size_t thumbnailOmitted{0};
};

/// The sheet of `doc` with `summary` (from `ToolpathSession::summary()` on the same document).
/// `title` is the job's name (empty = "Untitled job").
JobSheet makeJobSheet(const document::Document& doc, const JobSummary& summary, const std::string& title = {});

/// `text` cut to at most `kJobSheetMaxNameBytes` bytes (at a UTF-8 boundary) with "..." appended
/// when it was longer.
std::string clipJobSheetText(const std::string& text);

/// HTML-escapes &, <, >, " and ' and replaces control characters by spaces.
std::string escapeHtml(const std::string& text);

/// A complete, self-contained HTML5 document (no scripts, no external resources).
std::string renderJobSheetHtml(const JobSheet& sheet);
/// Plain text, UTF-8, LF line ends.
std::string renderJobSheetText(const JobSheet& sheet);

/// Pdf is only a name for the app layer (`checkJobSheetPath`, extension); `writeJobSheet` refuses it.
enum class JobSheetFormat { Html, Text, Pdf };

/// ".html", ".txt" or ".pdf".
const char* jobSheetExtension(JobSheetFormat format);

/// A safe default file name for `title`: sanitised stem (or "job-sheet") plus the extension.
std::string defaultJobSheetFileName(const std::string& title, JobSheetFormat format);

/// Empty when `path` is acceptable as a job sheet target, else why not: empty path, a ".."
/// component, a file name that is not its own sanitised form (control characters, separators,
/// reserved device names, trailing dots), an extension other than the format's (".htm" also
/// for HTML), or a path that is an existing directory.
std::string checkJobSheetPath(const std::filesystem::path& path, JobSheetFormat format);

struct JobSheetWriteResult {
    bool ok{false};
    std::string error;
    std::filesystem::path written;
};

/// Renders `sheet` in `format` and writes it to `path` through `writeOutput` (staged under a
/// random name, moved into place atomically, never through a link). Refuses hostile paths
/// (`checkJobSheetPath`) and, without `overwrite`, an existing file.
JobSheetWriteResult writeJobSheet(const JobSheet& sheet, JobSheetFormat format, const std::filesystem::path& path,
                                  bool overwrite);

}  // namespace carvetto::viewmodel
