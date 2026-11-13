#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/viewmodel/vector_selector.hpp"

// Toolpath definitions and their calculation as a pure function (T-0220, ADR-0031, ADR-0018).
//
// `ToolpathDefinition` is everything the user chooses: the operation's parameters (the core's own
// params structs, mm and radians), the tool (tool-database ids, with the machine and material that
// pick its cutting data), the V-carve's clearance tools and the vector selector. A `ToolpathJob`
// is a definition plus a snapshot of everything it needs (the contours, the resolved tools); its
// `run()` calls the core operation exactly as `carvetto-cli toolpath` does and returns a
// `ToolpathCalculation`: the records to store, byte for byte what the CLI stores for the same
// inputs, and the messages (core diagnostics with severity and the vector they refer to, refusals
// as errors). `run()` touches no document, shares nothing and may run on a worker thread; the
// session applies its result in ONE undo step (`ToolpathSession::apply`).

namespace carvetto::viewmodel {

/// One toolpath operation's settings.
struct ToolpathDefinition {
    /// The operation and its parameters (the variant index is the operation).
    document::ToolpathParams params{toolpath::ProfileParams{}};
    /// The tool (the carving tool for a V-carve) and the cutting data it uses.
    document::ToolSource tool;
    /// V-carve only: the end mills that clear the flat areas, in order.
    std::vector<document::ToolSource> clearanceTools;
    /// The vectors it machines.
    VectorSelector selector;

    document::ToolpathOperation operation() const { return static_cast<document::ToolpathOperation>(params.index()); }
    /// The toolpath name of the parameters.
    const std::string& name() const;
    void setName(std::string name);
    bool operator==(const ToolpathDefinition&) const = default;
};

/// A fresh definition of `operation` with the core's default parameters.
ToolpathDefinition defaultDefinition(document::ToolpathOperation operation);

/// Empty when every number of `params` is finite and within the range a form accepts (lengths and
/// times up to 1e9, angles up to 360000 degrees, counts in their range) and every option is one of
/// its choices; else the first offender. A definition built in code is held to what the forms
/// allow; the core operations judge the rest.
std::string validateParams(const document::ToolpathParams& params);

/// Longest toolpath name (bytes) the session accepts.
inline constexpr std::size_t kMaxToolpathNameBytes = 256;
/// Empty when `name` is usable: non-empty, at most `kMaxToolpathNameBytes`, no control characters
/// (a name reaches the G-code as a comment; a line break would end it).
std::string validateToolpathName(const std::string& name);

enum class MessageSeverity { Info, Warning, Error };

/// One message of a calculation: a core diagnostic, a refusal or a note about the input.
struct ToolpathMessage {
    MessageSeverity severity{MessageSeverity::Warning};
    /// The core issue it reports, when it comes from the operation.
    std::optional<toolpath::ToolpathIssue> issue;
    std::string text;
    /// The vector (drawing object) it refers to, when it names one.
    std::optional<document::ObjectId> vector;
    /// The index of the contour in the calculation's input, when it names one.
    std::optional<std::size_t> contour;

    bool operator==(const ToolpathMessage&) const = default;
};

/// A tool picked from the tool database, with the cutting data chosen for it.
struct ResolvedTool {
    tooling::Tool tool;
    /// Empty = the tool's default cutting data (no machine/material chosen).
    std::optional<tooling::CuttingData> data;
    /// True when the machine/material had no entry and the tool's default data stands in.
    bool fallback{false};
};

struct ToolLookup {
    std::optional<ResolvedTool> tool;
    /// Why the tool cannot be used (empty when `tool` is set).
    std::string error;
};

/// Finds `source`'s tool in `db` as carvetto-cli does: the tool must exist; machine and material
/// are both set or both empty and must exist; no entry for the pair falls back to the tool's
/// default data (`fallback`).
ToolLookup resolveTool(const tooling::ToolDatabase& db, const document::ToolSource& source);

/// Hash of everything of a tool and its cutting data that changes a toolpath.
std::uint64_t toolFingerprint(const ResolvedTool& tool);

/// The result of calculating one definition.
struct ToolpathCalculation {
    /// True when at least one toolpath was produced.
    bool ok{false};
    /// True when the run was cancelled (nothing is produced).
    bool cancelled{false};
    /// One record per toolpath with moves, in machining order (clearance tools first, the carving
    /// tool last). Ids are null; the session assigns them. For a Pocket/Profile/Drill/Engrave
    /// there is at most one.
    struct Item {
        /// -1 for the operation's own (carving) tool, i for clearance tool i.
        int role{-1};
        document::ToolpathRecord record;
    };
    std::vector<Item> items;
    std::vector<ToolpathMessage> messages;
    /// The definition calculated (normalised: selector deduplicated).
    ToolpathDefinition definition;
    /// Number of contours that were machined (the calculation's input).
    std::size_t vectorCount{0};
    /// What the toolpath was calculated from (for staleness), copied from the job.
    std::uint64_t vectorsFingerprint{0};
    std::uint64_t toolsFingerprint{0};
    /// The toolpath this recalculates (null for a new one).
    document::ToolpathId target;
    /// The records of the target (its V-carve) as they were when the job was prepared; `apply`
    /// refuses the result when they have changed since (an edit or rename in between).
    std::vector<document::ToolpathRecord> targetRecords;

    bool hasError() const;
    /// All error messages, joined by "; ".
    std::string errorText() const;
};

/// What an image engrave reads, copied from the document so the job stays independent of it (T-0275):
/// the encoded image (decoded on the worker thread, where the cancel flag is honoured) and where it
/// is placed.
struct PhotoInput {
    document::ObjectId bitmap;
    geometry::Box2 placement;
    std::vector<std::uint8_t> bytes;
    document::BitmapDecoder decoder;
};

/// Everything one calculation needs, copied from the document and the tool database. Independent
/// of both: it may be run on another thread, and run more than once with the same result.
struct ToolpathJob {
    ToolpathDefinition definition;
    /// Image Engrave only: the bitmap. The contours are then the vectors of the Selected Vectors
    /// strategy (none for Raster and Hatch).
    std::optional<PhotoInput> photo;
    std::vector<geometry::Contour> contours;
    /// The drawing object of each contour.
    std::vector<document::ObjectId> owners;
    /// The selection stored in the records.
    document::VectorSelection selection;
    std::optional<ResolvedTool> tool;
    std::vector<ResolvedTool> clearance;
    /// Notes found while preparing (vectors left out, cutting data fallbacks).
    std::vector<ToolpathMessage> notes;
    /// Non-empty: the job cannot run; `run()` returns this as an error.
    std::string refusal;
    std::uint64_t vectorsFingerprint{0};
    std::uint64_t toolsFingerprint{0};
    document::ToolpathId target;
    /// The sheet the records are linked to (T-0261): the active sheet of a double-sided job, else null.
    document::SheetId sheet;
    /// The records of the target (its V-carve) as they were when the job was prepared; `apply`
    /// refuses the result when they have changed since (an edit or rename in between).
    std::vector<document::ToolpathRecord> targetRecords;

    /// Runs the core operation. Deterministic. A set `cancel` flag is checked before the run and
    /// after it (the core operations themselves are not interruptible); a cancelled run produces
    /// nothing. Out-of-memory and other exceptions of the operation become error messages.
    ToolpathCalculation run(const std::atomic<bool>* cancel = nullptr) const;
};

/// Most spans a job accepts over all its contours, the largest coordinate magnitude (mm) and the
/// longest total contour length (mm); larger input is refused before the operation runs. The core
/// operations' cost grows with the length of the vectors (a 1e6 mm edge costs 14 s of CPU in a
/// Debug build, 1e7 mm minutes), and no machine works over 100 m.
inline constexpr std::size_t kMaxToolpathSpans = 2'000'000;
inline constexpr double kMaxToolpathCoordinateMm = 1.0e5;
inline constexpr double kMaxToolpathLengthMm = 1.0e7;

/// Builds a job from `definition`, the resolved vectors and the tool database (never throws; a
/// problem with the input becomes `refusal`). `selection` is what the records will store.
ToolpathJob makeToolpathJob(const ToolpathDefinition& definition, const ResolvedVectors& vectors,
                            const tooling::ToolDatabase& db, const document::VectorSelection& selection);

/// Gives `job` the bitmap of an image engrave (a no-op for other operations): copies the encoded
/// image and placement of `bitmap`, or sets the job's refusal when there is none or it is unusable.
void attachPhotoInput(ToolpathJob& job, const document::Document& doc, const ResolvedBitmap& bitmap,
                      const document::BitmapDecoder& decoder);

/// Hash of what the toolpath of `params` reads from `selector` in `doc`: the resolved vectors, or
/// for an image engrave its bitmap (and, for Selected Vectors, the vectors too).
std::uint64_t inputsFingerprint(const document::Document& doc, const VectorSelector& selector,
                                const document::ToolpathParams& params);

/// Hash of the tools `definition` uses in `db` (see `toolFingerprint`); 0 when one is missing.
std::uint64_t toolsFingerprintOf(const ToolpathDefinition& definition, const tooling::ToolDatabase& db);

/// True when the operation accepts `type` as its tool (`clearance`: a V-carve clearance tool).
/// Profile and Pocket accept any type (the core refuses unusable geometry).
bool toolAllowedFor(document::ToolpathOperation operation, tooling::ToolType type, bool clearance = false);

/// Name of an operation for messages: "profile", "pocket", "drill", "engrave", "v_carve", "image_engrave".
const char* operationName(document::ToolpathOperation operation);

}  // namespace carvetto::viewmodel
