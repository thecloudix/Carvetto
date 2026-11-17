#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/relief/relief.hpp"
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
    /// Calculation edit boxes (T-0257, T-0271): field id -> the `=...` text; stored in the records.
    std::map<std::string, std::string> expressions;

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
    /// Thread mill with "create circles for internal threads" (T-0262, ADR-0056): the bore circles
    /// to insert into the document when the result is applied, in the same undo step as the toolpath.
    struct BoreCircle {
        geometry::Contour circle;
        /// The vector the thread was made in: the circle goes onto its layer and sheet.
        document::ObjectId source;
    };
    std::vector<BoreCircle> boreCircles;
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

/// The toolpath a projection onto the model re-heights (T-0311, ADR-0091), copied from the document: the moves
/// are shared and immutable.
struct ProjectSource {
    document::ToolpathId id;
    std::string name;
    toolpath::ToolSnapshot tool;
    document::SharedMoves moves;
};

/// A prior toolpath of a 3D finish (T-0314): the record's moves with the tool it was cut with.
struct PriorInput {
    ProjectSource source;
    ResolvedTool tool;
};

/// Everything one calculation needs, copied from the document and the tool database. Independent
/// of both: it may be run on another thread, and run more than once with the same result.
struct ToolpathJob {
    ToolpathDefinition definition;
    /// Image Engrave, Laser Picture and Laser Sketch (bitmap source): the bitmap. The contours are then the
    /// vectors of the Selected Vectors strategy / the Vectors boundary (none otherwise).
    std::optional<PhotoInput> photo;
    /// Laser Sketch with the machine limit boundary only (T-0324, `attachMachineLimits`): where the machine can burn.
    std::optional<geometry::Box2> machineLimits;
    /// 3D rough and 3D finish only (T-0303, ADR-0085): the composed relief model of the sheet at the
    /// finest cell size, shared and immutable. The contours are then the Vectors boundary (if used).
    std::shared_ptr<const relief::HeightField> model;
    /// Project Toolpath onto Model / 3D Tabs only (T-0311): the toolpath to project (`attachSource`).
    std::optional<ProjectSource> source;
    /// 3D finish only (T-0314): the toolpaths whose result is the starting stock (`attachPriors`).
    std::vector<PriorInput> priors;
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
    /// V-carve inlay plug only: the bounds of the sheet the plug is cut on (the plug sheet's, else the
    /// vectors' sheet's material rectangle); empty when unknown (`attachPlugSheet`).
    geometry::Box2 plugSheetBounds;
    /// The sheet the records are linked to (T-0261): the active sheet of a double-sided job, else null.
    document::SheetId sheet;
    /// The records of the target (its V-carve) as they were when the job was prepared; `apply`
    /// refuses the result when they have changed since (an edit or rename in between).
    std::vector<document::ToolpathRecord> targetRecords;
    /// Rotary job with the Z zero on the cylinder axis (T-0343, ADR-0117): the Z (mm) of the cylinder surface,
    /// added to every move of the surface-referenced operations so their depths are measured from the surface.
    double zShift{0.0};

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

/// Gives `job` the sheet a V-carve inlay plug is cut on (a no-op for other operations): its bounds
/// for Sheet Limits and centring, and (for a plug on another sheet, in a double-sided job) the
/// sheet the records are linked to. Refuses when the named sheet does not exist.
void attachPlugSheet(ToolpathJob& job, const document::Document& doc);

/// Gives `job` the bitmap of a image engrave (a no-op for other operations): copies the encoded
/// image and placement of `bitmap`, or sets the job's refusal when there is none or it is unusable.
void attachPhotoInput(ToolpathJob& job, const document::Document& doc, const ResolvedBitmap& bitmap,
                      const document::BitmapDecoder& decoder);

/// True for the operations that machine the composed relief model (3D rough, 3D finish, and the projection
/// of a toolpath onto it).
bool usesModel(document::ToolpathOperation operation);

/// True when the toolpath of `params` reads a bitmap of the selection: an image engrave, a laser picture, a laser
/// sketch with the bitmap source (T-0324).
bool usesBitmap(const document::ToolpathParams& params);

/// False for a Rounding toolpath (T-0322): it machines the rotary cylinder itself, so no vectors are selected.
bool needsVectors(const document::ToolpathParams& params);

/// True when the toolpath of `params` reads the sheet's composed relief model: `usesModel`, and a laser sketch with
/// the model source (T-0324).
bool readsModel(const document::ToolpathParams& params);

/// True when the selected vectors are an input of a bitmap or model operation (T-0326: public for the form): a 3D
/// toolpath's Vectors boundary, the Selected Vectors strategy of an image engrave or laser picture, a laser
/// sketch's Vectors boundary.
bool boundaryUsesVectors(const document::ToolpathParams& params);

/// The machine limits a laser sketch honours (ADR-0105): the active sheet's material rectangle, cut down to the
/// machine's work area measured from the rectangle's lower-left corner when `machine` states one. Empty when the
/// sheet has no area.
std::optional<geometry::Box2> machineLimitsOf(const document::Document& doc, const tooling::Machine* machine);

/// Hash of the machine limits without a machine (the sheet's material rectangle), for staleness.
std::uint64_t machineLimitsFingerprint(const document::Document& doc);

/// Gives a laser sketch job with the machine limit boundary its limits (a no-op for other jobs); refuses when the
/// sheet has no area. The machine is the one the tool's cutting data was chosen for, if any.
void attachMachineLimits(ToolpathJob& job, const document::Document& doc, const tooling::ToolDatabase& db);

/// Gives `job` the composed relief model of the document's active sheet (a no-op for operations that do not read
/// it: `readsModel`), or sets the job's refusal when there is none or it is too large to compose.
void attachModel(ToolpathJob& job, const document::Document& doc);

/// Gives `job` the toolpath a Project Toolpath onto Model / 3D Tabs job projects (a no-op for other operations):
/// the record named by `Project3dParams::sourceToolpath`. Refuses when it does not exist or has no moves.
void attachSource(ToolpathJob& job, const document::Document& doc);

/// Gives `job` the prior toolpaths of a 3D finish (a no-op for other operations): the records
/// `Finish3dParams::priorToolpaths` names, each with its tool resolved in `db`. Refuses when one does not
/// exist, is not calculated, has no tool, is listed twice or is one of `own` (the toolpaths the job replaces).
void attachPriors(ToolpathJob& job, const document::Document& doc, const tooling::ToolDatabase& db,
                  const std::vector<document::ToolpathId>& own);

/// Hash of the moves of the toolpath record `sourceToolpath` (a fixed value when it does not exist).
std::uint64_t sourceFingerprint(const document::Document& doc, std::uint64_t sourceToolpath);

/// Why the active sheet's relief model cannot be machined (no levels, nothing visible, too large);
/// empty when it can.
std::string modelProblem(const document::Document& doc);

/// Hash of the visible relief model of `sheet` (levels, components and every height value).
std::uint64_t modelFingerprint(const document::Document& doc, document::SheetId sheet);

/// Defaults for a new 3D toolpath from the job and the model: Z zero at the material surface puts
/// the stock top at 0 and the bottom at -thickness (MachineBed: thickness and 0); the model's highest
/// point is placed at the stock top (`zOffset`); the safe Z is 5 mm above the stock top. On a rotary job the
/// stock top is the cylinder surface (the radius when Z zero is the axis) and the bottom the model band's.
struct ModelDefaults {
    bool valid{false};
    double zOffset{0.0};
    double stockTop{0.0};
    double stockBottom{-20.0};
    double safeZ{5.0};
};
ModelDefaults modelDefaults(const document::Document& doc);

/// Z (mm) added to the moves of `operation` on `job` (T-0343, ADR-0117): the surface Z of a rotary job whose Z zero
/// is the cylinder axis, for the operations whose depths are measured from the surface; 0 for every other job and
/// for the operations that take absolute Z (the model toolpaths, Moulding and Rounding).
double surfaceDepthShift(const document::Job& job, document::ToolpathOperation operation);

/// Hash of what the toolpath of `params` reads from `selector` in `doc`: the resolved vectors, or
/// for an image engrave its bitmap (and, for Selected Vectors, the vectors too), for a 3D rough or
/// finish the relief model (and, for the Vectors boundary, the vectors too).
std::uint64_t inputsFingerprint(const document::Document& doc, const VectorSelector& selector,
                                const document::ToolpathParams& params);

/// Hash of the tools `definition` uses in `db` (see `toolFingerprint`); 0 when one is missing.
std::uint64_t toolsFingerprintOf(const ToolpathDefinition& definition, const tooling::ToolDatabase& db);

/// True when the operation accepts `type` as its tool (`clearance`: a V-carve clearance tool).
/// Profile and Pocket accept any type (the core refuses unusable geometry).
bool toolAllowedFor(document::ToolpathOperation operation, tooling::ToolType type, bool clearance = false);

/// True for the operations whose calculation takes a list of earlier tools (V-carve and V-carve inlay: clearance
/// tools; 3D rough and 3D finish: rest tools; moulding: one larger-area clearance tool). Each tool's toolpath is
/// its own record and the records of one calculation form a group (T-0314, ADR-0094).
bool hasClearanceTools(document::ToolpathOperation operation);

/// True for the 3D operations whose tool list is a rest-tool list (3D rough, 3D finish, moulding).
bool hasRestTools(document::ToolpathOperation operation);

/// Name of an operation for messages: "profile", "pocket", "drill", "engrave", "v_carve", "image_engrave".
const char* operationName(document::ToolpathOperation operation);

}  // namespace carvetto::viewmodel
