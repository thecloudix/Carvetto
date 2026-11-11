#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/tooling/machine.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "carvetto/toolpath/time.hpp"
#include "carvetto/viewmodel/keepout.hpp"
#include "carvetto/viewmodel/toolpath_calc.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"
#include "carvetto/viewmodel/vector_selector.hpp"

// The toolpath session of the app (T-0220, ADR-0031, ADR-0014, ADR-0018, ADR-0021, ADR-0007): a
// Qt-free object over a `Document` that owns the toolpath list, the form contracts of the five
// operations and the calculation.
//
//   * List: add, edit (`update`), duplicate, delete, reorder, rename, enable/disable. Every edit
//     is ONE undo step (a macro of the core's toolpath commands); a refused edit changes nothing.
//   * Calculation is a pure function: `prepare()` copies what is needed into a `ToolpathJob`, the
//     job's `run()` (worker-ready) returns a `ToolpathCalculation`, and only `apply()` touches the
//     document, in one undo step, after checking that the inputs have not changed meanwhile.
//     `add`, `update` and `recalculate` do the three steps in a row.
//   * Staleness: a computed toolpath is "needs recalculation" when the vectors it machines (its
//     VectorSelector re-resolved now), the tool (database values) or its parameters differ from
//     what its moves were computed from. It is recalculated only on request. The comparison is on
//     content, so undoing a change that made a toolpath stale makes it current again.
//   * Records are exactly what `carvetto-cli toolpath` stores for the same inputs (name, params,
//     selection, tool snapshot and source, moves).
//   * `summary()` gives the job-level numbers (time estimate with the machine's rates, moves,
//     bounds, tool changes).
//
// Session-only state (not in the .carv file, not undoable): which toolpaths are enabled, the last
// messages of each calculation, the vector selector filters and V-carve clearance tool lists of
// toolpaths calculated in this session. A reloaded file gets them back from the records (selector
// = the stored selection, clearance tools = the adjacent records of the same V-carve).
//
// One thread at a time. The session never keeps a pointer into the document's vectors beyond a
// call.

namespace carvetto::viewmodel {

enum class ToolpathStatus { NotComputed, Current, NeedsRecalculation };

struct ToolpathListItem {
    document::ToolpathId id;
    std::string name;
    document::ToolpathOperation operation{document::ToolpathOperation::Profile};
    /// Name of the tool the moves were computed with.
    std::string toolName;
    ToolpathStatus status{ToolpathStatus::NotComputed};
    /// Why it needs recalculation (any may hold with `NeedsRecalculation`).
    bool vectorsChanged{false};
    bool parametersChanged{false};
    bool toolChanged{false};
    /// Can it be recalculated now (tool available, V-carve complete)? `note` says why not.
    bool canRecalculate{true};
    std::string note;
    /// Enabled toolpaths are in the job summary and are the ones to post.
    bool enabled{true};
    std::size_t moveCount{0};
};

/// Result of one operation on the list or of one calculation.
struct ToolpathOutcome {
    bool ok{false};
    /// For a message box: the reason when `!ok`, else a short confirmation.
    std::string message;
    /// The calculation's messages (diagnostics with severity and vector).
    std::vector<ToolpathMessage> messages;
    /// The toolpaths added or changed (job order).
    std::vector<document::ToolpathId> ids;
};

struct BatchOutcome {
    /// True when no toolpath failed.
    bool ok{false};
    std::string message;
    /// One entry per toolpath that was tried, in job order (a V-carve group counts once).
    std::vector<ToolpathOutcome> items;
    /// Number of toolpaths whose records were replaced.
    std::size_t recalculated{0};
    std::size_t failed{0};
};

/// Machine rates for the time estimate (feature-inventory "Time estimation"). Rapid rate in
/// mm/min (0 = not set: rapid moves are then left out of the estimate and `rapidRateMissing` is
/// reported), acceleration in mm/s² (infinity = instantaneous).
struct MachineSetup {
    double rapidRateMmPerMin{0.0};
    double accelerationMmPerS2{std::numeric_limits<double>::infinity()};
    bool operator==(const MachineSetup&) const = default;
};

struct ToolpathSummaryEntry {
    document::ToolpathId id;
    std::string name;
    bool included{false};
    std::size_t moves{0};
    geometry::Box2 bounds;
    double lengthMm{0.0};
    std::optional<double> minZ;
    std::optional<double> maxZ;
    toolpath::TimeEstimate time;
    int toolNumber{0};
    std::string toolName;
};

struct JobSummary {
    std::size_t toolpaths{0};
    std::size_t computed{0};
    std::size_t needRecalculation{0};
    /// Enabled, computed toolpaths: the ones the totals cover.
    std::size_t included{0};
    std::size_t moves{0};
    geometry::Box2 bounds;
    std::optional<double> minZ;
    std::optional<double> maxZ;
    /// Time in seconds by move kind, over the included toolpaths (each measured on its own: the
    /// first move has zero length, as `toolpath::estimateTime`).
    toolpath::TimeEstimate time;
    /// True when rapid moves exist but the rapid rate is not set (they are not in `time`).
    bool rapidRateMissing{false};
    /// Times the tool changes between consecutive included toolpaths (a different tool number or
    /// name or diameter), and the number of distinct tools.
    std::size_t toolChanges{0};
    std::size_t distinctTools{0};
    std::vector<ToolpathSummaryEntry> entries;
    /// Keep-out zones (T-0256): rapids of the included toolpaths replaced by a detour round a zone
    /// (the time and length above count the detours), left straight because an end lies inside a
    /// zone or no path exists, and left straight because the zones are too complex to route.
    std::size_t keepOutRouted{0};
    std::size_t keepOutUnroutable{0};
    std::size_t keepOutTooComplex{0};
};

class ToolpathSession {
   public:
    explicit ToolpathSession(document::Document* doc = nullptr);
    ~ToolpathSession();
    ToolpathSession(const ToolpathSession&) = delete;
    ToolpathSession& operator=(const ToolpathSession&) = delete;

    /// Switches document (the old one may already be gone): forgets all session state.
    void setDocument(document::Document* doc);
    document::Document* document() const { return doc_; }

    /// Decodes the non-Netpbm images (PNG, JPEG, ...) of a photo V-carve; the app supplies QImage's.
    /// Without one only Netpbm bitmaps can be carved. Must be thread-safe: it runs on the worker.
    void setBitmapDecoder(document::BitmapDecoder decoder) { bitmapDecoder_ = std::move(decoder); }

    void setToolDatabase(tooling::ToolDatabase db);
    const tooling::ToolDatabase& toolDatabase() const { return db_; }
    /// Rates for `summary()`. Non-finite or negative values are rejected (returns false).
    bool setMachineSetup(const MachineSetup& setup);
    /// Rapid rate from the user, acceleration from the machine: the smallest positive of its axes
    /// (0 = instantaneous).
    bool setMachine(const tooling::Machine& machine, double rapidRateMmPerMin);
    const MachineSetup& machineSetup() const { return setup_; }
    /// Forget cached staleness after a change made without a command (direct document mutators,
    /// switching the active sheet).
    void invalidate();

    // --- The list ----------------------------------------------------------------------------
    std::vector<ToolpathListItem> list();
    std::optional<ToolpathListItem> item(document::ToolpathId id);
    /// Messages of the last calculation of `id` (empty when none).
    std::vector<ToolpathMessage> lastMessages(document::ToolpathId id) const;
    bool isEnabled(document::ToolpathId id) const;
    /// False when `id` is not a toolpath.
    bool setEnabled(document::ToolpathId id, bool enabled);
    /// Enabled toolpaths with computed moves, in job order: what to post.
    std::vector<document::ToolpathId> enabledToolpaths();

    // --- Forms and definitions ---------------------------------------------------------------
    /// A form for a new toolpath: defaults, units of the job, the tool database's first suitable
    /// tool is NOT chosen (the user picks), the selector is the current selection when something
    /// is selected, else every vector.
    ToolpathForm newForm(document::ToolpathOperation operation);
    /// The definition of toolpath `id` (a clearance toolpath of a V-carve gives its V-carve's).
    std::optional<ToolpathDefinition> definition(document::ToolpathId id);
    std::optional<ToolpathForm> form(document::ToolpathId id);
    /// Number of vectors (contours) `selector` selects now ("objects count readout").
    std::size_t countVectors(const VectorSelector& selector) const;
    /// Photo V-carve: why `selector` names no usable bitmap now (empty when it does).
    std::string bitmapProblem(const VectorSelector& selector) const;

    // --- Calculation -------------------------------------------------------------------------
    /// Copies everything a calculation needs. `target` = recalculate that toolpath (its V-carve).
    ToolpathJob prepare(const ToolpathDefinition& definition, std::optional<document::ToolpathId> target = std::nullopt);
    /// Stores `calculation` in the document in ONE undo step: adds a new toolpath (or the
    /// V-carve's several), or replaces those of the target. Refused (nothing changes) when the
    /// calculation failed, was cancelled, or the vectors, tools or target changed since `prepare`.
    ToolpathOutcome apply(const ToolpathCalculation& calculation);
    /// Several calculations in ONE undo step; each is checked as in `apply` and the ones that pass
    /// are applied.
    BatchOutcome applyAll(const std::vector<ToolpathCalculation>& calculations);

    /// The toolpath to pass to `prepare` to recalculate or edit `id` (the carving toolpath of its
    /// V-carve, else `id` itself); nullopt when `id` is not a toolpath.
    std::optional<document::ToolpathId> targetOf(document::ToolpathId id);
    /// The jobs `recalculateAll(onlyStale)` would run (one per toolpath or V-carve, in job order),
    /// ready to run on a worker thread; apply the results with `applyAll`. Toolpaths that cannot be
    /// recalculated are not in the list; their outcomes (failures with the reason) go to `refused`
    /// when given.
    std::vector<ToolpathJob> prepareRecalculateAll(bool onlyStale, std::vector<ToolpathOutcome>* refused = nullptr);

    /// `prepare` + `run` + `apply`.
    ToolpathOutcome add(const ToolpathDefinition& definition);
    /// Recalculates toolpath `id` with a new definition and replaces it (and the other toolpaths of
    /// its V-carve). The old records stay when the calculation fails.
    ToolpathOutcome update(document::ToolpathId id, const ToolpathDefinition& definition);
    ToolpathOutcome recalculate(document::ToolpathId id);
    /// Recalculates every toolpath that needs it (`onlyStale`) or all, in ONE undo step. Toolpaths
    /// that cannot be recalculated keep their records and are reported.
    BatchOutcome recalculateAll(bool onlyStale);

    // --- Other edits (one undo step each) -----------------------------------------------------
    /// Copies toolpath `id` (and its V-carve's others) after the last one, named "<name> (copy)".
    ToolpathOutcome duplicate(document::ToolpathId id);
    ToolpathOutcome deleteToolpaths(const std::vector<document::ToolpathId>& ids);
    /// Moves toolpath `id` to position `newIndex` of the job order.
    ToolpathOutcome reorder(document::ToolpathId id, std::size_t newIndex);
    /// Renames toolpath `id` (a V-carve with clearance tools renames all of its toolpaths:
    /// "<name> [<tool>]").
    ToolpathOutcome rename(document::ToolpathId id, const std::string& name);

    // --- Job summary -------------------------------------------------------------------------
    JobSummary summary();
    /// The enabled, computed toolpaths with their rapids routed round the keep-out zones (as Save
    /// Toolpaths posts them); empty `toolpaths` when the document has no zones. Cached.
    const RoutedJob& keepOutRouting();

   private:
    using MovesKey = std::weak_ptr<const std::vector<toolpath::Move>>;

    /// What a toolpath's moves were computed from. Keyed by the identity of the moves (shared by
    /// the undo history), so undo and redo find the right one again.
    struct Baseline {
        ToolpathDefinition def;
        std::uint64_t vectors{0};
        std::uint64_t tools{0};
        /// V-carve with clearance tools: the toolpaths of one calculation share a group (> 0).
        std::uint64_t group{0};
        /// -1 the operation's own (carving) tool, i >= 0 clearance tool i, -2 a clearance
        /// toolpath whose carving toolpath is missing.
        int role{-1};
    };

    struct Staleness {
        bool computed{false};
        bool vectors{false};
        bool params{false};
        bool tool{false};
        bool canRecalculate{true};
        std::string note;
    };

    struct Stats {
        toolpath::MachineRates rates;
        bool rapidSet{false};
        std::size_t moves{0};
        geometry::Box2 bounds;
        double length{0.0};
        std::optional<double> minZ;
        std::optional<double> maxZ;
        toolpath::TimeEstimate time;
        bool rapidMissing{false};
    };

    void sync();
    void adopt(const document::ToolpathRecord& record);
    void inferGroups(const std::vector<bool>& adopted);
    Baseline* baselineOf(const document::ToolpathRecord& record);
    ToolpathDefinition definitionOf(const document::ToolpathRecord& record);
    /// The toolpaths of the record's V-carve (just the record otherwise), in job order.
    std::vector<document::ToolpathId> membersOf(const document::ToolpathRecord& record);
    /// The record that carries the V-carve's definition (the carving toolpath), or `record`.
    const document::ToolpathRecord* carvingOf(const document::ToolpathRecord& record);
    void computeStaleness();
    Stats statsOf(const document::ToolpathRecord& record, const document::SharedMoves* moves = nullptr);
    ToolpathOutcome applyOne(const ToolpathCalculation& calculation, const char* text);
    BatchOutcome applyMany(const std::vector<ToolpathCalculation>& calculations, const char* text);
    ToolpathOutcome runAndApply(const ToolpathDefinition& definition, std::optional<document::ToolpathId> target,
                                const char* text);
    std::string recordName(const document::ToolpathRecord& record, const std::string& paramsName);

    document::Document* doc_{nullptr};
    tooling::ToolDatabase db_;
    document::BitmapDecoder bitmapDecoder_;
    MachineSetup setup_;

    std::map<MovesKey, Baseline, std::owner_less<>> baselines_;
    std::map<MovesKey, Stats, std::owner_less<>> stats_;
    KeepOutRouter router_;
    std::uint64_t nextGroup_{1};
    std::set<document::ToolpathId> disabled_;
    std::map<document::ToolpathId, std::vector<ToolpathMessage>> messages_;

    bool staleValid_{false};
    std::uint64_t staleGeneration_{0};
    std::uint64_t staleDb_{0};
    std::uint64_t staleUser_{0};
    document::SheetId staleSheet_;
    std::uint64_t dbRevision_{1};
    std::uint64_t userRevision_{0};
    std::map<document::ToolpathId, Staleness> staleness_;
};

}  // namespace carvetto::viewmodel
