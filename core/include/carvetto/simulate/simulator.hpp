#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/vec3.hpp"
#include "carvetto/simulate/stock.hpp"
#include "carvetto/simulate/tool_shape.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Material-removal simulation (T-0222, ADR-0032; ARCHITECTURE.md "simulate"). A `Simulator` owns a
// heightfield of the stock and cuts it along the move lists of one or more toolpaths, in order,
// with the exact surface swept by the tool profile at every cell centre. Playback is incremental:
// `advanceToMove` / `advanceToTime` continue from where the last call stopped.
//
// Cost model (per move): O(cells inside the move's swept footprint, clipped to the stock), i.e.
// the capsule of the path widened by the tool radius; a move that stays above the stock top costs
// O(1). Total cost is near-linear in (moves x footprint cells). Memory: 5 bytes per cell
// (9 with per-cell pass ids) plus 72 bytes per queued move.

namespace carvetto::simulate {

enum class SimStatus {
    Ok,
    InvalidStock,        ///< The stock failed `checkStock` (dimensions or cell size).
    TooManyCells,        ///< The stock needs more cells than its limit (a clear message says how many).
    InvalidTool,         ///< The tool shape is not valid.
    InvalidMove,         ///< A move has non-finite or out-of-range coordinates, or a degenerate arc.
    TooManyMoves,        ///< More queued moves than `SimOptions::maxMoves`.
    WorkBudgetExceeded,  ///< The cell-evaluation budget `SimOptions::maxCellWork` ran out.
};

struct SimError {
    SimStatus status{SimStatus::Ok};
    std::string message;
    /// Global move index of an InvalidMove (index into the queue), or the move at which the work
    /// budget ran out.
    std::optional<std::size_t> move;
    bool ok() const { return status == SimStatus::Ok; }
};

enum class DiagnosticKind {
    RapidInMaterial,           ///< A rapid move whose tool would remove material that is still there.
    BelowTable,                ///< The tool tip goes below the bottom of the stock (the table/clamp).
    ToolTooDeep,               ///< Material stands above the flute top within the tool's reach (the shank would meet it).
    ToolChangeWithoutRetract,  ///< A different tool starts while the previous one was still inside the stock.
};

struct SimDiagnostic {
    DiagnosticKind kind{DiagnosticKind::RapidInMaterial};
    /// Global move index (for ToolChangeWithoutRetract: the first move of the new toolpath).
    std::size_t move{0};
    /// Id the toolpath was added with.
    std::uint32_t toolpathId{0};
    /// End point of the move.
    geometry::Vec3 position;
    std::string message;
};

/// Machine and limits for a simulation.
struct SimOptions {
    /// Rapid rate in mm/min used to time Rapid moves (feed and plunge rates come from each
    /// toolpath's `ToolSnapshot`).
    double rapidRate{5000.0};
    /// Acceleration in mm/s^2 for timing; infinity = none.
    double acceleration{std::numeric_limits<double>::infinity()};
    /// Slack in mm of the collision diagnostics (a rapid along a floor cut to the same Z is not a
    /// collision). Also the depth below the table that counts as BelowTable.
    double collisionTolerance{1e-3};
    /// Most moves that can be queued.
    std::size_t maxMoves{8'000'000};
    /// Most diagnostics kept; further ones are only counted.
    std::size_t maxDiagnostics{10'000};
    /// Budget of cell evaluations (the CPU bound): a run stops with WorkBudgetExceeded beyond it.
    double maxCellWork{1e10};
};

class Simulator {
public:
    /// Creates the heightfield (allocating `stock.cellCount * 5..9` bytes unless the stock is
    /// refused: then `ok()` is false, `error()` says why, and nothing is allocated).
    explicit Simulator(const Stock& stock, const SimOptions& options = {});
    ~Simulator();
    Simulator(Simulator&&) noexcept;
    Simulator& operator=(Simulator&&) noexcept;
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;

    bool ok() const;
    const SimError& error() const;

    /// Queues a toolpath, cut with `tool` after everything queued before it. `id` labels its cells
    /// in the pass-id layer (use non-zero ids; 0 means uncut). `from` is the tool position before
    /// the first move: it defaults to the end of the previous toolpath, and when there is none, the
    /// first move has zero length (a cutting first move then stamps the tool at its end point).
    /// All moves are validated first; on refusal nothing is queued and `error()` says why (InvalidTool,
    /// InvalidMove with the move's global index, TooManyMoves). Returns true when queued.
    bool addToolpath(const toolpath::Toolpath& path, const ToolShape& tool, std::uint32_t id,
                     std::optional<geometry::Vec3> from = std::nullopt);

    /// Number of queued moves (all toolpaths) and the number completely cut so far.
    std::size_t moveCount() const;
    std::size_t movesDone() const;
    /// Cuts moves up to (excluding) global index `n`, continuing from `movesDone()`. A target at or
    /// before `movesDone()` does nothing. Returns false when the simulator is in an error state.
    bool advanceToMove(std::size_t n);
    /// Cuts everything that is completed by `seconds` of machine time, plus the part of the move in
    /// progress (a prefix of its path proportional to the time; ignoring acceleration within it).
    /// Continues from the current state; a time before the current one does nothing.
    bool advanceToTime(double seconds);
    bool advanceAll() { return advanceToMove(moveCount()); }
    /// Random access: like advanceToMove/advanceToTime but rewinds (reset, then replay) when the
    /// target is behind the current state.
    bool seekMove(std::size_t n);
    bool seekTime(double seconds);
    /// Restores the uncut stock, clears diagnostics and the error of a spent work budget; keeps the queue.
    void reset();

    /// Machine time in seconds at the end of every queued move (cumulative; a pure estimate from
    /// the toolpaths' rates and `SimOptions`), and of the whole queue.
    const std::vector<double>& moveEndTimes() const;
    double totalSeconds() const;
    /// Time reached by the state: end time of the last completed move, plus the fraction of the
    /// move in progress after `advanceToTime`.
    double currentSeconds() const;
    /// Fraction in [0, 1) of the move at `movesDone()` that has been cut by `advanceToTime`; 0 otherwise.
    double partialFraction() const;

    const Heightfield& heightfield() const;
    const std::vector<SimDiagnostic>& diagnostics() const;
    /// Diagnostics beyond `maxDiagnostics`, not kept.
    std::size_t droppedDiagnostics() const;
    /// Cell evaluations so far (the CPU work counter).
    double cellWork() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace carvetto::simulate
