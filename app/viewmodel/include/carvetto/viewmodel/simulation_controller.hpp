#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/viewmodel/relief_render.hpp"
#include "carvetto/viewmodel/simulation_setup.hpp"

// Runs the material-removal simulation off the UI thread (T-0223, ADR-0032 amendment, ADR-0014).
// Qt-free. The controller owns a copy of a `SimulationSetup` (shared move lists, tool shapes, the
// stock by value) and one worker thread that owns the `simulate::Simulator`; nothing refers to the
// Document, so the document may be edited, recalculated or replaced while it runs. The UI asks for
// a move count (`seek`), the worker cuts (chunked, so cancel and a newer seek take effect within
// one chunk) and publishes an immutable `SimulationFrame` (shaded relief raster + diagnostics) when
// the target is reached. Forward seeks continue from where the simulator is; backward seeks reset
// and replay (the latest request wins, so dragging a scrubber costs one replay, not one per step).
//
// All public functions are for the owner's thread; the destructor cancels and joins.

namespace carvetto::viewmodel {

enum class SimState {
    Idle,       ///< Nothing started.
    Preparing,  ///< The worker is allocating the heightfield and queueing the moves.
    Working,    ///< Cutting towards the requested move.
    Ready,      ///< At the requested move; a frame is available.
    Failed,     ///< The simulator refused or failed; `error()` says why.
};

struct SimProgress {
    SimState state{SimState::Idle};
    /// Moves cut so far (the simulator's position), the move asked for, and the total.
    std::size_t movesDone{0};
    /// Position the current run started from (0 after a rewind): `fraction` is measured from it.
    std::size_t runBase{0};
    std::size_t target{0};
    std::size_t moveCount{0};
    /// Fraction of the way to the target in [0, 1] (1 when ready).
    double fraction() const;
    /// The last run was stopped by `cancel()` before it reached its target.
    bool cancelled{false};
};

/// Fixed facts of a prepared simulation.
struct SimulationMeta {
    std::size_t moveCount{0};
    std::size_t nx{0};
    std::size_t ny{0};
    double cellSize{0.0};
    double totalSeconds{0.0};
    std::size_t heightfieldBytes{0};
    /// Cumulative machine seconds at the end of each move.
    std::shared_ptr<const std::vector<double>> moveEndTimes;
};

/// The picture at one move count.
struct SimulationFrame {
    std::uint64_t serial{0};
    std::size_t move{0};
    double seconds{0.0};
    std::size_t cutCells{0};
    std::shared_ptr<const Raster> relief;
    std::vector<simulate::SimDiagnostic> diagnostics;
    std::size_t droppedDiagnostics{0};
    /// Wall time the worker spent cutting and shading to reach it.
    double workSeconds{0.0};
    /// Maps pass ids of `relief` in Toolpath colour mode: id i names `toolpathIds[i - 1]`.
    std::vector<std::uint64_t> toolpathIds;
};

struct SimulationShared;

class SimulationController {
   public:
    SimulationController() = default;
    ~SimulationController();
    SimulationController(const SimulationController&) = delete;
    SimulationController& operator=(const SimulationController&) = delete;

    /// Starts a simulation of `setup` (must be `ok`), replacing and joining a previous one. The
    /// first frame (move 0, the uncut stock) is produced as soon as the worker has prepared.
    /// False (nothing started) when the setup is not ok.
    bool start(SimulationSetup setup, const ReliefOptions& options = {});
    /// Stops the worker and discards frames and state (the owner went away or the document was replaced).
    void stop();
    bool active() const { return worker_.joinable(); }
    const SimulationSetup& setup() const { return setup_; }

    /// Asks for the state after `move` moves (clamped). Latest request wins.
    void seek(std::size_t move);
    /// Re-shades the current state with other options (no re-simulation).
    void setReliefOptions(const ReliefOptions& options);
    /// Stops the current run at the move reached so far; a frame at that move follows.
    void cancel();

    SimProgress progress() const;
    std::string error() const;
    std::shared_ptr<const SimulationMeta> meta() const;
    /// The newest frame (null before the first). Immutable; safe to keep.
    std::shared_ptr<const SimulationFrame> frame() const;

    /// Test hook: the worker waits this long (ms) after every chunk and limits chunks to 16 moves (to
    /// make runs long enough to cancel or replace under test).
    void setChunkDelayForTest(int milliseconds);

   private:
    SimulationSetup setup_;
    std::shared_ptr<SimulationShared> shared_;
    std::thread worker_;
};

}  // namespace carvetto::viewmodel
