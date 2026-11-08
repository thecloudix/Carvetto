#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/simulate/simulator.hpp"
#include "carvetto/simulate/stock.hpp"
#include "carvetto/simulate/tool_shape.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

// Everything a simulation run needs, copied out of the session (T-0223, ADR-0032 amendment).
// A `SimulationSetup` holds shared immutable move lists, tool shapes and the stock by value and
// no pointer into the document, so a worker may run it while the document is edited or replaced;
// `checkSimulationCurrent` later says whether the document still matches what was simulated.

namespace carvetto::viewmodel {

/// Largest heightfield the app builds (cells). A finer quality on a big stock gets a coarser cell
/// size (reported), never more memory: 16 M cells are 144 MB with pass ids.
inline constexpr std::size_t kSimulationMaxCells = 16'000'000;
/// Rapid rate used to pace the playback when the machine setup has none (mm/min).
inline constexpr double kDefaultPlaybackRapidRate = 5000.0;

struct SimulationRequest {
    simulate::Quality quality{simulate::Quality::Medium};
    std::size_t maxCells{kSimulationMaxCells};
    /// Most moves accepted (the simulator's own limit applies too).
    std::size_t maxMoves{simulate::SimOptions{}.maxMoves};
    /// Rapid rate for timing, mm/min; 0 = use the session's machine setup, else the default.
    double rapidRateMmPerMin{0.0};
};

struct SimulationToolpath {
    document::ToolpathId id;
    std::string name;
    document::SharedMoves moves;
    toolpath::ToolSnapshot tool;
    simulate::ToolShape shape;
    /// The database tool was missing or not simulable: shown with a flat cutter of the snapshot's diameter.
    bool shapeApproximated{false};
};

struct SimulationSetup {
    bool ok{false};
    /// Why `ok` is false.
    std::string problem;
    /// Things the user should know: coarsened cells, approximated tools, stale toolpaths.
    std::vector<std::string> notes;
    simulate::Stock stock;
    simulate::Quality quality{simulate::Quality::Medium};
    double requestedCellSize{0.0};
    bool coarsened{false};
    simulate::SimOptions options;
    bool rapidRateAssumed{false};
    /// Enabled, computed toolpaths in job order; the id given to the simulator is the index + 1.
    std::vector<SimulationToolpath> toolpaths;
    std::size_t totalMoves{0};
    std::size_t staleAtStart{0};
};

/// Copies the enabled, computed toolpaths, the tools (shapes from the session's tool database) and
/// the stock (the active sheet's material, the job's thickness and Z zero) out of `session`.
SimulationSetup makeSimulationSetup(ToolpathSession& session, const SimulationRequest& request = {});

struct SimulationValidity {
    bool current{false};
    /// Why not (empty when current).
    std::string reason;
};

/// Whether the session's document still matches `setup`: the same toolpaths with the same move
/// lists (identity), the same stock, none needing recalculation. A replaced document never matches.
SimulationValidity checkSimulationCurrent(const SimulationSetup& setup, ToolpathSession& session);

}  // namespace carvetto::viewmodel
