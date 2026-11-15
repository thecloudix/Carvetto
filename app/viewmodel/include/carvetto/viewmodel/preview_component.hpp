#pragma once

#include <atomic>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/simulate/preview_component.hpp"
#include "carvetto/simulate/stock.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

// Create Component From Toolpath Preview (T-0311, ADR-0091; feature-inventory "Model" menu): the stock surface left
// by simulating every enabled toolpath becomes a relief component of the active sheet. Qt-free; the simulation runs
// to the end synchronously (a cancel flag is polled between chunks of moves), so the app calls it from a worker or
// accepts the wait for modest jobs. The document is touched only at the end, in ONE undo step.

namespace carvetto::viewmodel {

struct PreviewComponentRequest {
    std::string name{"Toolpath Preview"};
    /// All = uncut stock keeps the stock top height; CutOnly = only cells the tool lowered (the rest transparent).
    simulate::PreviewCells cells{simulate::PreviewCells::All};
    relief::CombineMode mode{relief::CombineMode::Add};
    simulate::Quality quality{simulate::Quality::Medium};
    /// Most cells of the simulated stock (a finer quality on a big stock gets a coarser cell size).
    std::size_t maxCells{4'000'000};
    /// Model height = simulator Z - zOffset. Empty = the stock bottom, so the component stands on height 0 and
    /// its top is the stock thickness (reading recorded in ADR-0091).
    std::optional<double> zOffset;
};

struct PreviewComponentOutcome {
    bool ok{false};
    /// The reason when `!ok`, else a short confirmation.
    std::string message;
    /// The component added (null when `!ok`).
    document::ComponentId component;
    /// Notes of the simulation (coarsened cells, approximated tools, stale toolpaths).
    std::vector<std::string> notes;
    bool cancelled{false};
};

/// Simulates the session's enabled, computed toolpaths and adds the resulting surface as a component on the top
/// level of the active sheet (a level is added when the sheet has none) and selects it. Nothing changes when it fails.
PreviewComponentOutcome createComponentFromToolpathPreview(document::Document& doc, ToolpathSession& session,
                                                           const PreviewComponentRequest& request = {},
                                                           const std::atomic<bool>* cancel = nullptr);

}  // namespace carvetto::viewmodel
