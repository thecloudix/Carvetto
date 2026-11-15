#include "carvetto/viewmodel/preview_component.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "carvetto/document/relief_commands.hpp"
#include "carvetto/simulate/simulator.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"
#include "carvetto/viewmodel/simulation_setup.hpp"

namespace carvetto::viewmodel {

namespace {

PreviewComponentOutcome refused(std::string why) {
    PreviewComponentOutcome out;
    out.message = std::move(why);
    return out;
}

constexpr std::size_t kChunk = 4096;

}  // namespace

PreviewComponentOutcome createComponentFromToolpathPreview(document::Document& doc, ToolpathSession& session,
                                                           const PreviewComponentRequest& request,
                                                           const std::atomic<bool>* cancel) {
    const auto cancelled = [&] { return cancel != nullptr && cancel->load(); };
    SimulationRequest sim;
    sim.quality = request.quality;
    sim.maxCells = request.maxCells;
    SimulationSetup setup = makeSimulationSetup(session, sim);
    if (!setup.ok) {
        return refused(setup.problem.empty() ? std::string("there is nothing to simulate") : setup.problem);
    }
    if (request.zOffset.has_value() && !std::isfinite(*request.zOffset)) {
        return refused("the Z offset is not a finite number");
    }
    simulate::Simulator simulator(setup.stock, setup.options);
    if (!simulator.ok()) {
        return refused(simulator.error().message);
    }
    for (std::size_t i = 0; i < setup.toolpaths.size(); ++i) {
        const SimulationToolpath& tp = setup.toolpaths[i];
        toolpath::Toolpath path;
        path.name = tp.name;
        path.tool = tp.tool;
        path.moves = *tp.moves;
        if (!simulator.addToolpath(path, tp.shape, static_cast<std::uint32_t>(i + 1))) {
            return refused(simulator.error().message.empty()
                               ? "the simulator refused toolpath \"" + tp.name + "\""
                               : simulator.error().message);
        }
    }
    while (simulator.movesDone() < simulator.moveCount()) {
        if (cancelled()) {
            PreviewComponentOutcome out = refused("the simulation was cancelled");
            out.cancelled = true;
            return out;
        }
        if (!simulator.advanceToMove(std::min(simulator.moveCount(), simulator.movesDone() + kChunk))) {
            return refused(simulator.error().message.empty() ? std::string("the simulation failed")
                                                             : simulator.error().message);
        }
    }

    simulate::PreviewComponentOptions options;
    options.name = request.name;
    options.cells = request.cells;
    options.mode = request.mode;
    options.zOffset = request.zOffset.value_or(setup.stock.bottomZ());
    relief::Component component;
    if (const relief::Status s = simulate::componentFromPreview(simulator.heightfield(), options, component);
        s != relief::Status::Ok) {
        return refused(std::string("the preview cannot become a component: ") + relief::statusMessage(s));
    }

    const document::SheetId sheet = doc.activeSheet();
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Create component from toolpath preview");
    const std::vector<document::LevelId> levels = doc.reliefLevelsOnSheet(sheet);
    document::LevelId level;
    if (levels.empty()) {
        const EditResult added = addReliefLevel(doc, sheet, {}, &level);
        if (!added.ok) {
            doc.undoStack().endMacro();
            doc.undoStack().breakCoalescing();
            return refused(added.message);
        }
    } else {
        level = levels.back();
    }
    auto cmd = std::make_unique<document::AddReliefComponentCommand>(level, std::move(component));
    const document::AddReliefComponentCommand* raw = cmd.get();
    const bool done = doc.execute(std::move(cmd));
    const document::ComponentId id = done ? raw->createdId() : document::ComponentId{};
    doc.undoStack().endMacro();
    doc.undoStack().breakCoalescing();
    if (!done || !id) {
        return refused("the component could not be added (a size limit was reached)");
    }
    selectReliefComponents(doc, {id});
    PreviewComponentOutcome out;
    out.ok = true;
    out.component = id;
    out.notes = std::move(setup.notes);
    out.message = "Added component \"" + request.name + "\" from the preview of " +
                  std::to_string(setup.toolpaths.size()) + " toolpath(s)";
    return out;
}

}  // namespace carvetto::viewmodel
