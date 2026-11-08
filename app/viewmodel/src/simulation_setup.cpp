#include "carvetto/viewmodel/simulation_setup.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "carvetto/document/document.hpp"

namespace carvetto::viewmodel {

namespace {

struct StockResult {
    bool ok{false};
    std::string problem;
    simulate::Stock stock;
};

/// The stock of the active sheet at the requested cell size (not yet fitted to the cell cap).
StockResult stockOf(const document::Document& doc, double cellSize) {
    StockResult r;
    const document::Job& job = doc.job();
    if (job.zZero == document::ZZero::CylinderAxis || job.type == document::JobType::Rotary) {
        r.problem = "Rotary jobs cannot be simulated yet.";
        return r;
    }
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    if (sheet == nullptr) {
        r.problem = "There is no active sheet.";
        return r;
    }
    const geometry::Box2 box = sheet->materialBoundsMm();
    if (box.isEmpty() || !(box.width() > 0.0) || !(box.height() > 0.0) || !(job.sizeMm.z > 0.0)) {
        r.problem = "The material has no size.";
        return r;
    }
    r.stock.width = box.width();
    r.stock.height = box.height();
    r.stock.thickness = job.sizeMm.z;
    r.stock.origin = box.min;
    r.stock.topZ = job.zZero == document::ZZero::MachineBed ? job.sizeMm.z : 0.0;
    r.stock.cellSize = cellSize;
    r.stock.trackPassIds = true;
    r.ok = true;
    return r;
}

std::string pluralToolpaths(std::size_t n) {
    return std::to_string(n) + (n == 1 ? " toolpath" : " toolpaths");
}

}  // namespace

SimulationSetup makeSimulationSetup(ToolpathSession& session, const SimulationRequest& request) {
    SimulationSetup setup;
    setup.quality = request.quality;
    setup.requestedCellSize = simulate::cellSizeFor(request.quality);
    document::Document* doc = session.document();
    if (doc == nullptr) {
        setup.problem = "There is no document.";
        return setup;
    }
    StockResult sr = stockOf(*doc, setup.requestedCellSize);
    if (!sr.ok) {
        setup.problem = sr.problem;
        return setup;
    }
    const std::size_t cap = std::min(request.maxCells, simulate::kDefaultMaxCells);
    const double fitted = simulate::fitCellSize(sr.stock, setup.requestedCellSize, cap);
    if (!(fitted > 0.0)) {
        setup.problem = "The material size is not usable for a simulation.";
        return setup;
    }
    sr.stock.cellSize = fitted;
    sr.stock.maxCells = cap;
    setup.coarsened = fitted > setup.requestedCellSize * (1.0 + 1e-9);
    const simulate::StockCheck check = simulate::checkStock(sr.stock);
    if (!check.ok()) {
        setup.problem = check.message;
        return setup;
    }
    setup.stock = sr.stock;
    if (setup.coarsened) {
        char buf[160];
        std::snprintf(buf, sizeof buf, "The material is large: cells are %.3g mm instead of %.3g mm to stay within %zu million cells.",
                      fitted, setup.requestedCellSize, cap / 1'000'000);
        setup.notes.emplace_back(buf);
    }

    // Rates.
    setup.options.acceleration = session.machineSetup().accelerationMmPerS2;
    double rapid = request.rapidRateMmPerMin > 0.0 ? request.rapidRateMmPerMin : session.machineSetup().rapidRateMmPerMin;
    if (!(rapid > 0.0) || !std::isfinite(rapid)) {
        rapid = kDefaultPlaybackRapidRate;
        setup.rapidRateAssumed = true;
        setup.notes.push_back("The machine's rapid rate is not set: playback time assumes 5000 mm/min for rapids.");
    }
    setup.options.rapidRate = rapid;
    setup.options.maxMoves = request.maxMoves;

    // Toolpaths.
    const std::vector<ToolpathListItem> items = session.list();
    for (const document::ToolpathRecord& rec : doc->toolpaths()) {
        const ToolpathListItem* item = nullptr;
        for (const ToolpathListItem& it : items) {
            if (it.id == rec.id) {
                item = &it;
                break;
            }
        }
        if (item == nullptr || !item->enabled || !rec.hasMoves() || rec.moves->empty()) {
            continue;
        }
        SimulationToolpath tp;
        tp.id = rec.id;
        tp.name = rec.name;
        tp.moves = rec.moves;
        tp.tool = rec.tool;
        if (const tooling::Tool* dbTool = session.toolDatabase().findTool(rec.toolSource.toolId)) {
            if (auto shape = simulate::ToolShape::fromTool(*dbTool)) {
                tp.shape = *shape;
            }
        }
        if (!tp.shape.valid()) {
            tp.shapeApproximated = true;
            auto flat = simulate::ToolShape::flatEndMill(rec.tool.diameter);
            if (!flat) {
                setup.problem = "Toolpath \"" + rec.name + "\" has no usable tool diameter.";
                return setup;
            }
            tp.shape = *flat;
            setup.notes.push_back("Toolpath \"" + rec.name + "\": its tool is not in the tool database or not simulable; shown with a flat cutter of " +
                                  std::to_string(rec.tool.diameter) + " mm.");
        }
        setup.totalMoves += rec.moves->size();
        if (item->status == ToolpathStatus::NeedsRecalculation) {
            ++setup.staleAtStart;
        }
        setup.toolpaths.push_back(std::move(tp));
    }
    if (setup.toolpaths.empty()) {
        setup.problem = "There are no computed, enabled toolpaths to simulate.";
        return setup;
    }
    if (setup.totalMoves > request.maxMoves) {
        setup.problem = "Too many moves to simulate (" + std::to_string(setup.totalMoves) + ", at most " +
                        std::to_string(request.maxMoves) + ").";
        return setup;
    }
    if (setup.staleAtStart > 0) {
        setup.notes.push_back(pluralToolpaths(setup.staleAtStart) + (setup.staleAtStart == 1 ? " needs" : " need") +
                              " recalculation: the simulation uses the old moves.");
    }
    setup.ok = true;
    return setup;
}

SimulationValidity checkSimulationCurrent(const SimulationSetup& setup, ToolpathSession& session) {
    SimulationValidity v;
    document::Document* doc = session.document();
    if (doc == nullptr) {
        v.reason = "There is no document.";
        return v;
    }
    const StockResult sr = stockOf(*doc, setup.stock.cellSize);
    if (!sr.ok) {
        v.reason = sr.problem;
        return v;
    }
    simulate::Stock mine = sr.stock;
    mine.maxCells = setup.stock.maxCells;
    if (!(mine == setup.stock)) {
        v.reason = "The material changed since the simulation.";
        return v;
    }
    const std::vector<ToolpathListItem> items = session.list();
    std::size_t k = 0;
    std::size_t stale = 0;
    for (const document::ToolpathRecord& rec : doc->toolpaths()) {
        const ToolpathListItem* item = nullptr;
        for (const ToolpathListItem& it : items) {
            if (it.id == rec.id) {
                item = &it;
                break;
            }
        }
        if (item == nullptr || !item->enabled || !rec.hasMoves() || rec.moves->empty()) {
            continue;
        }
        if (k >= setup.toolpaths.size() || setup.toolpaths[k].id != rec.id || setup.toolpaths[k].moves != rec.moves) {
            v.reason = "The toolpaths changed since the simulation (recalculated, edited, added, removed or disabled).";
            return v;
        }
        stale += item->status == ToolpathStatus::NeedsRecalculation ? 1 : 0;
        ++k;
    }
    if (k != setup.toolpaths.size()) {
        v.reason = "The toolpaths changed since the simulation (recalculated, edited, added, removed or disabled).";
        return v;
    }
    if (stale > 0) {
        v.reason = pluralToolpaths(stale) + (stale == 1 ? " needs" : " need") +
                   " recalculation: the simulation shows their old moves.";
        return v;
    }
    v.current = true;
    return v;
}

}  // namespace carvetto::viewmodel
