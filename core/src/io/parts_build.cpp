#include "carvetto/io/parts_build.hpp"

#include <cmath>
#include <map>
#include <memory>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/contour.hpp"

namespace carvetto::io {

namespace {

constexpr const char* kBuild = "parts";

struct Instance {
    std::size_t part{0};
    std::size_t copy{0};
};

struct Group {
    std::string material;
    double thicknessMm{0.0};
    std::vector<Instance> instances;
};

/// Millimetres rounded to micrometres, the key two thicknesses must share to be one group.
long long micro(double mm) { return std::llround(mm * 1000.0); }

}  // namespace

BuildResult buildPanels(document::Document& doc, const PartsList& list, const PartsMapping& mapping) {
    using namespace carvetto::document;
    BuildResult out;
    DiagnosticSink sink;
    auto finish = [&](bool ok) {
        out.ok = ok;
        out.diagnostics = std::move(sink.items);
        return out;
    };

    // Existing layers of the mapping's names must be usable.
    auto findLayerByName = [&](const std::string& name) -> const Layer* {
        for (const Layer& l : doc.layers()) {
            if (l.name == name) return &l;
        }
        return nullptr;
    };
    for (const std::string* n : {&mapping.panelLayer, &mapping.holeLayer, &mapping.dadoLayer}) {
        const Layer* l = findLayerByName(*n);
        if (l && (l->locked || !l->visible)) {
            sink.error(kBuild, 0, "layer \"" + *n + "\" exists but is hidden or locked");
            return finish(false);
        }
    }

    // Group the instances by material and thickness, in order of first appearance.
    std::vector<Group> groups;
    std::map<std::pair<std::string, long long>, std::size_t> groupOf;
    for (std::size_t p = 0; p < list.parts.size(); ++p) {
        const Part& part = list.parts[p];
        const auto key = std::make_pair(part.material, micro(part.thicknessMm));
        auto it = groupOf.find(key);
        if (it == groupOf.end()) {
            it = groupOf.emplace(key, groups.size()).first;
            groups.push_back(Group{part.material, part.thicknessMm, {}});
        }
        for (std::size_t c = 0; c < part.quantity; ++c) groups[it->second].instances.push_back({p, c});
    }
    if (groups.empty()) {
        sink.error(kBuild, 0, "there are no panels to build");
        return finish(false);
    }

    // Nest every group first, so that nothing is touched when nothing fits.
    NestOptions nopt;
    nopt.sheetLengthMm = mapping.sheetLengthMm;
    nopt.sheetWidthMm = mapping.sheetWidthMm;
    nopt.gapMm = mapping.gapMm;
    nopt.marginMm = mapping.marginMm;
    nopt.allowRotate = mapping.rotate;
    std::vector<NestResult> nests;
    std::size_t placedTotal = 0;
    std::size_t sheetsTotal = 0;
    for (const Group& g : groups) {
        std::vector<NestItem> items;
        items.reserve(g.instances.size());
        for (const Instance& in : g.instances) {
            items.push_back({list.parts[in.part].lengthMm, list.parts[in.part].widthMm});
        }
        nests.push_back(nestRectangles(items, nopt));
        placedTotal += nests.back().placements.size();
        sheetsTotal += nests.back().sheetCount;
        for (std::size_t u : nests.back().unplaced) {
            const Part& part = list.parts[g.instances[u].part];
            sink.error(kBuild, part.line,
                       "panel \"" + part.label + "\" (" + std::to_string(part.lengthMm) + " x " + std::to_string(part.widthMm) +
                           " mm) does not fit a sheet of " + std::to_string(mapping.sheetLengthMm) + " x " +
                           std::to_string(mapping.sheetWidthMm) + " mm with the margin and sheet limit");
            ++out.unplaced;
        }
    }
    out.placed = placedTotal;
    if (placedTotal == 0) return finish(false);
    if (out.unplaced > 0) {
        // Whole import refused: a cabinet with a missing panel is worse than no cabinet.
        return finish(false);
    }

    UndoStack& undo = doc.undoStack();
    undo.beginMacro("Import parts");
    bool failed = false;
    auto run = [&](std::unique_ptr<Command> cmd) {
        if (!failed && !doc.execute(std::move(cmd))) failed = true;
    };

    // Layers.
    auto ensureLayer = [&](const std::string& name, Colour colour) -> LayerId {
        if (const Layer* l = findLayerByName(name)) return l->id;
        Layer layer;
        layer.name = name;
        layer.colour = colour;
        auto cmd = std::make_unique<AddLayerCommand>(layer);
        const AddLayerCommand* raw = cmd.get();
        run(std::move(cmd));
        return failed ? LayerId{} : raw->id();
    };
    out.panelLayer = ensureLayer(mapping.panelLayer, Colour{0, 0, 255});
    out.holeLayer = ensureLayer(mapping.holeLayer, Colour{255, 0, 0});
    out.dadoLayer = ensureLayer(mapping.dadoLayer, Colour{0, 160, 0});

    bool reuseActive = doc.objectsOnSheet(doc.activeSheet()).empty();
    bool first = true;
    for (std::size_t gi = 0; gi < groups.size() && !failed; ++gi) {
        const Group& g = groups[gi];
        const NestResult& nest = nests[gi];
        std::vector<SheetId> sheetIds;
        for (std::size_t s = 0; s < nest.sheetCount && !failed; ++s) {
            Sheet sheet = Sheet::fromJob(doc.job(), "Sheet");
            std::string name = g.material.empty() ? std::string("Panels") : g.material;
            name += " " + std::to_string(static_cast<long long>(std::llround(g.thicknessMm))) + "mm #" + std::to_string(s + 1);
            if (name.size() > 120) name.resize(120);
            SheetId id;
            if (reuseActive) {
                reuseActive = false;
                sheet = *doc.findSheet(doc.activeSheet());
                sheet.name = name;
                sheet.sizeMm = JobSize{mapping.sheetLengthMm, mapping.sheetWidthMm, g.thicknessMm};
                sheet.datum = Datum::BottomLeft;
                sheet.datumOffsetMm = {};
                id = sheet.id;
                run(std::make_unique<SetSheetPropertiesCommand>(sheet));
            } else {
                sheet.name = name;
                sheet.sizeMm = JobSize{mapping.sheetLengthMm, mapping.sheetWidthMm, g.thicknessMm};
                sheet.datum = Datum::BottomLeft;
                sheet.datumOffsetMm = {};
                auto cmd = std::make_unique<AddSheetCommand>(sheet, first);
                const AddSheetCommand* raw = cmd.get();
                run(std::move(cmd));
                id = failed ? SheetId{} : raw->id();
            }
            first = false;
            sheetIds.push_back(id);
            BuiltSheet bs;
            bs.sheet = id;
            bs.material = g.material;
            bs.thicknessMm = g.thicknessMm;
            out.sheets.push_back(std::move(bs));
        }
        const std::size_t base = out.sheets.size() - sheetIds.size();
        for (const NestPlacement& pl : nest.placements) {
            if (failed) break;
            const Instance& inst = g.instances[pl.item];
            const Part& part = list.parts[inst.part];
            BuiltSheet& bs = out.sheets[base + pl.sheet];
            const SheetId sheetId = sheetIds[pl.sheet];
            const double w = pl.rotated ? part.widthMm : part.lengthMm;
            const double h = pl.rotated ? part.lengthMm : part.widthMm;
            // Part frame (x along length, y along width) -> sheet; a quarter turn maps (x, y) to (width - y, x).
            auto place = [&](double x, double y) {
                return pl.rotated ? geometry::Vec2{pl.xMm + part.widthMm - y, pl.yMm + x}
                                  : geometry::Vec2{pl.xMm + x, pl.yMm + y};
            };
            std::string objName = part.label;
            if (part.quantity > 1) objName += " (" + std::to_string(inst.copy + 1) + "/" + std::to_string(part.quantity) + ")";

            {
                Object obj = Object::vector(geometry::Contour::rectangle(geometry::Box2{{pl.xMm, pl.yMm}, {pl.xMm + w, pl.yMm + h}}),
                                            out.panelLayer, sheetId);
                obj.name = objName;
                auto cmd = std::make_unique<AddObjectCommand>(std::move(obj));
                const AddObjectCommand* raw = cmd.get();
                run(std::move(cmd));
                if (failed) break;
                bs.panels.push_back(raw->id());
            }
            for (const PartHole& hole : part.holes) {
                Object obj = Object::vector(geometry::Contour::circle(place(hole.xMm, hole.yMm), hole.diameterMm * 0.5),
                                            out.holeLayer, sheetId);
                obj.name = objName + " hole";
                auto cmd = std::make_unique<AddObjectCommand>(std::move(obj));
                const AddObjectCommand* raw = cmd.get();
                run(std::move(cmd));
                if (failed) break;
                bs.holes.push_back({raw->id(), hole.depthMm});
            }
            for (const PartDado& d : part.dados) {
                if (failed) break;
                const geometry::Vec2 a = d.alongLength ? place(0.0, d.posMm) : place(d.posMm, 0.0);
                const geometry::Vec2 b = d.alongLength ? place(part.lengthMm, d.posMm + d.widthMm)
                                                       : place(d.posMm + d.widthMm, part.widthMm);
                Object obj = Object::vector(geometry::Contour::rectangle(geometry::Box2{a, b}), out.dadoLayer, sheetId);
                obj.name = objName + " dado";
                auto cmd = std::make_unique<AddObjectCommand>(std::move(obj));
                const AddObjectCommand* raw = cmd.get();
                run(std::move(cmd));
                if (failed) break;
                bs.dados.push_back({raw->id(), d.depthMm});
            }
        }
    }
    undo.endMacro();
    if (failed) {
        sink.error(kBuild, 0, "the document could not take all the objects (identifier space exhausted); the partial import is one undo step");
        return finish(false);
    }
    return finish(true);
}

}  // namespace carvetto::io
