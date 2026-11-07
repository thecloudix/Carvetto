#include "carvetto/viewmodel/dxf_exchange.hpp"

#include <algorithm>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/io/dxf.hpp"

namespace carvetto::viewmodel {

namespace {

/// Closes an open undo macro on every exit path.
class MacroScope {
   public:
    MacroScope(document::UndoStack& stack, std::string text) : stack_(stack) {
        stack_.beginMacro(std::move(text));
    }
    ~MacroScope() { stack_.endMacro(); }
    MacroScope(const MacroScope&) = delete;
    MacroScope& operator=(const MacroScope&) = delete;

   private:
    document::UndoStack& stack_;
};

document::Colour colourOf(std::uint32_t rgb) {
    return {static_cast<std::uint8_t>((rgb >> 16) & 0xFF), static_cast<std::uint8_t>((rgb >> 8) & 0xFF),
            static_cast<std::uint8_t>(rgb & 0xFF)};
}

std::uint32_t rgbOf(document::Colour c) {
    return (static_cast<std::uint32_t>(c.r) << 16) | (static_cast<std::uint32_t>(c.g) << 8) | c.b;
}

}  // namespace

const std::vector<DxfUnitChoice>& dxfUnitChoices() {
    static const std::vector<DxfUnitChoice> choices = {
        {"From the file ($INSUNITS)", std::nullopt},
        {"Millimetres", 1.0},
        {"Centimetres", 10.0},
        {"Metres", 1000.0},
        {"Inches", 25.4},
        {"Feet", 304.8},
    };
    return choices;
}

DxfImportOutcome importDxfIntoDocument(document::Document& doc, std::span<const std::uint8_t> bytes,
                                       std::optional<double> forcedMmPerUnit) {
    DxfImportOutcome out;
    try {
        io::DxfImportOptions options;
        options.forcedScale = forcedMmPerUnit;
        auto imported = io::importDxf(bytes, options);
        if (!imported) {
            out.error = "Cannot read the DXF file: " + imported.error().describe();
            return out;
        }
        const io::DxfDrawing& dxf = imported.value();
        for (const io::DxfWarning& w : dxf.warnings) {
            out.warnings.push_back(w.describe());
        }
        out.textSkipped = dxf.texts.size();
        if (out.textSkipped > 0) {
            out.warnings.push_back(std::to_string(out.textSkipped) +
                                   " text item(s) skipped: text import is not supported yet");
        }
        if (dxf.contours.empty()) {
            out.warnings.push_back("the drawing holds no vectors; nothing was imported");
            out.ok = true;
            return out;
        }

        // Match DXF layers to existing layers by name; the rest are created.
        std::vector<document::LayerId> layerIds(dxf.layers.size());
        std::vector<bool> isNew(dxf.layers.size(), false);
        std::size_t newLayers = 0;
        for (std::size_t i = 0; i < dxf.layers.size(); ++i) {
            const auto it = std::find_if(doc.layers().begin(), doc.layers().end(),
                                         [&](const document::Layer& l) { return l.name == dxf.layers[i].name; });
            if (it != doc.layers().end()) {
                layerIds[i] = it->id;
            } else {
                isNew[i] = true;
                ++newLayers;
            }
        }
        if (!doc.idGenerator().canAllocate(newLayers + dxf.contours.size())) {
            out.error = "Cannot import: the document has no free object ids left.";
            return out;
        }

        const std::uint64_t firstNewId = doc.idGenerator().peek();
        {
            MacroScope macro(doc.undoStack(), "Import DXF");
            for (std::size_t i = 0; i < dxf.layers.size(); ++i) {
                if (!isNew[i]) {
                    continue;
                }
                document::Layer layer;
                layer.name = dxf.layers[i].name;
                layer.colour = colourOf(dxf.layers[i].rgb);
                layer.visible = dxf.layers[i].visible;
                auto cmd = std::make_unique<document::AddLayerCommand>(std::move(layer));
                const document::AddLayerCommand* raw = cmd.get();
                if (doc.execute(std::move(cmd))) {
                    layerIds[i] = raw->id();
                    ++out.layersAdded;
                }
            }
            for (const io::DxfContour& c : dxf.contours) {
                const document::LayerId layer =
                    c.layer < layerIds.size() && !layerIds[c.layer].isNull() ? layerIds[c.layer] : doc.defaultLayer();
                if (doc.execute(std::make_unique<document::AddObjectCommand>(
                        document::Object::vector(c.contour, layer, doc.activeSheet())))) {
                    ++out.objectsAdded;
                }
            }
        }
        std::vector<document::ObjectId> added;
        for (auto it = doc.objects().lower_bound(document::ObjectId{firstNewId}); it != doc.objects().end(); ++it) {
            added.push_back(it->first);
        }
        doc.selection().assign(added);
        out.ok = true;
    } catch (const std::exception& e) {
        out.ok = false;
        out.error = std::string("Cannot import the DXF file: ") + e.what();
    }
    return out;
}

DxfExportOutcome exportDocumentDxf(const document::Document& doc, bool selectionOnly) {
    DxfExportOutcome out;
    try {
        std::vector<document::ObjectId> ids;
        if (selectionOnly) {
            ids = doc.withDescendants(doc.selection().ids());
        } else {
            ids = doc.objectsOnSheet(doc.activeSheet());
        }
        std::vector<io::DxfLayer> layers;
        std::vector<io::DxfContour> contours;
        std::map<document::LayerId, std::size_t> layerIndex;
        for (const document::ObjectId id : ids) {
            const document::Object* obj = doc.findObject(id);
            const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
            if (vec == nullptr || obj->sheet != doc.activeSheet()) {
                continue;
            }
            for (const geometry::Contour& contour : vec->contours) {
                if (contour.isEmpty()) {
                    continue;
                }
                auto found = layerIndex.find(obj->layer);
                if (found == layerIndex.end()) {
                    const document::Layer* layer = doc.findLayer(obj->layer);
                    io::DxfLayer dl;
                    dl.name = layer != nullptr ? layer->name : "0";
                    dl.aci = 0;
                    dl.rgb = layer != nullptr ? rgbOf(layer->colour) : 0xFFFFFF;
                    dl.visible = layer == nullptr || layer->visible;
                    found = layerIndex.emplace(obj->layer, layers.size()).first;
                    layers.push_back(std::move(dl));
                }
                contours.push_back({found->second, contour});
            }
        }
        if (contours.empty()) {
            out.error = selectionOnly ? "Nothing to export: the selection holds no vectors."
                                      : "Nothing to export: the active sheet holds no vectors.";
            return out;
        }
        auto exported = io::exportDxf(layers, contours);
        if (!exported) {
            out.error = "Cannot export DXF: " + exported.error().describe();
            return out;
        }
        out.bytes = std::move(exported.value());
        out.contoursWritten = contours.size();
        out.ok = true;
    } catch (const std::exception& e) {
        out.ok = false;
        out.error = std::string("Cannot export DXF: ") + e.what();
    }
    return out;
}

}  // namespace carvetto::viewmodel
