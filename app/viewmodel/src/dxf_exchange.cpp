#include "carvetto/viewmodel/dxf_exchange.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/dimension.hpp"
#include "carvetto/document/text_commands.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/transform2.hpp"
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

/// A Text object made from a DXF text item, ready to add.
struct ImportedText {
    std::size_t layer{0};
    document::TextParams params;
    std::vector<geometry::Contour> contours;
};

/// Cap height of the font in em (height of "H"), 0.7 when the font has no usable H.
double capHeightEm(const text::Font& font) {
    document::TextParams p;
    p.text = "H";
    p.sizeMm = 1.0;
    const document::TextBuild b = document::buildTextContours(font, p);
    if (b.ok() && !b.missingGlyphs && !b.contours.empty()) {
        geometry::Box2 box;
        for (const geometry::Contour& c : b.contours) {
            box.expand(c.bounds());
        }
        if (box.max.y > 0.05) {
            return box.max.y;
        }
    }
    return 0.7;
}

/// Maps one DXF text item; nullopt when it cannot be laid out (reason in `why`). `missing` is set
/// when the font lacks glyphs for the text (it shows as .notdef boxes).
///
/// TEXT justification (DXF spec): with a non-default horizontal or vertical justification the
/// anchor is the second alignment point (group 11) and group 10 is only the computed baseline start;
/// Aligned and Fit run the baseline from group 10 to group 11 (Aligned scales the height with the
/// width, Fit only stretches the width), fitted to the laid-out text's extent from its origin.
/// The "upside down" generation flag is not parsed by the importer and is ignored.
std::optional<ImportedText> mapDxfText(const io::DxfText& t, const text::Font& font, const std::string& key,
                                       double capEm, std::string* why, bool* missing) {
    if (!(t.height > 0.0) || !std::isfinite(t.height) || t.text.empty()) {
        *why = "empty text or no character height";
        return std::nullopt;
    }
    document::TextParams p;
    p.text = t.text;
    p.fontKey = key;
    p.sizeMm = t.height / capEm;
    const bool isText = t.kind != io::DxfTextKind::MText;
    const bool stretched =
        isText && t.alignmentPoint && (t.hAlign == io::DxfHAlign::Aligned || t.hAlign == io::DxfHAlign::Fit);
    switch (t.hAlign) {
        case io::DxfHAlign::Center:
        case io::DxfHAlign::Middle:
            p.align = document::TextAlign::Centre;
            break;
        case io::DxfHAlign::Right:
            p.align = document::TextAlign::Right;
            break;
        case io::DxfHAlign::Aligned:
        case io::DxfHAlign::Fit:
        case io::DxfHAlign::Left:
            p.align = document::TextAlign::Left;
            break;
    }
    geometry::Vec2 anchor = t.insertion;
    double rotation = t.rotation;
    double dx = 0.0;
    double dy = 0.0;
    double stretchX = 1.0;
    if (!isText && t.referenceWidth > 0.0) {
        // Text in a box: the box top is at the insertion height, the layout puts the first baseline
        // below it; the attachment column decides where the box sits horizontally.
        p.mode = document::TextMode::Box;
        p.boxWidthMm = t.referenceWidth;
        dx = t.hAlign == io::DxfHAlign::Right    ? -t.referenceWidth
             : t.hAlign == io::DxfHAlign::Center ? -t.referenceWidth * 0.5
                                                 : 0.0;
    } else {
        if (isText && t.alignmentPoint && !stretched) {
            anchor = *t.alignmentPoint;
        }
        if (t.vAlign == io::DxfVAlign::Top) {
            dy = -t.height;
        } else if (t.vAlign == io::DxfVAlign::Middle || t.hAlign == io::DxfHAlign::Middle) {
            dy = -t.height * 0.5;
        } else if (t.vAlign == io::DxfVAlign::Bottom) {
            const double desc = static_cast<double>(font.metrics().descender) * p.sizeMm /
                                static_cast<double>(font.metrics().unitsPerEm);
            dy = -desc;  // lift the baseline so the descender touches the anchor
        }
        if (stretched) {
            const double ex = t.alignmentPoint->x - t.insertion.x;
            const double ey = t.alignmentPoint->y - t.insertion.y;
            const double len = std::hypot(ex, ey);
            if (len > 0.0) {
                rotation = std::atan2(ey, ex);
                const document::TextBuild pb = document::buildTextContours(font, p);
                geometry::Box2 box;
                for (const geometry::Contour& c : pb.contours) {
                    box.expand(c.bounds());
                }
                if (pb.ok() && !box.isEmpty() && box.max.x > 0.0) {
                    stretchX = len / (box.max.x * t.widthFactor);
                    if (t.hAlign == io::DxfHAlign::Aligned) {
                        p.sizeMm *= stretchX;
                        stretchX = 1.0;
                    }
                }
            }
        }
    }
    const geometry::Transform2 scale =
        geometry::Transform2::scaling((t.mirrored ? -t.widthFactor : t.widthFactor) * stretchX, 1.0);
    p.placement = geometry::Transform2::translation(anchor) * geometry::Transform2::rotation(rotation) *
                  geometry::Transform2::translation({dx, dy}) * scale;
    document::TextBuild b = document::buildTextContours(font, p);
    if (!b.ok()) {
        *why = b.message.empty() ? "the text could not be laid out" : b.message;
        return std::nullopt;
    }
    if (b.contours.empty()) {
        *why = "the font has no outlines for this text";
        return std::nullopt;
    }
    *missing = b.missingGlyphs;
    ImportedText out;
    out.params = std::move(p);
    out.contours = std::move(b.contours);
    return out;
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
                                       std::optional<double> forcedMmPerUnit,
                                       std::span<const std::uint8_t> textFontBytes) {
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
        // Text: laid out with the caller's font before anything is touched.
        std::vector<ImportedText> texts;
        document::FontAsset fontAsset;
        if (!dxf.texts.empty() && !textFontBytes.empty()) {
            std::vector<std::uint8_t> fontCopy(textFontBytes.begin(), textFontBytes.end());
            fontAsset.key = document::fontKeyFor(fontCopy);
            fontAsset.bytes = std::move(fontCopy);
            text::FontLoad loaded = text::Font::load(fontAsset.bytes);
            const std::string embedProblem = doc.fontProblem(fontAsset);
            if (loaded.ok() && !embedProblem.empty()) {
                out.textSkipped = dxf.texts.size();
                out.warnings.push_back(std::to_string(out.textSkipped) +
                                       " text item(s) skipped: cannot embed the font: " + embedProblem);
            } else if (loaded.ok()) {
                const double capEm = capHeightEm(loaded.font);
                std::string firstWhy;
                std::size_t notdef = 0;
                for (const io::DxfText& t : dxf.texts) {
                    std::string why;
                    bool missing = false;
                    if (auto mapped = mapDxfText(t, loaded.font, fontAsset.key, capEm, &why, &missing)) {
                        notdef += missing ? 1U : 0U;
                        mapped->layer = t.layer;
                        texts.push_back(std::move(*mapped));
                    } else if (firstWhy.empty()) {
                        firstWhy = why;
                    }
                }
                if (notdef > 0) {
                    out.warnings.push_back(std::to_string(notdef) +
                                           " text item(s) use characters the font has no glyph for (drawn as "
                                           "boxes); choose a font with full coverage in the Text tool");
                }
                out.textSkipped = dxf.texts.size() - texts.size();
                if (out.textSkipped > 0) {
                    out.warnings.push_back(std::to_string(out.textSkipped) + " text item(s) skipped: " + firstWhy);
                }
            } else {
                out.textSkipped = dxf.texts.size();
                out.warnings.push_back(std::to_string(out.textSkipped) +
                                       " text item(s) skipped: the chosen font cannot be used");
            }
        } else if (!dxf.texts.empty()) {
            out.textSkipped = dxf.texts.size();
            out.warnings.push_back(std::to_string(out.textSkipped) +
                                   " text item(s) skipped: choose a font in the Text tool to import text");
        }
        if (dxf.contours.empty() && texts.empty()) {
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
        if (!doc.idGenerator().canAllocate(newLayers + dxf.contours.size() + texts.size())) {
            out.error = "Cannot import: the document has no free object ids left.";
            return out;
        }

        const std::uint64_t firstNewId = doc.idGenerator().peek();
        {
            MacroScope macro(doc.undoStack(), "Import DXF");
            if (!texts.empty()) {
                doc.execute(std::make_unique<document::AddFontCommand>(std::move(fontAsset)));
            }
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
            for (ImportedText& t : texts) {
                const document::LayerId layer =
                    t.layer < layerIds.size() && !layerIds[t.layer].isNull() ? layerIds[t.layer] : doc.defaultLayer();
                document::Object obj = document::Object::vector(std::move(t.contours), layer, doc.activeSheet(),
                                                                std::move(t.params));
                obj.name = "Text";
                if (doc.execute(std::make_unique<document::AddObjectCommand>(std::move(obj)))) {
                    ++out.textAdded;
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
            if (obj == nullptr || obj->sheet != doc.activeSheet()) {
                continue;
            }
            // Dimensions are exported exploded: their lines and arrowheads as plain geometry on
            // the dimension's layer (the text is not exported).
            std::vector<geometry::Contour> exploded;
            const document::VectorObject* vec = obj->asVector();
            if (vec == nullptr) {
                const document::DimensionObject* dim = obj->asDimension();
                if (dim == nullptr) {
                    continue;
                }
                exploded = document::dimensionGeometry(doc, *dim).contours;
            }
            for (const geometry::Contour& contour : vec != nullptr ? vec->contours : exploded) {
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
