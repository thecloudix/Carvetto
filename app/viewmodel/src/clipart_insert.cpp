#include "carvetto/viewmodel/clipart_insert.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/viewmodel/bitmap_import.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"

namespace carvetto::viewmodel {

namespace {

constexpr double kEmptyMaterialFitMm = 100.0;

ClipartInsertOutcome refuse(std::string why) {
    ClipartInsertOutcome o;
    o.error = std::move(why);
    return o;
}

/// Closes an open undo macro on every exit path.
class MacroScope {
   public:
    MacroScope(document::UndoStack& stack, std::string text) : stack_(stack) { stack_.beginMacro(std::move(text)); }
    ~MacroScope() { stack_.endMacro(); }
    MacroScope(const MacroScope&) = delete;
    MacroScope& operator=(const MacroScope&) = delete;

   private:
    document::UndoStack& stack_;
};

std::vector<document::ObjectId> idsFrom(const document::Document& doc, std::uint64_t firstId) {
    std::vector<document::ObjectId> ids;
    for (auto it = doc.objects().lower_bound(document::ObjectId{firstId}); it != doc.objects().end(); ++it) {
        ids.push_back(it->first);
    }
    return ids;
}

bool finiteBox(const geometry::Box2& b) {
    return !b.isEmpty() && std::isfinite(b.min.x) && std::isfinite(b.min.y) && std::isfinite(b.max.x) &&
           std::isfinite(b.max.y);
}

}  // namespace

ClipartInsertOutcome insertClipartBytes(document::Document& doc, std::span<const std::uint8_t> bytes,
                                        const std::string& name, const ClipartInsertOptions& options,
                                        const document::BitmapDecoder& decoder) {
    try {
        if (!(options.fitFraction > 0.0) || options.fitFraction > 1.0) {
            return refuse("the fit fraction must be above 0 and at most 1");
        }
        const ClipartFile file = checkClipartBytes(bytes, name, options.limits);
        if (!file.ok) {
            return refuse(file.error);
        }
        const document::Layer* layer = doc.findLayer(doc.activeLayer());
        if (layer == nullptr || !layer->visible || layer->locked) {
            return refuse("the active layer is hidden or locked; choose another layer to insert the clipart into");
        }
        const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
        const geometry::Box2 material = sheet != nullptr ? sheet->materialBoundsMm() : geometry::Box2{};
        const bool haveMaterial = !material.isEmpty() && material.width() > 0.0 && material.height() > 0.0;
        const geometry::Vec2 materialCentre = haveMaterial ? material.centre() : geometry::Vec2{0.0, 0.0};
        const double fitW = haveMaterial ? material.width() * options.fitFraction : kEmptyMaterialFitMm * options.fitFraction;
        const double fitH = haveMaterial ? material.height() * options.fitFraction : kEmptyMaterialFitMm * options.fitFraction;
        const geometry::Vec2 target = options.at ? *options.at : materialCentre;
        if (!std::isfinite(target.x) || !std::isfinite(target.y)) {
            return refuse("the insertion point is not a finite position");
        }

        ClipartInsertOutcome out;
        out.kind = file.kind;
        // Parsed first, with the browser's tight limits, so a hostile vector file is refused before
        // the document is touched (and its warnings are known).
        std::vector<geometry::Contour> svgContours;
        if (file.kind != ClipartKind::Png) {
            ClipartGeometry g = parseClipartVectors(file.bytes, file.kind, options.limits);
            if (!g.ok) {
                return refuse(name + ": " + g.error);
            }
            out.warnings = g.warnings;
            if (g.contours.empty() && (file.kind == ClipartKind::Svg || g.textItems == 0)) {
                return refuse(name + ": the drawing holds no vectors");
            }
            if (file.kind == ClipartKind::Svg) {
                svgContours = std::move(g.contours);
            }
        }
        if (file.kind == ClipartKind::Svg && !doc.idGenerator().canAllocate(svgContours.size())) {
            return refuse("cannot insert: the document has no free object ids left");
        }

        const std::size_t stepsBefore = doc.undoStack().count();
        const std::uint64_t firstId = doc.idGenerator().peek();
        bool failed = false;
        std::string failure;
        {
            MacroScope macro(doc.undoStack(), "Insert clipart");
            if (file.kind == ClipartKind::Svg) {
                for (geometry::Contour& c : svgContours) {
                    doc.execute(std::make_unique<document::AddObjectCommand>(
                        document::Object::vector(std::move(c), doc.activeLayer(), doc.activeSheet())));
                }
            } else if (file.kind == ClipartKind::Dxf) {
                const DxfImportOutcome r = importDxfIntoDocument(doc, file.bytes, std::nullopt, options.textFontBytes);
                if (!r.ok) {
                    failed = true;
                    failure = r.error;
                }
                for (const std::string& w : r.warnings) {
                    out.warnings.push_back(w);
                }
            } else {
                const std::size_t slash = name.find_last_of("/\\");
                const BitmapImportOutcome r = importBitmapIntoDocument(
                    doc, std::vector<std::uint8_t>(file.bytes), slash == std::string::npos ? name : name.substr(slash + 1), decoder);
                if (!r.ok) {
                    failed = true;
                    failure = r.error;
                }
            }
            const std::vector<document::ObjectId> ids = failed ? std::vector<document::ObjectId>{} : idsFrom(doc, firstId);
            if (!failed && ids.empty()) {
                failed = true;
                failure = "the file holds nothing that can be inserted";
            }
            if (!failed) {
                geometry::Box2 box;
                for (const document::ObjectId id : ids) {
                    if (const auto b = doc.objectBounds(id)) {
                        box.expand(*b);
                    }
                }
                const double w = box.width();
                const double h = box.height();
                double scale = 1.0;
                if (!finiteBox(box) || !std::isfinite(w) || !std::isfinite(h)) {
                    failed = true;
                    failure = "the clipart has no measurable size";
                } else {
                    if (w > 0.0 && h > 0.0) {
                        scale = std::min(fitW / w, fitH / h);
                    } else if (w > 0.0) {
                        scale = fitW / w;
                    } else if (h > 0.0) {
                        scale = fitH / h;
                    }
                    if (!(scale > 0.0) || !std::isfinite(scale)) {
                        failed = true;
                        failure = "the clipart cannot be scaled to fit";
                    }
                }
                if (!failed) {
                    const geometry::Vec2 c = box.centre();
                    const geometry::Transform2 m = geometry::Transform2::translation(target) *
                                                   geometry::Transform2::scaling(scale) *
                                                   geometry::Transform2::translation({-c.x, -c.y});
                    doc.execute(std::make_unique<document::TransformObjectsCommand>(ids, m));
                    geometry::Box2 placed;
                    for (const document::ObjectId id : ids) {
                        if (const auto b = doc.objectBounds(id)) {
                            placed.expand(*b);
                        }
                    }
                    if (!finiteBox(placed)) {
                        failed = true;
                        failure = "the clipart could not be placed";
                    } else {
                        out.placedBox = placed;
                        out.scale = scale;
                        out.objectsAdded = ids.size();
                    }
                }
            }
        }
        if (failed) {
            // The macro recorded whatever the importer had added; take it back out completely.
            if (doc.undoStack().count() > stepsBefore) {
                doc.undoStack().undoAndDiscard(doc);
            }
            return refuse(name + ": " + failure);
        }
        doc.selection().assign(idsFrom(doc, firstId));
        out.ok = true;
        return out;
    } catch (const std::exception& e) {
        return refuse(std::string("cannot insert the clipart: ") + e.what());
    }
}

ClipartInsertOutcome insertClipartFile(document::Document& doc, const std::string& path,
                                       const ClipartInsertOptions& options, const document::BitmapDecoder& decoder) {
    const ClipartFile file = readClipartFile(path, options.limits);
    if (!file.ok) {
        return refuse(file.error);
    }
    return insertClipartBytes(doc, file.bytes, path, options, decoder);
}

}  // namespace carvetto::viewmodel
