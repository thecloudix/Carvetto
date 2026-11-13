#include "carvetto/viewmodel/bitmap_import.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <utility>

namespace carvetto::viewmodel {

namespace {

BitmapImportOutcome refuse(std::string why) {
    BitmapImportOutcome out;
    out.error = std::move(why);
    return out;
}

}  // namespace

BitmapImportOutcome importBitmapIntoDocument(document::Document& doc, std::vector<std::uint8_t> bytes,
                                             const std::string& name, const document::BitmapDecoder& decoder) {
    if (bytes.empty()) {
        return refuse("the image file is empty");
    }
    if (bytes.size() > document::kMaxEmbeddedBitmapBytes) {
        return refuse("the image file is too large to embed (more than 64 MiB)");
    }
    const document::Layer* layer = doc.findLayer(doc.activeLayer());
    if (layer == nullptr || !layer->visible || layer->locked) {
        return refuse("the active layer is hidden or locked; choose another layer to import the bitmap into");
    }
    const geometry::ShapeResult<io::RasterImage> raster =
        document::decodeBitmapBytes(std::span<const std::uint8_t>(bytes), decoder);
    if (!raster.ok()) {
        return refuse("the image cannot be read: " + raster.error());
    }

    document::BitmapImport in;
    in.widthPx = raster.value().width;
    in.heightPx = raster.value().height;
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    const geometry::Box2 material = sheet != nullptr ? sheet->materialBoundsMm() : geometry::Box2{};
    const double aspect = static_cast<double>(in.widthPx) / static_cast<double>(in.heightPx);
    double w = 100.0;
    double h = 100.0 / aspect;
    geometry::Vec2 centre{0.0, 0.0};
    if (!material.isEmpty() && material.width() > 0.0 && material.height() > 0.0) {
        const double scale = std::min(material.width() / in.widthPx, material.height() / in.heightPx);
        w = scale * in.widthPx;
        h = scale * in.heightPx;
        centre = {0.5 * (material.min.x + material.max.x), 0.5 * (material.min.y + material.max.y)};
    }
    in.placementMm = geometry::Box2{{centre.x - 0.5 * w, centre.y - 0.5 * h}, {centre.x + 0.5 * w, centre.y + 0.5 * h}};
    in.bytes = std::move(bytes);
    in.layer = doc.activeLayer();
    in.sheet = doc.activeSheet();
    in.name = name;
    if (const std::string bad = document::bitmapImportProblem(doc, in); !bad.empty()) {
        return refuse(bad);
    }
    std::unique_ptr<document::Command> cmd = document::makeImportBitmapCommand(doc, std::move(in));
    if (cmd == nullptr || !doc.execute(std::move(cmd))) {
        return refuse("the bitmap could not be added");
    }
    BitmapImportOutcome out;
    out.ok = true;
    out.widthPx = raster.value().width;
    out.heightPx = raster.value().height;
    // Objects are kept in id order and ids only grow: the newest one is the bitmap just added.
    out.object = doc.objects().rbegin()->first;
    doc.selection().assign({out.object});
    return out;
}

}  // namespace carvetto::viewmodel
