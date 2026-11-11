#include "carvetto/document/bitmap_store.hpp"

#include <cmath>
#include <memory>
#include <string_view>

#include "carvetto/document/commands.hpp"

namespace carvetto::document {

BitmapAsset makeBitmapAsset(std::vector<std::uint8_t> bytes) {
    BitmapAsset a;
    a.key = fontKeyFor(bytes);
    a.bytes = std::move(bytes);
    return a;
}

bool AddBitmapAssetCommand::apply(Document& doc) {
    added_ = doc.insertBitmap(image_);
    return added_;
}

void AddBitmapAssetCommand::revert(Document& doc) {
    if (added_) {
        doc.takeBitmap(image_.key);
        added_ = false;
    }
}

std::string bitmapImportProblem(const Document& doc, const BitmapImport& in) {
    if (in.bytes.empty()) {
        return "the image file is empty";
    }
    if (in.widthPx <= 0 || in.heightPx <= 0 || in.widthPx > io::kMaxImageEdge || in.heightPx > io::kMaxImageEdge ||
        static_cast<std::size_t>(in.widthPx) * static_cast<std::size_t>(in.heightPx) > io::kMaxImagePixels) {
        return "the image dimensions are out of range";
    }
    const geometry::Box2& b = in.placementMm;
    if (!std::isfinite(b.min.x) || !std::isfinite(b.min.y) || !std::isfinite(b.max.x) || !std::isfinite(b.max.y) ||
        !(b.max.x > b.min.x) || !(b.max.y > b.min.y)) {
        return "the placement rectangle is empty or not finite";
    }
    return doc.bitmapProblem(makeBitmapAsset(in.bytes));
}

std::unique_ptr<Command> makeImportBitmapCommand(const Document& doc, BitmapImport in) {
    if (!bitmapImportProblem(doc, in).empty()) {
        return nullptr;
    }
    BitmapAsset asset = makeBitmapAsset(std::move(in.bytes));
    Object obj;
    obj.layer = in.layer;
    obj.sheet = in.sheet;
    obj.name = std::move(in.name);
    BitmapObject b;
    b.widthPx = in.widthPx;
    b.heightPx = in.heightPx;
    b.placementMm = in.placementMm;
    b.sourceRef = asset.key;
    obj.data = std::move(b);
    auto macro = std::make_unique<MacroCommand>("Import bitmap");
    macro->add(std::make_unique<AddBitmapAssetCommand>(std::move(asset)));
    macro->add(std::make_unique<AddObjectCommand>(std::move(obj)));
    return macro;
}

geometry::ShapeResult<io::RasterImage> decodeBitmapObject(const Document& doc, const BitmapObject& obj,
                                                           const BitmapDecoder& decoder) {
    using R = geometry::ShapeResult<io::RasterImage>;
    const BitmapAsset* asset = isFontKey(obj.sourceRef) ? doc.findBitmap(obj.sourceRef) : nullptr;
    if (asset == nullptr) {
        return R(std::string("the bitmap has no embedded image data"));
    }
    const std::string_view head(reinterpret_cast<const char*>(asset->bytes.data()), asset->bytes.size());
    if (head.size() >= 2 && head[0] == 'P' && head[1] >= '1' && head[1] <= '6') {
        return io::decodePnm(head);
    }
    if (!decoder) {
        return R(std::string("this image format needs the application's image decoder"));
    }
    R r = decoder(std::span<const std::uint8_t>(asset->bytes));
    if (r.ok() && !r.value().valid()) {
        return R(std::string("the decoder returned an invalid image"));
    }
    return r;
}

}  // namespace carvetto::document
