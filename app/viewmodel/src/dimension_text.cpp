#include "carvetto/viewmodel/dimension_text.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <utility>

#include "carvetto/geometry/transform2.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/text/layout.hpp"
#include "carvetto/viewmodel/starter_font.hpp"

namespace carvetto::viewmodel {

namespace {

/// Parsed fonts by key ("" is the starter font). A Font is not thread-safe, so every use holds the
/// mutex. A key maps to null when its bytes were refused (not retried). Bounded.
struct FontCache {
    std::mutex mutex;
    std::map<std::string, std::unique_ptr<text::Font>> fonts;
};

FontCache& cache() {
    static FontCache c;
    return c;
}

constexpr std::size_t kMaxCachedFonts = 8;

/// The parsed font for `key` (caller holds the lock). Null when it cannot be loaded.
const text::Font* loadLocked(FontCache& c, const document::Document& doc, const std::string& key) {
    const document::FontAsset* asset = key.empty() ? nullptr : doc.findFont(key);
    const std::string effective = asset != nullptr ? key : std::string{};
    const auto it = c.fonts.find(effective);
    if (it != c.fonts.end()) {
        return it->second.get();
    }
    if (c.fonts.size() >= kMaxCachedFonts) {
        c.fonts.clear();
    }
    text::FontLoad load = asset != nullptr ? text::Font::load(asset->bytes) : text::Font::load(starterFontBytes());
    std::unique_ptr<text::Font> font;
    if (load.ok()) {
        font = std::make_unique<text::Font>(std::move(load.font));
    }
    const text::Font* raw = font.get();
    c.fonts[effective] = std::move(font);
    return raw;
}

}  // namespace

std::vector<geometry::Contour> dimensionTextContours(const document::Document& doc,
                                                     const document::DimensionObject& dim,
                                                     const document::DimensionGeometry& g,
                                                     const std::string& str) {
    std::vector<geometry::Contour> out;
    if (!g.valid || str.empty() || !(dim.textHeightMm > 0.0)) {
        return out;
    }
    FontCache& c = cache();
    const std::lock_guard<std::mutex> lock(c.mutex);
    const text::Font* font = loadLocked(c, doc, dim.fontKey);
    if (font == nullptr && !dim.fontKey.empty()) {
        font = loadLocked(c, doc, std::string{});  // unreadable embedded font: the starter font
    }
    if (font == nullptr) {
        return out;
    }
    text::TextStyle style;
    style.sizeMm = dim.textHeightMm / kDimensionCapHeightRatio;
    style.alignment = text::Alignment::Centre;
    const text::TextLayout layout = text::layoutText(*font, str, style);
    if (!layout.ok() || layout.contours.empty()) {
        return out;
    }
    const geometry::Vec2 centre = layout.bounds.centre();
    const geometry::Transform2 toCentre = geometry::Transform2::translation(-centre);
    const geometry::Transform2 turn = geometry::Transform2::rotation(g.textAngle);
    const geometry::Transform2 place = geometry::Transform2::translation(g.textCentre);
    out.reserve(layout.contours.size());
    for (const geometry::Contour& contour : layout.contours) {
        out.push_back(contour.transformed(toCentre).transformed(turn).transformed(place));
    }
    return out;
}

std::vector<geometry::Contour> dimensionTextContours(const document::Document& doc,
                                                     const document::DimensionObject& dim) {
    const document::DimensionGeometry g = document::dimensionGeometry(doc, dim);
    return dimensionTextContours(doc, dim, g, document::dimensionText(doc, dim, doc.job().displayUnits));
}

std::string dimensionFontName(const document::Document& doc, const std::string& fontKey) {
    FontCache& c = cache();
    const std::lock_guard<std::mutex> lock(c.mutex);
    const text::Font* font = loadLocked(c, doc, fontKey);
    if (font == nullptr) {
        return starterFontName();
    }
    const std::string effective = !fontKey.empty() && doc.findFont(fontKey) != nullptr ? fontKey : std::string{};
    if (effective.empty()) {
        return starterFontName();
    }
    return font->familyName().empty() ? effective : font->familyName();
}

}  // namespace carvetto::viewmodel
