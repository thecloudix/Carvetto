#include "carvetto/viewmodel/tiling_form.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace carvetto::viewmodel {

namespace {

constexpr std::size_t kMaxOutlineSegments = 4000;

bool usesWidth(toolpath::TileStrategy s) {
    return s != toolpath::TileStrategy::FeedThroughY;
}

bool usesHeight(toolpath::TileStrategy s) {
    return s != toolpath::TileStrategy::FeedThroughX;
}

bool sameBox(const geometry::Box2& a, const geometry::Box2& b) {
    return a.min.x == b.min.x && a.min.y == b.min.y && a.max.x == b.max.x && a.max.y == b.max.y;
}

}  // namespace

TilingForm defaultTilingForm(const tooling::Machine* machine) {
    TilingForm f;
    if (machine != nullptr) {
        f.width = std::max(0.0, machine->workAreaX);
        f.height = std::max(0.0, machine->workAreaY);
    }
    return f;
}

std::string validateTilingForm(const TilingForm& form) {
    const bool w = usesWidth(form.strategy);
    const bool h = usesHeight(form.strategy);
    if ((w && !(form.width > 0.0 && std::isfinite(form.width))) ||
        (h && !(form.height > 0.0 && std::isfinite(form.height)))) {
        return "the tile size must be greater than zero";
    }
    if (!(form.overlap >= 0.0) || !std::isfinite(form.overlap) || (w && form.overlap >= form.width) ||
        (h && form.overlap >= form.height)) {
        return "the overlap must be at least zero and smaller than the tile size";
    }
    return {};
}

std::optional<toolpath::TileSpec> tileSpecOf(const TilingForm& form) {
    if (!form.enabled || !validateTilingForm(form).empty()) {
        return std::nullopt;
    }
    toolpath::TileSpec spec;
    spec.strategy = form.strategy;
    spec.width = form.width;
    spec.height = form.height;
    spec.overlap = form.overlap;
    spec.smallestFirst = form.smallestFirst;
    spec.originalPosition = form.originalPosition;
    return spec;
}

void applyTiling(SaveOptions& options, const TilingForm& form, const tooling::Machine* machine) {
    options.workAreaX = machine != nullptr ? std::max(0.0, machine->workAreaX) : 0.0;
    options.workAreaY = machine != nullptr ? std::max(0.0, machine->workAreaY) : 0.0;
    if (form.enabled) {
        // An invalid form still turns tiling on (with the raw numbers) so that saving reports the
        // error instead of silently writing untiled files.
        toolpath::TileSpec spec;
        spec.strategy = form.strategy;
        spec.width = form.width;
        spec.height = form.height;
        spec.overlap = form.overlap;
        spec.smallestFirst = form.smallestFirst;
        spec.originalPosition = form.originalPosition;
        options.tiling = spec;
    } else {
        options.tiling.reset();
    }
}

TilePreview previewTiling(const std::vector<document::ToolpathRecord>& records, const TilingForm& form,
                          double workAreaX, double workAreaY) {
    TilePreview out;
    if (!form.enabled) {
        out.error = "tiling is off";
        return out;
    }
    out.error = validateTilingForm(form);
    if (!out.ok()) {
        return out;
    }
    std::vector<toolpath::Toolpath> tps;
    for (const document::ToolpathRecord& r : records) {
        if (r.hasMoves()) {
            tps.push_back(r.toolpath());
        }
    }
    if (tps.empty()) {
        out.error = "there are no calculated toolpaths to tile";
        return out;
    }
    for (const toolpath::Toolpath& tp : tps) {
        out.extent.expand(tp.bounds());
    }
    out.jobExceedsWorkArea = (workAreaX > 0.0 && out.extent.width() > workAreaX + 1e-9) ||
                             (workAreaY > 0.0 && out.extent.height() > workAreaY + 1e-9);

    const toolpath::TileSpec spec = *tileSpecOf(form);
    const toolpath::TilingResult res = toolpath::tileToolpaths(tps, spec);
    if (!res.ok()) {
        out.error = res.error;
        return out;
    }
    std::string err;
    const std::vector<geometry::Box2> rects = toolpath::tileRects(out.extent, spec, &err);
    std::vector<bool> used(res.tiles.size(), false);
    for (const geometry::Box2& rect : rects) {
        TilePreviewEntry e;
        e.rect = rect;
        for (std::size_t k = 0; k < res.tiles.size(); ++k) {
            if (!used[k] && sameBox(res.tiles[k].rect, rect)) {
                used[k] = true;
                e.number = k + 1;
                e.oversize = !toolpath::tileFitsWorkArea(res.tiles[k], workAreaX, workAreaY);
                break;
            }
        }
        if (e.number != 0) {
            ++out.saved;
            if (e.oversize) {
                ++out.oversize;
            }
        }
        out.tiles.push_back(e);
    }

    // Outline for drawing: cutting moves as segments, thinned to a bounded count.
    std::vector<std::pair<geometry::Vec2, geometry::Vec2>> all;
    for (const toolpath::Toolpath& tp : tps) {
        std::optional<geometry::Vec2> at;
        for (const toolpath::Move& m : tp.moves) {
            const geometry::Vec2 to{m.to.x, m.to.y};
            const bool cut = m.kind == toolpath::MoveKind::Feed || m.kind == toolpath::MoveKind::ArcCW ||
                             m.kind == toolpath::MoveKind::ArcCCW;
            if (cut && at && (at->x != to.x || at->y != to.y)) {
                all.emplace_back(*at, to);
            }
            if (m.kind != toolpath::MoveKind::Dwell) {
                at = to;
            }
        }
    }
    const std::size_t stride = all.size() / kMaxOutlineSegments + 1;
    for (std::size_t i = 0; i < all.size(); i += stride) {
        out.outline.push_back(all[i]);
    }
    return out;
}

std::string describeTilePreview(const TilePreview& preview) {
    if (!preview.ok()) {
        return preview.error;
    }
    std::ostringstream s;
    s << preview.saved << " tile(s) to save";
    if (preview.tiles.size() > preview.saved) {
        s << ", " << (preview.tiles.size() - preview.saved) << " empty skipped";
    }
    if (preview.oversize > 0) {
        s << "; " << preview.oversize << " larger than the machine work area (saving is refused)";
    }
    return s.str();
}

}  // namespace carvetto::viewmodel
