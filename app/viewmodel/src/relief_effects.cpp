#include "carvetto/viewmodel/relief_effects.hpp"

#include <memory>
#include <sstream>
#include <utility>

#include "carvetto/document/relief_commands.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/viewmodel/vector_selector.hpp"

namespace carvetto::viewmodel {

namespace {

using document::Document;
using document::LevelId;

EditResult refused(std::string message) {
    EditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

EditResult run(Document& doc, std::unique_ptr<document::Command> cmd, const char* failure) {
    doc.undoStack().breakCoalescing();
    const bool ok = doc.execute(std::move(cmd));
    doc.undoStack().breakCoalescing();
    if (!ok) {
        return refused(failure);
    }
    EditResult r;
    r.changed = true;
    return r;
}

}  // namespace

const std::vector<relief::MirrorMode>& allMirrorModes() {
    static const std::vector<relief::MirrorMode> modes = {
        relief::MirrorMode::None,          relief::MirrorMode::LeftToRight,   relief::MirrorMode::RightToLeft,
        relief::MirrorMode::TopToBottom,   relief::MirrorMode::BottomToTop,   relief::MirrorMode::QuadTopRight,
        relief::MirrorMode::QuadTopLeft,   relief::MirrorMode::QuadBottomLeft, relief::MirrorMode::QuadBottomRight};
    return modes;
}

const char* mirrorModeName(relief::MirrorMode mode) {
    switch (mode) {
        case relief::MirrorMode::None: return "None";
        case relief::MirrorMode::LeftToRight: return "Left to right";
        case relief::MirrorMode::RightToLeft: return "Right to left";
        case relief::MirrorMode::TopToBottom: return "Top to bottom";
        case relief::MirrorMode::BottomToTop: return "Bottom to top";
        case relief::MirrorMode::QuadTopRight: return "Top right quadrant";
        case relief::MirrorMode::QuadTopLeft: return "Top left quadrant";
        case relief::MirrorMode::QuadBottomLeft: return "Bottom left quadrant";
        case relief::MirrorMode::QuadBottomRight: return "Bottom right quadrant";
    }
    return "None";
}

LevelId effectsTargetLevel(const Document& doc, LevelId picked) {
    if (picked && doc.findReliefLevel(picked) != nullptr) {
        return picked;
    }
    if (const document::ComponentId one = doc.componentSelection().single()) {
        if (const auto loc = doc.locateReliefComponent(one)) {
            return loc->level;
        }
    }
    const std::vector<LevelId> levels = doc.reliefLevelsOnSheet(doc.activeSheet());
    return levels.empty() ? LevelId{} : levels.back();
}

relief::Clip clipFromSelection(const Document& doc) {
    relief::Clip clip;
    const ResolvedVectors rv = resolveVectors(doc, VectorSelector::fromSelection(doc));
    for (const ContourRef& ref : rv.contours) {
        if (ref.contour == nullptr || !ref.contour->closed) {
            continue;
        }
        const geometry::Polyline pl = ref.contour->polygonise(geometry::Tolerance::defaults());
        if (pl.vertexCount() < 3) {
            continue;
        }
        std::vector<relief::Point> poly;
        poly.reserve(pl.vertexCount());
        for (const geometry::Vec2& v : pl.points) {
            poly.push_back({v.x, v.y});
        }
        clip.polygons.push_back(std::move(poly));
    }
    return clip;
}

std::optional<relief::Point> modelCentre(const Document& doc, document::SheetId sheet) {
    relief::Grid g;
    if (relief::unionGrid(doc.reliefModel(sheet), g) != relief::Status::Ok) {
        return std::nullopt;
    }
    return relief::Point{g.origin.x + 0.5 * static_cast<double>(g.width - 1) * g.cellSize,
                         g.origin.y + 0.5 * static_cast<double>(g.height - 1) * g.cellSize};
}

std::string LevelEffectsSummary::text() const {
    std::ostringstream os;
    os << levelName << ": ";
    if (clipPolygons == 0) {
        os << "not clipped";
    } else {
        os << "clipped by " << clipPolygons << (clipPolygons == 1 ? " polygon" : " polygons");
    }
    os << "; mirror: " << mirrorModeName(mirror);
    if (wrapping) {
        os << "; wrapped";
    }
    return os.str();
}

std::optional<LevelEffectsSummary> summarizeEffects(const Document& doc, LevelId level) {
    const document::ReliefLevel* l = doc.findReliefLevel(level);
    if (l == nullptr) {
        return std::nullopt;
    }
    LevelEffectsSummary s;
    s.levelName = l->name;
    s.clipPolygons = l->effects.clip.polygons.size();
    for (const auto& p : l->effects.clip.polygons) {
        s.clipVertices += p.size();
    }
    s.mirror = l->effects.mirror;
    s.mirrorCenter = l->effects.mirrorCenter;
    s.wrapping = l->effects.wrapping.enabled;
    s.selectedClosedVectors = clipFromSelection(doc).polygons.size();
    return s;
}

EditResult applyLevelClip(Document& doc, LevelId level) {
    if (doc.findReliefLevel(level) == nullptr) {
        return refused("Pick a level first");
    }
    relief::Clip clip = clipFromSelection(doc);
    if (clip.polygons.empty()) {
        return refused("Select one or more closed vectors to clip by");
    }
    std::size_t vertices = 0;
    for (const auto& p : clip.polygons) {
        vertices += p.size();
    }
    if (clip.polygons.size() > relief::kMaxClipPolygons || vertices > relief::kMaxClipVertices) {
        return refused("The selected vectors are too complex to clip by (at most 256 outlines, 65536 points)");
    }
    return run(doc, std::make_unique<document::SetReliefLevelClipCommand>(level, std::move(clip)),
               "The clipping is unchanged or not valid");
}

EditResult removeLevelClip(Document& doc, LevelId level) {
    const document::ReliefLevel* l = doc.findReliefLevel(level);
    if (l == nullptr) {
        return refused("Pick a level first");
    }
    if (l->effects.clip.polygons.empty()) {
        return refused("The level is not clipped");
    }
    return run(doc, std::make_unique<document::SetReliefLevelClipCommand>(level, relief::Clip{}),
               "The clipping could not be removed");
}

EditResult setLevelMirror(Document& doc, LevelId level, relief::MirrorMode mode) {
    const document::ReliefLevel* l = doc.findReliefLevel(level);
    if (l == nullptr) {
        return refused("Pick a level first");
    }
    relief::Point centre = l->effects.mirrorCenter;
    if (const auto c = modelCentre(doc, l->sheet)) {
        centre = *c;
    }
    if (mode == l->effects.mirror && (mode == relief::MirrorMode::None ||
                                      (centre.x == l->effects.mirrorCenter.x && centre.y == l->effects.mirrorCenter.y))) {
        return refused("The mirror mode is already set");
    }
    return run(doc, std::make_unique<document::SetReliefLevelMirrorCommand>(level, mode, centre),
               "The mirror mode could not be set");
}

EditResult setLevelWrapping(Document& doc, LevelId level, bool enabled) {
    const document::ReliefLevel* l = doc.findReliefLevel(level);
    if (l == nullptr) {
        return refused("Pick a level first");
    }
    relief::Wrapping w;
    if (enabled) {
        const document::Job& job = doc.job();
        if (job.type != document::JobType::Rotary) {
            return refused("Wrapping needs a rotary job");
        }
        const geometry::Box2 area = document::materialBounds(job.datum, job.sizeMm.x, job.sizeMm.y, job.datumOffsetMm);
        w.enabled = true;
        w.circumferenceIsY = job.rotaryAxis == document::RotaryAxis::X;
        w.circumference = job.circumferenceMm();
        w.start = w.circumferenceIsY ? area.min.y : area.min.x;
    }
    if (w == l->effects.wrapping) {
        return refused(enabled ? "The level is already wrapped" : "The level is not wrapped");
    }
    return run(doc, std::make_unique<document::SetReliefLevelWrappingCommand>(level, w),
               "The wrapping could not be set");
}

EditResult bakeLevelEffects(Document& doc, LevelId level) {
    return run(doc, std::make_unique<document::BakeReliefLevelEffectsCommand>(level),
               "Nothing to bake: the level needs a clip, mirror mode or wrapping and a visible component");
}

}  // namespace carvetto::viewmodel
