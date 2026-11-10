#include "carvetto/viewmodel/toolpath_preview.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <utility>

namespace carvetto::viewmodel {

using geometry::Box2;
using geometry::Vec2;
using toolpath::Move;
using toolpath::MoveKind;
using toolpath::Vec3;

PreviewColour colourOf(MoveClass cls) {
    switch (cls) {
        case MoveClass::Rapid:
            return {200, 60, 60};
        case MoveClass::Feed:
            return {30, 120, 220};
        case MoveClass::Plunge:
            return {230, 140, 20};
        case MoveClass::Retract:
            return {150, 90, 200};
        case MoveClass::Ramp:
            return {30, 170, 110};
    }
    return {0, 0, 0};
}

const char* nameOf(MoveClass cls) {
    switch (cls) {
        case MoveClass::Rapid:
            return "Rapid";
        case MoveClass::Feed:
            return "Feed";
        case MoveClass::Plunge:
            return "Plunge";
        case MoveClass::Retract:
            return "Retract";
        case MoveClass::Ramp:
            return "Ramp / helix";
    }
    return "";
}

std::optional<MoveClass> classify(const Move& move, const Vec3& from) {
    switch (move.kind) {
        case MoveKind::Rapid:
            return MoveClass::Rapid;
        case MoveKind::Plunge:
            return MoveClass::Plunge;
        case MoveKind::Retract:
            return MoveClass::Retract;
        case MoveKind::Dwell:
            return std::nullopt;
        case MoveKind::Feed:
        case MoveKind::ArcCW:
        case MoveKind::ArcCCW: {
            const bool dz = std::fabs(move.to.z - from.z) > 1e-9;
            const bool arc = toolpath::isArc(move.kind);
            const bool dxy = arc || std::fabs(move.to.x - from.x) > 1e-9 || std::fabs(move.to.y - from.y) > 1e-9;
            return dz && dxy ? MoveClass::Ramp : MoveClass::Feed;
        }
    }
    return MoveClass::Feed;
}

bool PreviewStyle::operator<(const PreviewStyle& o) const {
    return std::tie(played, stale, highlighted, cls) < std::tie(o.played, o.stale, o.highlighted, o.cls);
}

namespace {

bool finite(const Vec3& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/// Points after the start of an arc move (the last one is exactly the end point).
void arcPoints(const Vec3& from, const Move& m, std::vector<Vec2>& out) {
    const Vec2 c = m.centre;
    const Vec2 s{from.x, from.y};
    const Vec2 e{m.to.x, m.to.y};
    const double r = std::hypot(s.x - c.x, s.y - c.y);
    if (!std::isfinite(r) || !std::isfinite(c.x) || !std::isfinite(c.y) || r < 1e-12) {
        out.push_back(e);
        return;
    }
    const double a0 = std::atan2(s.y - c.y, s.x - c.x);
    double sweep = 0.0;
    const bool full = std::fabs(e.x - s.x) < 1e-9 && std::fabs(e.y - s.y) < 1e-9;
    const double a1 = std::atan2(e.y - c.y, e.x - c.x);
    constexpr double twoPi = 2.0 * geometry::kPi;
    if (m.kind == MoveKind::ArcCCW) {
        sweep = full ? twoPi : a1 - a0;
        while (sweep <= 0.0) {
            sweep += twoPi;
        }
    } else {
        sweep = full ? -twoPi : a1 - a0;
        while (sweep >= 0.0) {
            sweep -= twoPi;
        }
    }
    const double tol = kPreviewArcTolerance;
    const double step = r > tol ? 2.0 * std::acos(std::max(-1.0, 1.0 - tol / r)) : geometry::kPi / 2.0;
    const double want = std::ceil(std::fabs(sweep) / std::max(step, 1e-6));
    const std::size_t n = static_cast<std::size_t>(std::clamp(want, 2.0, static_cast<double>(kPreviewMaxArcPoints)));
    for (std::size_t i = 1; i < n; ++i) {
        const double a = a0 + sweep * (static_cast<double>(i) / static_cast<double>(n));
        out.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
    }
    out.push_back(e);
}

}  // namespace

std::shared_ptr<const PreviewGeometry> buildPreviewGeometry(const std::vector<Move>& moves,
                                                            const std::vector<std::uint32_t>* originalIndex,
                                                            std::optional<Vec3> startAt) {
    auto geo = std::make_shared<PreviewGeometry>();
    if (moves.empty()) {
        return geo;
    }
    PreviewChunk open;
    bool isOpen = false;
    auto close = [&]() {
        if (isOpen && !open.points.empty()) {
            geo->points += open.points.size();
            geo->bounds.expand(open.bounds);
            geo->chunks.push_back(std::move(open));
        }
        open = PreviewChunk{};
        isOpen = false;
    };

    Vec3 cur = startAt.value_or(moves.front().to);
    const bool zeroFirst = !startAt.has_value();
    std::vector<Vec2> pts;
    for (std::size_t i = 0; i < moves.size(); ++i) {
        const Move& m = moves[i];
        if (!finite(m.to)) {
            continue;
        }
        const Vec3 from = i == 0 && zeroFirst ? m.to : cur;
        const std::optional<MoveClass> cls = classify(m, from);
        if (!cls) {
            continue;
        }
        const std::size_t source = originalIndex != nullptr && i < originalIndex->size() ? (*originalIndex)[i] : i;
        const std::uint32_t idx = static_cast<std::uint32_t>(std::min<std::size_t>(source, 0xFFFFFFFEu));
        ++geo->movesByClass[static_cast<std::size_t>(*cls)];
        pts.clear();
        if (toolpath::isArc(m.kind) && (i > 0 || !zeroFirst)) {
            arcPoints(from, m, pts);
        } else {
            pts.push_back({m.to.x, m.to.y});
        }
        const Vec2 start{from.x, from.y};
        const bool still = (i == 0 && zeroFirst) || (std::fabs(m.to.x - from.x) < 1e-12 && std::fabs(m.to.y - from.y) < 1e-12 &&
                                      !toolpath::isArc(m.kind));
        cur = m.to;
        if (i == 0 && zeroFirst) {
            continue;
        }
        if (still) {
            // A vertical move: a dot where it happens (once for a plunge/retract pair repeats).
            if (*cls == MoveClass::Plunge || *cls == MoveClass::Retract) {
                close();
                PreviewChunk dot;
                dot.cls = *cls;
                dot.firstMove = dot.lastMove = idx;
                dot.points.push_back(start);
                dot.bounds.expand(start);
                if (!geo->chunks.empty()) {
                    const PreviewChunk& last = geo->chunks.back();
                    if (last.cls == dot.cls && last.points.size() == 1 && last.points.front().x == start.x &&
                        last.points.front().y == start.y) {
                        geo->chunks.back().lastMove = idx;
                        continue;
                    }
                }
                geo->points += 1;
                geo->bounds.expand(dot.bounds);
                geo->chunks.push_back(std::move(dot));
            }
            continue;
        }
        if (!isOpen || open.cls != *cls || open.points.size() + pts.size() > kPreviewChunkPoints) {
            close();
            open.cls = *cls;
            open.firstMove = idx;
            open.points.push_back(start);
            open.bounds.expand(start);
            isOpen = true;
        }
        for (const Vec2& p : pts) {
            open.points.push_back(p);
            open.bounds.expand(p);
        }
        open.lastMove = idx;
    }
    close();
    return geo;
}

// --- PreviewModel -----------------------------------------------------------------------------

void PreviewModel::clear() {
    scene_ = PreviewScene{};
    selected_ = document::ToolpathId{};
    hidden_.clear();
    ++revision_;
}

void PreviewModel::setEnabled(bool on) {
    if (enabled_ != on) {
        enabled_ = on;
        ++revision_;
    }
}

void PreviewModel::setVisible(document::ToolpathId id, bool visible) {
    const bool changed = visible ? hidden_.erase(id) > 0 : hidden_.insert(id).second;
    if (changed) {
        ++revision_;
    }
}

void PreviewModel::setSelectedOnly(bool on) {
    if (selectedOnly_ != on) {
        selectedOnly_ = on;
        ++revision_;
    }
}

void PreviewModel::setSelected(document::ToolpathId id) {
    if (selected_ != id) {
        selected_ = id;
        ++revision_;
    }
}

void PreviewModel::sync(ToolpathSession& session) {
    document::Document* doc = session.document();
    PreviewScene next;
    if (doc == nullptr) {
        if (!scene_.toolpaths.empty()) {
            scene_ = PreviewScene{};
            ++revision_;
        }
        return;
    }
    const std::vector<ToolpathListItem> items = session.list();
    const RoutedJob& routing = session.keepOutRouting();
    std::map<document::ToolpathId, const PreviewToolpath*> old;
    for (const PreviewToolpath& tp : scene_.toolpaths) {
        old[tp.id] = &tp;
    }
    std::size_t nextMove = 0;
    for (const document::ToolpathRecord& rec : doc->toolpaths()) {
        PreviewToolpath tp;
        tp.id = rec.id;
        tp.name = rec.name;
        tp.moves = rec.moves;
        tp.routedMoves = rec.moves;
        tp.computed = rec.hasMoves();
        tp.moveCount = rec.moveCount();
        for (const ToolpathListItem& it : items) {
            if (it.id == rec.id) {
                tp.stale = it.status == ToolpathStatus::NeedsRecalculation;
                tp.enabled = it.enabled;
                break;
            }
        }
        const RoutedToolpath* routed = tp.enabled && tp.computed ? routing.find(rec.id) : nullptr;
        if (routed != nullptr) {
            tp.routedMoves = routed->routed;
            tp.routing = routed->report;
        }
        if (tp.computed) {
            const auto it = old.find(rec.id);
            if (it != old.end() && it->second->moves == rec.moves && it->second->routedMoves == tp.routedMoves &&
                it->second->geometry) {
                tp.geometry = it->second->geometry;
            } else if (routed != nullptr && routed->changed()) {
                tp.geometry = buildPreviewGeometry(*routed->routed, routed->originalIndex.get(), routed->start);
            } else {
                tp.geometry = buildPreviewGeometry(*rec.moves);
            }
            if (tp.enabled) {
                next.routing.routed += tp.routing.routed;
                next.routing.unroutable += tp.routing.unroutable;
                next.routing.tooComplex += tp.routing.tooComplex;
                tp.firstMove = nextMove;
                nextMove += tp.moveCount;
                if (!tp.geometry->bounds.isEmpty()) {
                    next.bounds.expand(tp.geometry->bounds);
                }
            }
        }
        next.toolpaths.push_back(std::move(tp));
    }
    next.totalMoves = nextMove;
    // Same picture? (identity of the geometry, flags, names)
    bool same = next.toolpaths.size() == scene_.toolpaths.size() && next.totalMoves == scene_.totalMoves;
    for (std::size_t i = 0; same && i < next.toolpaths.size(); ++i) {
        const PreviewToolpath& a = next.toolpaths[i];
        const PreviewToolpath& b = scene_.toolpaths[i];
        same = a.id == b.id && a.stale == b.stale && a.enabled == b.enabled && a.computed == b.computed &&
               a.firstMove == b.firstMove && a.moves == b.moves && a.routedMoves == b.routedMoves && a.name == b.name;
    }
    if (!same) {
        scene_ = std::move(next);
        ++revision_;
    }
    if (!selected_.isNull()) {
        bool found = false;
        for (const PreviewToolpath& tp : scene_.toolpaths) {
            found = found || tp.id == selected_;
        }
        if (!found) {
            selected_ = document::ToolpathId{};
            ++revision_;
        }
    }
}

bool PreviewModel::drawn(const PreviewToolpath& tp) const {
    if (!enabled_ || !tp.enabled || !tp.computed || !tp.geometry || !isVisible(tp.id)) {
        return false;
    }
    return !selectedOnly_ || tp.id == selected_;
}

std::size_t PreviewModel::staleDrawn() const {
    std::size_t n = 0;
    for (const PreviewToolpath& tp : scene_.toolpaths) {
        if (drawn(tp) && tp.stale) {
            ++n;
        }
    }
    return n;
}

PreviewDrawList PreviewModel::draw(const ViewTransform& view, const PreviewDrawOptions& options) const {
    PreviewDrawList out;
    if (!enabled_) {
        return out;
    }
    const Box2 visible = view.visibleWorld().inflated(2.0 / std::max(view.scale(), 1e-9));
    std::map<PreviewStyle, std::size_t> batchOf;
    const double spacing = std::max(options.minPointSpacingPx, 0.0);
    for (const PreviewToolpath& tp : scene_.toolpaths) {
        if (!drawn(tp)) {
            continue;
        }
        out.stats.chunksTotal += tp.geometry->chunks.size();
        for (const PreviewChunk& chunk : tp.geometry->chunks) {
            out.stats.pointsIn += chunk.points.size();
            const std::size_t g = tp.firstMove + chunk.lastMove;
            if (g < options.fromMove || g >= options.toMove ||
                !options.classVisible[static_cast<std::size_t>(chunk.cls)] || !visible.intersects(chunk.bounds)) {
                continue;
            }
            PreviewStyle style;
            style.cls = chunk.cls;
            style.highlighted = tp.id == selected_;
            style.stale = tp.stale;
            style.played = options.played;
            auto found = batchOf.find(style);
            if (found == batchOf.end()) {
                found = batchOf.emplace(style, out.batches.size()).first;
                out.batches.emplace_back();
                out.batches.back().style = style;
            }
            PreviewBatch& batch = out.batches[found->second];
            const std::size_t before = batch.points.size();
            ++out.stats.chunksDrawn;
            const Box2 px = view.worldToScreen(chunk.bounds);
            if (chunk.points.size() > 1 && px.width() < spacing && px.height() < spacing) {
                // Smaller than a pixel: one short stroke across its extent keeps the density.
                batch.points.push_back(view.worldToScreen(chunk.points.front()));
                batch.points.push_back(view.worldToScreen(chunk.points.back()));
            } else {
                Vec2 last = view.worldToScreen(chunk.points.front());
                batch.points.push_back(last);
                for (std::size_t i = 1; i + 1 < chunk.points.size(); ++i) {
                    const Vec2 p = view.worldToScreen(chunk.points[i]);
                    if (std::fabs(p.x - last.x) + std::fabs(p.y - last.y) >= spacing) {
                        batch.points.push_back(p);
                        last = p;
                    }
                }
                if (chunk.points.size() > 1) {
                    batch.points.push_back(view.worldToScreen(chunk.points.back()));
                }
            }
            batch.runLengths.push_back(static_cast<std::uint32_t>(batch.points.size() - before));
            out.stats.pointsOut += batch.points.size() - before;
        }
    }
    std::sort(out.batches.begin(), out.batches.end(),
              [](const PreviewBatch& a, const PreviewBatch& b) { return a.style < b.style; });
    return out;
}

std::optional<Vec2> PreviewModel::positionAfter(std::size_t globalMove) const {
    if (scene_.totalMoves == 0) {
        return std::nullopt;
    }
    const std::size_t index = globalMove == 0 ? 0 : std::min(globalMove, scene_.totalMoves) - 1;
    for (const PreviewToolpath& tp : scene_.toolpaths) {
        if (!tp.enabled || !tp.computed || tp.moveCount == 0) {
            continue;
        }
        if (index >= tp.firstMove && index < tp.firstMove + tp.moveCount) {
            const Move& m = (*tp.moves)[index - tp.firstMove];
            return Vec2{m.to.x, m.to.y};
        }
    }
    return std::nullopt;
}

}  // namespace carvetto::viewmodel
