#include "carvetto/viewmodel/keepout.hpp"

#include <cmath>
#include <memory>
#include <sstream>
#include <utility>

#include "carvetto/document/keepout_commands.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/offset.hpp"

namespace carvetto::viewmodel {

using document::Document;

namespace {

std::string mmText(double mm) {
    std::ostringstream os;
    os << mm;
    return os.str();
}

}  // namespace

bool canCreateKeepOutFromSelection(const Document& doc) {
    return !document::closedContoursOf(doc, doc.selection().ids()).empty();
}

KeepOutResult createKeepOutFromSelection(Document& doc) {
    KeepOutResult r;
    if (doc.selection().empty()) {
        r.message = "Select the closed vectors to use as keep-out zones first.";
        return r;
    }
    auto cmd = std::make_unique<document::CreateKeepOutZonesCommand>(doc, doc.selection().ids());
    const document::CreateKeepOutZonesCommand* raw = cmd.get();
    if (!doc.execute(std::move(cmd))) {
        r.message = document::closedContoursOf(doc, doc.selection().ids()).empty()
                        ? "The selection has no closed vectors: nothing to make a zone from."
                        : "The keep-out zone limit would be exceeded.";
        return r;
    }
    r.ok = true;
    r.count = raw->createdIds().size();
    r.message = "Created " + std::to_string(r.count) + " keep-out zone(s).";
    return r;
}

KeepOutResult clearKeepOutZones(Document& doc) {
    KeepOutResult r;
    const std::size_t n = doc.keepOutZones().size();
    if (n == 0 || !doc.execute(std::make_unique<document::RemoveKeepOutZonesCommand>(
                      document::RemoveKeepOutZonesCommand::all()))) {
        r.message = "There are no keep-out zones to clear.";
        return r;
    }
    r.ok = true;
    r.count = n;
    r.message = "Cleared " + std::to_string(n) + " keep-out zone(s).";
    return r;
}

KeepOutResult setKeepOutClearanceMm(Document& doc, double mm) {
    KeepOutResult r;
    if (!std::isfinite(mm) || mm < 0.0 || mm > document::kMaxKeepOutClearanceMm) {
        r.message = "The clearance must be between 0 and " + mmText(document::kMaxKeepOutClearanceMm) + " mm.";
        return r;
    }
    if (!doc.execute(std::make_unique<document::SetKeepOutClearanceCommand>(mm))) {
        r.message = "The clearance is already " + mmText(mm) + " mm.";
        return r;
    }
    r.ok = true;
    r.message = "Keep-out clearance set to " + mmText(mm) + " mm.";
    return r;
}

bool setKeepOutVisible(Document& doc, bool visible) {
    if (doc.keepOutVisible() == visible) {
        return false;
    }
    doc.setKeepOutVisible(visible);
    return true;
}

std::string describeKeepOut(const Document& doc) {
    const std::size_t n = doc.keepOutZones().size();
    return std::to_string(n) + (n == 1 ? " keep-out zone" : " keep-out zones") + ", clearance " +
           mmText(doc.keepOutClearance()) + " mm, " + (doc.keepOutVisible() ? "shown" : "hidden");
}

std::vector<geometry::Contour> keepOutOutlines(const Document& doc) {
    std::vector<geometry::Contour> out;
    out.reserve(doc.keepOutZones().size());
    for (const document::KeepOutZone& z : doc.keepOutZones()) {
        out.push_back(z.outline);
    }
    return out;
}

std::vector<geometry::Contour> keepOutHalo(const Document& doc) {
    if (doc.keepOutZones().empty() || !(doc.keepOutClearance() > 0.0)) {
        return {};
    }
    const geometry::Tolerance tol = geometry::Tolerance::defaults();
    std::vector<geometry::Contour> region = geometry::unionContours(keepOutOutlines(doc), tol);
    return geometry::offsetContours(region, doc.keepOutClearance(), geometry::JoinType::Round, tol);
}

// --- Routing -----------------------------------------------------------------------------------

const RoutedToolpath* RoutedJob::find(document::ToolpathId id) const {
    for (const RoutedToolpath& t : toolpaths) {
        if (t.id == id) {
            return &t;
        }
    }
    return nullptr;
}

void KeepOutRouter::clear() {
    zones_.clear();
    clearance_ = 0.0;
    field_.reset();
    cache_.clear();
    job_ = RoutedJob{};
    ++generation_;
}

const RoutedJob& KeepOutRouter::route(const Document& doc, const std::vector<document::ToolpathId>& ids) {
    job_ = RoutedJob{};
    if (doc.keepOutZones().empty()) {
        if (!zones_.empty() || field_) {
            clear();
        }
        return job_;
    }
    std::vector<geometry::Contour> zones = keepOutOutlines(doc);
    if (zones != zones_ || doc.keepOutClearance() != clearance_ || !field_) {
        zones_ = std::move(zones);
        clearance_ = doc.keepOutClearance();
        field_ = std::make_unique<toolpath::KeepOutField>(zones_, clearance_);
        cache_.clear();
        ++generation_;
    }
    std::optional<geometry::Vec3> at;
    std::map<document::ToolpathId, Entry> next;
    for (const document::ToolpathId id : ids) {
        const document::ToolpathRecord* rec = doc.findToolpath(id);
        if (rec == nullptr || !rec->hasMoves()) {
            continue;
        }
        if (!at.has_value()) {
            at = geometry::Vec3{0.0, 0.0, rec->safeZ()};
        }
        const std::optional<geometry::Vec3> from = at;
        const auto hit = cache_.find(id);
        if (hit != cache_.end() && hit->second.original == rec->moves && hit->second.from == from &&
            hit->second.generation == generation_) {
            next[id] = std::move(hit->second);
        } else {
            Entry e;
            e.original = rec->moves;
            e.from = from;
            e.generation = generation_;
            e.result.id = id;
            e.result.original = rec->moves;
            e.result.routed = rec->moves;
            toolpath::Toolpath tp = rec->toolpath();
            e.result.report = toolpath::routeRapids(tp, *field_, from);
            if (e.result.report.routed > 0) {
                std::uint32_t orig = 0;
                bool any = false;
                std::vector<std::uint32_t> index;
                index.reserve(tp.moves.size());
                for (const toolpath::Move& m : tp.moves) {
                    if (m.route != toolpath::RapidRoute::DetourNext) {
                        if (any) {
                            ++orig;
                        }
                        any = true;
                    }
                    index.push_back(orig);
                }
                e.result.originalIndex = std::make_shared<const std::vector<std::uint32_t>>(std::move(index));
                if (!tp.moves.empty() && tp.moves.front().route == toolpath::RapidRoute::DetourFirst) {
                    e.result.start = from;
                }
                e.result.routed = std::make_shared<const std::vector<toolpath::Move>>(std::move(tp.moves));
            }
            next[id] = std::move(e);
        }
        const Entry& e = next[id];
        if (!rec->moves->empty()) {
            at = rec->moves->back().to;
        }
        job_.total.routed += e.result.report.routed;
        job_.total.unroutable += e.result.report.unroutable;
        job_.total.tooComplex += e.result.report.tooComplex;
        job_.toolpaths.push_back(e.result);
    }
    cache_ = std::move(next);
    return job_;
}

}  // namespace carvetto::viewmodel
