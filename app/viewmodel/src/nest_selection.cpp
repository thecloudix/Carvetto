#include "carvetto/viewmodel/nest_selection.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <unordered_set>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Polyline;
using geometry::Vec2;

namespace {

constexpr double kChordalMm = 0.05;
constexpr double kMaxCoordinateMm = 1.0e9;
constexpr std::size_t kMaxLinkWork = 20'000'000;

bool finiteIn(double v, double limit) { return std::isfinite(v) && std::fabs(v) <= limit; }

bool usable(const Document& doc, const Object& obj) {
    const document::Layer* layer = doc.findLayer(obj.layer);
    return layer != nullptr && layer->visible && !layer->locked;
}

struct Member {
    ObjectId id;
    std::vector<Polyline> rings;  // closed outlines
    Box2 box;
};

bool inRing(const Polyline& r, Vec2 p) {
    bool in = false;
    const std::size_t n = r.points.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2 a = r.points[i];
        const Vec2 b = r.points[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

double side(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

bool crosses(Vec2 p1, Vec2 p2, Vec2 q1, Vec2 q2) {
    const double d1 = side(q1, q2, p1);
    const double d2 = side(q1, q2, p2);
    const double d3 = side(p1, p2, q1);
    const double d4 = side(p1, p2, q2);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

bool boxesOverlap(const Box2& a, const Box2& b) {
    return a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y;
}

enum class Rel { Apart, Inside, Crossing };

/// Relation of two closed outlines: edges cross, one contains the other, or they are apart.
Rel relation(const Polyline& a, const Polyline& b, std::size_t& work) {
    if (!boxesOverlap(a.bounds(), b.bounds())) return Rel::Apart;
    work += a.points.size() * b.points.size();
    if (work > kMaxLinkWork) return Rel::Crossing;  // too complex: take the conservative reading
    const std::size_t na = a.points.size();
    const std::size_t nb = b.points.size();
    for (std::size_t i = 0; i < na; ++i) {
        for (std::size_t j = 0; j < nb; ++j) {
            if (crosses(a.points[i], a.points[(i + 1) % na], b.points[j], b.points[(j + 1) % nb])) return Rel::Crossing;
        }
    }
    if (inRing(b, a.points[0]) || inRing(a, b.points[0])) return Rel::Inside;
    return Rel::Apart;
}

std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x != b.x ? a.x < b.x : a.y < b.y; });
    pts.erase(std::unique(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }), pts.end());
    if (pts.size() < 3) return pts;
    std::vector<Vec2> h(2 * pts.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && side(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
        h[k++] = pts[i];
    }
    for (std::size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && side(h[k - 2], h[k - 1], pts[i - 1]) <= 0) --k;
        h[k++] = pts[i - 1];
    }
    h.resize(k - 1);
    return h;
}

struct PartInfo {
    std::vector<std::size_t> members;   // indices into members
    std::vector<ObjectId> open;         // open vectors attached
    geometry::NestShape shape;
};

struct Collected {
    std::vector<Member> members;
    std::vector<ObjectId> openIds;
    std::vector<Vec2> openStart;
};

Collected collect(const Document& doc) {
    Collected c;
    for (ObjectId id : doc.selection().ids()) {
        const Object* obj = doc.findObject(id);
        if (obj == nullptr || obj->sheet != doc.activeSheet() || obj->parent.has_value() || !usable(doc, *obj)) continue;
        const document::VectorObject* vec = obj->asVector();
        if (vec == nullptr) continue;
        Member m;
        m.id = id;
        geometry::Tolerance tol;
        tol.chordal = kChordalMm;
        Vec2 firstOpen;
        bool haveOpen = false;
        for (const geometry::Contour& ct : vec->contours) {
            if (ct.isEmpty()) continue;
            if (ct.closed) {
                Polyline pl = ct.polygonise(tol);
                pl.closed = true;
                if (pl.points.size() >= 3) {
                    m.box.expand(pl.bounds());
                    m.rings.push_back(std::move(pl));
                }
            } else if (!haveOpen) {
                firstOpen = ct.startPoint();
                haveOpen = true;
            }
        }
        if (!m.rings.empty()) {
            c.members.push_back(std::move(m));
        } else if (haveOpen) {
            c.openIds.push_back(id);
            c.openStart.push_back(firstOpen);
        }
    }
    return c;
}

bool coordinatesOk(const Document& doc, const Collected& c) {
    for (const Member& m : c.members) {
        const Object* o = doc.findObject(m.id);
        const Box2 b = o->asVector()->bounds();
        if (!finiteIn(b.min.x, kMaxCoordinateMm) || !finiteIn(b.min.y, kMaxCoordinateMm) ||
            !finiteIn(b.max.x, kMaxCoordinateMm) || !finiteIn(b.max.y, kMaxCoordinateMm)) {
            return false;
        }
    }
    return true;
}

std::vector<PartInfo> buildParts(const Collected& c, bool& tooComplex) {
    const std::size_t n = c.members.size();
    std::vector<std::size_t> parent(n);
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    const auto find = [&](std::size_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    std::vector<char> crossing(n, 0);
    std::size_t work = 0;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            if (!boxesOverlap(c.members[i].box, c.members[j].box)) continue;
            Rel worst = Rel::Apart;
            for (const Polyline& a : c.members[i].rings) {
                for (const Polyline& b : c.members[j].rings) {
                    const Rel r = relation(a, b, work);
                    if (r == Rel::Crossing) worst = Rel::Crossing;
                    else if (r == Rel::Inside && worst == Rel::Apart) worst = Rel::Inside;
                }
            }
            if (worst != Rel::Apart) {
                const std::size_t ri = find(i);
                const std::size_t rj = find(j);
                if (ri != rj) parent[rj] = ri;
                if (worst == Rel::Crossing) crossing[i] = crossing[j] = 1;
            }
        }
    }
    tooComplex = work > kMaxLinkWork;

    std::vector<PartInfo> parts;
    std::vector<std::size_t> rootToPart(n, n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t r = find(i);
        if (rootToPart[r] == n) {
            rootToPart[r] = parts.size();
            parts.emplace_back();
        }
        parts[rootToPart[r]].members.push_back(i);
    }
    for (PartInfo& p : parts) {
        std::vector<const Polyline*> rings;
        bool cross = false;
        for (std::size_t m : p.members) {
            cross = cross || crossing[m] != 0;
            for (const Polyline& r : c.members[m].rings) rings.push_back(&r);
        }
        // Depth of every ring = number of other rings containing its first vertex.
        std::vector<int> depth(rings.size(), 0);
        std::vector<std::size_t> container(rings.size(), rings.size());
        std::vector<double> area(rings.size());
        for (std::size_t i = 0; i < rings.size(); ++i) area[i] = std::fabs(rings[i]->signedArea());
        if (!cross && rings.size() > 1) {
            for (std::size_t i = 0; i < rings.size(); ++i) {
                for (std::size_t j = 0; j < rings.size(); ++j) {
                    if (i == j) continue;
                    if (area[j] > area[i] && inRing(*rings[j], rings[i]->points[0])) {
                        ++depth[i];
                        if (container[i] == rings.size() || area[j] < area[container[i]]) container[i] = j;
                    }
                }
            }
        }
        std::size_t outer = 0;
        for (std::size_t i = 1; i < rings.size(); ++i) {
            if (area[i] > area[outer]) outer = i;
        }
        bool simple = !cross;
        for (int d : depth) simple = simple && d <= 1;
        if (simple) {
            std::size_t roots = 0;
            for (int d : depth) roots += d == 0 ? 1 : 0;
            simple = roots == 1;
        }
        if (simple) {
            p.shape.outer = *rings[outer];
            for (std::size_t i = 0; i < rings.size(); ++i) {
                if (i != outer) p.shape.holes.push_back(*rings[i]);
            }
        } else if (!cross) {
            p.shape.outer = *rings[outer];  // islands inside holes: holes dropped, the outer encloses them
        } else {
            std::vector<Vec2> pts;
            for (const Polyline* r : rings) pts.insert(pts.end(), r->points.begin(), r->points.end());
            p.shape.outer.points = convexHull(std::move(pts));
        }
        p.shape.outer.closed = true;
    }
    // Attach open vectors to the smallest part whose outline contains their start.
    for (std::size_t k = 0; k < c.openIds.size(); ++k) {
        std::size_t best = parts.size();
        double bestArea = 0.0;
        for (std::size_t i = 0; i < parts.size(); ++i) {
            const Polyline& o = parts[i].shape.outer;
            if (!inRing(o, c.openStart[k])) continue;
            const double a = std::fabs(o.signedArea());
            if (best == parts.size() || a < bestArea) {
                best = i;
                bestArea = a;
            }
        }
        if (best != parts.size()) parts[best].open.push_back(c.openIds[k]);
    }
    return parts;
}

document::CreationParams mapParams(const document::CreationParams& params, const geometry::Transform2& m) {
    if (const auto* text = std::get_if<document::TextParams>(&params)) {
        document::TextParams q = *text;
        q.placement = m * q.placement;
        return q;
    }
    return document::NoParams{};
}

Object transformedCopy(const Object& src, const geometry::Transform2& m, document::SheetId sheet) {
    Object obj = src;
    obj.id = {};
    obj.parent.reset();
    obj.sheet = sheet;
    if (document::VectorObject* vec = obj.asVector(); vec != nullptr) {
        vec->params = mapParams(vec->params, m);
        vec->transform(m);
        if (m.a * m.d - m.b * m.c < 0.0) {
            for (geometry::Contour& c : vec->contours) c.reverse();  // a mirror flips orientation
        }
    }
    return obj;
}

NestSelectionResult failure(std::string message) {
    NestSelectionResult r;
    r.message = std::move(message);
    return r;
}

}  // namespace

std::string validateNestOptions(const NestSelectionOptions& o) {
    for (double v : {o.toolDiameterMm, o.clearanceMm, o.borderGapMm}) {
        if (!(std::isfinite(v) && v >= 0.0 && v <= 1.0e6)) return "Spacing and border gap must be 0 or more (at most 1e6 mm).";
    }
    if (!std::isfinite(o.rotationStepRad) || o.rotationStepRad < 0.0 || o.rotationStepRad > 6.283185307179586) {
        return "The rotation step must be between 0 and 360 degrees.";
    }
    if (o.copies < 1 || o.copies > kMaxNestCopies) return "Copies must be between 1 and 1000.";
    if (o.maxSheets < 1 || o.maxSheets > 1000) return "The sheet limit must be between 1 and 1000.";
    for (const geometry::NestSheet& s : o.sheets) {
        if (!(std::isfinite(s.lengthMm) && std::isfinite(s.widthMm) && s.lengthMm > 0.0 && s.widthMm > 0.0 &&
              s.lengthMm <= 1.0e6 && s.widthMm <= 1.0e6)) {
            return "Sheet sizes must be positive (at most 1e6 mm).";
        }
    }
    return {};
}

bool canNestSelection(const Document& doc) {
    const Collected c = collect(doc);
    return !c.members.empty();
}

NestSelectionResult nestSelection(Document& doc, const NestSelectionOptions& o) {
    if (std::string bad = validateNestOptions(o); !bad.empty()) return failure(std::move(bad));
    const Collected c = collect(doc);
    if (c.members.empty()) {
        return failure("Select closed vectors to nest first (hidden and locked layers, groups and open vectors alone are skipped).");
    }
    if (c.members.size() + c.openIds.size() > kMaxNestObjects) return failure("Too many objects selected to nest (over 2000).");
    if (!coordinatesOk(doc, c)) return failure("The selection has coordinates that are not finite or beyond 1e9 mm.");
    bool tooComplex = false;
    std::vector<PartInfo> parts = buildParts(c, tooComplex);
    if (tooComplex) return failure("The selection is too complex to group into parts.");

    geometry::NestingOptions n;
    n.gapMm = o.toolDiameterMm + o.clearanceMm;
    n.marginMm = o.borderGapMm;
    n.rotationStepRad = o.rotationStepRad;
    n.mirror = o.mirror;
    n.allowInside = o.allowInside;
    n.corner = o.corner;
    n.direction = o.direction;
    n.chordalMm = kChordalMm;
    n.maxSheets = o.maxSheets;
    const document::Sheet* active = doc.findSheet(doc.activeSheet());
    if (active == nullptr) return failure("There is no active sheet.");

    if (o.area == NestArea::NewSheets) {
        n.sheets = o.sheets;
        if (n.sheets.empty()) n.sheets.push_back({active->sizeMm.x, active->sizeMm.y});
    } else {
        n.maxSheets = 1;
        if (o.area == NestArea::ActiveSheet) {
            const Box2 b = active->materialBoundsMm();
            Polyline r;
            r.closed = true;
            r.points = {b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}};
            n.boundary.push_back(std::move(r));
        } else {
            const std::unordered_set<ObjectId> selected(doc.selection().ids().begin(), doc.selection().ids().end());
            geometry::Tolerance tol;
            tol.chordal = kChordalMm;
            for (ObjectId id : doc.objectsOnSheet(doc.activeSheet())) {
                const Object* obj = doc.findObject(id);
                if (obj == nullptr || obj->layer != o.boundaryLayer || selected.contains(id) || obj->asVector() == nullptr) continue;
                for (const geometry::Contour& ct : obj->asVector()->contours) {
                    if (!ct.closed || ct.isEmpty()) continue;
                    Polyline pl = ct.polygonise(tol);
                    pl.closed = true;
                    if (pl.points.size() >= 3) n.boundary.push_back(std::move(pl));
                }
            }
            if (n.boundary.empty()) return failure("The boundary layer has no closed vectors on this sheet.");
        }
    }

    std::vector<geometry::NestPart> nestParts;
    nestParts.reserve(parts.size());
    for (const PartInfo& p : parts) {
        geometry::NestPart np;
        np.shape = p.shape;
        np.copies = static_cast<std::size_t>(o.copies);
        nestParts.push_back(std::move(np));
    }
    const geometry::NestingResult res = geometry::nestParts(nestParts, n);

    NestSelectionResult out;
    out.parts = parts.size();
    out.placed = res.placements.size();
    out.unplaced = res.unplaced.size();
    out.skippedOpen = c.openIds.size();
    for (const PartInfo& p : parts) out.skippedOpen -= p.open.size();
    if (res.placements.empty()) {
        out.message = res.cancelled ? "The nest was cancelled."
                      : res.budgetExhausted ? "The nest ran out of its work budget before placing anything."
                                            : "No part fits the sheet or boundary with this spacing.";
        return out;
    }

    // Count the ids needed before touching the document.
    std::uint64_t ids = o.area == NestArea::NewSheets ? res.sheetCount : 0;
    for (const geometry::NestPlacementResult& pl : res.placements) {
        ids += parts[pl.part].members.size() + parts[pl.part].open.size();
    }
    if (!doc.idGenerator().canAllocate(ids)) return failure("The document has no free object ids left.");

    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Nest selected vectors");
    std::vector<document::SheetId> sheetIds;
    if (o.area == NestArea::NewSheets) {
        for (std::size_t s = 0; s < res.sheetCount; ++s) {
            document::Sheet sheet = document::Sheet::fromJob(doc.job(), "Nest " + std::to_string(s + 1));
            const geometry::NestSheet& sz = n.sheets[std::min(s, n.sheets.size() - 1)];
            sheet.sizeMm = document::JobSize{sz.lengthMm, sz.widthMm, active->sizeMm.z};
            sheet.datum = document::Datum::BottomLeft;
            sheet.datumOffsetMm = {};
            sheet.zZero = active->zZero;
            auto cmd = std::make_unique<document::AddSheetCommand>(sheet, s == 0);
            const document::AddSheetCommand* raw = cmd.get();
            doc.execute(std::move(cmd));
            sheetIds.push_back(raw->id());
        }
        out.sheets = sheetIds;
    }
    const document::SheetId origSheet = doc.activeSheet();
    std::unordered_set<std::size_t> partsDone;
    for (const geometry::NestPlacementResult& pl : res.placements) {
        const document::SheetId target = o.area == NestArea::NewSheets ? sheetIds[pl.sheet] : origSheet;
        partsDone.insert(pl.part);
        std::vector<ObjectId> srcIds;
        for (std::size_t m : parts[pl.part].members) srcIds.push_back(c.members[m].id);
        srcIds.insert(srcIds.end(), parts[pl.part].open.begin(), parts[pl.part].open.end());
        for (ObjectId src : srcIds) {
            const Object* so = doc.findObject(src);
            if (so == nullptr) continue;
            auto cmd = std::make_unique<document::AddObjectCommand>(transformedCopy(*so, pl.transform, target));
            const document::AddObjectCommand* raw = cmd.get();
            if (doc.execute(std::move(cmd))) out.created.push_back(raw->id());
        }
    }
    if (o.removeOriginals) {
        std::vector<ObjectId> gone;
        for (std::size_t p : partsDone) {
            for (std::size_t m : parts[p].members) gone.push_back(c.members[m].id);
            gone.insert(gone.end(), parts[p].open.begin(), parts[p].open.end());
        }
        std::sort(gone.begin(), gone.end());
        if (!gone.empty()) doc.execute(std::make_unique<document::RemoveObjectsCommand>(gone));
    }
    doc.undoStack().endMacro();
    doc.selection().assign(out.created);
    out.ok = true;
    if (out.unplaced > 0) {
        out.message = std::to_string(out.unplaced) + " part(s) did not fit and were left out.";
    }
    return out;
}

}  // namespace carvetto::viewmodel
