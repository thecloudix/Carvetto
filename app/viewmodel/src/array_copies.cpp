#include "carvetto/viewmodel/array_copies.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_set>
#include <utility>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/transform2.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Transform2;
using geometry::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool finiteIn(double v, double limit) { return std::isfinite(v) && std::fabs(v) <= limit; }

bool spanFinite(const geometry::Span& span) {
    return std::visit(
        [](const auto& sp) {
            using T = std::decay_t<decltype(sp)>;
            if constexpr (std::is_same_v<T, geometry::LineSpan>) {
                return finiteIn(sp.start.x, kMaxCopyCoordinateMm) && finiteIn(sp.start.y, kMaxCopyCoordinateMm) &&
                       finiteIn(sp.end.x, kMaxCopyCoordinateMm) && finiteIn(sp.end.y, kMaxCopyCoordinateMm);
            } else if constexpr (std::is_same_v<T, geometry::ArcSpan>) {
                return finiteIn(sp.centre.x, kMaxCopyCoordinateMm) && finiteIn(sp.centre.y, kMaxCopyCoordinateMm) &&
                       finiteIn(sp.radius, kMaxCopyCoordinateMm) && std::isfinite(sp.startAngle) &&
                       std::isfinite(sp.endAngle);
            } else {
                return finiteIn(sp.p0.x, kMaxCopyCoordinateMm) && finiteIn(sp.p0.y, kMaxCopyCoordinateMm) &&
                       finiteIn(sp.p1.x, kMaxCopyCoordinateMm) && finiteIn(sp.p1.y, kMaxCopyCoordinateMm) &&
                       finiteIn(sp.p2.x, kMaxCopyCoordinateMm) && finiteIn(sp.p2.y, kMaxCopyCoordinateMm) &&
                       finiteIn(sp.p3.x, kMaxCopyCoordinateMm) && finiteIn(sp.p3.y, kMaxCopyCoordinateMm);
            }
        },
        span);
}

bool boxFinite(const Box2& b) {
    return finiteIn(b.min.x, kMaxCopyCoordinateMm) && finiteIn(b.min.y, kMaxCopyCoordinateMm) &&
           finiteIn(b.max.x, kMaxCopyCoordinateMm) && finiteIn(b.max.y, kMaxCopyCoordinateMm);
}

// --- Eligibility --------------------------------------------------------------------------------

bool layerUsable(const Document& doc, document::LayerId id) {
    const document::Layer* layer = doc.findLayer(id);
    return layer != nullptr && layer->visible && !layer->locked;
}

bool chainUsable(const Document& doc, const Object& obj) {
    const Object* cur = &obj;
    for (std::size_t guard = 0; guard <= doc.objectCount(); ++guard) {
        if (!layerUsable(doc, cur->layer)) {
            return false;
        }
        if (!cur->parent.has_value()) {
            return true;
        }
        cur = doc.findObject(*cur->parent);
        if (cur == nullptr) {
            return false;
        }
    }
    return false;
}

bool ancestorIn(const Document& doc, const Object& obj, const std::unordered_set<ObjectId>& set) {
    const Object* cur = &obj;
    for (std::size_t guard = 0; guard <= doc.objectCount(); ++guard) {
        if (!cur->parent.has_value()) {
            return false;
        }
        if (set.contains(*cur->parent)) {
            return true;
        }
        cur = doc.findObject(*cur->parent);
        if (cur == nullptr) {
            return false;
        }
    }
    return false;
}

// --- Source plan --------------------------------------------------------------------------------

struct Entry {
    Object proto;  ///< Id and parent cleared.
    int parent{-1};
};

struct Plan {
    std::vector<Entry> entries;  ///< Pre-order: a group precedes its children.
    Box2 bounds;
    std::size_t spans{0};
    std::size_t roots{0};
    bool badGeometry{false};  ///< A source has non-finite or out-of-range coordinates.

    std::size_t unitCost() const { return entries.size() + spans; }
};

bool addTree(const Document& doc, const Object& obj, int parent, Plan& plan, std::size_t depth) {
    if (depth > doc.objectCount()) {
        return false;
    }
    Entry entry;
    entry.proto = obj;
    entry.proto.id = {};
    entry.proto.parent.reset();
    entry.parent = parent;
    if (const document::GroupObject* group = obj.asGroup(); group != nullptr) {
        const std::size_t first = plan.entries.size();
        const int self = static_cast<int>(first);
        entry.proto.data = document::GroupObject{};
        plan.entries.push_back(std::move(entry));
        bool any = false;
        for (ObjectId child : group->children) {
            const Object* c = doc.findObject(child);
            if (c != nullptr && layerUsable(doc, c->layer)) {
                any = addTree(doc, *c, self, plan, depth + 1) || any;
            }
        }
        if (!any) {
            plan.entries.resize(first);
        }
        return any;
    }
    if (const document::VectorObject* vec = obj.asVector(); vec != nullptr) {
        for (const Contour& c : vec->contours) {
            plan.spans += c.spans.size();
            for (const geometry::Span& sp : c.spans) {
                plan.badGeometry = plan.badGeometry || !spanFinite(sp);
            }
        }
        plan.bounds.expand(vec->bounds());
    } else if (const document::BitmapObject* bmp = obj.asBitmap(); bmp != nullptr) {
        plan.badGeometry = plan.badGeometry || !boxFinite(bmp->bounds());
        plan.bounds.expand(bmp->bounds());
    }
    plan.entries.push_back(std::move(entry));
    return true;
}

/// The eligible selection (minus `exclude`) as a copy template.
Plan collectPlan(const Document& doc, ObjectId exclude) {
    Plan plan;
    const std::unordered_set<ObjectId> selected(doc.selection().ids().begin(), doc.selection().ids().end());
    for (ObjectId id : doc.selection().ids()) {
        const Object* obj = doc.findObject(id);
        if (obj == nullptr || id == exclude || obj->sheet != doc.activeSheet() || !chainUsable(doc, *obj) ||
            ancestorIn(doc, *obj, selected)) {
            continue;
        }
        if (addTree(doc, *obj, -1, plan, 0)) {
            ++plan.roots;
        }
    }
    return plan;
}

// --- Creation parameters under a transform ------------------------------------------------------

bool nearValue(double a, double b) { return std::fabs(a - b) < 1e-9; }

/// Quarter turns (0..3) when the linear part of `m` is a rotation by a multiple of 90 degrees.
int quarterTurns(const Transform2& m) {
    if (!nearValue(m.a, m.d) || !nearValue(m.b, -m.c) || !nearValue(m.a * m.a + m.b * m.b, 1.0)) {
        return -1;
    }
    if (nearValue(m.a, 1.0) && nearValue(m.c, 0.0)) {
        return 0;
    }
    if (nearValue(m.a, 0.0) && nearValue(m.c, 1.0)) {
        return 1;
    }
    if (nearValue(m.a, -1.0) && nearValue(m.c, 0.0)) {
        return 2;
    }
    if (nearValue(m.a, 0.0) && nearValue(m.c, -1.0)) {
        return 3;
    }
    return -1;
}

document::CreationParams transformedParams(const document::CreationParams& params, const Transform2& m) {
    const int turns = quarterTurns(m);
    if (turns < 0 || std::holds_alternative<document::NoParams>(params)) {
        return document::NoParams{};
    }
    const double turn = turns * (kPi / 2.0);
    return std::visit(
        [&](const auto& p) -> document::CreationParams {
            using T = std::decay_t<decltype(p)>;
            T q = p;
            if constexpr (std::is_same_v<T, document::NoParams>) {
                return q;
            } else {
                q.centreMm = m.apply(p.centreMm);
                if constexpr (std::is_same_v<T, document::RectangleParams>) {
                    if (turns % 2 == 1) {
                        std::swap(q.widthMm, q.heightMm);
                    }
                } else if constexpr (std::is_same_v<T, document::EllipseParams> ||
                                     std::is_same_v<T, document::PolygonParams> ||
                                     std::is_same_v<T, document::StarParams>) {
                    q.angle = p.angle + turn;
                }
                return q;
            }
        },
        params);
}

// --- Planned copies -----------------------------------------------------------------------------

struct Planned {
    Object obj;
    int parent{-1};
    bool top{false};
};

/// Appends a transformed copy of `plan`. Top-level entries hang under `groupParent` when >= 0.
void instantiate(const Plan& plan, const Transform2& m, int groupParent, std::vector<Planned>& out) {
    const int base = static_cast<int>(out.size());
    for (const Entry& e : plan.entries) {
        Planned p;
        p.obj = e.proto;
        if (document::VectorObject* vec = p.obj.asVector(); vec != nullptr) {
            vec->params = transformedParams(vec->params, m);
            vec->transform(m);
        } else if (document::BitmapObject* bmp = p.obj.asBitmap(); bmp != nullptr) {
            bmp->transform(m);
        }
        if (e.parent >= 0) {
            p.parent = base + e.parent;
        } else {
            p.parent = groupParent;
            p.top = groupParent < 0;
        }
        out.push_back(std::move(p));
    }
}

Planned makeGroup(const Plan& plan) {
    Planned g;
    g.obj = plan.entries.front().proto;
    g.obj.data = document::GroupObject{};
    g.obj.name.clear();
    g.top = true;
    return g;
}

/// Applies the plan as one undo step. Selection becomes the top-level copies.
CopyResult commit(Document& doc, const char* text, std::vector<Planned> planned, bool newLayer,
                  std::size_t copies) {
    CopyResult result;
    const std::uint64_t ids = planned.size() + (newLayer ? 1U : 0U);
    if (planned.empty() || !doc.idGenerator().canAllocate(ids)) {
        result.message = planned.empty() ? "There is nothing to copy." : "The document has no free object ids left.";
        return result;
    }
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro(text);
    document::LayerId layer;
    if (newLayer) {
        document::Layer fresh;
        fresh.name = "Copies";
        auto cmd = std::make_unique<document::AddLayerCommand>(std::move(fresh), false);
        const document::AddLayerCommand* raw = cmd.get();
        if (doc.execute(std::move(cmd))) {
            layer = raw->id();
        }
    }
    std::vector<ObjectId> created(planned.size());
    for (std::size_t i = 0; i < planned.size(); ++i) {
        Object obj = std::move(planned[i].obj);
        if (newLayer) {
            obj.layer = layer;
        }
        if (planned[i].parent >= 0) {
            obj.parent = created[static_cast<std::size_t>(planned[i].parent)];
        }
        obj.sheet = doc.activeSheet();
        auto cmd = std::make_unique<document::AddObjectCommand>(std::move(obj));
        const document::AddObjectCommand* raw = cmd.get();
        if (doc.execute(std::move(cmd))) {
            created[i] = raw->id();
        }
        if (planned[i].top && created[i]) {
            result.created.push_back(created[i]);
        }
    }
    doc.undoStack().endMacro();
    doc.selection().assign(result.created);
    result.copies = copies;
    result.ok = true;
    return result;
}

/// Empty when the work fits the cap.
std::string checkWork(double copies, const Plan& plan) {
    if (copyWorkExceeded(copies, plan.unitCost())) {
        return "The request is too large (copies times objects and spans over 500000).";
    }
    return {};
}

std::string checkSources(const Plan& plan) {
    if (plan.badGeometry || (!plan.bounds.isEmpty() && !boxFinite(plan.bounds))) {
        return "The selection has coordinates that are not finite or beyond 1e9 mm.";
    }
    if (plan.roots == 0 || plan.bounds.isEmpty()) {
        return "Select objects to copy first (hidden and locked layers are skipped).";
    }
    return {};
}

CopyResult failure(std::string message) {
    CopyResult r;
    r.message = std::move(message);
    return r;
}

}  // namespace

bool copyWorkExceeded(double copies, std::size_t unitCost) {
    return !(copies * static_cast<double>(unitCost) <= static_cast<double>(kMaxCopyWork));
}

bool canArrayCopy(const Document& doc) { return collectPlan(doc, {}).roots > 0; }
bool canCircularCopy(const Document& doc) { return collectPlan(doc, {}).roots > 0; }

ObjectId defaultGuideObject(const Document& doc) {
    const std::vector<ObjectId>& ids = doc.selection().ids();
    for (auto it = ids.rbegin(); it != ids.rend(); ++it) {
        const Object* obj = doc.findObject(*it);
        if (obj != nullptr && obj->isVector() && obj->sheet == doc.activeSheet()) {
            return *it;
        }
    }
    return {};
}

bool canCopyAlongVectors(const Document& doc) { return static_cast<bool>(defaultGuideObject(doc)); }

// --- Array --------------------------------------------------------------------------------------

std::string validateArrayOptions(const ArrayOptions& o) {
    if (o.rows < 1 || o.columns < 1 || o.rows > kMaxArrayLines || o.columns > kMaxArrayLines) {
        return "Rows and columns must be between 1 and 100000.";
    }
    if (static_cast<std::int64_t>(o.rows) * o.columns < 2) {
        return "An array needs at least two cells.";
    }
    if (static_cast<double>(o.rows) * static_cast<double>(o.columns) > static_cast<double>(kMaxCopyWork)) {
        return "The array has too many cells.";
    }
    for (double v : {o.xMm, o.yMm, o.rowDisplacementMm, o.columnDisplacementMm}) {
        if (!finiteIn(v, kMaxCopyCoordinateMm)) {
            return "Spacing and displacement must be finite numbers within 1e9 mm.";
        }
    }
    return {};
}

CopyResult applyArrayCopy(Document& doc, const ArrayOptions& o) {
    if (std::string bad = validateArrayOptions(o); !bad.empty()) {
        return failure(std::move(bad));
    }
    const Plan plan = collectPlan(doc, {});
    if (std::string bad = checkSources(plan); !bad.empty()) {
        return failure(std::move(bad));
    }
    const std::size_t cells = static_cast<std::size_t>(o.rows) * static_cast<std::size_t>(o.columns);
    const std::size_t copies = cells - 1;
    if (std::string bad = checkWork(static_cast<double>(copies), plan); !bad.empty()) {
        return failure(std::move(bad));
    }
    const double stepX = o.spacing == ArraySpacing::Gap ? plan.bounds.width() + o.xMm : o.xMm;
    const double stepY = o.spacing == ArraySpacing::Gap ? plan.bounds.height() + o.yMm : o.yMm;
    if (!finiteIn(stepX, kMaxCopyCoordinateMm) || !finiteIn(stepY, kMaxCopyCoordinateMm)) {
        return failure("The array spacing is out of range.");
    }
    if (static_cast<double>(o.columns) * std::fabs(stepX) > kMaxCopyCoordinateMm ||
        static_cast<double>(o.rows) * std::fabs(stepY) > kMaxCopyCoordinateMm) {
        return failure("The array extends beyond the supported range (1e9 mm).");
    }
    const Vec2 centre = plan.bounds.centre();
    const bool mirrorCols = o.symmetry == ArraySymmetry::MirrorColumns || o.symmetry == ArraySymmetry::MirrorBoth;
    const bool mirrorRows = o.symmetry == ArraySymmetry::MirrorRows || o.symmetry == ArraySymmetry::MirrorBoth;

    std::vector<Planned> planned;
    planned.reserve(copies * (plan.entries.size() + (o.groupCopies ? 1U : 0U)));
    for (int r = 0; r < o.rows; ++r) {
        for (int c = 0; c < o.columns; ++c) {
            if (r == 0 && c == 0) {
                continue;
            }
            const bool oddRow = (r % 2) == 1;
            const bool oddCol = (c % 2) == 1;
            const double dx = c * stepX + (oddRow ? o.rowDisplacementMm : 0.0);
            const double dy = r * stepY + (oddCol ? o.columnDisplacementMm : 0.0);
            double sx = (mirrorCols && oddCol) ? -1.0 : 1.0;
            double sy = (mirrorRows && oddRow) ? -1.0 : 1.0;
            if (o.symmetry == ArraySymmetry::Rotate180 && ((r + c) % 2) == 1) {
                sx = -1.0;
                sy = -1.0;
            }
            const Transform2 m = Transform2::translation({dx, dy}) * Transform2::scaling(sx, sy, centre);
            int group = -1;
            if (o.groupCopies) {
                group = static_cast<int>(planned.size());
                planned.push_back(makeGroup(plan));
            }
            instantiate(plan, m, group, planned);
        }
    }
    return commit(doc, "Array copy", std::move(planned), false, copies);
}

// --- Circular -----------------------------------------------------------------------------------

std::string validateCircularOptions(const CircularOptions& o) {
    if (o.count < 2) {
        return "Circular copy needs at least 2 copies (the original counts as one).";
    }
    if (o.count > kMaxCopyCount) {
        return "Too many copies requested (at most 100000).";
    }
    if (!finiteIn(o.centreMm.x, kMaxCopyCoordinateMm) || !finiteIn(o.centreMm.y, kMaxCopyCoordinateMm)) {
        return "The rotation centre must be finite and within 1e9 mm.";
    }
    if (!std::isfinite(o.angleRad) || o.angleRad == 0.0 || std::fabs(o.angleRad) > kTwoPi + 1e-12) {
        return "The angle must be non-zero and at most 360 degrees.";
    }
    return {};
}

CopyResult applyCircularCopy(Document& doc, const CircularOptions& o) {
    if (std::string bad = validateCircularOptions(o); !bad.empty()) {
        return failure(std::move(bad));
    }
    const Plan plan = collectPlan(doc, {});
    if (std::string bad = checkSources(plan); !bad.empty()) {
        return failure(std::move(bad));
    }
    const std::size_t copies = static_cast<std::size_t>(o.count) - 1;
    if (std::string bad = checkWork(static_cast<double>(copies), plan); !bad.empty()) {
        return failure(std::move(bad));
    }
    double step = o.angleRad;
    if (o.mode == CircularAngleMode::TotalAngle) {
        step = std::fabs(o.angleRad) >= kTwoPi - 1e-9 ? o.angleRad / o.count : o.angleRad / (o.count - 1);
    }
    const Vec2 ref = plan.bounds.centre();
    std::vector<Planned> planned;
    planned.reserve(copies * plan.entries.size());
    for (int i = 1; i < o.count; ++i) {
        const Transform2 turn = Transform2::rotation(-i * step, o.centreMm);  // positive = clockwise
        const Transform2 m = o.rotateCopies ? turn : Transform2::translation(turn.apply(ref) - ref);
        instantiate(plan, m, -1, planned);
    }
    return commit(doc, "Circular copy", std::move(planned), false, copies);
}

// --- Along vectors ------------------------------------------------------------------------------

namespace {

/// Position and direction along a contour by arc length.
class PathSampler {
   public:
    explicit PathSampler(const Contour& contour) : spans_(contour.spans) {
        double total = 0.0;
        for (const geometry::Span& s : spans_) {
            cum_.push_back(total);
            if (const auto* b = std::get_if<geometry::BezierSpan>(&s)) {
                tables_.emplace_back(buildTable(*b));
                total += tables_.back().back();
            } else {
                tables_.emplace_back();
                total += geometry::length(s);
            }
        }
        total_ = total;
    }

    double length() const { return total_; }

    void locate(double s, Vec2& point, Vec2& tangent) const {
        s = std::clamp(s, 0.0, total_);
        std::size_t i = static_cast<std::size_t>(std::upper_bound(cum_.begin(), cum_.end(), s) - cum_.begin());
        i = i == 0 ? 0 : i - 1;
        // Skip zero-length spans so the tangent is meaningful.
        const double local = s - cum_[i];
        const geometry::Span& span = spans_[i];
        double t = 0.0;
        if (const auto* b = std::get_if<geometry::BezierSpan>(&span)) {
            t = bezierParameter(*b, tables_[i], local);
        } else {
            const double len = geometry::length(span);
            t = len > 0.0 ? std::clamp(local / len, 0.0, 1.0) : 0.0;
        }
        point = geometry::pointAt(span, t);
        tangent = geometry::tangentAt(span, t);
    }

   private:
    static constexpr int kCells = 64;

    static double speed(const geometry::BezierSpan& b, double t) { return b.derivativeAt(t).length(); }

    /// Arc length of the curve on [t0, t1] by 3-point Gauss-Legendre.
    static double integrate(const geometry::BezierSpan& b, double t0, double t1) {
        constexpr double kNode = 0.7745966692414834;  // sqrt(3/5)
        const double mid = 0.5 * (t0 + t1);
        const double half = 0.5 * (t1 - t0);
        return half * ((5.0 / 9.0) * speed(b, mid - kNode * half) + (8.0 / 9.0) * speed(b, mid) +
                       (5.0 / 9.0) * speed(b, mid + kNode * half));
    }

    static std::vector<double> buildTable(const geometry::BezierSpan& b) {
        std::vector<double> table(kCells + 1, 0.0);
        for (int k = 0; k < kCells; ++k) {
            // Each cell is integrated in 4 pieces: plenty for the smooth cubic speed function.
            double sum = 0.0;
            for (int q = 0; q < 4; ++q) {
                sum += integrate(b, (k + q / 4.0) / kCells, (k + (q + 1) / 4.0) / kCells);
            }
            table[static_cast<std::size_t>(k) + 1] = table[static_cast<std::size_t>(k)] + sum;
        }
        return table;
    }

    static double bezierParameter(const geometry::BezierSpan& b, const std::vector<double>& table, double u) {
        const double total = table.back();
        if (total <= 0.0) {
            return 0.0;
        }
        u = std::clamp(u, 0.0, total);
        std::size_t k = static_cast<std::size_t>(std::upper_bound(table.begin(), table.end(), u) - table.begin());
        k = std::min(k == 0 ? 0 : k - 1, static_cast<std::size_t>(kCells - 1));
        const double t0 = static_cast<double>(k) / kCells;
        const double t1 = static_cast<double>(k + 1) / kCells;
        const double cell = table[k + 1] - table[k];
        double t = cell > 0.0 ? t0 + (u - table[k]) / cell * (t1 - t0) : t0;
        for (int it = 0; it < 4; ++it) {
            const double f = table[k] + integrate(b, t0, t) - u;
            const double v = speed(b, t);
            if (v <= 1e-12) {
                break;
            }
            t = std::clamp(t - f / v, t0, t1);
        }
        return t;
    }

    std::vector<geometry::Span> spans_;
    std::vector<double> cum_;
    std::vector<std::vector<double>> tables_;
    double total_{0.0};
};

}  // namespace

std::string validateAlongVectorOptions(const AlongVectorOptions& o) {
    if (o.kind == AlongCopyKind::Circles &&
        (!finiteIn(o.circleDiameterMm, kMaxCopyCoordinateMm) || o.circleDiameterMm <= 0.0)) {
        return "The circle diameter must be a positive number.";
    }
    if (o.spacing == AlongSpacing::Distance) {
        if (!finiteIn(o.distanceMm, kMaxCopyCoordinateMm) || o.distanceMm <= 0.0) {
            return "The distance between copies must be a positive number.";
        }
    } else if (o.count < 1 || o.count > kMaxCopyCount) {
        return "The number of copies must be between 1 and 100000.";
    }
    return {};
}

CopyResult applyCopyAlongVectors(Document& doc, const AlongVectorOptions& o) {
    if (std::string bad = validateAlongVectorOptions(o); !bad.empty()) {
        return failure(std::move(bad));
    }
    const Object* guide = o.guide ? doc.findObject(o.guide) : nullptr;
    if (guide == nullptr || !guide->isVector() || guide->sheet != doc.activeSheet()) {
        return failure("Select the vector to copy along.");
    }
    const Contour* path = nullptr;
    for (const Contour& c : guide->asVector()->contours) {
        if (!c.spans.empty()) {
            path = &c;
            break;
        }
    }
    if (path == nullptr) {
        return failure("The chosen vector has no geometry.");
    }
    Plan plan;
    if (o.kind == AlongCopyKind::Objects) {
        plan = collectPlan(doc, o.guide);
        if (std::string bad = checkSources(plan); !bad.empty()) {
            return failure(std::move(bad));
        }
    } else if (!layerUsable(doc, doc.activeLayer())) {
        return failure("The active layer is hidden or locked.");
    }

    const Contour oriented = o.reverse ? path->reversed() : *path;
    for (const geometry::Span& sp : oriented.spans) {
        if (!spanFinite(sp)) {
            return failure("The chosen vector has coordinates outside the supported range or not finite.");
        }
    }
    const PathSampler sampler(oriented);
    const double len = sampler.length();
    if (!(len > 1e-9)) {
        return failure("The chosen vector has no length.");
    }
    const bool closed = oriented.closed;

    // Positions as arc lengths.
    std::vector<double> at;
    if (o.spacing == AlongSpacing::Count) {
        const int n = o.count;
        if (closed) {
            for (int k = 0; k < n; ++k) {
                at.push_back(len * k / n);
            }
        } else if (n == 1) {
            at.push_back(0.0);
        } else {
            for (int k = 0; k < n; ++k) {
                at.push_back(k == n - 1 ? len : len * k / (n - 1));
            }
        }
    } else {
        const double ratio = len / o.distanceMm;
        if (ratio > static_cast<double>(kMaxCopyWork)) {
            return failure("The request is too large (too many copies along the vector).");
        }
        if (o.forceEvenSpacing) {
            const long long n = std::max(1LL, std::llround(ratio));
            const long long positions = closed ? n : n + 1;
            for (long long k = 0; k < positions; ++k) {
                at.push_back(k == n ? len : len * static_cast<double>(k) / static_cast<double>(n));
            }
        } else if (closed) {
            const long long m = std::max(1LL, static_cast<long long>(std::ceil(ratio - 1e-9)));
            for (long long k = 0; k < m; ++k) {
                at.push_back(o.distanceMm * static_cast<double>(k));
            }
        } else {
            const long long m = static_cast<long long>(std::floor(ratio + 1e-9)) + 1;
            for (long long k = 0; k < m; ++k) {
                at.push_back(std::min(o.distanceMm * static_cast<double>(k), len));
            }
        }
    }
    Plan costPlan = plan;
    if (o.kind == AlongCopyKind::Circles) {
        costPlan.entries.resize(1);
        costPlan.spans = 1;
    }
    if (std::string bad = checkWork(static_cast<double>(at.size()), costPlan); !bad.empty()) {
        return failure(std::move(bad));
    }

    std::vector<Planned> planned;
    planned.reserve(at.size() * costPlan.entries.size());
    const Vec2 ref = plan.bounds.centre();
    for (double s : at) {
        Vec2 point;
        Vec2 tangent;
        sampler.locate(s, point, tangent);
        if (o.kind == AlongCopyKind::Circles) {
            const double r = 0.5 * o.circleDiameterMm;
            Planned p;
            p.obj = Object::vector(Contour::circle(point, r), doc.activeLayer(), doc.activeSheet(),
                                   document::CircleParams{point, r});
            p.top = true;
            planned.push_back(std::move(p));
            continue;
        }
        Transform2 m = Transform2::translation(point - ref);
        if (o.alignToCurve && (tangent.x != 0.0 || tangent.y != 0.0)) {
            m = Transform2::translation(point) * Transform2::rotation(std::atan2(tangent.y, tangent.x)) *
                Transform2::translation(ref * -1.0);
        }
        instantiate(plan, m, -1, planned);
    }
    const std::size_t copies = at.size();
    return commit(doc, "Copy along vector", std::move(planned), o.newLayer, copies);
}

}  // namespace carvetto::viewmodel
