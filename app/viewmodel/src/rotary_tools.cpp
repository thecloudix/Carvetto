#include "carvetto/viewmodel/rotary_tools.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/cylinder_map.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/relief/mesh_raster.hpp"
#include "carvetto/relief/rotary_raster.hpp"

namespace carvetto::viewmodel {

namespace {

using document::Document;

bool isRotary(const Document& doc) {
    return doc.job().type == document::JobType::Rotary && doc.job().radiusMm() > 0.0;
}

geometry::CylinderMap mapOf(const document::Job& job) {
    geometry::CylinderMap m;
    m.alongX = job.rotaryAxis == document::RotaryAxis::X;
    m.radius = job.radiusMm();
    m.flip = job.flipDesign;
    return m;
}

geometry::Box2 flatArea(const document::Job& job) {
    return document::materialBounds(job.datum, job.sizeMm.x, job.sizeMm.y, job.datumOffsetMm);
}

}  // namespace

VectorOpResult unwrapSelectedVectors(Document& doc, const UnwrapperSettings& s) {
    VectorOpResult result;
    if (!isRotary(doc)) {
        result.message = "The Vector Unwrapper needs a rotary job.";
        return result;
    }
    if (s.refine && !(std::isfinite(s.tolerance) && s.tolerance > 0.0)) {
        result.message = "The tolerance must be a positive number.";
        return result;
    }
    const document::Job& job = doc.job();
    const geometry::Box2 area = flatArea(job);
    geometry::UnwrapOptions o;
    o.centre = s.centre;
    o.customCentre = s.customCentre;
    o.workspaceCentre = (area.min + area.max) * 0.5;
    o.radius = job.radiusMm();
    o.flip = job.flipDesign;
    o.refine = s.refine;
    o.tolerance = s.tolerance;
    geometry::Tolerance flatten = geometry::Tolerance::defaults();
    if (s.refine) {
        flatten.chordal = s.tolerance;
    }
    struct Pending {
        std::vector<geometry::Contour> contours;
        document::LayerId layer;
    };
    std::vector<Pending> pending;
    for (const document::ObjectId id : doc.selection().ids()) {
        const document::Object* obj = doc.findObject(id);
        const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
        if (vec == nullptr || obj->sheet != doc.activeSheet()) {
            continue;
        }
        Pending p;
        p.layer = obj->layer;
        for (const geometry::Contour& c : vec->contours) {
            geometry::Polyline poly = c.polygonise(flatten);
            if (poly.points.size() < 2) {
                continue;
            }
            auto r = geometry::unwrapCrossSection(poly, o);
            if (!r.ok()) {
                result.message = r.error();
                return result;
            }
            p.contours.push_back(geometry::Contour::fromPolyline(r.value().contour));
        }
        if (!p.contours.empty()) {
            pending.push_back(std::move(p));
        }
    }
    if (pending.empty()) {
        result.message = "Select one or more vectors to unwrap.";
        return result;
    }
    if (!doc.idGenerator().canAllocate(pending.size())) {
        result.message = "The document has no free object ids left.";
        return result;
    }
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Unwrap vectors");
    for (Pending& p : pending) {
        auto cmd = std::make_unique<document::AddObjectCommand>(
            document::Object::vector(std::move(p.contours), p.layer, doc.activeSheet()));
        const document::AddObjectCommand* raw = cmd.get();
        if (doc.execute(std::move(cmd))) {
            result.created.push_back(raw->id());
        }
    }
    doc.undoStack().endMacro();
    doc.selection().assign(result.created);
    result.ok = !result.created.empty();
    if (!result.ok) {
        result.message = "The unwrapped vectors could not be added.";
    }
    return result;
}

MeshImportOutcome importMeshIntoRotaryJob(Document& doc, document::SheetId sheet, const io::TriangleMesh& original,
                                          const RotaryImportSettings& s, const std::string& name) {
    MeshImportOutcome out;
    auto fail = [&out](std::string message) {
        out.ok = false;
        out.error = std::move(message);
        return out;
    };
    if (!isRotary(doc)) {
        return fail("This import needs a rotary job.");
    }
    const document::Job& job = doc.job();
    const geometry::CylinderMap map = mapOf(job);
    const geometry::Box2 area = flatArea(job);
    const double cell = s.mesh.cellSize;
    if (!(std::isfinite(cell) && cell > 0.0)) {
        return fail("The cell size must be a positive number.");
    }
    if (s.moveAxis && !(std::isfinite(s.axisShift.x) && std::isfinite(s.axisShift.y))) {
        return fail("The axis movement must be numbers.");
    }
    if (!std::isfinite(s.alongOffset)) {
        return fail("The start along the axis must be a number.");
    }
    MeshImportSettings prep = s.mesh;
    prep.centre = false;  // placement is decided here
    auto prepared = prepareMesh(original, prep);
    if (!prepared.ok()) {
        return fail(prepared.error());
    }
    io::TriangleMesh& mesh = prepared.value();

    relief::Grid grid;
    grid.cellSize = cell;
    grid.width = static_cast<std::size_t>(std::ceil((area.max.x - area.min.x) / cell));
    grid.height = static_cast<std::size_t>(std::ceil((area.max.y - area.min.y) / cell));
    grid.origin = {area.min.x + 0.5 * cell, area.min.y + 0.5 * cell};

    relief::Component c;
    c.name = name.empty() ? "Model" : name;
    if (s.type == RotaryModelType::Flat) {
        // The mesh is modelled flat and wrapped by the job: raster it onto the flat design area, centred on it.
        if (s.mesh.centre) {
            const io::Vec3 mc = io::meshBounds(mesh).centre();
            for (io::Vec3& v : mesh.vertices) {
                v.x += 0.5 * (area.min.x + area.max.x) - mc.x;
                v.y += 0.5 * (area.min.y + area.max.y) - mc.y;
            }
        }
        relief::MeshRasterOptions o;
        o.cellSize = cell;
        o.zeroPlane = effectiveZeroPlane(io::meshBounds(mesh), s.mesh);
        o.discardBelowZero = s.mesh.discardBelowZero;
        o.bothSides = false;
        o.detectUndercuts = true;
        o.grid = grid;
        auto raster = relief::meshToHeightField(mesh, o);
        if (!raster.ok()) {
            return fail(raster.error());
        }
        c.field = std::move(raster.value().top);
        out.undercutCells = raster.value().undercutCount;
    } else {
        const bool alongX = map.alongX;
        const auto bounds = [&mesh]() { return io::meshBounds(mesh); };
        if (s.fitToMaterial) {
            // Uniform scale about the middle of the model so that it fits the length and the radius.
            const io::MeshBounds b = bounds();
            const io::Vec3 mid = b.centre();
            const double ay = alongX ? mid.y : mid.x;
            double rMax = 0.0;
            for (const io::Vec3& v : mesh.vertices) {
                rMax = std::max(rMax, std::hypot((alongX ? v.y : v.x) - ay, v.z - mid.z));
            }
            const double extent = alongX ? b.size().x : b.size().y;
            double k = extent > 0.0 ? job.lengthMm() / extent : 1.0;
            if (rMax > 0.0) {
                k = std::min(k, job.radiusMm() / rMax);
            }
            if (k > 0.0 && std::isfinite(k)) {
                for (io::Vec3& v : mesh.vertices) {
                    v.x = mid.x + (v.x - mid.x) * k;
                    v.y = mid.y + (v.y - mid.y) * k;
                    v.z = mid.z + (v.z - mid.z) * k;
                }
            }
        }
        const io::MeshBounds b = bounds();
        const io::Vec3 mid = b.centre();
        relief::RotaryRasterOptions o;
        o.map = map;
        o.grid = grid;
        o.axisPoint = {(alongX ? mid.y : mid.x) + (s.moveAxis ? s.axisShift.x : 0.0),
                       mid.z + (s.moveAxis ? s.axisShift.y : 0.0)};
        o.meshAlongOrigin = alongX ? b.min.x : b.min.y;
        const double extent = alongX ? b.size().x : b.size().y;
        const double areaStart = alongX ? area.min.x : area.min.y;
        const double areaLength = alongX ? area.max.x - area.min.x : area.max.y - area.min.y;
        o.alongStart = areaStart + (s.centreAlong ? 0.5 * (areaLength - extent) : s.alongOffset);
        o.zeroRadius = 0.0;
        auto raster = relief::meshToRotaryField(mesh, o);
        if (!raster.ok()) {
            return fail(raster.error());
        }
        c.field = std::move(raster.value().field);
    }
    MeshImportOutcome added = addImportedComponent(doc, sheet, std::move(c), "Import 3D model into rotary job");
    added.undercutCells = out.undercutCells;
    return added;
}

}  // namespace carvetto::viewmodel
