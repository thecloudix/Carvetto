// T-0321: Wrapping level effect actions, Vector Unwrapper form and rotary 3D import (ADR-0114).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/relief_effects.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"
#include "carvetto/viewmodel/rotary_tools.hpp"
#include "carvetto/viewmodel/toolpath_calc.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;
using Catch::Approx;

namespace {

constexpr double kPi = geometry::kPi;

Document rotaryDoc(RotaryAxis axis = RotaryAxis::X, bool flip = false) {
    return Document(Job::rotary(40.0, 20.0, Units::Millimetres, axis, ZZero::CylinderAxis, flip));
}

LevelId addLevel(Document& doc) {
    LevelId id;
    REQUIRE(addReliefLevel(doc, doc.activeSheet(), {}, &id).changed);
    return id;
}

ObjectId addVector(Document& doc, geometry::Contour contour) {
    Object o;
    o.layer = doc.defaultLayer();
    o.sheet = doc.activeSheet();
    VectorObject v;
    v.contours.push_back(std::move(contour));
    o.data = std::move(v);
    auto cmd = std::make_unique<AddObjectCommand>(std::move(o));
    const AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

io::TriangleMesh prism(int n, double rc, double x0, double x1) {
    io::TriangleMesh m;
    for (const double x : {x0, x1}) {
        for (int k = 0; k < n; ++k) {
            const double a = 2.0 * kPi * k / n;
            m.vertices.push_back({x, rc * std::sin(a), rc * std::cos(a)});
        }
    }
    m.vertices.push_back({x0, 0, 0});
    m.vertices.push_back({x1, 0, 0});
    const auto c0 = static_cast<std::uint32_t>(2 * n), c1 = c0 + 1;
    for (int k = 0; k < n; ++k) {
        const auto a = static_cast<std::uint32_t>(k), b = static_cast<std::uint32_t>((k + 1) % n);
        const auto a1 = a + static_cast<std::uint32_t>(n), b1 = b + static_cast<std::uint32_t>(n);
        m.triangles.push_back({a, b, b1});
        m.triangles.push_back({a, b1, a1});
        m.triangles.push_back({c0, b, a});
        m.triangles.push_back({c1, a1, b1});
    }
    return m;
}

double polygonRadius(int n, double rc, double theta) {
    const double step = 2.0 * kPi / n;
    const double t = theta - step * std::floor(theta / step) - step / 2.0;
    return rc * std::cos(kPi / n) / std::cos(t);
}

const relief::Component& lastComponent(const Document& doc, LevelId level) {
    return doc.findReliefLevel(level)->components.back().data;
}

}  // namespace

TEST_CASE("wrapping actions: rotary jobs only; circumference and start come from the job", "[viewmodel][relief][effects][wrapping]") {
    {
        Document flat;
        const LevelId level = addLevel(flat);
        const EditResult r = setLevelWrapping(flat, level, true);
        CHECK_FALSE(r.ok);
        CHECK(r.message.find("rotary") != std::string::npos);
        CHECK_FALSE(flat.findReliefLevel(level)->effects.any());
    }
    Document doc = rotaryDoc();
    const LevelId level = addLevel(doc);
    CHECK_FALSE(setLevelWrapping(doc, LevelId{}, true).ok);
    CHECK_FALSE(setLevelWrapping(doc, level, false).ok);  // not wrapped yet
    REQUIRE(setLevelWrapping(doc, level, true).ok);
    const relief::Wrapping& w = doc.findReliefLevel(level)->effects.wrapping;
    CHECK(w.enabled);
    CHECK(w.circumferenceIsY);
    CHECK(w.circumference == Approx(kPi * 20.0));
    CHECK(w.start == 0.0);
    CHECK(summarizeEffects(doc, level)->wrapping);
    CHECK(summarizeEffects(doc, level)->text().find("wrapped") != std::string::npos);
    CHECK_FALSE(setLevelWrapping(doc, level, true).ok);  // already
    REQUIRE(setLevelWrapping(doc, level, false).ok);
    CHECK_FALSE(doc.findReliefLevel(level)->effects.any());
    CHECK(summarizeEffects(doc, level)->text().find("wrapped") == std::string::npos);
    REQUIRE(doc.undo());
    CHECK(doc.findReliefLevel(level)->effects.wrapping.enabled);

    Document alongY = rotaryDoc(RotaryAxis::Y);
    const LevelId ly = addLevel(alongY);
    REQUIRE(setLevelWrapping(alongY, ly, true).ok);
    CHECK_FALSE(alongY.findReliefLevel(ly)->effects.wrapping.circumferenceIsY);
    CHECK(alongY.findReliefLevel(ly)->effects.wrapping.circumference == Approx(kPi * 20.0));
}

TEST_CASE("wrapping actions: the window starts where the design area does; the model fingerprint sees the effect", "[viewmodel][relief][effects][wrapping]") {
    Job j = Job::rotary(40.0, 20.0, Units::Millimetres, RotaryAxis::X, ZZero::CylinderAxis);
    j.datumOffsetMm = {5.0, 7.0};
    Document doc(j);
    const LevelId level = addLevel(doc);
    const std::uint64_t before = modelFingerprint(doc, doc.activeSheet());
    REQUIRE(setLevelWrapping(doc, level, true).ok);
    CHECK(doc.findReliefLevel(level)->effects.wrapping.start == Approx(7.0));
    const std::uint64_t wrapped = modelFingerprint(doc, doc.activeSheet());
    CHECK(wrapped != before);
    REQUIRE(doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(level, [] {
        relief::Wrapping w;
        w.enabled = true;
        w.circumference = 30.0;
        return w;
    }())));
    CHECK(modelFingerprint(doc, doc.activeSheet()) != wrapped);
    Job jy = Job::rotary(40.0, 20.0, Units::Millimetres, RotaryAxis::Y, ZZero::CylinderAxis);
    jy.datumOffsetMm = {5.0, 7.0};
    Document docY(jy);
    const LevelId ly = addLevel(docY);
    REQUIRE(setLevelWrapping(docY, ly, true).ok);
    CHECK(docY.findReliefLevel(ly)->effects.wrapping.start == Approx(5.0));
}

TEST_CASE("vector unwrapper: unwraps the selected vectors into new objects, one undo step", "[viewmodel][rotary][unwrapper]") {
    Document doc = rotaryDoc();
    UnwrapperSettings s;
    CHECK_FALSE(unwrapSelectedVectors(doc, s).ok);  // nothing selected
    const ObjectId circle = addVector(doc, geometry::Contour::circle({20.0, 30.0}, 6.0));
    doc.selection().assign({circle});
    const std::size_t before = doc.objects().size();
    const VectorOpResult r = unwrapSelectedVectors(doc, s);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 1);
    CHECK(doc.objects().size() == before + 1);
    CHECK(doc.objects().count(circle) == 1);  // the original stays
    const geometry::Contour& out = doc.findObject(r.created[0])->asVector()->contours[0];
    CHECK_FALSE(out.closed);  // one full turn round the centre
    // Contour centre = the circle's centre; radius 6 on a cylinder of radius 10: x runs 2 pi 10, y = 6.
    CHECK(out.startPoint().y == Approx(6.0).margin(0.01));  // the bounds of the flattened circle centre it to within the chordal tolerance
    CHECK(out.endPoint().y == Approx(6.0).margin(0.01));
    CHECK(out.endPoint().x - out.startPoint().x == Approx(2.0 * kPi * 10.0).margin(1e-6));
    CHECK(doc.selection().ids() == r.created);
    REQUIRE(doc.undo());
    CHECK(doc.objects().size() == before);

    // Workspace centre: the middle of the design area (20, 31.4159); a circle round it has distance 6 too.
    UnwrapperSettings ws;
    ws.centre = geometry::UnwrapCentre::Workspace;
    doc.selection().assign({circle});
    const VectorOpResult w = unwrapSelectedVectors(doc, ws);
    REQUIRE(w.ok);
    const geometry::Contour& wo = doc.findObject(w.created[0])->asVector()->contours[0];
    const double dist = std::hypot(26.0 - 20.0, 30.0 - kPi * 10.0);  // circle start (26, 30)
    CHECK(wo.startPoint().y == Approx(dist).margin(1e-6));

    // Custom centre far away: the circle does not enclose it, so the result stays closed.
    UnwrapperSettings cu;
    cu.centre = geometry::UnwrapCentre::Custom;
    cu.customCentre = {-50.0, 0.0};
    doc.selection().assign({circle});
    const VectorOpResult c = unwrapSelectedVectors(doc, cu);
    REQUIRE(c.ok);
    CHECK(doc.findObject(c.created[0])->asVector()->contours[0].closed);

    UnwrapperSettings bad;
    bad.tolerance = 0.0;
    doc.selection().assign({circle});
    CHECK_FALSE(unwrapSelectedVectors(doc, bad).ok);

    Document flat;
    const ObjectId o = addVector(flat, geometry::Contour::circle({0, 0}, 2.0));
    flat.selection().assign({o});
    const VectorOpResult f = unwrapSelectedVectors(flat, s);
    CHECK_FALSE(f.ok);
    CHECK(f.message.find("rotary") != std::string::npos);
}

TEST_CASE("rotary import: a full 3D model is mapped onto the cylinder and centred along the axis", "[viewmodel][rotary][import]") {
    Document doc = rotaryDoc();  // length 40, diameter 20, circumference 62.83
    RotaryImportSettings s;
    s.mesh.cellSize = 1.0;
    const int n = 8;
    const io::TriangleMesh mesh = prism(n, 5.0, 0.0, 20.0);
    const MeshImportOutcome r = importMeshIntoRotaryJob(doc, doc.activeSheet(), mesh, s, "Pole");
    REQUIRE(r.ok);
    const std::vector<LevelId> levels = doc.reliefLevelsOnSheet(doc.activeSheet());
    REQUIRE(levels.size() == 1);
    const relief::Component& c = lastComponent(doc, levels[0]);
    CHECK(c.name == "Pole");
    CHECK(c.field.width() == 40);
    CHECK(c.field.height() == 63);
    CHECK(c.field.cellSize() == 1.0);
    for (std::size_t j = 0; j < 63; ++j) {
        const double theta = (0.5 + static_cast<double>(j)) / 10.0;
        for (std::size_t i = 0; i < 40; ++i) {
            if (i < 10 || i >= 30) {
                CHECK(std::isnan(c.field.at(i, j)));  // the model is centred: flat x 10 .. 30
            } else {
                CHECK(c.field.at(i, j) == Approx(polygonRadius(n, 5.0, theta)).margin(1e-4));
            }
        }
    }
    REQUIRE(doc.undo());
    CHECK(doc.reliefLevelsOnSheet(doc.activeSheet()).empty());  // the level made by the import goes with it
}

TEST_CASE("rotary import: fit to material, placement, orientation by axis and flip", "[viewmodel][rotary][import]") {
    const int n = 8;
    const io::TriangleMesh mesh = prism(n, 5.0, 0.0, 20.0);
    {  // fit: length 20 -> 40 and radius 5 -> 10 (scale 2)
        Document doc = rotaryDoc();
        RotaryImportSettings s;
        s.mesh.cellSize = 1.0;
        s.fitToMaterial = true;
        REQUIRE(importMeshIntoRotaryJob(doc, doc.activeSheet(), mesh, s, "").ok);
        const relief::Component& c = lastComponent(doc, doc.reliefLevelsOnSheet(doc.activeSheet())[0]);
        CHECK(c.name == "Model");
        CHECK_FALSE(std::isnan(c.field.at(0, 0)));
        CHECK_FALSE(std::isnan(c.field.at(39, 10)));
        CHECK(c.field.at(20, 4) == Approx(polygonRadius(n, 10.0, 4.5 / 10.0)).margin(1e-4));
    }
    {  // not centred: starts alongOffset from the start of the design area
        Document doc = rotaryDoc();
        RotaryImportSettings s;
        s.mesh.cellSize = 1.0;
        s.centreAlong = false;
        s.alongOffset = 4.0;
        REQUIRE(importMeshIntoRotaryJob(doc, doc.activeSheet(), mesh, s, "m").ok);
        const relief::Component& c = lastComponent(doc, doc.reliefLevelsOnSheet(doc.activeSheet())[0]);
        CHECK(std::isnan(c.field.at(3, 0)));
        CHECK_FALSE(std::isnan(c.field.at(4, 0)));
        CHECK_FALSE(std::isnan(c.field.at(23, 0)));
        CHECK(std::isnan(c.field.at(24, 0)));
    }
    {  // axis along Y: the flat design area is circumference x length; the mesh lies along its Y
        Document doc = rotaryDoc(RotaryAxis::Y);
        RotaryImportSettings s;
        s.mesh.cellSize = 1.0;
        io::TriangleMesh m = mesh;
        for (io::Vec3& v : m.vertices) v = {v.y, v.x, v.z};
        REQUIRE(importMeshIntoRotaryJob(doc, doc.activeSheet(), m, s, "y").ok);
        const relief::Component& c = lastComponent(doc, doc.reliefLevelsOnSheet(doc.activeSheet())[0]);
        CHECK(c.field.width() == 63);
        CHECK(c.field.height() == 40);
        CHECK(c.field.at(5, 15) == Approx(polygonRadius(n, 5.0, 5.5 / 10.0)).margin(1e-4));
        CHECK(std::isnan(c.field.at(5, 5)));
    }
    {  // Flip Design reverses the angle
        Document doc = rotaryDoc(RotaryAxis::X, true);
        RotaryImportSettings s;
        s.mesh.cellSize = 1.0;
        s.mesh.orientation = {};
        REQUIRE(importMeshIntoRotaryJob(doc, doc.activeSheet(), mesh, s, "f").ok);
        const relief::Component& c = lastComponent(doc, doc.reliefLevelsOnSheet(doc.activeSheet())[0]);
        CHECK(c.field.at(15, 7) == Approx(polygonRadius(n, 5.0, -7.5 / 10.0)).margin(1e-4));
    }
    {  // Rotation Axis Movement: moving the axis changes what is seen
        Document doc = rotaryDoc();
        RotaryImportSettings s;
        s.mesh.cellSize = 1.0;
        s.moveAxis = true;
        s.axisShift = {1.0, 0.0};
        REQUIRE(importMeshIntoRotaryJob(doc, doc.activeSheet(), mesh, s, "a").ok);
        Document plain = rotaryDoc();
        s.moveAxis = false;
        REQUIRE(importMeshIntoRotaryJob(plain, plain.activeSheet(), mesh, s, "a").ok);
        const float moved = lastComponent(doc, doc.reliefLevelsOnSheet(doc.activeSheet())[0]).field.at(15, 0);
        const float straight = lastComponent(plain, plain.reliefLevelsOnSheet(plain.activeSheet())[0]).field.at(15, 0);
        CHECK(moved != straight);
    }
}

TEST_CASE("rotary import: flat model is rastered onto the flat design area; refusals", "[viewmodel][rotary][import]") {
    Document doc = rotaryDoc();
    RotaryImportSettings s;
    s.type = RotaryModelType::Flat;
    s.mesh.cellSize = 1.0;
    // A 10 x 10 x 3 block.
    io::TriangleMesh box;
    const double X[2] = {0, 10}, Y[2] = {0, 10}, Z[2] = {0, 3};
    for (int a = 0; a < 2; ++a) for (int b = 0; b < 2; ++b) for (int c = 0; c < 2; ++c) box.vertices.push_back({X[a], Y[b], Z[c]});
    const std::uint32_t f[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                                    {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
    for (const auto& t : f) box.triangles.push_back({t[0], t[1], t[2]});
    const MeshImportOutcome r = importMeshIntoRotaryJob(doc, doc.activeSheet(), box, s, "Flat");
    REQUIRE(r.ok);
    const relief::Component& c = lastComponent(doc, doc.reliefLevelsOnSheet(doc.activeSheet())[0]);
    CHECK(c.field.width() == 40);
    CHECK(c.field.height() == 63);
    // Centred on the area (20, 31.4): the block covers x 15..25 and y 26.4..36.4.
    CHECK(c.field.at(20, 31) == Approx(3.0).margin(1e-5));
    CHECK(std::isnan(c.field.at(5, 5)));

    Document flatJob;
    CHECK_FALSE(importMeshIntoRotaryJob(flatJob, flatJob.activeSheet(), box, s, "x").ok);
    RotaryImportSettings badCell = s;
    badCell.mesh.cellSize = 0.0;
    CHECK_FALSE(importMeshIntoRotaryJob(doc, doc.activeSheet(), box, badCell, "x").ok);
    RotaryImportSettings badAxis;
    badAxis.moveAxis = true;
    badAxis.axisShift = {std::nan(""), 0};
    CHECK_FALSE(importMeshIntoRotaryJob(doc, doc.activeSheet(), box, badAxis, "x").ok);
    CHECK_FALSE(importMeshIntoRotaryJob(doc, doc.activeSheet(), io::TriangleMesh{}, s, "x").ok);
}

TEST_CASE("rotary import: a degenerate mesh (every vertex the same point) is refused, not imported empty", "[viewmodel][rotary][import]") {
    Document doc = rotaryDoc();
    const std::size_t before = doc.reliefLevels().size();
    io::TriangleMesh dot;
    dot.vertices = {{5, 5, 5}, {5, 5, 5}, {5, 5, 5}};
    dot.triangles = {{0, 1, 2}};
    RotaryImportSettings s;
    s.type = RotaryModelType::Full3D;
    s.mesh.cellSize = 1.0;
    const MeshImportOutcome r = importMeshIntoRotaryJob(doc, doc.activeSheet(), dot, s, "Dot");
    CHECK_FALSE(r.ok);
    CHECK_FALSE(r.error.empty());
    CHECK(doc.reliefLevels().size() == before);
    CHECK(doc.undoStack().count() == 0);
}
