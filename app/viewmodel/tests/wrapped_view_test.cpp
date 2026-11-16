// T-0320: the Auto-Wrapping View (ADR-0103): the flat/wrapped toggle, the job's cylinder map, the wrapped
// software render of surfaces, paths and the simulated stock, and the wrapped simulation of a rotary job.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/viewmodel/simulation_setup.hpp"
#include "carvetto/viewmodel/wrapped_view.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using Catch::Approx;
using toolpath::Move;

namespace {

constexpr double kPi = std::numbers::pi;
constexpr std::uint32_t kBackground = 0xFFE2E4E8u;
constexpr std::uint32_t kPathColour = 0xFF00FF00u;

struct Extent {
    long x0{std::numeric_limits<long>::max()}, x1{-1}, y0{std::numeric_limits<long>::max()}, y1{-1};
    std::size_t count{0};
};

Extent paintedExtent(const Raster& r, std::uint32_t background = kBackground) {
    Extent e;
    for (std::size_t y = 0; y < r.height; ++y) {
        for (std::size_t x = 0; x < r.width; ++x) {
            if (r.pixels[y * r.width + x] != background) {
                e.x0 = std::min(e.x0, static_cast<long>(x));
                e.x1 = std::max(e.x1, static_cast<long>(x));
                e.y0 = std::min(e.y0, static_cast<long>(y));
                e.y1 = std::max(e.y1, static_cast<long>(y));
                ++e.count;
            }
        }
    }
    return e;
}

std::size_t countColour(const Raster& r, std::uint32_t colour) {
    return static_cast<std::size_t>(std::count(r.pixels.begin(), r.pixels.end(), colour));
}

/// A flat grid at machine Z `z` over length x circumference of a cylinder of diameter `d`.
WrappedSurface flatSurface(double length, double d, double z, double cell = 1.0) {
    WrappedSurface s;
    const double circ = kPi * d;
    s.cellSize = cell;
    s.nx = static_cast<std::size_t>(length / cell) + 1;
    s.ny = static_cast<std::size_t>(circ / cell) + 1;
    s.origin = {0.0, 0.0};
    s.z.assign(s.nx * s.ny, static_cast<float>(z));
    return s;
}

OrbitCamera sideView() {
    OrbitCamera c;
    c.azimuthRad = 0.0;
    c.elevationRad = 0.0;
    return c;
}

OrbitCamera topView() {
    OrbitCamera c;
    c.azimuthRad = 0.0;
    c.elevationRad = kPi / 2.0;
    return c;
}

}  // namespace

TEST_CASE("wrapped view: the toggle exists only for rotary jobs", "[wrapped][rotary]") {
    const document::Job rotary = document::Job::rotary(100, 20, document::Units::Millimetres);
    const document::Job flat = document::Job::singleSided(100, 100, 10, document::Units::Millimetres);
    RotaryViewState state;
    CHECK(RotaryViewState::available(rotary));
    CHECK_FALSE(RotaryViewState::available(flat));
    CHECK(state.effective(rotary) == RotaryViewMode::Wrapped);
    CHECK(state.effective(flat) == RotaryViewMode::Flat);
    state.toggle();
    CHECK(state.effective(rotary) == RotaryViewMode::Flat);
    state.toggle();
    CHECK(state.effective(rotary) == RotaryViewMode::Wrapped);
}

TEST_CASE("wrapped view: the cylinder map follows the job", "[wrapped][rotary]") {
    document::Job job = document::Job::rotary(100, 20, document::Units::Millimetres, document::RotaryAxis::Y,
                                              document::ZZero::CylinderAxis, true);
    geometry::CylinderMap m = cylinderMapFor(job);
    CHECK_FALSE(m.alongX);
    CHECK(m.radius == Approx(10.0));
    CHECK(m.flip);
    CHECK(m.axisZ == Approx(0.0));
    job.zZero = document::ZZero::MaterialSurface;
    job.rotaryAxis = document::RotaryAxis::X;
    job.flipDesign = false;
    m = cylinderMapFor(job);
    CHECK(m.alongX);
    CHECK_FALSE(m.flip);
    CHECK(m.axisZ == Approx(-10.0));  // Z zero at the surface: the axis is a radius below
}

TEST_CASE("wrapped view: a flat surface wraps to the cylinder silhouette", "[wrapped][rotary]") {
    // Cylinder along X, 40 long, diameter 20, axis at Z 0, surface at Z 10, seen from the side.
    geometry::CylinderMap map;
    map.alongX = true;
    map.radius = 10.0;
    WrappedRenderOptions opt;
    opt.width = opt.height = 200;
    const WrappedSurface s = flatSurface(40.0, 20.0, 10.0, 0.5);
    const Raster r = renderWrapped(s, map, sideView(), opt);
    REQUIRE(r.pixels.size() == 200u * 200u);
    const Extent e = paintedExtent(r);
    // Scale: 0.9 * 200 / (2 * sqrt(20^2 + 10^2 + 10^2)) pixels per mm.
    const double scale = 0.9 * 200.0 / (2.0 * std::sqrt(600.0));
    CHECK(static_cast<double>(e.x1 - e.x0 + 1) == Approx(40.0 * scale).margin(3.0));
    CHECK(static_cast<double>(e.y1 - e.y0 + 1) == Approx(20.0 * scale).margin(3.0));
    // Deterministic.
    CHECK(renderWrapped(s, map, sideView(), opt).pixels == r.pixels);
}

TEST_CASE("wrapped view: a hole in the surface is left empty", "[wrapped][rotary]") {
    geometry::CylinderMap map;
    map.radius = 10.0;
    WrappedSurface s = flatSurface(40.0, 20.0, 10.0, 1.0);
    WrappedRenderOptions opt;
    opt.width = opt.height = 100;
    const std::size_t full = paintedExtent(renderWrapped(s, map, topView(), opt)).count;
    for (std::size_t j = 0; j < s.ny; ++j) {
        for (std::size_t i = 0; i < s.nx; ++i) {
            if (i >= 15 && i <= 25) {  // a band across the middle: the framing stays the same
                s.z[j * s.nx + i] = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    const std::size_t holed = paintedExtent(renderWrapped(s, map, topView(), opt)).count;
    CHECK(holed < full);
    CHECK_FALSE(WrappedSurface{}.ok());
    CHECK(paintedExtent(renderWrapped(WrappedSurface{}, map, topView(), opt)).count == 0);
}

TEST_CASE("wrapped view: paths follow the wrap and are hidden on the far side", "[wrapped][rotary]") {
    geometry::CylinderMap map;
    map.alongX = true;
    map.radius = 10.0;
    const WrappedSurface s = flatSurface(40.0, 20.0, 10.0, 1.0);
    WrappedRenderOptions opt;
    opt.width = opt.height = 200;
    ModelPath onTop;  // circumference coordinate 0: straight up
    onTop.colour = kPathColour;
    onTop.runs.push_back({{2, 0, 10}, {38, 0, 10}});
    ModelPath underneath;  // half a turn round: at the bottom, hidden from above
    underneath.colour = kPathColour;
    underneath.runs.push_back({{2, kPi * 10.0, 10}, {38, kPi * 10.0, 10}});

    opt.paths = {onTop};
    const std::size_t top = countColour(renderWrapped(s, map, topView(), opt), kPathColour);
    CHECK(top > 100);
    opt.paths = {underneath};
    CHECK(countColour(renderWrapped(s, map, topView(), opt), kPathColour) == 0);
    // From below the same path shows.
    OrbitCamera below = topView();
    below.elevationRad = -kPi / 2.0;
    CHECK(countColour(renderWrapped(s, map, below, opt), kPathColour) > 100);
    // A circumferential path (a full turn at X = 20) is a circle seen end on: it wraps, so its extent
    // spans the disc in both directions.
    ModelPath ring;
    ring.colour = kPathColour;
    ring.runs.push_back({{20, 0, 10}, {20, kPi * 5.0, 10}, {20, kPi * 10.0, 10}, {20, kPi * 15.0, 10}, {20, kPi * 20.0, 10}});
    opt.paths = {ring};
    OrbitCamera end = sideView();
    end.azimuthRad = kPi / 2.0;
    Raster rr = renderWrapped(s, map, end, opt);
    const Extent ringExtent = paintedExtent(rr, kBackground);
    CHECK(ringExtent.count > 0);
    CHECK(countColour(rr, kPathColour) > 50);
}

TEST_CASE("wrapped view: a rotary model draws wrapped and the toggle returns the flat view", "[wrapped][rotary]") {
    document::Document doc(document::Job::rotary(60, 20, document::Units::Millimetres));
    ModelSurface model;
    model.field = relief::HeightField();
    relief::Grid grid;
    grid.origin = {0.0, 0.0};
    grid.cellSize = 2.0;
    grid.width = 31;
    grid.height = 32;
    REQUIRE(relief::HeightField::create(grid, 3.0f, model.field) == relief::Status::Ok);
    REQUIRE(model.ok());
    ModelRenderOptions ro;
    ro.width = ro.height = 160;
    RotaryViewState state;
    OrbitCamera cam;
    const Raster wrapped = renderRotaryModel(model, doc.job(), state, cam, ro);
    state.toggle();
    const Raster flat = renderRotaryModel(model, doc.job(), state, cam, ro);
    CHECK(flat.pixels == renderModel(model, cam, ro).pixels);
    CHECK(wrapped.pixels != flat.pixels);
    CHECK(paintedExtent(wrapped, ro.background).count > 1000);
    // The model band starts at the inside gap: height h wraps at radius gapInside + h.
    document::Job job = doc.job();
    job.gapInsideMm = 2.0;
    const WrappedSurface ws = wrappedSurfaceFromModel(model, job);
    REQUIRE(ws.ok());
    CHECK(ws.z[0] == Approx(job.rotaryModelBottomZ() + 3.0));
    CHECK(ws.z[0] == Approx(5.0));  // radius 10, inside gap 2: band bottom at Z 2, plus 3
}

TEST_CASE("wrapped simulation: rotary jobs are simulated as an unwrapped cylinder", "[wrapped][simulation][rotary]") {
    document::Document doc(document::Job::rotary(60, 20, document::Units::Millimetres));
    {
        // One pass along the length at the top of the cylinder, 2 mm deep (surface Z 10, cut at 8).
        document::ToolpathRecord rec;
        rec.name = "groove";
        rec.tool.diameter = 3.0;
        rec.tool.feedRate = 1000.0;
        rec.tool.plungeRate = 300.0;
        rec.moves = document::shareMoves({Move::rapid({5, 31.4, 15}), Move::plunge({5, 31.4, 8}),
                                          Move::feed({55, 31.4, 8}), Move::retract({55, 31.4, 15})});
        REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(rec)));
    }
    ToolpathSession session(&doc);
    SimulationRequest req;
    req.quality = simulate::Quality::Coarse;
    const SimulationSetup setup = makeSimulationSetup(session, req);
    REQUIRE(setup.ok);
    CHECK(setup.stock.width == Approx(60.0));
    CHECK(setup.stock.height == Approx(kPi * 20.0));
    CHECK(setup.stock.topZ == Approx(10.0));
    CHECK(setup.stock.thickness == Approx(10.0));
    CHECK(setup.stock.bottomZ() == Approx(0.0));  // the cylinder axis

    simulate::Simulator sim(setup.stock, setup.options);
    REQUIRE(sim.ok());
    toolpath::Toolpath path;
    path.name = setup.toolpaths[0].name;
    path.tool = setup.toolpaths[0].tool;
    path.moves = *setup.toolpaths[0].moves;
    REQUIRE(sim.addToolpath(path, setup.toolpaths[0].shape, 1));
    REQUIRE(sim.advanceAll());
    CHECK(sim.heightfield().cutCells() > 0);

    const WrappedSurface ws = wrappedSurfaceFromSimulation(sim.heightfield());
    REQUIRE(ws.ok());
    float lowest = 100.0f;
    float highest = -100.0f;
    std::size_t tinted = 0;
    for (std::size_t k = 0; k < ws.z.size(); ++k) {
        lowest = std::min(lowest, ws.z[k]);
        highest = std::max(highest, ws.z[k]);
        if (ws.colours[k] != 0) {
            ++tinted;
        }
    }
    CHECK(highest == Approx(10.0f));
    CHECK(lowest == Approx(8.0f).margin(0.01));
    CHECK(tinted == sim.heightfield().cutCells());

    // The wrapped render of the cut stock differs from the uncut one and is deterministic.
    simulate::Simulator uncut(setup.stock, setup.options);
    WrappedRenderOptions opt;
    opt.width = opt.height = 160;
    const geometry::CylinderMap map = cylinderMapFor(doc.job());
    const Raster a = renderWrapped(ws, map, topView(), opt);
    const Raster b = renderWrapped(wrappedSurfaceFromSimulation(uncut.heightfield()), map, topView(), opt);
    CHECK(a.pixels != b.pixels);
    CHECK(renderWrapped(ws, map, topView(), opt).pixels == a.pixels);
}

TEST_CASE("wrapped simulation: cutting is limited by the cylinder axis", "[wrapped][simulation][rotary]") {
    document::Document doc(document::Job::rotary(60, 20, document::Units::Millimetres));
    document::ToolpathRecord rec;
    rec.name = "deep";
    rec.tool.diameter = 3.0;
    rec.tool.feedRate = 1000.0;
    rec.tool.plungeRate = 300.0;
    rec.moves = document::shareMoves({Move::rapid({5, 5, 15}), Move::plunge({5, 5, -5}), Move::feed({55, 5, -5})});
    REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(rec)));
    ToolpathSession session(&doc);
    const SimulationSetup setup = makeSimulationSetup(session);
    REQUIRE(setup.ok);
    simulate::Simulator sim(setup.stock, setup.options);
    toolpath::Toolpath path;
    path.tool = setup.toolpaths[0].tool;
    path.moves = *setup.toolpaths[0].moves;
    sim.addToolpath(path, setup.toolpaths[0].shape, 1);
    sim.advanceAll();
    const WrappedSurface ws = wrappedSurfaceFromSimulation(sim.heightfield());
    float lowest = 100.0f;
    for (const float z : ws.z) {
        lowest = std::min(lowest, z);
    }
    CHECK(lowest >= 0.0f);
}

TEST_CASE("wrapped paths: vectors and toolpaths in flat machine coordinates", "[wrapped][rotary]") {
    document::Document doc(document::Job::rotary(60, 20, document::Units::Millimetres));
    document::ToolpathRecord rec;
    rec.name = "arc";
    rec.moves = document::shareMoves({Move::rapid({10, 0, 15}), Move::plunge({10, 0, 8}),
                                      Move::arcCW({10, 0, 8}, {15, 0}), Move::retract({10, 0, 15})});
    const std::vector<ModelPath> paths = wrappedToolpathPaths({rec}, 0.1);
    REQUIRE(paths.size() == 1);
    REQUIRE(paths[0].runs.size() == 1);
    // The plunge start, the full circle in chords of at most 0.1 rad, and the retract.
    CHECK(paths[0].runs[0].size() > 60);
    CHECK(paths[0].runs[0].front().z == Approx(15.0));
    CHECK(wrappedVectorPaths(doc).empty());
}
