#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <variant>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/viewmodel/node_ops.hpp"
#include "carvetto/viewmodel/tools.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/far, GDI names).
#include "../../../core/tests/geometry/cpu_time.hpp"

using namespace carvetto::document;
using namespace carvetto::geometry;
using namespace carvetto::viewmodel;

namespace {

// Deterministic across platforms: raw mt19937 output, no <random> distributions.
struct Rng {
    std::mt19937 gen;
    explicit Rng(std::uint32_t seed) : gen(seed) {}
    std::uint32_t next(std::uint32_t n) { return static_cast<std::uint32_t>(gen() % n); }
    double unit() { return static_cast<double>(gen() % 1000001U) / 1000000.0; }
    double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
};

Contour randomContour(Rng& rng) {
    const double x = rng.range(10, 300);
    const double y = rng.range(10, 300);
    switch (rng.next(5)) {
        case 0:
            return Contour::rectangle(Box2{{x, y}, {x + rng.range(5, 80), y + rng.range(5, 60)}});
        case 1:
            return Contour::circle({x, y}, rng.range(3, 40));
        case 2: {
            Contour c;
            c.append(BezierSpan{{x, y}, {x + 10, y + 40}, {x + 40, y - 30}, {x + 60, y}});
            c.append(BezierSpan{{x + 60, y}, {x + 70, y + 20}, {x + 90, y + 20}, {x + 100, y}});
            return c;
        }
        case 3: {
            Contour c;
            c.append(LineSpan{{x, y}, {x + 40, y}});
            c.append(LineSpan{{x + 40, y}, {x + 40, y + 30}});
            c.append(*nodeops::arcFromBulge({x + 40, y + 30}, {x, y + 30}, rng.range(-1, 1) + 1e-3));
            c.closed = rng.next(2) == 0;
            if (c.closed) {
                c.append(LineSpan{{x, y + 30}, {x, y}});
            }
            return c;
        }
        default: {
            Contour c;
            c.append(LineSpan{{x, y}, {x + 20, y + 10}});
            c.append(LineSpan{{x + 20, y + 10}, {x + 45, y - 10}});
            return c;
        }
    }
}

void checkDocumentSane(const Document& doc) {
    for (const auto& [id, obj] : doc.objects()) {
        const auto* vec = obj.asVector();
        if (vec == nullptr) {
            continue;
        }
        for (const Contour& c : vec->contours) {
            REQUIRE_FALSE(c.spans.empty());
            for (const Span& s : c.spans) {
                REQUIRE(std::isfinite(startPoint(s).x));
                REQUIRE(std::isfinite(startPoint(s).y));
                REQUIRE(std::isfinite(endPoint(s).x));
                REQUIRE(std::isfinite(endPoint(s).y));
                REQUIRE(std::fabs(startPoint(s).x) <= 1e9 + 1);
                REQUIRE(std::fabs(startPoint(s).y) <= 1e9 + 1);
            }
        }
    }
}

/// Returns how many undo steps the scripted events recorded.
std::size_t runSeed(std::uint32_t seed) {
    Rng rng(seed);
    auto doc = std::make_unique<Document>(Job::singleSided(1000.0, 1000.0, 18.0, Units::Millimetres));
    ViewTransform view;
    view.setViewport(1000.0, 1000.0);
    view.set(2.0, {100.0, 900.0});
    ToolController tool;
    tool.setView(view);
    tool.setDocument(doc.get());
    tool.setTool(ToolKind::NodeEdit);
    if (rng.next(2) == 0) {
        tool.snapSettings().disableAll();
        tool.snapSettings().setEnabled(SnapKind::Endpoint, true);
    }
    std::vector<ObjectId> ids;
    const std::uint32_t objects = 1 + rng.next(4);
    for (std::uint32_t i = 0; i < objects; ++i) {
        auto cmd = std::make_unique<AddObjectCommand>(
            Object::vector(randomContour(rng), doc->activeLayer(), doc->activeSheet()));
        AddObjectCommand* raw = cmd.get();
        REQUIRE(doc->execute(std::move(cmd)));
        ids.push_back(raw->id());
    }
    const std::size_t setupSteps = doc->undoStack().count();
    const auto setupObjects = doc->objects();
    doc->selection().assign(ids);

    auto randomPoint = [&]() {
        // Mostly inside the drawing, sometimes exactly on a node, sometimes absurd.
        const std::uint32_t kind = rng.next(20);
        if (kind == 0) {
            return Vec2{std::numeric_limits<double>::quiet_NaN(), 5.0};
        }
        if (kind == 1) {
            return Vec2{1e12, -1e12};
        }
        if (kind < 8 && !ids.empty()) {
            const Object* obj = doc->findObject(ids[rng.next(static_cast<std::uint32_t>(ids.size()))]);
            if (obj != nullptr && obj->asVector() != nullptr && !obj->asVector()->contours.empty()) {
                const Contour& c = obj->asVector()->contours.front();
                return view.worldToScreen(startPoint(c.spans[rng.next(static_cast<std::uint32_t>(c.spans.size()))]));
            }
        }
        return view.worldToScreen({rng.range(0, 400), rng.range(0, 400)});
    };

    const int events = 60;
    for (int e = 0; e < events; ++e) {
        Modifiers m;
        m.shift = rng.next(5) == 0;
        m.ctrl = rng.next(5) == 0;
        m.alt = rng.next(8) == 0;
        switch (rng.next(12)) {
            case 0:
            case 1:
            case 2:
                tool.pointerMove(randomPoint(), m);
                break;
            case 3:
            case 4:
                tool.pointerPress(randomPoint(), m);
                break;
            case 5:
            case 6:
                tool.pointerRelease(randomPoint(), m);
                break;
            case 7:
                tool.key(ToolKey::Escape);
                break;
            case 8: {
                static const char keys[] = "idscblaxyjhvkp";
                tool.nodeEditor().handleKey(keys[rng.next(sizeof(keys) - 1)]);
                break;
            }
            case 9:
                tool.nodeEditor().selectAllNodes();
                break;
            case 10:
                tool.nodeEditor().setSelectedNodeCoordinates(rng.range(-50, 400), std::nullopt, rng.next(2) == 0);
                break;
            default:
                // Idle edits of the document by others: a lock toggle or a transform.
                if (!tool.nodeEditor().pressed() && !ids.empty() && rng.next(3) == 0) {
                    doc->execute(std::make_unique<TransformObjectsCommand>(
                        std::vector<ObjectId>{ids[rng.next(static_cast<std::uint32_t>(ids.size()))]},
                        Transform2::translation({rng.range(-5, 5), rng.range(-5, 5)})));
                }
                break;
        }
        checkDocumentSane(*doc);
        for (const NodeRef& n : tool.nodeEditor().selectedNodes()) {
            REQUIRE(tool.nodeEditor().nodePosition(n).has_value());
        }
        (void)tool.nodeEditor().handles();
    }
    const std::size_t recorded = doc->undoStack().count() - setupSteps;
    // Finish any drag, then undo everything: the setup state returns byte-identically.
    tool.pointerRelease({0, 0});
    tool.key(ToolKey::Escape);
    while (doc->undoStack().count() > setupSteps && doc->undoStack().canUndo()) {
        if (doc->undoStack().index() <= setupSteps) {
            break;
        }
        doc->undoStack().undo(*doc);
    }
    if (doc->undoStack().index() == setupSteps) {
        REQUIRE(doc->objects() == setupObjects);
    }
    return recorded;
}

}  // namespace

TEST_CASE("fuzz: scripted node-edit event sequences keep the document sane", "[viewmodel][node][edit][fuzz]") {
    std::size_t recorded = 0;
    for (std::uint32_t seed = 1; seed <= 150; ++seed) {
        INFO("seed " << seed);
        recorded += runSeed(seed);
    }
    std::cout << "[T-0206 fuzz] undo steps recorded by 150 seeds: " << recorded << "\n";
    CHECK(recorded > 300);  // the scripts really edit
}

// Run explicitly (and under ASan/UBSan) with "[fuzz-long]".
TEST_CASE("fuzz: several thousand seeds", "[.][fuzz-long]") {
    for (std::uint32_t seed = 1; seed <= 6000; ++seed) {
        INFO("seed " << seed);
        runSeed(seed);
    }
}

TEST_CASE("performance: a pointer step with 10k objects and one edited object", "[viewmodel][node][edit][perf]") {
    Document doc(Job::singleSided(5000.0, 5000.0, 18.0, Units::Millimetres));
    ViewTransform view;
    view.setViewport(1000.0, 1000.0);
    view.set(2.0, {50.0, 950.0});
    ToolController tool;
    tool.setView(view);
    tool.setDocument(&doc);
    tool.setTool(ToolKind::NodeEdit);
    const int n = 10000;
    ObjectId first;
    for (int i = 0; i < n; ++i) {
        const double x = 3.0 * (i % 100);
        const double y = 3.0 * (i / 100);
        auto cmd = std::make_unique<AddObjectCommand>(Object::vector(
            Contour::rectangle(Box2{{x, y}, {x + 2.0, y + 2.0}}), doc.activeLayer(), doc.activeSheet()));
        AddObjectCommand* raw = cmd.get();
        REQUIRE(doc.execute(std::move(cmd)));
        if (i == 5050) {
            first = raw->id();
        }
    }
    doc.selection().assign({first});
    const Vec2 corner{3.0 * 50 + 2.0, 3.0 * 50 + 2.0};  // node 2 of object 5050
    tool.pointerMove(view.worldToScreen(corner));       // builds the snap index once
    tool.pointerPress(view.worldToScreen(corner));
    const int steps = 200;
    // The first move starts the drag; measure from the second on.
    tool.pointerMove(view.worldToScreen(corner + Vec2{3, 3}));
    const double t0 = carvetto::test::processCpuSeconds();
    for (int i = 1; i <= steps; ++i) {
        tool.pointerMove(view.worldToScreen(corner + Vec2{3.0 + 0.05 * i, 3.0 + 0.04 * i}));
    }
    const double perStep = (carvetto::test::processCpuSeconds() - t0) / steps;
    tool.pointerRelease(view.worldToScreen(corner + Vec2{13, 11}));
    std::cout << "[T-0206 perf] objects=" << n << " step=" << perStep * 1e3 << " ms\n";
    CHECK(doc.undoStack().count() == static_cast<std::size_t>(n) + 1);
    CHECK(perStep < 0.016);
}
