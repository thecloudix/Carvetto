#include "carvetto/viewmodel/bitmap_trace.hpp"

#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour_ops.hpp"

using namespace carvetto;
using Catch::Approx;
using namespace carvetto::viewmodel;

namespace {

io::RasterImage blank(int w, int h, std::uint8_t v = 255) {
    io::RasterImage im;
    im.width = w;
    im.height = h;
    im.rgb.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3, v);
    return im;
}
void fill(io::RasterImage& im, int x0, int y0, int x1, int y1, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(im.width) + static_cast<std::size_t>(x)) * 3;
            im.rgb[i] = r;
            im.rgb[i + 1] = g;
            im.rgb[i + 2] = b;
        }
    }
}
double area(const io::TraceLayer& l) {
    double a = 0;
    for (const auto& c : l.contours) a += geometry::signedArea(c);
    return a;
}

}  // namespace

TEST_CASE("trace form: validation and size basis", "[trace][viewmodel]") {
    TraceForm f;
    CHECK(validateTraceForm(f, 100, 50).empty());
    CHECK_FALSE(validateTraceForm(f, 0, 50).empty());
    f.threshold = 256;
    CHECK_FALSE(validateTraceForm(f, 100, 50).empty());
    f.threshold = -1;
    CHECK_FALSE(validateTraceForm(f, 100, 50).empty());
    f = TraceForm{};
    f.mode = io::TraceMode::Colour;
    f.colourCount = 1;
    CHECK_FALSE(validateTraceForm(f, 10, 10).empty());
    f.colourCount = 17;
    CHECK_FALSE(validateTraceForm(f, 10, 10).empty());
    f.colourCount = 16;
    CHECK(validateTraceForm(f, 10, 10).empty());
    f = TraceForm{};
    f.noiseFilter = -1;
    CHECK_FALSE(validateTraceForm(f, 10, 10).empty());
    f = TraceForm{};
    f.cornerFit = 1.5;
    CHECK_FALSE(validateTraceForm(f, 10, 10).empty());
    f.cornerFit = std::nan("");
    CHECK_FALSE(validateTraceForm(f, 10, 10).empty());
    f = TraceForm{};
    f.sizeValue = 0.0;
    CHECK_FALSE(validateTraceForm(f, 10, 10).empty());
    f.sizeValue = -2.0;
    CHECK_FALSE(validateTraceForm(f, 10, 10).empty());
    f.sizeValue = 1.0e300;
    CHECK_FALSE(validateTraceForm(f, 10, 10).empty());

    f = TraceForm{};
    f.sizeBasis = TraceSizeBasis::PerPixel;
    f.sizeValue = 0.25;
    CHECK(traceMmPerPixel(f, 40, 20).value() == Approx(0.25));
    f.units = document::Units::Inches;
    CHECK(traceMmPerPixel(f, 40, 20).value() == Approx(6.35));
    f.units = document::Units::Millimetres;
    f.sizeBasis = TraceSizeBasis::TargetWidth;
    f.sizeValue = 80.0;
    CHECK(traceMmPerPixel(f, 40, 20).value() == Approx(2.0));
    f.sizeBasis = TraceSizeBasis::TargetHeight;
    f.sizeValue = 80.0;
    CHECK(traceMmPerPixel(f, 40, 20).value() == Approx(4.0));
    f.units = document::Units::Inches;
    f.sizeValue = 2.0;
    CHECK(traceMmPerPixel(f, 40, 20).value() == Approx(2.54));
    // Too-small pixels and too-large results are refused.
    f = TraceForm{};
    f.sizeValue = 1.0e-9;
    CHECK_FALSE(traceMmPerPixel(f, 40, 20).ok());
    f.sizeValue = 50.0;
    CHECK_FALSE(traceMmPerPixel(f, 4000, 4000).ok());

    f = TraceForm{};
    f.mode = io::TraceMode::Colour;
    f.colourCount = 5;
    f.threshold = 99;
    f.invert = true;
    f.noiseFilter = 7;
    f.cornerFit = 0.75;
    f.sizeValue = 0.5;
    f.originMm = {3, 4};
    const auto p = toTraceParams(f, 10, 10);
    REQUIRE(p.ok());
    CHECK(p.value().mode == io::TraceMode::Colour);
    CHECK(p.value().colourCount == 5);
    CHECK(p.value().threshold == 99);
    CHECK(p.value().invert);
    CHECK(p.value().noiseFilter == 7);
    CHECK(p.value().cornerFit == Approx(0.75));
    CHECK(p.value().mmPerPixel == Approx(0.5));
    CHECK(p.value().origin.x == Approx(3));
    CHECK(p.value().origin.y == Approx(4));
}

TEST_CASE("trace preview image keeps the covered area", "[trace][viewmodel]") {
    io::RasterImage im = blank(1000, 500);
    fill(im, 0, 0, 500, 500, 0, 0, 0);
    const PreviewImage small = previewImage(im, 100);
    CHECK(small.image.width == 100);
    CHECK(small.image.height == 50);
    CHECK(small.scale == Approx(10.0));
    CHECK(small.image.valid());
    CHECK(small.image.rgb[0] == 0);
    CHECK(small.image.rgb[(99) * 3] == 255);
    const PreviewImage same = previewImage(blank(30, 20), 100);
    CHECK(same.image.width == 30);
    CHECK(same.scale == 1.0);
    io::TraceParams p;
    p.mmPerPixel = 0.2;
    p.noiseFilter = 200;
    const io::TraceParams pp = previewParams(p, 10.0);
    CHECK(pp.mmPerPixel == Approx(2.0));
    CHECK(pp.noiseFilter == 2);
    CHECK(previewParams(p, 1.0).noiseFilter == 200);
    CHECK_FALSE(previewImage(io::RasterImage{}).image.valid());
}

TEST_CASE("trace runner: result, cancel and refusal", "[trace][viewmodel]") {
    auto img = std::make_shared<io::RasterImage>(blank(40, 40));
    fill(*img, 10, 10, 30, 30, 0, 0, 0);
    io::TraceParams p;
    p.noiseFilter = 0;
    TraceRunner runner;
    REQUIRE(runner.start(img, p));
    CHECK(runner.busy());
    CHECK_FALSE(runner.start(img, p));  // one at a time
    const auto out = runner.take();
    REQUIRE(out.has_value());
    CHECK_FALSE(runner.busy());
    REQUIRE(out->ok);
    REQUIRE(out->layers.size() == 1);
    CHECK(area(out->layers[0]) == Approx(400.0).epsilon(0.02));
    CHECK_FALSE(runner.take().has_value());

    // Cancelled before it starts: the outcome says so and has no layers.
    std::atomic<bool> flag{true};
    const TraceOutcome c = runTrace(*img, p, &flag);
    CHECK(c.cancelled);
    CHECK_FALSE(c.ok);
    CHECK(c.layers.empty());

    // An error from core (bad parameter) is passed on, not cancelled.
    io::TraceParams bad = p;
    bad.mmPerPixel = 0;
    const TraceOutcome e = runTrace(*img, bad);
    CHECK_FALSE(e.ok);
    CHECK_FALSE(e.cancelled);
    CHECK_FALSE(e.message.empty());

    // Destroying a busy runner cancels and joins.
    {
        TraceRunner r2;
        REQUIRE(r2.start(img, p));
    }
}

TEST_CASE("applyTrace: one undo step, one layer per colour", "[trace][viewmodel]") {
    io::RasterImage im = blank(60, 30);
    fill(im, 5, 5, 25, 25, 200, 0, 0);
    fill(im, 35, 5, 55, 25, 0, 0, 200);
    io::TraceParams p;
    p.mode = io::TraceMode::Colour;
    p.colourCount = 3;
    p.noiseFilter = 0;
    const TraceOutcome out = runTrace(im, p);
    REQUIRE(out.ok);
    REQUIRE(out.layers.size() == 3);

    document::Document doc;
    const std::size_t layersBefore = doc.layers().size();
    const std::size_t objectsBefore = doc.objectCount();
    const std::size_t stepsBefore = doc.undoStack().count();
    const TraceApplyResult r = applyTrace(doc, out.layers, io::TraceMode::Colour);
    REQUIRE(r.ok);
    CHECK(r.layers.size() == 3);
    CHECK(r.created.size() == 3);
    CHECK(doc.layers().size() == layersBefore + 3);
    CHECK(doc.objectCount() == objectsBefore + 3);
    CHECK(doc.undoStack().count() == stepsBefore + 1);
    CHECK(doc.selection().size() == 3);
    bool sawRed = false;
    for (const document::LayerId id : r.layers) {
        const document::Layer* l = doc.findLayer(id);
        REQUIRE(l != nullptr);
        CHECK(l->name.rfind("Trace #", 0) == 0);
        if (l->colour.r > 150 && l->colour.g < 60 && l->colour.b < 60) sawRed = true;
    }
    CHECK(sawRed);
    REQUIRE(doc.undo());
    CHECK(doc.layers().size() == layersBefore);
    CHECK(doc.objectCount() == objectsBefore);
    REQUIRE(doc.redo());
    CHECK(doc.layers().size() == layersBefore + 3);
    CHECK(doc.objectCount() == objectsBefore + 3);
}

TEST_CASE("applyTrace: black and white, empty result untouched", "[trace][viewmodel]") {
    document::Document doc;
    const std::size_t stepsBefore = doc.undoStack().count();
    const TraceApplyResult none = applyTrace(doc, {});
    CHECK_FALSE(none.ok);
    CHECK_FALSE(none.message.empty());
    CHECK(doc.undoStack().count() == stepsBefore);
    io::TraceLayer empty;
    CHECK_FALSE(applyTrace(doc, {empty}).ok);
    CHECK(doc.undoStack().count() == stepsBefore);

    io::RasterImage im = blank(30, 30);
    fill(im, 5, 5, 25, 25, 0, 0, 0);
    io::TraceParams p;
    const TraceOutcome out = runTrace(im, p);
    REQUIRE(out.ok);
    const TraceApplyResult r = applyTrace(doc, out.layers);
    REQUIRE(r.ok);
    REQUIRE(r.layers.size() == 1);
    CHECK(doc.findLayer(r.layers[0])->name == "Trace");
    CHECK(doc.undoStack().count() == stepsBefore + 1);
}
