// T-0266: Tiling Manager model behind the Save Toolpaths dialog's "Tile Toolpaths" box.

#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "carvetto/viewmodel/tiling_form.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using toolpath::Move;

namespace {

/// A 100 x 50 mm job: two cuts far apart (leaving empty tiles in a 3x3 grid of 40 x 20).
std::vector<document::ToolpathRecord> job() {
    document::ToolpathRecord r;
    r.name = "cut";
    r.moves = document::shareMoves(std::vector<Move>{
        Move::rapid({0, 0, 5}), Move::plunge({0, 0, -1}), Move::feed({100, 0, -1}), Move::retract({100, 0, 5}),
        Move::rapid({0, 50, 5}), Move::plunge({0, 50, -1}), Move::feed({100, 50, -1}), Move::retract({100, 50, 5})});
    return {r};
}

TilingForm form(double w, double h) {
    TilingForm f;
    f.enabled = true;
    f.width = w;
    f.height = h;
    return f;
}

}  // namespace

TEST_CASE("tiling form: machine work area is the default tile size", "[tiling_form]") {
    tooling::Machine m;
    m.workAreaX = 300;
    m.workAreaY = 200;
    const TilingForm f = defaultTilingForm(&m);
    CHECK(f.width == 300);
    CHECK(f.height == 200);
    CHECK_FALSE(f.enabled);
    CHECK(defaultTilingForm(nullptr).width == 0);
}

TEST_CASE("tiling form: validation and spec", "[tiling_form]") {
    TilingForm f = form(40, 20);
    CHECK(validateTilingForm(f).empty());
    REQUIRE(tileSpecOf(f).has_value());
    CHECK(tileSpecOf(f)->width == 40);
    f.overlap = 20;
    CHECK_FALSE(validateTilingForm(f).empty());
    CHECK_FALSE(tileSpecOf(f).has_value());
    f.overlap = 0;
    f.height = 0;
    CHECK_FALSE(validateTilingForm(f).empty());
    f.strategy = toolpath::TileStrategy::FeedThroughX;  // height unused
    CHECK(validateTilingForm(f).empty());
    f.enabled = false;
    CHECK_FALSE(tileSpecOf(f).has_value());
}

TEST_CASE("tiling form: applyTiling sets tiling and the machine work area", "[tiling_form]") {
    tooling::Machine m;
    m.workAreaX = 300;
    m.workAreaY = 200;
    SaveOptions o;
    applyTiling(o, form(40, 20), &m);
    REQUIRE(o.tiling.has_value());
    CHECK(o.workAreaX == 300);
    CHECK(o.workAreaY == 200);
    TilingForm off = form(40, 20);
    off.enabled = false;
    applyTiling(o, off, nullptr);
    CHECK_FALSE(o.tiling.has_value());
    CHECK(o.workAreaX == 0);
}

TEST_CASE("tiling preview: tiles, numbers, empty tiles and oversize", "[tiling_form]") {
    const auto records = job();
    const TilePreview p = previewTiling(records, form(40, 20), 0, 0);
    INFO(p.error);
    REQUIRE(p.ok());
    CHECK(p.extent.width() == 100);
    REQUIRE(p.tiles.size() == 9);  // 3 columns x 3 rows
    CHECK(p.saved == 6);           // the middle row has no cuts
    std::size_t last = 0;
    for (const TilePreviewEntry& e : p.tiles) {
        if (e.number != 0) {
            CHECK(e.number == last + 1);
            last = e.number;
        }
        CHECK_FALSE(e.oversize);
    }
    CHECK(p.oversize == 0);
    CHECK_FALSE(p.outline.empty());
    CHECK(describeTilePreview(p).find("6 tile(s)") != std::string::npos);

    // The same tiles as the core tiling the save path uses.
    const auto res = toolpath::tileToolpaths({records[0].toolpath()}, *tileSpecOf(form(40, 20)));
    REQUIRE(res.ok());
    CHECK(res.tiles.size() == p.saved);
    for (const TilePreviewEntry& e : p.tiles) {
        if (e.number != 0) {
            CHECK(res.tiles[e.number - 1].rect.min.x == e.rect.min.x);
            CHECK(res.tiles[e.number - 1].rect.min.y == e.rect.min.y);
        }
    }
}

TEST_CASE("tiling preview: oversize tiles are marked, job larger than the work area is flagged", "[tiling_form]") {
    const TilePreview p = previewTiling(job(), form(40, 20), 30, 0);
    REQUIRE(p.ok());
    CHECK(p.jobExceedsWorkArea);
    CHECK(p.oversize == 4);  // the two 40 mm columns, in two rows; the 20 mm column fits
    CHECK(describeTilePreview(p).find("work area") != std::string::npos);
    const TilePreview ok = previewTiling(job(), form(40, 20), 40, 20);
    CHECK(ok.oversize == 0);
    CHECK(ok.jobExceedsWorkArea);
}

TEST_CASE("tiling preview: errors", "[tiling_form]") {
    TilingForm f = form(40, 20);
    f.enabled = false;
    CHECK_FALSE(previewTiling(job(), f, 0, 0).ok());
    CHECK_FALSE(previewTiling(job(), form(0, 20), 0, 0).ok());
    CHECK_FALSE(previewTiling({}, form(40, 20), 0, 0).ok());
    CHECK(previewTiling(job(), form(1e-3, 1e-3), 0, 0).error.find("too many") != std::string::npos);
}

TEST_CASE("tiling preview: feed-through strips", "[tiling_form]") {
    TilingForm f = form(40, 0);
    f.strategy = toolpath::TileStrategy::FeedThroughX;
    const TilePreview p = previewTiling(job(), f, 0, 0);
    REQUIRE(p.ok());
    CHECK(p.tiles.size() == 3);
    CHECK(p.tiles[0].rect.height() == 50);
}
