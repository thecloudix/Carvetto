// T-0122: the CarvExtras member accessors used by carvetto-cli to keep computed toolpaths in a
// .carv (interim storage, ADR-0019).

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto;
using document::CarvExtras;

namespace {

std::vector<std::uint8_t> bytesOf(const std::string& s) {
    return {s.begin(), s.end()};
}

}  // namespace

TEST_CASE("CarvExtras::member is null for empty extras and unknown names", "[document][carv][extras]") {
    const CarvExtras none;
    CHECK(none.member("toolpaths/0001.json") == nullptr);
    const CarvExtras one = none.withMember("toolpaths/0001.json", bytesOf("{}"));
    CHECK(one.member("toolpaths/0002.json") == nullptr);
    REQUIRE(one.member("toolpaths/0001.json") != nullptr);
    CHECK(*one.member("toolpaths/0001.json") == bytesOf("{}"));
    CHECK_FALSE(one.empty());
    CHECK(none.empty());  // the original is unchanged
}

TEST_CASE("CarvExtras::withMember adds or replaces and leaves the source untouched", "[document][carv][extras]") {
    const CarvExtras a = CarvExtras{}.withMember("b.bin", bytesOf("1"));
    const CarvExtras b = a.withMember("a.bin", bytesOf("2"));
    const CarvExtras c = b.withMember("b.bin", bytesOf("3"));
    CHECK(a.memberNames() == std::vector<std::string>{"b.bin"});
    CHECK(b.memberNames() == std::vector<std::string>{"a.bin", "b.bin"});
    CHECK(*b.member("b.bin") == bytesOf("1"));
    CHECK(*c.member("b.bin") == bytesOf("3"));
    CHECK(*c.member("a.bin") == bytesOf("2"));
}

TEST_CASE("members added with withMember are saved and loaded back", "[document][carv][extras]") {
    document::Document doc;
    doc.insertObject(document::Object::vector(geometry::Contour::rectangle(geometry::Box2{{0, 0}, {10, 5}}),
                                              doc.defaultLayer(), doc.activeSheet()));
    const CarvExtras extras = CarvExtras{}
                                  .withMember("toolpaths/0001.json", bytesOf(R"({"name":"Profile"})"))
                                  .withMember("toolpaths/0002.json", bytesOf(R"({"name":"Pocket"})"));
    const auto saved = document::saveCarv(doc, extras);
    REQUIRE(saved.ok());
    const auto loaded = document::loadCarv(saved.value());
    REQUIRE(loaded.ok());
    const CarvExtras& back = loaded.value().extras;
    CHECK(back.memberNames() == std::vector<std::string>{"toolpaths/0001.json", "toolpaths/0002.json"});
    REQUIRE(back.member("toolpaths/0002.json") != nullptr);
    CHECK(*back.member("toolpaths/0002.json") == bytesOf(R"({"name":"Pocket"})"));
    // Saving again with the loaded extras is byte-identical.
    const auto again = document::saveCarv(loaded.value().document, back);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("withMember refuses names saveCarv cannot write", "[document][carv][extras]") {
    const CarvExtras e;
    for (const char* name : {"", "manifest.json", "job.json", "/abs.json", "a\\b.json", "c:x", "toolpaths/",
                             "toolpaths//a.json", "./a", "a/../b", ".."}) {
        INFO(name);
        CHECK_THROWS_AS(e.withMember(name, {}), std::invalid_argument);
    }
    CHECK_THROWS_AS(e.withMember(std::string(256, 'a'), {}), std::invalid_argument);
    CHECK_NOTHROW(e.withMember(std::string(255, 'a'), {}));
    CHECK_NOTHROW(e.withMember("toolpaths/0001.json", {}));
}
