#include "carvetto/document/selection.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace carvetto::document;

TEST_CASE("selection keeps insertion order and rejects duplicates and null ids", "[document][selection]") {
    Selection sel;
    CHECK(sel.empty());
    CHECK(sel.single().isNull());

    CHECK(sel.add(ObjectId{3}));
    CHECK(sel.add(ObjectId{1}));
    CHECK_FALSE(sel.add(ObjectId{3}));
    CHECK_FALSE(sel.add(ObjectId{}));
    CHECK(sel.size() == 2);
    CHECK(sel.ids() == std::vector<ObjectId>{ObjectId{3}, ObjectId{1}});
    CHECK(sel.contains(ObjectId{1}));
    CHECK_FALSE(sel.contains(ObjectId{2}));
    CHECK(sel.single().isNull());

    CHECK(sel.remove(ObjectId{3}));
    CHECK_FALSE(sel.remove(ObjectId{3}));
    CHECK(sel.single() == ObjectId{1});

    CHECK(sel.toggle(ObjectId{5}));
    CHECK_FALSE(sel.toggle(ObjectId{5}));
    CHECK(sel.ids() == std::vector<ObjectId>{ObjectId{1}});

    sel.assign({ObjectId{7}, ObjectId{8}, ObjectId{7}});
    CHECK(sel.ids() == std::vector<ObjectId>{ObjectId{7}, ObjectId{8}});

    sel.clear();
    CHECK(sel.empty());
    CHECK_FALSE(sel.contains(ObjectId{7}));
}

TEST_CASE("ids are strongly typed, comparable and hashable", "[document][ids]") {
    constexpr ObjectId a{1};
    constexpr ObjectId b{2};
    static_assert(a < b);
    static_assert(ObjectId{}.isNull());
    static_assert(!ObjectId{} && static_cast<bool>(a));
    CHECK(std::hash<ObjectId>{}(a) == std::hash<ObjectId>{}(ObjectId{1}));

    IdGenerator gen;
    CHECK(gen.allocate() == 1);
    CHECK(gen.allocate() == 2);
    gen.reserveThrough(10);
    CHECK(gen.allocate() == 11);
    gen.reserveThrough(5);  // never moves backwards
    CHECK(gen.peek() == 12);
}
