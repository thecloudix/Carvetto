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

// --- T-0209: O(1) amortised removal, same observable behaviour ------------------------------------

#include <algorithm>
#include <cstdint>
#include <random>

#include "../geometry/cpu_scaling.hpp"

namespace {

/// The obvious implementation: an ordered vector searched linearly.
struct NaiveSelection {
    std::vector<ObjectId> order;
    bool contains(ObjectId id) const { return std::find(order.begin(), order.end(), id) != order.end(); }
    bool add(ObjectId id) {
        if (id.isNull() || contains(id)) {
            return false;
        }
        order.push_back(id);
        return true;
    }
    bool remove(ObjectId id) {
        const auto it = std::find(order.begin(), order.end(), id);
        if (it == order.end()) {
            return false;
        }
        order.erase(it);
        return true;
    }
    bool toggle(ObjectId id) {
        if (contains(id)) {
            remove(id);
            return false;
        }
        return add(id);
    }
    void assign(const std::vector<ObjectId>& ids) {
        order.clear();
        for (ObjectId id : ids) {
            add(id);
        }
    }
};

void requireSame(const Selection& s, const NaiveSelection& n) {
    REQUIRE(s.ids() == n.order);
    REQUIRE(s.size() == n.order.size());
    REQUIRE(s.empty() == n.order.empty());
    REQUIRE(s.single() == (n.order.size() == 1 ? n.order.front() : ObjectId{}));
}

}  // namespace

TEST_CASE("selection matches a naive reference on random operation sequences", "[document][selection]") {
    std::mt19937 rng(12345);
    for (int round = 0; round < 40; ++round) {
        const std::uint64_t range = round % 3 == 0 ? 6 : (round % 3 == 1 ? 40 : 200);
        Selection s;
        NaiveSelection n;
        for (int step = 0; step < 600; ++step) {
            const ObjectId id{rng() % (range + 1)};  // includes the null id and absent ids
            switch (rng() % 8) {
                case 0:
                case 1:
                    CHECK(s.add(id) == n.add(id));
                    break;
                case 2:
                case 3:
                case 4:
                    CHECK(s.remove(id) == n.remove(id));
                    break;
                case 5:
                    CHECK(s.toggle(id) == n.toggle(id));
                    break;
                case 6: {
                    std::vector<ObjectId> ids;
                    for (std::uint64_t k = rng() % 6; k > 0; --k) {
                        ids.push_back(ObjectId{rng() % (range + 1)});
                    }
                    s.assign(ids);
                    n.assign(ids);
                    break;
                }
                default:
                    if (rng() % 5 == 0) {
                        s.clear();
                        n.order.clear();
                    }
                    break;
            }
            CHECK(s.contains(id) == n.contains(id));
            if (step % 7 == 0) {
                requireSame(s, n);  // ids() compacts; other steps exercise lazy tombstones
            }
        }
        requireSame(s, n);
        Selection copy = s;
        CHECK(copy == s);
        CHECK(copy.ids() == n.order);
    }
}

TEST_CASE("selection keeps order across removals and later additions", "[document][selection]") {
    Selection s;
    for (std::uint64_t i = 1; i <= 100; ++i) {
        s.add(ObjectId{i});
    }
    for (std::uint64_t i = 1; i <= 100; i += 2) {
        s.remove(ObjectId{i});  // enough tombstones to trigger compaction inside remove
    }
    s.add(ObjectId{1});
    s.remove(ObjectId{100});
    s.add(ObjectId{500});
    std::vector<ObjectId> expected;
    for (std::uint64_t i = 2; i < 100; i += 2) {
        expected.push_back(ObjectId{i});
    }
    expected.push_back(ObjectId{1});
    expected.push_back(ObjectId{500});
    CHECK(s.ids() == expected);
    for (ObjectId id : expected) {
        CHECK(s.contains(id));
    }
    CHECK_FALSE(s.contains(ObjectId{3}));
    // Removing the tail repeatedly leaves the rest intact.
    s.remove(ObjectId{500});
    s.remove(ObjectId{1});
    expected.resize(expected.size() - 2);
    CHECK(s.ids() == expected);
}

namespace {

enum class RemoveOrder { Forward, Reverse, Random };

void removeAll(std::uint64_t count, RemoveOrder order) {
    std::vector<ObjectId> ids;
    ids.reserve(count);
    for (std::uint64_t i = 1; i <= count; ++i) {
        ids.push_back(ObjectId{i});
    }
    Selection s;
    s.assign(ids);
    if (order == RemoveOrder::Reverse) {
        std::reverse(ids.begin(), ids.end());
    } else if (order == RemoveOrder::Random) {
        std::mt19937 rng(7);
        std::shuffle(ids.begin(), ids.end(), rng);
    }
    for (ObjectId id : ids) {
        s.remove(id);
    }
    REQUIRE(s.empty());
}

}  // namespace

TEST_CASE("selection removal scales near-linearly in every order", "[document][selection][perf]") {
    for (const RemoveOrder order : {RemoveOrder::Forward, RemoveOrder::Reverse, RemoveOrder::Random}) {
        const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
            3, [&] { removeAll(50'000, order); }, [&] { removeAll(100'000, order); });
        INFO("order " << static_cast<int>(order) << ": " << t.smallSeconds << " -> " << t.largeSeconds << " s, ratio "
                      << t.ratio());
        // Linear is 2 (hash-table cache effects add some noise in random order); quadratic is 4.
        CHECK(t.ratio() <= 3.0);
        CHECK(t.largeSeconds < 2.0);
    }
}
