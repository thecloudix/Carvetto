// T-0014 AC4: Document::checkIntegrity() is O(N log N) in the number of objects whatever the group
// depth or width, because file loading (T-0012) runs it on untrusted input. Documents are built with
// the raw mutators (they do not maintain group links, so the tests set both sides explicitly).
//
// T-0313: the tests count work instead of timing it. checkIntegrity() reports the elementary steps
// of its object, link and parent-chain passes (one per object, per listed child, per parent-chain
// object walked or marked), so the check cannot be affected by scheduler load. Linear work is
// about 5 steps per object (a chain: 1 object + 1 child + 1 listed pair + walk + mark); the
// quadratic behaviour this replaced (walking every chain to its end) takes ~N^2/2 steps.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/object.hpp"

using namespace carvetto::document;
using Kind = IntegrityIssue::Kind;

namespace {

/// Most steps per object the checks may take (linear work is ~5; quadratic is >= N/2).
constexpr std::size_t kStepsPerObject = 8;

std::size_t countKind(const std::vector<IntegrityIssue>& issues, Kind kind) {
    return static_cast<std::size_t>(std::count_if(
        issues.begin(), issues.end(), [kind](const IntegrityIssue& i) { return i.kind == kind; }));
}

std::string firstMessages(const std::vector<IntegrityIssue>& issues) {
    std::string out;
    for (std::size_t i = 0; i < issues.size() && i < 5; ++i) {
        out += issues[i].message + "; ";
    }
    return out;
}

ObjectId idAt(std::uint64_t base, std::size_t i) {
    return ObjectId{base + static_cast<std::uint64_t>(i)};
}

/// A chain of `depth` nested groups, each listing the next as its only child; the innermost
/// object is a vector. Returns the ids from outermost to innermost.
std::vector<ObjectId> buildChain(Document& doc, std::size_t depth) {
    const std::uint64_t base = doc.idGenerator().peek();
    std::vector<ObjectId> ids;
    ids.reserve(depth);
    for (std::size_t i = 0; i < depth; ++i) {
        Object obj;
        obj.id = idAt(base, i);
        obj.layer = doc.defaultLayer();
        obj.sheet = doc.activeSheet();
        if (i > 0) {
            obj.parent = idAt(base, i - 1);
        }
        if (i + 1 < depth) {
            obj.data = GroupObject{{idAt(base, i + 1)}};
        }
        ids.push_back(doc.insertObject(std::move(obj)));
    }
    return ids;
}

/// One group listing `width` vector children. Returns the group id.
ObjectId buildWide(Document& doc, std::size_t width) {
    const std::uint64_t base = doc.idGenerator().peek();
    Object groupObj;
    groupObj.id = idAt(base, 0);
    groupObj.layer = doc.defaultLayer();
    groupObj.sheet = doc.activeSheet();
    GroupObject g;
    g.children.reserve(width);
    for (std::size_t i = 1; i <= width; ++i) {
        g.children.push_back(idAt(base, i));
    }
    groupObj.data = std::move(g);
    const ObjectId groupId = doc.insertObject(std::move(groupObj));
    for (std::size_t i = 1; i <= width; ++i) {
        Object child;
        child.id = idAt(base, i);
        child.layer = doc.defaultLayer();
        child.sheet = doc.activeSheet();
        child.parent = groupId;
        doc.insertObject(std::move(child));
    }
    return groupId;
}

/// checkIntegrity() steps for `doc`, which must report no issues.
std::size_t checkSteps(const Document& doc) {
    std::size_t steps = 0;
    const std::vector<IntegrityIssue> issues = doc.checkIntegrity(&steps);
    INFO(firstMessages(issues));
    REQUIRE(issues.empty());
    return steps;
}

}  // namespace

TEST_CASE("checkIntegrity is fast on a 60 000-deep chain of nested groups", "[document][integrity][perf]") {
    for (const std::size_t depth : {std::size_t{30000}, std::size_t{60000}}) {
        Document doc;
        buildChain(doc, depth);
        const std::size_t steps = checkSteps(doc);
        INFO("depth " << depth << " steps " << steps);
        CHECK(steps <= kStepsPerObject * depth);
    }
}

TEST_CASE("checkIntegrity is fast on one group with 200 000 children", "[document][integrity][perf]") {
    for (const std::size_t width : {std::size_t{100000}, std::size_t{200000}}) {
        Document doc;
        buildWide(doc, width);
        const std::size_t steps = checkSteps(doc);
        INFO("width " << width << " steps " << steps);
        CHECK(steps <= kStepsPerObject * width);
    }
}

TEST_CASE("checkIntegrity finds a cycle closed at depth 50 000 of a 60 000-deep chain",
          "[document][integrity][perf]") {
    Document doc;
    const std::vector<ObjectId> ids = buildChain(doc, 60000);
    constexpr std::size_t kDepth = 50000;
    // Close the loop: the outermost group becomes a child of the group at depth 50 000 (both sides
    // of the link set, so the only issues are the loop itself).
    doc.findObject(ids[0])->parent = ids[kDepth];
    doc.findObject(ids[kDepth])->asGroup()->children.push_back(ids[0]);

    std::size_t steps = 0;
    const std::vector<IntegrityIssue> issues = doc.checkIntegrity(&steps);
    INFO("steps " << steps << "; " << firstMessages(issues));
    CHECK(steps <= kStepsPerObject * ids.size());
    // Every object's parent chain ends in the loop, so each one is reported once, in id order, and
    // nothing else is.
    REQUIRE(countKind(issues, Kind::Cycle) == ids.size());
    REQUIRE(issues.size() == ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        INFO("object index " << i);
        REQUIRE(issues[i].object == ids[i]);
        // A loop object's chain first repeats itself; an object below the loop first repeats the
        // object where its chain enters the loop.
        REQUIRE(issues[i].related == (i <= kDepth ? ids[i] : ids[kDepth]));
    }
}

TEST_CASE("checkIntegrity reports loops the same way at small scale", "[document][integrity]") {
    Document doc;
    const std::vector<ObjectId> ids = buildChain(doc, 6);
    SECTION("self-parent") {
        doc.findObject(ids[5])->parent = ids[5];
        const auto issues = doc.checkIntegrity();
        REQUIRE(countKind(issues, Kind::Cycle) == 1);
        const auto it = std::find_if(issues.begin(), issues.end(),
                                     [](const IntegrityIssue& i) { return i.kind == Kind::Cycle; });
        CHECK(it->object == ids[5]);
        CHECK(it->related == ids[5]);
    }
    SECTION("loop through the middle with a tail") {
        doc.findObject(ids[2])->parent = ids[4];  // loop 2 -> 4 -> 3 -> 2; 5 hangs below 4
        const auto issues = doc.checkIntegrity();
        REQUIRE(countKind(issues, Kind::Cycle) == 4);
        std::vector<std::pair<ObjectId, ObjectId>> cycles;
        for (const auto& issue : issues) {
            if (issue.kind == Kind::Cycle) {
                cycles.emplace_back(issue.object, issue.related);
            }
        }
        const std::vector<std::pair<ObjectId, ObjectId>> expected{
            {ids[2], ids[2]}, {ids[3], ids[3]}, {ids[4], ids[4]}, {ids[5], ids[4]}};
        CHECK(cycles == expected);
        // Objects 0 and 1 are above the loop and are not reported as cycles.
    }
}
