// T-0014 AC4: Document::checkIntegrity() is O(N log N) in the number of objects whatever the group
// depth or width, because file loading (T-0012) runs it on untrusted input. Documents are built with
// the raw mutators (they do not maintain group links, so the tests set both sides explicitly).
//
// Timing method, as for the AC4 undo/redo test in commands_test.cpp: process CPU time from
// `std::clock()` (on POSIX, unaffected by other processes under parallel ctest), best of two runs.
// Each shape gets an absolute bound (1 s with NDEBUG; 10 s in Debug, where the Debug standard
// library is several times slower) and a machine-independent scaling check, time(2N) < 3 x time(N):
// N log N work gives a ratio slightly above 2, while the quadratic behaviour this replaces gave >= 4.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/object.hpp"

using namespace carvetto::document;
using Kind = IntegrityIssue::Kind;

namespace {

#ifdef NDEBUG
constexpr double kBoundSeconds = 1.0;
#else
constexpr double kBoundSeconds = 10.0;
#endif
/// Lower bound for the time(N) term of the scaling check, so a coarse `std::clock()` tick
/// (~16 ms on Windows) cannot fail it on a fast optimised build.
constexpr double kClockFloorSeconds = 0.05;

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

/// Best-of-two process CPU seconds for one checkIntegrity() call; `issues` receives the result.
double timeCheck(const Document& doc, std::vector<IntegrityIssue>& issues) {
    double best = std::numeric_limits<double>::infinity();
    for (int run = 0; run < 2; ++run) {
        const std::clock_t start = std::clock();
        issues = doc.checkIntegrity();
        const std::clock_t stop = std::clock();
        best = std::min(best, static_cast<double>(stop - start) / static_cast<double>(CLOCKS_PER_SEC));
    }
    return best;
}

}  // namespace

TEST_CASE("checkIntegrity is fast on a 60 000-deep chain of nested groups", "[document][integrity][perf]") {
    Document small;
    buildChain(small, 30000);
    Document big;
    buildChain(big, 60000);
    std::vector<IntegrityIssue> issues;
    const double tSmall = timeCheck(small, issues);
    INFO(firstMessages(issues));
    REQUIRE(issues.empty());
    const double tBig = timeCheck(big, issues);
    INFO(firstMessages(issues));
    REQUIRE(issues.empty());
    INFO("CPU seconds: 30k = " << tSmall << ", 60k = " << tBig);
    CHECK(tBig < kBoundSeconds);
    CHECK(tBig < 3.0 * std::max(tSmall, kClockFloorSeconds));
}

TEST_CASE("checkIntegrity is fast on one group with 200 000 children", "[document][integrity][perf]") {
    Document small;
    buildWide(small, 100000);
    Document big;
    buildWide(big, 200000);
    std::vector<IntegrityIssue> issues;
    const double tSmall = timeCheck(small, issues);
    INFO(firstMessages(issues));
    REQUIRE(issues.empty());
    const double tBig = timeCheck(big, issues);
    INFO(firstMessages(issues));
    REQUIRE(issues.empty());
    INFO("CPU seconds: 100k = " << tSmall << ", 200k = " << tBig);
    CHECK(tBig < kBoundSeconds);
    CHECK(tBig < 3.0 * std::max(tSmall, kClockFloorSeconds));
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

    std::vector<IntegrityIssue> issues;
    const double t = timeCheck(doc, issues);
    INFO("CPU seconds: " << t << "; " << firstMessages(issues));
    CHECK(t < kBoundSeconds);
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
