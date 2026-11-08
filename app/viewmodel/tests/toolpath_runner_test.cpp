// T-0221: the off-thread runner of toolpath jobs, and the session's recalculate-all preparation it
// serves. The runner works on copies only, so replacing or editing the document while a batch runs
// must be safe and the stale result must be refused by the session.

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <thread>

#include "carvetto/viewmodel/toolpath_runner.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;

namespace {

struct Setup {
    document::Document doc;
    tooling::ToolDatabase db{cliToolDatabase()};
    ToolpathSession session{&doc};
    document::ObjectId square1;
    Setup() {
        session.setToolDatabase(db);
        square1 = addVector(doc, square(0, 0, 30));
        doc.undoStack().clear();
    }
    ToolpathDefinition def() const {
        ToolpathDefinition d = pocketDefinition(db);
        d.selector = VectorSelector::fromObjects({square1});
        return d;
    }
};

void waitFinished(ToolpathRunner& runner) {
    for (int i = 0; i < 20000 && !runner.finished(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    REQUIRE(runner.finished());
}

}  // namespace

TEST_CASE("runner: a batch runs on a worker and equals the synchronous result", "[toolpath][runner]") {
    Setup s;
    std::vector<ToolpathJob> jobs;
    jobs.push_back(s.session.prepare(s.def()));
    const ToolpathCalculation direct = jobs.front().run();
    ToolpathRunner runner;
    CHECK_FALSE(runner.busy());
    CHECK_FALSE(runner.take().has_value());
    std::atomic<int> notified{0};
    REQUIRE(runner.start(std::move(jobs), [&notified]() { notified.fetch_add(1); }));
    CHECK(runner.busy());
    CHECK_FALSE(runner.start({}, {}));  // one batch at a time
    waitFinished(runner);
    auto results = runner.take();
    REQUIRE(results.has_value());
    REQUIRE(results->size() == 1);
    CHECK(notified.load() == 1);
    CHECK_FALSE(runner.busy());
    REQUIRE((*results)[0].ok);
    CHECK((*results)[0].items.front().record == direct.items.front().record);
    const auto out = s.session.apply((*results)[0]);
    INFO(out.message);
    CHECK(out.ok);
}

TEST_CASE("runner: cancel before the run skips the jobs", "[toolpath][runner]") {
    Setup s;
    std::vector<ToolpathJob> jobs;
    for (int i = 0; i < 3; ++i) {
        jobs.push_back(s.session.prepare(s.def()));
    }
    ToolpathRunner runner;
    REQUIRE(runner.start(std::move(jobs)));
    runner.cancel();
    auto results = runner.take();  // blocks until done
    REQUIRE(results.has_value());
    REQUIRE(results->size() == 3);
    // The first job may have started before the flag was set; later ones may not have produced.
    CHECK((*results)[2].cancelled);
    for (const auto& r : *results) {
        CHECK((r.cancelled || r.ok));
    }
    CHECK(s.doc.toolpaths().empty());
}

TEST_CASE("runner: destroying a running runner cancels and joins", "[toolpath][runner]") {
    Setup s;
    std::vector<ToolpathJob> jobs;
    for (int i = 0; i < 5; ++i) {
        jobs.push_back(s.session.prepare(s.def()));
    }
    {
        ToolpathRunner runner;
        REQUIRE(runner.start(std::move(jobs)));
    }  // no crash, no leak (ASan)
    SUCCEED();
}

TEST_CASE("runner: the document may change or die while a job runs; apply refuses the stale result",
          "[toolpath][runner]") {
    {
        Setup s;
        std::vector<ToolpathJob> jobs;
        jobs.push_back(s.session.prepare(s.def()));
        ToolpathRunner runner;
        REQUIRE(runner.start(std::move(jobs)));
        // Edit the vector while the worker runs.
        REQUIRE(s.doc.execute(std::make_unique<document::TransformObjectsCommand>(
            std::vector<document::ObjectId>{s.square1}, geometry::Transform2::translation({5, 0}))));
        auto results = runner.take();
        REQUIRE(results.has_value());
        const std::size_t steps = s.doc.undoStack().index();
        const auto out = s.session.apply(results->front());
        CHECK_FALSE(out.ok);
        CHECK_FALSE(out.message.empty());
        CHECK(s.doc.toolpaths().empty());
        CHECK(s.doc.undoStack().index() == steps);
    }
    {
        // The document is destroyed while the worker runs: the job holds copies only.
        auto doc = std::make_unique<document::Document>();
        ToolpathSession session(doc.get());
        const tooling::ToolDatabase db = cliToolDatabase();
        session.setToolDatabase(db);
        const auto id = addVector(*doc, square(0, 0, 30));
        ToolpathDefinition d = pocketDefinition(db);
        d.selector = VectorSelector::fromObjects({id});
        std::vector<ToolpathJob> jobs;
        jobs.push_back(session.prepare(d));
        ToolpathRunner runner;
        REQUIRE(runner.start(std::move(jobs)));
        session.setDocument(nullptr);
        doc.reset();
        auto results = runner.take();
        REQUIRE(results.has_value());
        CHECK(results->front().ok);
        const auto out = session.apply(results->front());
        CHECK_FALSE(out.ok);  // no document
    }
}

TEST_CASE("session: prepareRecalculateAll + applyAll equals recalculateAll", "[toolpath][runner]") {
    Setup s;
    REQUIRE(s.session.add(s.def()).ok);
    REQUIRE(s.session.add(vCarveDefinition(s.db, true)).ok);
    REQUIRE(s.doc.execute(std::make_unique<document::TransformObjectsCommand>(
        std::vector<document::ObjectId>{s.square1}, geometry::Transform2::translation({2, 0}))));
    std::vector<ToolpathOutcome> refused;
    std::vector<ToolpathJob> jobs = s.session.prepareRecalculateAll(true, &refused);
    CHECK(refused.empty());
    REQUIRE_FALSE(jobs.empty());
    std::vector<ToolpathCalculation> calcs;
    for (const ToolpathJob& j : jobs) {
        calcs.push_back(j.run());
    }
    const std::size_t steps = s.doc.undoStack().index();
    const BatchOutcome batch = s.session.applyAll(calcs);
    INFO(batch.message);
    CHECK(batch.ok);
    CHECK(s.doc.undoStack().index() == steps + 1);  // one undo step
    for (const ToolpathListItem& it : s.session.list()) {
        CHECK(it.status == ToolpathStatus::Current);
    }
    CHECK(s.session.prepareRecalculateAll(true).empty());
}
