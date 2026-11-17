// T-0334: the rotary refusals and warnings of the post, picked out of a Save Toolpaths run; and the wrapped
// surface that rotary simulation frames carry for the app's wrapped playback.

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "carvetto/viewmodel/rotary_post_report.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "carvetto/viewmodel/simulation_controller.hpp"
#include "carvetto/viewmodel/wrapped_view.hpp"
#include "toolpath_test_support.hpp"

namespace fs = std::filesystem;
using namespace carvetto;
using namespace carvetto::viewmodel;
using toolpath::Move;

namespace {

document::Document rotaryDoc(double cutZ, document::RotaryAxis axis = document::RotaryAxis::X) {
    document::Document doc(document::Job::rotary(60, 20, document::Units::Millimetres, axis));
    document::ToolpathRecord rec;
    rec.name = "groove";
    rec.tool.diameter = 3.0;
    rec.tool.spindleSpeed = 12000.0;
    rec.tool.feedRate = 1000.0;
    rec.tool.plungeRate = 300.0;
    rec.moves = document::shareMoves({Move::rapid({5, 10, 15}), Move::plunge({5, 10, cutZ}),
                                      Move::feed({55, 10, cutZ}), Move::retract({55, 10, 15})});
    REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(rec)));
    return doc;
}

/// The shipped GRBL post, with `extra` lines appended, written to the work directory.
PostInfo postWith(const std::string& name, const std::string& extra) {
    const std::string base = testing::readTextFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp");
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "rotary_post_report";
    fs::create_directories(dir);
    const fs::path path = dir / (name + ".pp");
    {
        std::ofstream out(path, std::ios::binary);
        out << base << "\n" << extra << "\n";
    }
    PostInfo info;
    info.name = name;
    info.path = path;
    info.extension = "nc";
    info.units = post::Units::Millimetres;
    return info;
}

SaveOutput postRun(const document::Document& doc, const PostInfo& post) {
    SaveOptions opt;
    opt.directory = fs::path(CARVETTO_VM_WORK_DIR) / "rotary_post_report";
    opt.jobName = "r";
    const SaveJob job = prepareSave(doc, {}, post, opt);
    REQUIRE(job.error.empty());
    return generateOutput(job);
}

bool anyMentions(const RotaryPostReport& r, const std::string& text) {
    for (const RotaryPostIssue& i : r.issues) {
        if (i.message.find(text) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("rotary post report: a post that does not wrap is a warning with advice", "[rotary][post_report]") {
    const document::Document doc = rotaryDoc(8.0);
    const SaveOutput out = postRun(doc, postWith("plain", ""));
    INFO(out.error);
    REQUIRE(out.ok);
    const RotaryPostReport r = rotaryPostReport(out, doc.job());
    REQUIRE(r.issues.size() == 1);
    CHECK_FALSE(r.refused());
    CHECK_FALSE(r.issues[0].refusal);
    CHECK(anyMentions(r, "does not wrap"));
    CHECK(r.issues[0].advice.find("ROTARY_WRAP") != std::string::npos);
}

TEST_CASE("rotary post report: wrapping the axis the cylinder lies along is a refusal", "[rotary][post_report]") {
    const document::Document doc = rotaryDoc(8.0);  // along X
    const SaveOutput out = postRun(doc, postWith("wrongaxis", "ROTARY_WRAP_X = \"A\""));
    REQUIRE_FALSE(out.ok);
    const RotaryPostReport r = rotaryPostReport(out, doc.job());
    REQUIRE(r.issues.size() == 1);
    CHECK(r.refused());
    CHECK(r.issues[0].refusal);
    CHECK(anyMentions(r, "lies along X"));
    CHECK_FALSE(r.issues[0].advice.empty());
}

TEST_CASE("rotary post report: a move at or below the cylinder axis is a refusal", "[rotary][post_report]") {
    const document::Document doc = rotaryDoc(-1.0);
    const SaveOutput out = postRun(doc, postWith("belowaxis", "ROTARY_WRAP_Y = \"A\""));
    REQUIRE_FALSE(out.ok);
    const RotaryPostReport r = rotaryPostReport(out, doc.job());
    REQUIRE(r.issues.size() == 1);
    CHECK(r.refused());
    CHECK(anyMentions(r, "at or below the cylinder axis"));
}

TEST_CASE("rotary post report: a clean wrapped run has nothing to report, other jobs never do", "[rotary][post_report]") {
    const document::Document doc = rotaryDoc(8.0);
    const SaveOutput out = postRun(doc, postWith("good", "ROTARY_WRAP_Y = \"A\""));
    INFO(out.error);
    REQUIRE(out.ok);
    CHECK(rotaryPostReport(out, doc.job()).empty());

    SaveOutput fake;
    fake.ok = true;
    fake.warnings = {"x: the cylinder is odd"};
    const document::Job flat = document::Job::singleSided(100, 100, 18, document::Units::Millimetres);
    CHECK(rotaryPostReport(fake, flat).empty());
    CHECK(rotaryPostReport(fake, doc.job()).issues.size() == 1);
}

TEST_CASE("rotary simulation frames carry the wrapped surface; flat jobs do not", "[rotary][simulation]") {
    const auto frameFor = [](document::Document& doc) {
        ToolpathSession session(&doc);
        SimulationRequest req;
        req.quality = simulate::Quality::Coarse;
        SimulationSetup setup = makeSimulationSetup(session, req);
        REQUIRE(setup.ok);
        const bool rotary = doc.job().type == document::JobType::Rotary;
        CHECK(setup.cylinder.has_value() == rotary);
        SimulationController c;
        REQUIRE(c.start(setup));
        c.seek(setup.totalMoves);
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(120);
        while (std::chrono::steady_clock::now() < end) {
            const auto f = c.frame();
            if (f && f->move == setup.totalMoves && c.progress().state == SimState::Ready) {
                return f;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        FAIL("the simulation did not finish");
        return std::shared_ptr<const SimulationFrame>();
    };
    document::Document doc = rotaryDoc(8.0);
    const auto frame = frameFor(doc);
    REQUIRE(frame->wrapped);
    REQUIRE(frame->wrapped->ok());
    float lowest = 100.0f;
    for (const float z : frame->wrapped->z) {
        lowest = std::min(lowest, z);
    }
    CHECK(lowest < 9.0f);  // the groove was cut

    document::Document flat(document::Job::singleSided(60, 60, 10, document::Units::Millimetres));
    document::ToolpathRecord rec;
    rec.name = "g";
    rec.tool.diameter = 3.0;
    rec.tool.feedRate = 1000.0;
    rec.tool.plungeRate = 300.0;
    rec.moves = document::shareMoves({Move::rapid({5, 10, 15}), Move::plunge({5, 10, -1}),
                                      Move::feed({55, 10, -1}), Move::retract({55, 10, 15})});
    REQUIRE(flat.execute(std::make_unique<document::AddToolpathCommand>(rec)));
    CHECK_FALSE(frameFor(flat)->wrapped);
}

TEST_CASE("a rotary job without a model still draws its cylinder and paths", "[rotary][wrapped]") {
    const document::Job job = document::Job::rotary(60, 20, document::Units::Millimetres);
    const WrappedSurface blank = wrappedBlankCylinder(job);
    REQUIRE(blank.ok());
    CHECK(blank.z.front() == static_cast<float>(job.rotarySurfaceZ()));
    CHECK_FALSE(wrappedBlankCylinder(document::Job::singleSided(0, 0, 18, document::Units::Millimetres)).ok());

    ModelSurface none;  // no model
    ModelRenderOptions ro;
    ro.width = ro.height = 160;
    OrbitCamera cam;
    cam.viewTop();
    RotaryViewState state;
    const Raster wrapped = renderRotaryModel(none, job, state, cam, ro);
    const Raster empty = renderRotaryModel(none, document::Job::singleSided(60, 60, 10, document::Units::Millimetres),
                                           state, cam, ro);
    CHECK(wrapped.pixels != empty.pixels);
    std::size_t painted = 0;
    for (const std::uint32_t p : wrapped.pixels) {
        painted += p != ro.background ? 1u : 0u;
    }
    CHECK(painted > 1000);
}
