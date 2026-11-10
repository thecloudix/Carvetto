#pragma once

// Shared fixtures of the toolpath session tests (T-0220): the CLI test tool database, small
// documents and helpers. Header-only, test use only.

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

namespace carvetto::viewmodel::testing {

inline std::string readTextFile(const std::string& path) {
    std::ifstream in(std::filesystem::path(path), std::ios::binary);
    REQUIRE(in);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// The tool database of the CLI end-to-end tests: end-mill-3mm, end-mill-6mm, drill-3mm, v-bit-90.
inline tooling::ToolDatabase cliToolDatabase() {
    const tooling::ImportResult r =
        tooling::importToolDatabaseJson(readTextFile(std::string(CARVETTO_CLI_DATA_DIR) + "/tools.json"));
    REQUIRE(r.ok);
    return r.database;
}

inline document::ToolSource source(const std::string& tool, const std::string& machine = {},
                                   const std::string& material = {}) {
    return document::ToolSource{tool, machine, material};
}

inline geometry::Contour square(double x, double y, double size) {
    return geometry::Contour::rectangle(geometry::Box2{{x, y}, {x + size, y + size}});
}

inline document::ObjectId addVector(document::Document& doc, geometry::Contour contour,
                                    document::LayerId layer = {}) {
    document::Object obj = document::Object::vector(std::move(contour), layer ? layer : doc.defaultLayer(),
                                                    doc.activeSheet());
    auto cmd = std::make_unique<document::AddObjectCommand>(std::move(obj));
    const document::AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

inline document::LayerId addLayer(document::Document& doc, const std::string& name) {
    document::Layer l;
    l.name = name;
    return doc.insertLayer(l);
}

/// A pocket-ready definition: a 6 mm end mill, cut depth 3.
inline ToolpathDefinition pocketDefinition(const tooling::ToolDatabase& db, double depth = 3.0) {
    ToolpathForm form(document::ToolpathOperation::Pocket, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-6mm")).ok);
    REQUIRE(form.setField("cutDepth", std::to_string(depth)).ok);
    return form.definition();
}

inline ToolpathDefinition profileDefinition(const tooling::ToolDatabase& db, double depth = 3.0) {
    ToolpathForm form(document::ToolpathOperation::Profile, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setField("cutDepth", std::to_string(depth)).ok);
    return form.definition();
}

inline ToolpathDefinition drillDefinition(const tooling::ToolDatabase& db) {
    ToolpathForm form(document::ToolpathOperation::Drill, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source("drill-3mm")).ok);
    REQUIRE(form.setField("cutDepth", "4").ok);
    return form.definition();
}

inline ToolpathDefinition engraveDefinition(const tooling::ToolDatabase& db) {
    ToolpathForm form(document::ToolpathOperation::Engrave, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    REQUIRE(form.setField("depth", "0.5").ok);
    return form.definition();
}

inline ToolpathDefinition chamferDefinition(const tooling::ToolDatabase& db) {
    ToolpathForm form(document::ToolpathOperation::Chamfer, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    REQUIRE(form.setField("width", "2").ok);
    return form.definition();
}

inline ToolpathDefinition threadMillDefinition(const tooling::ToolDatabase& db) {
    ToolpathForm form(document::ToolpathOperation::ThreadMill, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    return form.definition();
}

inline ToolpathDefinition inlayDefinition(const tooling::ToolDatabase& db) {
    ToolpathForm form(document::ToolpathOperation::Inlay, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    return form.definition();
}

inline ToolpathDefinition vCarveDefinition(const tooling::ToolDatabase& db, bool withClearance) {
    ToolpathForm form(document::ToolpathOperation::VCarving, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    REQUIRE(form.setField("chordal", "0.05").ok);
    if (withClearance) {
        REQUIRE(form.setField("flatDepth", "1.5").ok);
        REQUIRE(form.setClearanceTools(db, {source("end-mill-3mm")}).ok);
    }
    return form.definition();
}

}  // namespace carvetto::viewmodel::testing
