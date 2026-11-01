#pragma once

// Small helpers for the pocket tests (T-0123): pocket tools are tool-database tools with cutting
// data, and diagnostics carry an issue code plus a message.

#include <string>
#include <vector>

#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/pocket.hpp"

namespace carvetto::test::pocket {

/// An end mill as a pocket tool: the stepover and pass depth go into its default cutting data.
inline toolpath::PocketTool endMill(const std::string& name, int number, double diameter, double stepover,
                                    double passDepth, double feed = 1000.0, double plunge = 300.0,
                                    double spindle = 18000.0) {
    toolpath::PocketTool t;
    t.tool.id = name;
    t.tool.name = name;
    t.tool.type = tooling::ToolType::EndMill;
    t.tool.diameter = diameter;
    tooling::CuttingData& cd = t.tool.defaultCuttingData;
    cd.toolNumber = number;
    cd.feedRate = feed;
    cd.plungeRate = plunge;
    cd.spindleSpeed = spindle;
    cd.stepover = stepover;
    cd.passDepth = passDepth;
    return t;
}

/// The cutting data the pocket uses for `t` (its default data; the tests set no override).
inline tooling::CuttingData& cuttingOf(toolpath::PocketTool& t) {
    return t.tool.defaultCuttingData;
}
inline const tooling::CuttingData& cuttingOf(const toolpath::PocketTool& t) {
    return t.tool.defaultCuttingData;
}

/// The messages of `diagnostics`, in order.
inline std::vector<std::string> messagesOf(const std::vector<toolpath::ToolpathDiagnostic>& diagnostics) {
    std::vector<std::string> out;
    for (const toolpath::ToolpathDiagnostic& d : diagnostics) {
        out.push_back(d.message);
    }
    return out;
}

}  // namespace carvetto::test::pocket
