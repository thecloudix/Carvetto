#include "carvetto/toolpath/operation.hpp"

#include <algorithm>
#include <optional>
#include <vector>

namespace carvetto::toolpath {

bool hasIssue(const std::vector<ToolpathDiagnostic>& diagnostics, ToolpathIssue issue) {
    return std::any_of(diagnostics.begin(), diagnostics.end(),
                       [&](const ToolpathDiagnostic& d) { return d.issue == issue; });
}

const tooling::CuttingData& effectiveCuttingData(const tooling::Tool& tool,
                                                  const std::optional<tooling::CuttingData>& chosen) {
    return chosen ? *chosen : tool.defaultCuttingData;
}

ToolSnapshot toolSnapshot(const tooling::Tool& tool, const tooling::CuttingData& data) {
    ToolSnapshot snap;
    snap.name = tool.name;
    snap.toolNumber = data.toolNumber;
    snap.diameter = tool.diameter;
    snap.feedRate = data.feedRate;
    snap.plungeRate = data.plungeRate;
    snap.spindleSpeed = data.spindleSpeed;
    return snap;
}

PassTool passTool(const tooling::Tool& tool, const tooling::CuttingData& data, double bottomDepth) {
    PassTool p;
    p.snapshot = toolSnapshot(tool, data);
    p.radius = 0.5 * tooling::cuttingWidthAtDepth(tool, bottomDepth);
    p.stepover = data.stepover;
    p.passDepth = data.passDepth;
    return p;
}

}  // namespace carvetto::toolpath
