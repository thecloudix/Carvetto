#include "carvetto/toolpath/operation.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace carvetto::toolpath {

ToolpathDiagnostic refusedContourDiagnostic(std::size_t contour, const std::string& vectorName) {
    return ToolpathDiagnostic{ToolpathIssue::ContourRefused, contour,
                              vectorName +
                                  " has an arc or curve too large to approximate within the tolerance (it would need "
                                  "more than a million chords; a radius beyond about 1e9 mm); it is not cut",
                              std::nullopt};
}

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
