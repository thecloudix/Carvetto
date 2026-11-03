#include <cstdio>
#include <fstream>
#include <sstream>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/drill.hpp"
using namespace carvetto; using namespace carvetto::toolpath;
int main() {
  tooling::Tool t; t.id="d"; t.name="D"; t.type=tooling::ToolType::Drill; t.diameter=3; t.includedAngle=2; t.defaultCuttingData = {2.5,0,0,0,0.0,600,250,0,2};
  auto r = drillToolpath({geometry::Contour::circle({0,0},3)}, t, std::nullopt, DrillParams{.cutDepth = 3});
  std::ifstream in("../posts/grbl_mm.pp"); std::stringstream s; s << in.rdbuf();
  auto pp = post::PostProcessor::parse(s.str()); post::PostContext ctx; auto res = pp.value().run({r.toolpath}, ctx);
  std::printf("toolpath ok=%d post ok=%d %s\n", r.ok(), res.ok(), res.ok() ? "" : res.error().describe().c_str());
}
