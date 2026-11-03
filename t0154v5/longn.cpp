// T-0154 r5: long drag toolpath on every shipped post (Mach3 [N] reaches 6 digits after 10k lines).
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"
using namespace carvetto;
using toolpath::Move; using toolpath::Toolpath; using toolpath::ToolSnapshot;
int main(int argc, char** argv) {
  int n = argc > 1 ? std::atoi(argv[1]) : 12000;
  ToolSnapshot t; t.name="Drag 90"; t.toolNumber=3; t.diameter=0.5; t.feedRate=600; t.plungeRate=200; t.spindleSpeed=0; t.spindleOff=true; t.toolType=toolpath::kDiamondDragToolType;
  Toolpath tp; tp.name="Engrave"; tp.tool=t; tp.moves.push_back(Move::rapid({0,0,5})); tp.moves.push_back(Move::plunge({0,0,-0.1}));
  for (int i = 1; i <= n; ++i) tp.moves.push_back(Move::feed({0.01*i, (i%2)*0.5, -0.1}));
  tp.moves.push_back(Move::retract({0.01*n,0,5}));
  for (const char* p : {"grbl_mm","grbl_inch","linuxcnc_mm","linuxcnc_inch","mach3_mm","mach3_inch","mach4_mm","marlin_mm"}) {
    std::ifstream f(std::string(CARVETTO_POSTS_DIR) + "/" + p + ".pp"); std::stringstream ss; ss << f.rdbuf();
    auto pp = post::PostProcessor::parse(ss.str());
    auto r = pp.value().run({tp}, test::fixtureContext());
    printf("%-14s %d moves: %s\n", p, n, r.ok() ? "ok" : r.error().describe().substr(0, 220).c_str());
  }
}
