#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "post_fixture.hpp"
using namespace carvetto;
using toolpath::Move; using toolpath::Toolpath; using toolpath::ToolSnapshot;
static ToolSnapshot mill(int n, std::string nm="End Mill 6") { ToolSnapshot t; t.name=nm; t.toolNumber=n; t.diameter=6; t.feedRate=1000; t.plungeRate=300; t.spindleSpeed=18000; return t; }
static ToolSnapshot drg(int n, std::string nm="Diamond Drag (90 deg)") { ToolSnapshot t=mill(n,nm); t.diameter=0.3; t.spindleSpeed=0; t.spindleOff=true; return t; }
static Toolpath tp(std::string nm, ToolSnapshot t, double x){ double d = t.spindleOff? -0.1 : -3.0; Toolpath p; p.name=nm; p.tool=t;
  p.moves={Move::rapid({x,10,5}),Move::plunge({x,10,d}),Move::feed({x+5,10,d}),Move::arcCW({x+10,10,d},{x+7.5,10}),Move::retract({x+10,10,5})}; return p; }
int main(int, char** argv) {
  std::string dir = argv[1];
  const char* stems[]={"grbl_mm","grbl_inch","mach3_mm","mach3_inch","mach4_mm","linuxcnc_mm","linuxcnc_inch","marlin_mm"};
  std::vector<std::pair<std::string,std::vector<Toolpath>>> sc = {
    {"s1_first", {tp("D",drg(3),0)}},
    {"s2_same", {tp("D1",drg(3),0), tp("D2",drg(3),20)}},
    {"s3_mill_to_drag", {tp("M",mill(1),0), tp("D",drg(3),20)}},
    {"s4_drag_to_mill", {tp("D",drg(3),0), tp("M",mill(1),20)}},
    {"s5_drag_to_drag_other", {tp("D3",drg(3),0), tp("D4",drg(4,"Drag 120"),20)}},
    {"s6_same_numname_spin_to_off", {tp("M",mill(3,"X"),0), tp("D",drg(3,"X"),20)}},
    {"s7_same_numname_off_to_spin", {tp("D",drg(3,"X"),0), tp("M",mill(3,"X"),20)}},
    {"s8_mixed", {tp("M1",mill(1),0), tp("D",drg(3),20), tp("D'",drg(3),40), tp("M2",mill(2,"Mill2"),60), tp("D4",drg(4,"Drag 120"),80), tp("M2b",mill(2,"Mill2"),100)}},
  };
  for (auto st : stems) {
    std::ifstream in(std::string(CARVETTO_POSTS_DIR)+"/"+st+".pp"); std::stringstream ss; ss<<in.rdbuf();
    auto pp = post::PostProcessor::parse(ss.str()); if(!pp.ok()){printf("parse %s\n",st);return 1;}
    for (auto& [name, tps] : sc) {
      auto r = pp.value().run(tps, test::fixtureContext());
      if (!r.ok()) { printf("%s %s REFUSED %s\n", st, name.c_str(), r.error().describe().c_str()); continue; }
      if (!r.value().warnings.empty()) printf("%s %s warnings %zu\n", st, name.c_str(), r.value().warnings.size());
      std::ofstream(dir+"/"+st+"__"+name+".nc", std::ios::binary) << r.value().text;
    }
  }
  printf("done\n");
}
