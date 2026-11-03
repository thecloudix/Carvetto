#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
int main(int argc,char**argv){ std::ifstream f(argv[1]); std::stringstream s; s<<f.rdbuf();
  auto pr=PostProcessor::parse(s.str()); if(!pr.ok()){std::cout<<pr.error().describe()<<"\n";return 1;}
  Toolpath t; t.name="p"; t.tool.name="EM"; t.tool.toolNumber=1; t.tool.diameter=6; t.tool.feedRate=std::atof(argv[2]); t.tool.plungeRate=std::atof(argv[3]); t.tool.spindleSpeed=std::atof(argv[4]);
  t.moves={Move::rapid({0,0,5}),Move::plunge({0,0,-1}),Move::feed({20,0,-1}),Move::retract({20,0,5})};
  PostContext c; c.safeZ=5; auto r=pr.value().run({t},c);
  std::cout<<(r.ok()?r.value().text:"ERR "+r.error().describe()+"\n");
}
