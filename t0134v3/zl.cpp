#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
// usage: zl <pp> <feed> <plunge> <override|-> [zero-length feed move? 0/1] [tiny length]
int main(int argc,char**argv){ std::ifstream f(argv[1]); std::stringstream s; s<<f.rdbuf();
  auto pr=PostProcessor::parse(s.str()); if(!pr.ok()){std::cout<<pr.error().describe()<<"\n";return 1;}
  Toolpath t; t.name="p"; t.tool.name="EM"; t.tool.toolNumber=1; t.tool.diameter=6; t.tool.feedRate=std::strtod(argv[2],nullptr); t.tool.plungeRate=std::strtod(argv[3],nullptr); t.tool.spindleSpeed=18000;
  double len=argc>5?std::strtod(argv[5],nullptr):20;
  t.moves={Move::rapid({0,0,5}),Move::plunge({0,0,-1}),Move::feed({len,0,-1}),Move::feed({len,0,-1}),Move::arcCW({len,0,-1},{len+5,0}),Move::retract({len,0,5})};
  if(std::string(argv[4])!="-") t.moves[2].feedOverride=std::strtod(argv[4],nullptr), t.moves[3].feedOverride=std::strtod(argv[4],nullptr), t.moves[4].feedOverride=std::strtod(argv[4],nullptr);
  PostContext c; c.safeZ=5; auto r=pr.value().run({t},c);
  std::cout<<(r.ok()?r.value().text:"ERR "+r.error().describe()+"\n");
}
