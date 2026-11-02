#include <iostream>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
int main(){
  auto pp=PostProcessor::parse("VAR X = [X|A|X|1.3]\nVAR Y = [Y|A|Y|1.3]\nVAR Z = [Z|A|Z|1.3]\nbegin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\nbegin FEED_MOVE\n\"G1[X][Y][Z]\"\nend\n").value();
  Toolpath tp; tp.name="semi"; tp.tool.feedRate=1000; tp.tool.plungeRate=300; tp.tool.spindleSpeed=1e4;
  // 25 mm radius semicircle, centre (0,0): (25,0) -> (-25,0) CCW over the top
  tp.moves={Move::rapid({25,0,-1}), Move::arcCCW({-25,0,-1},{0,0})};
  for(double tol: {0.01, 1e-12, 1e-15, 1e-17, 1e-20}){
    PostContext c; c.arcTolerance=tol; auto r=pp.run({tp},c);
    std::string t=r.ok()? r.value().text : "ERR "+r.error().describe();
    size_t lines=0; for(char ch:t) lines+=ch=='\n';
    std::cout<<"tol="<<tol<<" lines="<<lines<<"\n"; if(lines<5) std::cout<<t;
  }
}
