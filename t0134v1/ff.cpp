#include <iostream>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
int main(){ for(const char* ws:{"\f","\v","\r"}){
  auto p=PostProcessor::parse(std::string("begin HEADER\n\"(")+ws+"[TOOLPATH_NAME])\"\nend\nbegin RAPID_MOVE\nend\n");
  if(!p.ok()){std::cout<<"PARSE "<<p.error().describe()<<"\n";continue;}
  Toolpath t; t.name="MSG,hi"; t.tool.feedRate=1000; t.tool.plungeRate=300; t.tool.spindleSpeed=1e4; t.moves={Move::rapid({0,0,5})};
  auto r=p.value().run({t},{}); for(unsigned char c: r.value().text){ if(c<0x20) std::cout<<"\\x"<<std::hex<<int(c)<<std::dec; else std::cout<<c;} std::cout<<"\n"; } }
