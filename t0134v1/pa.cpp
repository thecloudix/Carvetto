// T-0134 verifier: parser control-char positions
#include <iostream>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "text_util.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
static void T(const std::string& name, const std::string& src){
  auto p=PostProcessor::parse(src);
  std::cout<<name<<": "<<(p.ok()?std::string("OK"):p.error().describe())<<"\n";
}
int main(){
  for(int c=0;c<0x80;++c){ if(c>=0x20&&c!=0x7f) continue; if(c=='\n') continue;
    std::string ch(1,char(c));
    auto a=PostProcessor::parse("begin HEADER\n\"(x"+ch+"y)\"\nend\n");
    bool refused=!a.ok();
    if((c=='\t')==refused) std::cout<<"MISMATCH output line 0x"<<std::hex<<c<<std::dec<<"\n";
  }
  std::string F="\f";
  T("output line", "begin HEADER\n\"(\f[TOOLPATH_NAME])\"\nend\n");
  T("VAR prefix", "VAR N = [TOOLPATH_NAME|A|(\f|1.0]\n");
  T("VAR name", "VAR N\fX = [TOOLPATH_NAME|A||1.0]\n");
  T("setting value", "POST_NAME = \"a\fb\"\n");
  T("SUBSTITUTE", "SUBSTITUTE = \"(\f)\"\n");
  T("SUBSTITUTE end", "SUBSTITUTE = \"([)]\f\"\n");
  T("begin name", "begin HEA\fDER\nend\n");
  T("comment +", "+ a\fb\n");
  T("comment in block", "begin HEADER\n+ a\fb\n\"x\"\nend\n");
  T("leading ff (trimmed)", "begin HEADER\n\f\"(x)\"\nend\n");
  T("trailing ff (trimmed)", "begin HEADER\n\"(x)\"\f\nend\n");
  T("CRLF", "begin HEADER\r\n\"(x)\"\r\nend\r\n");
  T("lone CR mac", "begin HEADER\r\"(x)\"\rend\r");
  T("CRCRLF", "begin HEADER\r\r\n\"(x)\"\r\r\nend\r\n");
  T("CR before close quote", "begin HEADER\n\"(x)\r\"\nend\n");
  T("NUL mid", std::string("begin HEADER\n\"(x\0y)\"\nend\n",24));
  T("high byte 0xA0", "begin HEADER\n\"(\xA0[TOOLPATH_NAME])\"\nend\nbegin RAPID_MOVE\nend\n");
  T("0x85", "begin HEADER\n\"(\x85[TOOLPATH_NAME])\"\nend\n");
  // commentPositionAt
  using namespace carvetto::post::detail;
  for(const char* s:{"(\f","(\v","(\r","( \x1b","(\x7f","(LOG\fOPEN"}){ auto cp=commentPositionAt(s); std::cout<<"cpa plain="<<cp.plain<<" lead="<<cp.lead<<"\n";}
  // high byte run
  auto p=PostProcessor::parse("begin HEADER\n\"(\xA0[TOOLPATH_NAME])\"\nend\nbegin RAPID_MOVE\nend\n");
  if(p.ok()){Toolpath t; t.name="MSG,hi"; t.tool.feedRate=1000; t.tool.plungeRate=300; t.tool.spindleSpeed=1e4; t.moves={Move::rapid({0,0,5})};
   auto r=p.value().run({t},{}); std::cout<<"0xA0 run: "; for(unsigned char c: r.value().text){ if(c<0x20||c>0x7e) std::cout<<"\\x"<<std::hex<<int(c)<<std::dec; else std::cout<<c;} std::cout<<"\n";}
}
