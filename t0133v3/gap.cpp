// T-0133 r2: cross-value active comment repros (custom .pp) + documented gap + edge cases.
#include <iostream>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
static std::string esc(const std::string& s){std::string o;for(unsigned char c:s){if(c=='\n')o+="\\n\n";else if(c<0x20||c>=0x7f){char b[8];snprintf(b,8,"\\x%02x",c);o+=b;}else o+=char(c);}return o;}
static void run(const std::string& pp, const std::string& tp, const std::string& tool, PostContext ctx={}){
  auto p=PostProcessor::parse(pp); if(!p.ok()){std::cout<<"PARSE "<<p.error().describe()<<"\n";return;}
  Toolpath t; t.name=tp; t.tool.name=tool; t.tool.feedRate=1000; t.tool.plungeRate=300; t.tool.spindleSpeed=1e4; t.moves={Move::rapid({0,0,5})};
  auto r=p.value().run({t},ctx); std::cout<<(r.ok()?esc(r.value().text):"ERR "+r.error().describe()+"\n");
}
int main(){
  const std::string rm="begin RAPID_MOVE\nend\n";
  std::cout<<"--- adjacent tool name + toolpath name\n";
  run("begin HEADER\n\"([TOOLNAME][TOOLPATH_NAME])\"\n\"([TOOLNAME] [TOOLPATH_NAME])\"\n\";[TOOLNAME][TOOLPATH_NAME]\"\nend\n"+rm,",/home/cnc/linuxcnc/configs/mill/mill.hal","LOGOPEN");
  std::cout<<"--- TP_DIR + TP_FILENAME\n";
  { PostContext c; c.fileDir="LOGOPEN"; c.fileName=",/tmp/x"; run("begin HEADER\n\"([TP_DIR][TP_FILENAME])\"\nend\n"+rm,"a","b",c); }
  std::cout<<"--- documented gap: template comma\n";
  run("begin HEADER\n\"([TOOLPATH_NAME],x)\"\nend\n"+rm,"LOGOPEN","t");
  std::cout<<"--- probe split across values\n";
  run("begin HEADER\n\"([TOOLNAME][TOOLPATH_NAME])\"\nend\n"+rm,"OPEN /tmp/p","PROBE");
  std::cout<<"--- value with only spaces then value\n";
  run("begin HEADER\n\"([TOOLNAME][TOOLPATH_NAME])\"\nend\n"+rm,"MSG,x","\t \n");
  std::cout<<"--- prefix variants\n";
  run("VAR N = [TOOLPATH_NAME|A|  (  |1.0]\nbegin HEADER\n\"G0[TOOLPATH_NAME])\"\nend\n"+rm,"msg,x","t");
  run("VAR N = [TOOLPATH_NAME|A|;|1.0]\nbegin HEADER\n\"G0 [TOOLPATH_NAME]\"\nend\n"+rm,"PY,import os","t");
  std::cout<<"--- unbalanced/nested parens\n";
  run("begin HEADER\n\"((a)[TOOLPATH_NAME])\"\n\"(a))[TOOLPATH_NAME]\"\n\"(a;b[TOOLPATH_NAME])\"\nend\n"+rm,"MSG,)x(","t");
  std::cout<<"--- keyword variants at start\n";
  for(const char* n: {"m s g,x"," \t PyRun ,x","LoG,x","AXIS,stop","debug,#<_x>","logappend,f","probeclose","PROBEOPEN","LOG CLOSE","\xC3\x96,x","l\xC3\xB6g,x","msg\xEF\xBC\x8Cx","msg\xC0\xACx",",msg,x","_msg,x"})
    run("begin HEADER\n\"([TOOLPATH_NAME])\"\n\";[TOOLPATH_NAME]\"\nend\n"+rm,n,"t");
}
