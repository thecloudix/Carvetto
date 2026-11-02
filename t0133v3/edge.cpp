// T-0133 r3 edge cases: behaviour change, boundaries, contexts.
#include <cmath>
#include <iostream>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
static std::string esc(const std::string& s){std::string o;for(unsigned char c:s){if(c=='\n')o+="\\n";else if(c<0x20||c>=0x7f){char b[8];snprintf(b,8,"\\x%02x",c);o+=b;}else o+=char(c);}return o;}
static std::string run(const std::string& line,const std::string& name,const std::string& tool="t",PostContext ctx={},std::vector<Move> mv={Move::rapid({0,0,5})}, std::string extra=""){
  auto p=PostProcessor::parse(extra+"begin HEADER\n\""+line+"\"\nend\nbegin RAPID_MOVE\nend\nbegin FEED_MOVE\n\"G1[X][Y]\"\nend\n"); if(!p.ok()) return "PARSE "+p.error().describe();
  Toolpath t; t.name=name; t.tool.name=tool; t.tool.feedRate=1000; t.tool.plungeRate=300; t.tool.spindleSpeed=1e4; t.moves=mv;
  auto r=p.value().run({t},ctx); return r.ok()?esc(r.value().text):"ERR "+r.error().describe();
}
int main(){
  auto P=[](const char* l,const std::string& n,const std::string& t="t"){std::cout<<l<<"  name='"<<esc(n)<<"' tool='"<<esc(t)<<"' -> "<<run(l,n,t)<<"\n";};
  P("([TOOLPATH_NAME])","Profile,1,2");
  P("(Tool 1: [TOOLPATH_NAME])","a,b");
  P("([TOOLPATH_NAME])","a b,c-d,e");
  P("([TOOLPATH_NAME])",",,,");
  P("([TOOLPATH_NAME])","");
  P("([TOOLPATH_NAME])","   ");
  P("([TOOLPATH_NAME])","Probe Close");
  P("([TOOLPATH_NAME])","logclose x");
  P("([TOOLPATH_NAME])","P\tR\nO\rBEOPEN f");
  P("G0 X1 ;[TOOLPATH_NAME]","msg,x");
  P("G0 X1;[TOOLPATH_NAME]","(msg,x)");
  P(";([TOOLPATH_NAME]","msg,x");
  P("([TOOLS_USED])","x","End,Mill");
  P("(Tools: [TOOLS_USED])","x","End,Mill");
  P("([TOOLPATH_NAME][TOOLNAME][TP_FILENAME][TP_DIR])","P","R");
  { PostContext c; c.fileName="O"; c.fileDir="BEOPEN /f"; std::cout<<"4 values PROBEOPEN -> "<<run("([TOOLPATH_NAME][TOOLNAME][TP_FILENAME][TP_DIR])","P","R",c)<<"\n"; }
  { PostContext c; c.fileName="OPEN"; c.fileDir=",/f"; std::cout<<"4 values LOGOPEN, -> "<<run("([TOOLPATH_NAME][TOOLNAME][TP_FILENAME][TP_DIR])","L","OG",c)<<"\n"; }
  { PostContext c; c.fileName=" "; c.fileDir="  ,/f"; std::cout<<"spaces between -> "<<run("( [TOOLPATH_NAME] [TOOLNAME] [TP_FILENAME] [TP_DIR])","M","S G",c)<<"\n"; }
  // number tag between text values (digits are word chars)
  std::cout<<"num between -> "<<run("([TOOLPATH_NAME][T][TOOLNAME])","LOG","OPEN,/f")<<"\n";
  std::cout<<"num prefix after value (gap) -> "<<run("([TOOLPATH_NAME][T])","LOGOPEN","x",{}, {Move::rapid({0,0,5})},"VAR TOOL_NUMBER = [T|A|,|1.0]\n")<<"\n";
  // tolerance boundaries
  for(double tol:{1e-6, std::nextafter(1e-6,0.0), 1.0, std::nextafter(1.0,2.0)}){ PostContext c; c.arcTolerance=tol; std::cout<<"tol "<<tol<<" -> "<<run("(x)","n","t",c).substr(0,60)<<"\n"; }
  // huge arc at 1e-6: count lines
  { PostContext c; c.arcTolerance=1e-6; auto s=run("(x)","n","t",c,{Move::rapid({1e6,0,0}),Move::arcCCW({-1e6,0,0},{0,0})}); size_t n=0; for(size_t i=0;i+1<s.size();++i) if(s[i]=='\\'&&s[i+1]=='n') ++n; std::cout<<"r=1e6 semicircle @1e-6: lines="<<n<<" "<<(s.rfind("ERR",0)==0?s:"")<<"\n"; }
  { PostContext c; auto s=run("(x)","n","t",c,{Move::rapid({1e-12,0,0}),Move::arcCCW({1e-12,0,0},{0,0})}); std::cout<<"tiny full circle: "<<s.substr(0,120)<<"\n"; }
  // number edges
  std::cout<<"-0 coord -> "<<run("(x)","n","t",{}, {Move::rapid({0,0,5}),Move::feed({-0.0,-1e6,0})})<<"\n";
  std::cout<<"1e6+eps -> "<<run("(x)","n","t",{}, {Move::rapid({0,0,5}),Move::feed({std::nextafter(1e6,2e6),0,0})})<<"\n";
}
