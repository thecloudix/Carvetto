// Throwaway T-0133 verifier driver (not committed).
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"
using namespace carvetto::post;
using carvetto::toolpath::Move;
using carvetto::toolpath::Toolpath;
static std::string rd(const std::string& p){std::ifstream i(p,std::ios::binary);std::ostringstream s;s<<i.rdbuf();return s.str();}
static const char* kPosts[]={"grbl_mm","grbl_inch","mach3_mm","mach3_inch","mach4_mm","linuxcnc_mm","linuxcnc_inch","marlin_mm"};
static std::string dir="/home/chris/wt-T0133/posts/";
// strict controller-like split: block ends at LF or CR (any), code = outside (..) and before ';'
struct Block{std::string code;bool unclosed=false;bool nested=false;};
static std::vector<Block> blocks(const std::string& t){
  std::vector<Block> v; Block b; bool paren=false, semi=false;
  for(unsigned char c: t){
    if(c=='\n'||c=='\r'){ if(paren) b.unclosed=true; if(!b.code.empty()||true) v.push_back(b); b=Block{}; paren=false; semi=false; continue;}
    if(semi) continue;
    if(paren){ if(c=='(') b.nested=true; if(c==')') paren=false; continue;}
    if(c=='('){paren=true;continue;} if(c==';'){semi=true;continue;}
    if(c==' ') continue;
    b.code+=char(c);
  }
  if(!b.code.empty()) v.push_back(b);
  std::vector<Block> o; for(auto&x:v) if(!x.code.empty()||x.unclosed||x.nested) o.push_back(x); return o;
}
static std::string esc(const std::string& s){std::string o;for(unsigned char c:s){if(c<0x20||c>=0x7f){char b[8];snprintf(b,8,"\\x%02x",c);o+=b;}else o+=char(c);}return o;}
int main(){
  std::vector<std::string> hostile={"\n","\r","\r\n","\n\r","\x0b","\x0c","\x1c\x1d\x1e\x1f","\x7f","\x18",std::string("a\0b",3),
   "\x85","\xc2\x85","\xe2\x80\xa8","\xe2\x80\xa9",")","(","((","))",")(",";","(;)",")G0 Z-50(",") G0 Z-50","G0 Z-50 (",
   "x)\nG0 Z-50\n(y",";\nG0 Z-50","%\nG0 Z-50","\r\nM3 S30000\r\n","\x1b[2J","\xc3\x91\xc3\x91\xc3\x9a","~!?"};
  int fails=0;
  for(const char* stem: kPosts){
    auto pp=PostProcessor::parse(rd(dir+stem+".pp")).value();
    auto base=pp.run(carvetto::test::fixtureToolpaths(),carvetto::test::fixtureContext()).value().text;
    auto bb=blocks(base);
    for(size_t hi=0;hi<hostile.size();++hi){
      const std::string h="N"+hostile[hi]+"M";
      for(int src=0;src<10;++src){
        auto tps=carvetto::test::fixtureToolpaths(); auto ctx=carvetto::test::fixtureContext();
        switch(src){case 0: for(auto&t:tps)t.name=h;break; case 1: for(auto&t:tps)t.tool.name=h;break;
          case 2: ctx.fileName=h;break; case 3: ctx.fileDir=h;break; case 4: ctx.fileExt=h;break; case 5: ctx.pathName=h;break;
          case 6: ctx.date=h;break; case 7: ctx.time=h;break; case 8: ctx.product=h;break; case 9: ctx.fileNotes=h;break;}
        auto r=pp.run(tps,ctx); if(!r.ok()){std::cout<<stem<<" src"<<src<<" h"<<hi<<" ERR "<<r.error().describe()<<"\n";continue;}
        auto out=r.value().text; auto ob=blocks(out);
        bool bad = ob.size()!=bb.size();
        for(size_t i=0;!bad&&i<ob.size();++i){ if(ob[i].code!=bb[i].code||ob[i].unclosed||ob[i].nested) bad=true; }
        // count CR/LF
        auto cnt=[](const std::string&s){size_t n=0;for(char c:s) if(c=='\n'||c=='\r') ++n; return n;};
        if(cnt(out)!=cnt(base)) bad=true;
        for(unsigned char c: out) if((c<0x20&&c!='\n'&&c!='\r')||c==0x7f) bad=true;
        if(bad){++fails; std::cout<<"INJECT "<<stem<<" src"<<src<<" h="<<esc(hostile[hi])<<"\n";}
      }
    }
  }
  std::cout<<"text fails="<<fails<<"\n";
}
