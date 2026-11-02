// Would the branch's shipped-post tests notice a shipped post exposed to the documented gap?
// Re-implements the B3 test predicate and applies it to mutated copies of shipped posts.
#include <fstream>
#include <iostream>
#include <sstream>
#include "carvetto/post/post_processor.hpp"
#include "post_fixture.hpp"
using namespace carvetto::post;
static std::string rd(const std::string& p){std::ifstream i(p,std::ios::binary);std::ostringstream s;s<<i.rdbuf();return s.str();}
static bool b3fails(const std::string& out){
  std::istringstream in(out); std::string line; while(std::getline(in,line)){ if(!line.empty()&&line.back()=='\r') line.pop_back();
    std::size_t open=std::string::npos;
    for(size_t i=0;i<line.size();++i){ if(line[i]=='('||(line[i]==';'&&open==std::string::npos)){open=i; std::string sq;
      for(size_t k=i+1;k<line.size()&&line[k]!=')';++k) if(line[k]!=' ') sq+=char(std::toupper((unsigned char)line[k]));
      size_t w=sq.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_");
      if((w!=std::string::npos&&sq[w]==',')||sq.rfind("PROBEOPEN",0)==0) return true; }}}
  return false;
}
static void rep(std::string& s,const std::string& a,const std::string& b){size_t p=0;while((p=s.find(a,p))!=std::string::npos){s.replace(p,a.size(),b);p+=b.size();}}
int main(int,char**argv){
  for(const char* stem:{"grbl_mm","linuxcnc_mm","mach3_mm","marlin_mm"}){
    for(const char* add:{",x"," ,x","OPEN","CLOSE"}){
      std::string src=rd(std::string(argv[1])+"/"+stem+".pp");
      rep(src,"([TOOLPATH_NAME])",std::string("([TOOLPATH_NAME]")+add+")");
      rep(src,"; [TOOLPATH_NAME]\"",std::string("; [TOOLPATH_NAME]")+add+"\"");
      auto pp=PostProcessor::parse(src).value();
      bool caught=false;
      for(const char* name:{"LOGOPEN,/home/cnc/mill.hal"," msg ,hi","PROBEOPEN /tmp/p","py,import os"}){
        auto tps=carvetto::test::fixtureToolpaths(); for(auto&t:tps){t.name=name;t.tool.name=name;}
        auto ctx=carvetto::test::fixtureContext(); ctx.fileName=name;
        if(b3fails(pp.run(tps,ctx).value().text)) caught=true;
      }
      // is it actually exploitable? name "PROBE"/"LOGOPEN"/"LOG"
      std::string ex; for(const char* name:{"LOGOPEN","PROBE","LOG","MSG"}){ auto tps=carvetto::test::fixtureToolpaths(); for(auto&t:tps) t.name=name; auto o=pp.run(tps,carvetto::test::fixtureContext()).value().text; if(b3fails(o)) ex+=std::string(name)+" "; }
      std::cout<<stem<<" +'"<<add<<"' : B3 test "<<(caught?"FAILS (gap caught)":"passes (gap NOT caught)")<<"; exploitable with: "<<ex<<"\n";
    }
  }
}
