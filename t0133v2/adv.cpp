// T-0133 round-2 verifier: randomized text-injection fuzzer with a controller model.
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"
using namespace carvetto::post;
using carvetto::toolpath::Toolpath;
static std::string rd(const std::string& p){std::ifstream i(p,std::ios::binary);std::ostringstream s;s<<i.rdbuf();return s.str();}
static std::string esc(const std::string& s){std::string o;for(unsigned char c:s){if(c<0x20||c>=0x7f){char b[8];snprintf(b,8,"\\x%02x",c);o+=b;}else o+=char(c);}return o;}
static std::string lower(std::string s){for(auto&c:s) c=char(std::tolower((unsigned char)c));return s;}
struct Finding{std::string kind;std::string line;};
// Analyse one output line. expectCode: code of the benign rendering (spaces removed).
static void analyse(const std::string& line, std::vector<Finding>& f){
  for(unsigned char c: line){
    if(c<0x20||c==0x7f) f.push_back({"CTRL",line});
    else if(c>=0x80) f.push_back({"HIBYTE",line});
    else if(c=='~'||c=='!'||c=='?') f.push_back({"REALTIME",line});
  }
  std::vector<std::string> comments; bool paren=false; std::string cur;
  for(size_t i=0;i<line.size();++i){ char c=line[i];
    if(paren){ if(c=='(') f.push_back({"NESTED",line}); if(c==')'){paren=false;comments.push_back(cur);cur.clear();} else cur+=c; continue;}
    if(c=='('){paren=true;continue;}
    if(c==';'){comments.push_back(line.substr(i+1));break;}
  }
  if(paren) f.push_back({"UNCLOSED",line});
  static const char* kw[]={"msg,","debug,","print,","log,","logopen,","logappend,","logclose","pyrun,","py,","abort,","probeopen","probeclose"};
  for(auto& c: comments){
    size_t k=c.find_first_not_of(" \t"); std::string lc=lower(k==std::string::npos?"":c.substr(k));
    for(auto w: kw) if(lc.rfind(w,0)==0) f.push_back({std::string("ACTIVE_LCNC:")+w,line});
    std::string sq; for(char ch:c) if(ch!=' '&&ch!='\t') sq+=char(std::toupper((unsigned char)ch));
    size_t w=sq.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_");
    if(w!=std::string::npos && sq[w]==',') f.push_back({"WORDCOMMA",line});
    for(auto kk:{"PROBEOPEN","PROBECLOSE","LOGCLOSE"}) if(sq.rfind(kk,0)==0) f.push_back({"NOARGKW",line});
  }
}
static std::string codeOf(const std::string& line){std::string o;bool p=false;for(char c:line){if(p){if(c==')')p=false;continue;} if(c=='('){p=true;continue;} if(c==';')break; if(c!=' ')o+=c;}return o;}
static std::vector<std::string> split(const std::string& t){std::vector<std::string> v;std::string cur;for(char c:t){if(c=='\n'){if(!cur.empty()&&cur.back()=='\r')cur.pop_back();v.push_back(cur);cur.clear();}else cur+=c;}if(!cur.empty())v.push_back(cur);return v;}
int main(int argc,char**argv){
  const std::string dir=argv[1]; int iters=argc>2?atoi(argv[2]):3000;
  std::vector<std::pair<std::string,std::string>> posts;
  for(const char* s:{"grbl_mm","grbl_inch","mach3_mm","mach3_inch","mach4_mm","linuxcnc_mm","linuxcnc_inch","marlin_mm"}) posts.push_back({s,rd(dir+"/"+s+".pp")});
  const std::string tail="begin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\nbegin FEED_MOVE\n\"G1[X][Y][Z]\"\nend\n";
  posts.push_back({"c_positions","begin HEADER\n\"([TOOLPATH_NAME])\"\n\"( [TOOLPATH_NAME])\"\n\"(\t[TOOLPATH_NAME] )\"\n\"(Name: [TOOLPATH_NAME])\"\n\";[TOOLPATH_NAME]\"\n\"; [TOOLPATH_NAME]\"\n\"G0 X1 ([TOOLPATH_NAME])\"\n\"G0 X1 ;[TOOLPATH_NAME]\"\n\"(a)([TOOLPATH_NAME])\"\n\"([TOOLPATH_NAME]) ([TOOLNAME])\"\n\"([FILE_NOTES])\"\n\"([TOOLS_USED])\"\n\"([DATE] [TIME])\"\n\"([TP_DIR][TP_FILENAME].[TP_EXT])\"\n\"([PATHNAME])\"\n\"([PRODUCT])\"\nend\n"+tail});
  posts.push_back({"c_adjacent","begin HEADER\n\"([TOOLNAME][TOOLPATH_NAME])\"\n\"([TOOLNAME] [TOOLPATH_NAME])\"\n\"; [TOOLNAME][TOOLPATH_NAME]\"\nend\n"+tail});
  posts.push_back({"c_prefix_paren","VAR NAME = [TOOLPATH_NAME|A|(|1.0]\nbegin HEADER\n\"[TOOLPATH_NAME])\"\nend\n"+tail});
  posts.push_back({"c_prefix_paren_sp","VAR NAME = [TOOLPATH_NAME|A|( |1.0]\nbegin HEADER\n\"[TOOLPATH_NAME])\"\nend\n"+tail});
  posts.push_back({"c_prefix_semi","VAR NAME = [TOOLPATH_NAME|A|;|1.0]\nbegin HEADER\n\"[TOOLPATH_NAME]\"\nend\n"+tail});
  posts.push_back({"c_prefix_x_paren","VAR NAME = [TOOLPATH_NAME|A|(x)(|1.0]\nbegin HEADER\n\"[TOOLPATH_NAME])\"\nend\n"+tail});
  posts.push_back({"c_subst","SUBSTITUTE = \"x,y(z)\"\nbegin HEADER\n\"([TOOLPATH_NAME])\"\n\"; [TOOLPATH_NAME]\"\nend\n"+tail});
  std::vector<std::string> tok={"MSG","msg","LogOpen","LOGOPEN","log","OPEN","py","PyRun","DEBUG","print","ABORT","PROBEOPEN","probe","close","LOGCLOSE","AXIS",",",",",","," ","\t","  ","(",")",";","~","!","?","\n","\r",std::string(1,'\0'),"\x7f","\x18","\x1b","\xC3\x91","\xC3\xA9","\xC0\xAC","\xC0\x80","\xE0\x80\xAC","\xF0\x80\x80\xAC","\xFF","\x80","\xC2","\xEF\xBC\x8C","\xED\xA0\x80","\xF4\x90\x80\x80","G0 Z-50","%","$","x","1","_","/home/cnc/f",")G0 Z-50("};
  std::mt19937 rng(12345);
  auto rnd=[&](){int n=1+int(rng()%8);std::string s;for(int i=0;i<n;++i) s+=tok[rng()%tok.size()];return s;};
  std::map<std::string,std::pair<int,std::string>> agg;
  int runs=0, errs=0;
  for(auto&[pn,src]:posts){
    auto pr=PostProcessor::parse(src); if(!pr.ok()){std::cout<<pn<<" PARSE "<<pr.error().describe()<<"\n";continue;}
    auto pp=pr.value();
    auto baseTps=carvetto::test::fixtureToolpaths(); auto baseCtx=carvetto::test::fixtureContext();
    for(auto&t:baseTps){t.name="x";t.tool.name="x";}
    baseCtx.fileName=baseCtx.fileDir=baseCtx.fileExt=baseCtx.pathName=baseCtx.date=baseCtx.time=baseCtx.product=baseCtx.fileNotes="x";
    auto base=split(pp.run(baseTps,baseCtx).value().text);
    for(int it=0;it<iters;++it){
      std::string a=rnd(), b=rnd(), c=rnd();
      auto tps=carvetto::test::fixtureToolpaths(); auto ctx=carvetto::test::fixtureContext();
      for(auto&t:tps){t.name=a;t.tool.name=b;}
      ctx.fileName=c; ctx.fileDir=b; ctx.fileExt=a; ctx.pathName=c; ctx.date=a; ctx.time=b; ctx.product=c; ctx.fileNotes=a;
      auto r=pp.run(tps,ctx); ++runs; if(!r.ok()){++errs; continue;}
      auto lines=split(r.value().text);
      std::vector<Finding> f;
      if(lines.size()!=base.size()) f.push_back({"LINECOUNT",esc(a)+" | "+esc(b)});
      else for(size_t i=0;i<lines.size();++i){ analyse(lines[i],f); if(codeOf(lines[i])!=codeOf(base[i])) f.push_back({"CODE",lines[i]}); }
      for(auto&x:f){ auto key=pn+" "+x.kind; auto& e=agg[key]; if(e.first++==0) e.second=esc(x.line)+"   <- A="+esc(a)+" B="+esc(b)+" C="+esc(c);} 
    }
  }
  std::cout<<"runs="<<runs<<" errors="<<errs<<"\n";
  for(auto&[k,v]:agg) std::cout<<k<<" x"<<v.first<<"  e.g. "<<v.second<<"\n";
}
