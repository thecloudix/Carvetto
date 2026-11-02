// T-0134 verifier: [TOOLS_USED] separator fuzz against a LinuxCNC-ish active comment model
#include <iostream>
#include <random>
#include <map>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
static std::string low(std::string s){for(auto&c:s)c=char(std::tolower((unsigned char)c));return s;}
static const char* KW[]={"msg,","debug,","print,","log,","logopen,","logappend,","logclose","pyrun,","py,","abort,","probeopen","probeclose"};
static void comments(const std::string& line, std::vector<std::string>& out){
  bool p=false; std::string cur;
  for(size_t i=0;i<line.size();++i){char c=line[i];
    if(p){ if(c==')'){p=false;out.push_back(cur);cur.clear();} else cur+=c; continue;}
    if(c=='('){p=true;continue;} if(c==';'){out.push_back(line.substr(i+1));return;}}
  if(p) out.push_back(cur);
}
int main(){
  std::mt19937 rng(99);
  std::vector<std::string> tok={"MSG","msg","LOG","OPEN","CLOSE","PROBE","py","RUN",","," ",";","\t","(",")","","x","_","APPEND","DEBUG","\n"};
  std::vector<std::string> tpl={"([TOOLS_USED])","( [TOOLS_USED])","(\t[TOOLS_USED])","; [TOOLS_USED]",";[TOOLS_USED]","G0 X1 ([TOOLS_USED])","(PROBE[TOOLS_USED])","(LOG[TOOLS_USED])","(M S G[TOOLS_USED])","([TOOLS_USED][TOOLNAME])","([TOOLNAME][TOOLS_USED])"};
  std::map<std::string,std::pair<long,std::string>> agg; long runs=0;
  for(auto& t: tpl){
    auto pr=PostProcessor::parse("begin HEADER\n\""+t+"\"\nend\nbegin TOOLCHANGE\nend\nbegin RAPID_MOVE\nend\n"); if(!pr.ok()){std::cout<<"PARSE "<<pr.error().describe()<<"\n";continue;}
    for(int it=0;it<40000;++it){
      int nt=1+int(rng()%4); std::vector<Toolpath> tps;
      for(int k=0;k<nt;++k){ Toolpath tp; std::string n; int m=int(rng()%4); for(int j=0;j<m;++j) n+=tok[rng()%tok.size()];
        tp.name="x"; tp.tool.name=n; tp.tool.toolNumber=k+1; tp.tool.feedRate=1000; tp.tool.plungeRate=300; tp.tool.spindleSpeed=1e4; tp.moves={Move::rapid({0,0,5})}; tps.push_back(tp);}
      auto r=pr.value().run(tps,{}); ++runs; if(!r.ok()){agg["ERR"].first++;continue;}
      std::string line=r.value().text.substr(0,r.value().text.find('\n'));
      std::vector<std::string> cs; comments(line,cs);
      for(auto& cm:cs){ std::string sq; for(char ch:cm) if(ch!=' '&&ch!='\t') sq+=char(std::tolower((unsigned char)ch));
        for(auto w:KW) if(sq.rfind(w,0)==0){ bool authored=low(t).find("probe")!=std::string::npos||low(t).find("(log")!=std::string::npos||low(t).find("m s g")!=std::string::npos;
          std::string key=std::string(authored?"authored? ":"VALUE ")+w; auto& e=agg[key]; if(e.first++<1) e.second=t+" => "+line; }
      }
    }
  }
  std::cout<<"runs="<<runs<<"\n"; for(auto&[k,v]:agg) std::cout<<k<<" x"<<v.first<<"  e.g. "<<v.second<<"\n";
}
