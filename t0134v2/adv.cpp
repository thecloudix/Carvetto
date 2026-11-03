// T-0134 r2 verifier: AC3 adversarial driver. For every post x field x value x layout, prints
// key \t OK|ERR \t fnv-hash \t bad-words(F/S/P <= 0 outside comments) \t message
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
static std::string slurp(const char* p){std::ifstream f(p,std::ios::binary);std::stringstream s;s<<f.rdbuf();return s.str();}
static unsigned long long fnv(const std::string& s){unsigned long long h=1469598103934665603ull;for(unsigned char c:s){h^=c;h*=1099511628211ull;}return h;}
static std::string bad(const std::string& t){
  std::string out; std::istringstream is(t); std::string line;
  static const std::regex w("([FSP])([-+]?[0-9]*\\.?[0-9]*)");
  while(std::getline(is,line)){
    std::string code; int d=0; for(char c:line){ if(c==';'&&d==0)break; if(c=='('){++d;continue;} if(c==')'){--d;continue;} if(d==0)code+=c; }
    for(std::sregex_iterator it(code.begin(),code.end(),w),e;it!=e;++it){ std::string n=(*it)[2]; if(n.empty()||n=="."||n=="-"||n=="+")continue; double v=std::strtod(n.c_str(),nullptr); if(!(v>0)) out+=(*it)[0].str()+" "; }
  }
  return out;
}
static Toolpath mk(const std::string& name,int num,double feed,double plunge,double sp){
  Toolpath t; t.name=name; t.tool.name="T"+std::to_string(num); t.tool.toolNumber=num; t.tool.diameter=6; t.tool.feedRate=feed; t.tool.plungeRate=plunge; t.tool.spindleSpeed=sp;
  t.moves={Move::rapid({0,0,5}),Move::plunge({0,0,-1}),Move::feed({20,0,-1}),Move::arcCCW({20,20,-1},{20,10}),Move::feed({20,20,-1}) /*zero length*/,Move::dwell({20,20,-1},0.5),Move::retract({20,20,5})};
  return t;
}
int main(int argc,char**argv){
  const double NaN=std::numeric_limits<double>::quiet_NaN(), DEN=4.9e-324, INF=std::numeric_limits<double>::infinity();
  std::vector<std::pair<std::string,double>> vals={{"0",0.0},{"-0",-0.0},{"-5",-5.0},{"NaN",NaN},{"inf",INF},{"den",DEN},{"1e-9",1e-9},
    {"0.0004",0.0004},{"0.004",0.004},{"0.0049",0.0049},{"0.005",0.005},{"0.04",0.04},{"0.049",0.049},{"0.05",0.05},{"0.1",0.1},{"0.126",0.126},{"0.127",0.127},{"0.4",0.4},{"0.49",0.49},{"0.5",0.5},{"1",1.0},{"1.27",1.27},{"1.2699",1.2699},
    {"2.5",2.5},{"400",400.0},{"499",499.0},{"500",500.0},{"1000",1000.0}};
  const std::vector<std::string> fields={"spindle","feed","plunge","override","dwell","base"};
  const std::vector<std::string> layouts={"single","change","segment","empty"};
  for(int a=1;a<argc;++a){
    std::string path=argv[a]; std::string nm=path.substr(path.find_last_of('/')+1);
    auto pr=PostProcessor::parse(slurp(argv[a])); if(!pr.ok()){std::cout<<nm<<"\tPARSE\t"<<pr.error().describe()<<"\n";continue;}
    for(auto& f:fields) for(auto& [vn,v]:vals){ if(f=="base"&&vn!="1000")continue;
      for(auto& L:layouts){ if(L=="empty"&&f!="base"&&f!="spindle")continue;
        double feed=1000,plunge=300,sp=18000; 
        Toolpath good=mk("good",1,feed,plunge,sp);
        Toolpath b=mk("bad",L=="change"?2:1,feed,plunge,sp);
        if(f=="spindle") b.tool.spindleSpeed=v; else if(f=="feed") b.tool.feedRate=v; else if(f=="plunge") b.tool.plungeRate=v;
        else if(f=="override"){ b.moves[2].feedOverride=v; } else if(f=="dwell"){ b.moves[5].dwellSeconds=v; }
        std::vector<Toolpath> tps; if(L=="single") tps={b}; else if(L=="empty") tps={}; else tps={good,b};
        PostContext c; c.safeZ=5;
        auto r=pr.value().run(tps,c);
        std::string key=nm+"|"+f+"|"+vn+"|"+L;
        if(r.ok()) std::cout<<key<<"\tOK\t"<<std::hex<<fnv(r.value().text)<<std::dec<<"\t"<<bad(r.value().text)<<"\t\n";
        else std::cout<<key<<"\tERR\t-\t\t"<<r.error().describe()<<"\n";
      }
    }
  }
}
