// Throwaway T-0133 number driver: prints a deterministic report; diff branch vs main.
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
static std::string dir="/home/chris/wt-T0133/posts/";
static std::string esc(const std::string& s){std::string o;for(unsigned char c:s){if(c=='\n')o+="\\n";else if(c<0x20||c>=0x7f){char b[8];snprintf(b,8,"\\x%02x",c);o+=b;}else o+=char(c);}return o;}
static unsigned long long fnv(const std::string&s){unsigned long long h=1469598103934665603ull;for(unsigned char c:s){h^=c;h*=1099511628211ull;}return h;}
// longest digit run and presence of inf/nan in output
static std::string scan(const std::string& t){size_t run=0,mx=0;bool bad=false;for(size_t i=0;i<t.size();++i){char c=t[i];if(c>='0'&&c<='9'){++run;mx=std::max(mx,run);}else run=0;}
  for(const char* w:{"inf","nan","INF","NAN"}) if(t.find(w)!=std::string::npos) bad=true; return "maxdigits="+std::to_string(mx)+(bad?" INFNAN":"");}
int main(int argc,char**argv){
  bool full=argc>1;
  const double inf=std::numeric_limits<double>::infinity(), nan=std::numeric_limits<double>::quiet_NaN();
  std::vector<std::pair<std::string,double>> vals={{"0",0},{"-0",-0.0},{"denorm",5e-324},{"denorm2",-1e-310},{"1e-9",1e-9},{"999999.999",999999.999},
    {"1e6",1e6},{"next1e6",std::nextafter(1e6,inf)},{"1e6+1e-3",1e6+1e-3},{"-1e6",-1e6},{"-next1e6",-std::nextafter(1e6,inf)},{"1e9",1e9},{"1e300",1e300},{"inf",inf},{"-inf",-inf},{"nan",nan},{"-nan",-nan}};
  using F=std::function<void(std::vector<Toolpath>&,PostContext&,double)>;
  std::vector<std::pair<std::string,F>> tgts={
    {"rapidX",[](auto&t,auto&,double v){t[0].moves[0].to.x=v;}},
    {"plungeZ",[](auto&t,auto&,double v){t[0].moves[1].to.z=v;}},
    {"feedY",[](auto&t,auto&,double v){t[0].moves[3].to.y=v;}},
    {"arcEndX",[](auto&t,auto&,double v){t[1].moves[4].to.x=v;}},
    {"arcCentreY",[](auto&t,auto&,double v){t[1].moves[4].centre.y=v;}},
    {"feedOverride",[](auto&t,auto&,double v){t[0].moves[2].feedOverride=v;}},
    {"rapidFeedOverride",[](auto&t,auto&,double v){t[0].moves[0].feedOverride=v;}},
    {"dwell",[](auto&t,auto&,double v){for(auto&m:t[2].moves) if(m.kind==carvetto::toolpath::MoveKind::Dwell) m.dwellSeconds=v;}},
    {"toolFeed",[](auto&t,auto&,double v){t[0].tool.feedRate=v;}},
    {"toolPlunge",[](auto&t,auto&,double v){t[0].tool.plungeRate=v;}},
    {"spindle",[](auto&t,auto&,double v){t[0].tool.spindleSpeed=v;}},
    {"diameter",[](auto&t,auto&,double v){t[0].tool.diameter=v;}},
    {"safeZ",[](auto&,auto&c,double v){c.safeZ=v;}},
    {"homeX",[](auto&,auto&c,double v){c.home.x=v;}},
    {"matMaxX",[](auto&,auto&c,double v){c.materialMax.x=v;}},
    {"matMinZ",[](auto&,auto&c,double v){c.materialMin.z=v;}},
    {"arcTol",[](auto&,auto&c,double v){c.arcTolerance=v;}},
    {"rotDia",[](auto&,auto&c,double v){c.rotaryDiameter=v;}},
  };
  std::vector<std::pair<std::string,std::string>> posts;
  for(const char* s:{"grbl_mm","grbl_inch","mach3_mm","mach3_inch","mach4_mm","linuxcnc_mm","linuxcnc_inch","marlin_mm"}) posts.push_back({s,rd(dir+s+".pp")});
  // custom posts
  const std::string allTags = "begin HEADER\n\"([XMIN] [XMAX] [YMIN] [YMAX] [ZMIN] [ZMAX] [XLENGTH] [YLENGTH] [ZLENGTH] [XH] [YH] [ZH] [SAFEZ] [TDIA] [Radius] [Angle] [ArcMidX])\"\nend\n";
  const std::string mv = "begin RAPID_MOVE\n\"G0[X][Y][Z][A]\"\nend\nbegin FEED_MOVE\n\"G1[X][Y][Z][A][F][FI]\"\nend\nbegin PLUNGE_MOVE\n\"G1[X][Y][Z][FP]\"\nend\nbegin DWELL_MOVE\n\"G4[DWELL]\"\nend\nbegin TOOLCHANGE\n\"T[T]M6[S]\"\nend\n";
  posts.push_back({"inch_mult","UNITS = \"INCHES\"\nVAR X = [X|A|X|1.4|2.5]\nVAR Y = [Y|A|Y|1.4|-1000]\nVAR Z = [Z|A|Z|1.4]\nVAR FEED_RATE = [F|A|F|1.1|7]\nVAR DWELL = [DWELL|A|P|1.0|1000]\n"+allTags+mv});
  posts.push_back({"incremental","VAR X = [X|AI|X|1.3]\nVAR Y = [Y|CI|Y|1.3]\nVAR Z = [Z|AI|Z|1.3]\n"+allTags+mv});
  posts.push_back({"invtime","INVERSE_TIME_MODE = YES\nVAR FEED_RATE = [F|A|F|1.3]\n"+allTags+mv});
  posts.push_back({"wrapY","ROTARY_WRAP_Y = \"A\"\nVAR A = [A|A|A|1.3]\n"+mv});
  posts.push_back({"wrapXneg","ROTARY_WRAP_X = \"-A\"\nVAR A = [A|A|A|1.3]\n"+mv});
  posts.push_back({"arcblk","VAR X = [X|A|X|1.3]\nVAR Y = [Y|A|Y|1.3]\nVAR I = [I|A|I|1.3]\nVAR J = [J|A|J|1.3]\nVAR Radius = [Radius|A|R|1.3]\n"+mv+"begin CW_ARC_MOVE\n\"G2[X][Y][Z][I][J][Radius][F]\"\nend\nbegin CCW_ARC_MOVE\n\"G3[X][Y][Z][I][J][Radius][F]\"\nend\n"});
  for(auto&[pn,src]:posts){
    auto pr=PostProcessor::parse(src); if(!pr.ok()){std::cout<<pn<<" PARSE "<<pr.error().describe()<<"\n";continue;}
    auto pp=pr.value();
    for(auto&[tn,f]:tgts) for(auto&[vn,v]:vals){
      auto tps=carvetto::test::fixtureToolpaths(); auto ctx=carvetto::test::fixtureContext(); ctx.rotaryDiameter=20;
      f(tps,ctx,v);
      auto r=pp.run(tps,ctx);
      std::cout<<pn<<"|"<<tn<<"|"<<vn<<"|";
      if(!r.ok()) std::cout<<"ERR "<<esc(r.error().describe());
      else { const auto& t=r.value().text; std::cout<<"OK "<<std::hex<<fnv(t)<<std::dec<<" len="<<t.size()<<" "<<scan(t); if(full) std::cout<<"\n"<<t; }
      std::cout<<"\n";
    }
  }
}
