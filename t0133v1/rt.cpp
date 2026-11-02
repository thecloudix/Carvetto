#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include "carvetto/post/post_processor.hpp"
#include "post_fixture.hpp"
using namespace carvetto::post;
static std::string rd(const std::string& p){std::ifstream i(p,std::ios::binary);std::ostringstream s;s<<i.rdbuf();return s.str();}
int main(int argc,char**argv){
  auto pp=PostProcessor::parse(rd(std::string("/home/chris/wt-T0133/posts/")+argv[1]+".pp")).value();
  auto tps=carvetto::test::fixtureToolpaths();
  tps[0].name=argv[2]; tps[0].tool.name=argc>3?argv[3]:tps[0].tool.name;
  auto ctx=carvetto::test::fixtureContext();
  auto r=pp.run(tps,ctx); if(!r.ok()){std::cout<<"ERR "<<r.error().describe()<<"\n";return 1;}
  std::cout<<r.value().text;
}
