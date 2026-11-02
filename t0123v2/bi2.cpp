#include <cstdio>
#include <cmath>
#include <limits>
#include <random>
#include "carvetto/toolpath/box_index.hpp"
using namespace carvetto; using geometry::Box2;
int main() {
  std::mt19937_64 rng(424242); auto U=[&](double a,double b){return std::uniform_real_distribution<double>(a,b)(rng);};
  const double inf=std::numeric_limits<double>::infinity(), nan=std::nan("");
  double vals[]={0,1,-1,1e-300,1e300,-1e300,1.7e308,-1.7e308,inf,-inf,nan,5e-10,1e9};
  auto pick=[&]{ int k=int(U(0,16)); if(k<13) return vals[k]; return U(-100,100);};
  long bad=0, tot=0;
  for(int it=0; it<20000; ++it){
    int n=int(U(0,40)); std::vector<Box2> bs;
    for(int i=0;i<n;++i){ double a=pick(),b=pick(),c=pick(),d=pick(); Box2 x; if(U(0,1)<0.7){x.min={std::min(a,b),std::min(c,d)}; x.max={std::max(a,b),std::max(c,d)};} else {x.min={a,c};x.max={b,d};} if(U(0,1)<0.5){double cx=U(-50,50),cy=U(-50,50),w=U(0,10);x.min={cx,cy};x.max={cx+w,cy+w*U(0,2)};} bs.push_back(x);}
    toolpath::BoxIndex idx(bs); bool anyG=false; for(auto&x:bs){ if(!x.isEmpty()&&std::isfinite(x.min.x)&&std::isfinite(x.min.y)&&std::isfinite(x.max.x)&&std::isfinite(x.max.y)&&std::isfinite(x.width())&&std::isfinite(x.height())) anyG=true;} if(getenv("ONLYG") && !anyG) continue;
    for(int q=0;q<20;++q){ double a=pick(),b=pick(),c=pick(),d=pick(); Box2 Q; Q.min={std::min(a,b),std::min(c,d)};Q.max={std::max(a,b),std::max(c,d)}; if(U(0,1)<0.5){double cx=U(-60,60),cy=U(-60,60),w=U(0,20);Q.min={cx,cy};Q.max={cx+w,cy+w};}
      std::vector<size_t> br; for(size_t i=0;i<bs.size();++i) if(bs[i].intersects(Q)) br.push_back(i);
      ++tot; if(idx.query(Q)!=br){ if(bad<3){ std::printf("mismatch it %d q %d Q (%g,%g)-(%g,%g)\n",it,q,Q.min.x,Q.min.y,Q.max.x,Q.max.y); for(size_t i=0;i<bs.size();++i) std::printf("  b%zu (%g,%g)-(%g,%g) empty %d\n",i,bs[i].min.x,bs[i].min.y,bs[i].max.x,bs[i].max.y,(int)bs[i].isEmpty()); auto r=idx.query(Q); std::printf("  idx:"); for(auto i:r) std::printf(" %zu",i); std::printf("  brute:"); for(auto i:br) std::printf(" %zu",i); std::printf("\n");} ++bad; } }
  }
  std::printf("queries %ld mismatches %ld\n", tot, bad);
}
