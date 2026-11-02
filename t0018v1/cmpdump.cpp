#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>
struct P{double x,y;}; using Ring=std::vector<P>; struct E{std::string k; std::vector<Ring> r;};
static std::vector<E> load(const char* f){ std::vector<E> v; FILE* F=std::fopen(f,"r"); char key[512]; size_t n; while(std::fscanf(F," K %511s %zu",key,&n)==2){ E e; e.k=key; for(size_t i=0;i<n;++i){ size_t m; std::fscanf(F," R %zu",&m); Ring r(m); for(auto& p:r) std::fscanf(F,"%lf %lf",&p.x,&p.y); e.r.push_back(r);} v.push_back(e);} std::fclose(F); return v; }
static double d2seg(P p,P a,P b){ double dx=b.x-a.x,dy=b.y-a.y,l=dx*dx+dy*dy; double t=l>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l,0.0,1.0):0; double ex=p.x-a.x-t*dx,ey=p.y-a.y-t*dy; return ex*ex+ey*ey; }
static double dirH(const std::vector<Ring>& A,const std::vector<Ring>& B){ double h=0; for(auto& r:A) for(auto p:r){ double b=1e300; for(auto& s:B) for(size_t i=0;i<s.size();++i) b=std::min(b,d2seg(p,s[i],s[(i+1)%s.size()])); h=std::max(h,b);} return std::sqrt(h); }
int main(int c,char** v){ auto a=load(v[1]), b=load(v[2]); if(a.size()!=b.size()){ std::printf("size mismatch %zu %zu\n",a.size(),b.size()); return 1;}
 long same=0, diff=0, cnt=0, vdiff=0; double worst=0; std::string wk; double hist[6]={0};
 for(size_t i=0;i<a.size();++i){ if(a[i].k!=b[i].k){ std::printf("key mismatch %s %s\n",a[i].k.c_str(),b[i].k.c_str()); return 1;}
   bool ident = a[i].r.size()==b[i].r.size(); if(ident) for(size_t j=0;j<a[i].r.size()&&ident;++j){ if(a[i].r[j].size()!=b[i].r[j].size()) ident=false; else for(size_t k=0;k<a[i].r[j].size();++k) if(a[i].r[j][k].x!=b[i].r[j][k].x||a[i].r[j][k].y!=b[i].r[j][k].y){ident=false;break;} }
   if(ident){++same; continue;} ++diff; if(a[i].r.size()!=b[i].r.size()){ ++cnt; std::printf("RINGCOUNT %s new %zu old %zu\n",a[i].k.c_str(),a[i].r.size(),b[i].r.size()); continue; }
   double h=std::max(dirH(a[i].r,b[i].r),dirH(b[i].r,a[i].r)); if(h>worst){worst=h; wk=a[i].k;} int bin = h<1e-9?0: h<1e-7?1: h<1e-6?2: h<1e-5?3: h<1e-3?4:5; hist[bin]++; if(bin>=3) std::printf("H %.3e %s\n",h,a[i].k.c_str()); }
 std::printf("entries %zu identical %ld differ %ld (ringcount %ld) worstHausdorff %.3e at %s\n hist <1e-9:%g <1e-7:%g <1e-6:%g <1e-5:%g <1e-3:%g >=1e-3:%g\n",a.size(),same,diff,cnt,worst,wk.c_str(),hist[0],hist[1],hist[2],hist[3],hist[4],hist[5]); }
