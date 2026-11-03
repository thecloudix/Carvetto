#include <cstdio>
#include <ctime>
#include "shapes.hpp"
using namespace carvetto::geometry;
static double cpu(){timespec t;clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&t);return t.tv_sec+1e-9*t.tv_nsec;}
int main(){ Tolerance tol; for(int n:{500,2000,8000}) for(double ratio:{0.1,0.5,0.9}){ double R=std::sqrt((double)n)*2; auto c=vs::star({0,0},R,R*ratio,n);
  double t0=cpu(); MedialAxis ax=medialAxis({c},tol); double t=cpu()-t0; std::printf("star n=%d r/R=%.1f spans=%zu samples=%zu cpu=%.3f us/sample=%.1f capped=%d\n",n,ratio,c.spans.size(),ax.boundarySamples,t,1e6*t/ax.boundarySamples,(int)ax.accuracyCapped);} }
