#include "pp_impl.hpp"
int main(){ long diffs=0, n=0;
 for (int di=1; di<=100000; di+= (di<1000?1:7)) { double depth = di/1000.0; for (int k=1;k<=1000;k=(k<50?k+1:k+13)) { PP p{0,depth,depth/k,true,0}; ++n; if(levels<false>(p)!=levels<true>(p)){ if(diffs<10) std::printf("ENG depth=%.17g k=%d main=%zu new=%zu\n",depth,k,levels<false>(p).size(),levels<true>(p).size()); ++diffs;} } }
 // start depth != 0 with lpt and stretch offsets
 std::mt19937_64 rng(11); std::uniform_real_distribution<double> u(0,1);
 for (long i=0;i<5000000;++i){ double pass=(1+(int)(u(rng)*2000))/400.0; double cut=(1+(int)(u(rng)*24000))/400.0; double lpt=(i%2)?(int)(u(rng)*400)/1000.0:0; PP p{(int)(u(rng)*100)/100.0,cut,pass,(i%3)!=0,lpt}; ++n; if(levels<false>(p)!=levels<true>(p)){ if(diffs<20) std::printf("R pass=%.17g cut=%.17g lpt=%g\n",pass,cut,lpt); ++diffs;} }
 std::printf("n=%ld diffs=%ld\n",n,diffs);}
