#include <cmath>
#include <cstdio>
#include <vector>
#include <cstdint>
#include <random>
struct PP { double startDepth{0}, cutDepth{0}, passDepth{0}; bool maintainExactStep{true}; double lastPassThickness{0}; };
constexpr double kStepSlack = 0.15;
template<bool NEW>
void planStretch(const PP& plan, double top, double depth, std::vector<double>& out) {
    if (depth <= 0.0) return;
    constexpr double kEps = 1e-9;
    const double eps = NEW ? kEps * depth : kEps;
    if (plan.passDepth <= 0.0 || plan.passDepth >= depth - eps) { out.push_back(top + depth); return; }
    if (plan.maintainExactStep) {
        const auto full = static_cast<std::size_t>(std::floor(depth / plan.passDepth + kEps));
        for (std::size_t i = 1; i <= full; ++i) { const double d = double(i) * plan.passDepth; if (d < depth - eps) out.push_back(top + d); }
        out.push_back(top + depth); return;
    }
    auto n = static_cast<std::size_t>(std::ceil(depth / plan.passDepth - kEps));
    bool dec = NEW ? (n > 1 && depth / double(n - 1) <= plan.passDepth * (1.0 + kStepSlack) * (1.0 + kEps))
                   : (n > 1 && depth / double(n - 1) <= plan.passDepth * (1.0 + kStepSlack) + kEps);
    if (dec) --n;
    const double step = depth / double(n);
    for (std::size_t i = 1; i < n; ++i) out.push_back(top + double(i) * step);
    out.push_back(top + depth);
}
template<bool NEW> std::vector<double> levels(const PP& p) {
    std::vector<double> d; if (p.cutDepth <= 0) return {};
    if (p.lastPassThickness > 0 && p.lastPassThickness >= p.cutDepth) d.push_back(p.startDepth + p.cutDepth);
    else if (p.lastPassThickness > 0) { planStretch<NEW>(p, p.startDepth, p.cutDepth - p.lastPassThickness, d); d.push_back(p.startDepth + p.cutDepth); }
    else planStretch<NEW>(p, p.startDepth, p.cutDepth, d);
    for (double& x : d) x = -x; d.back() = -(p.startDepth + p.cutDepth); return d;
}
void show(const char* tag, PP p) {
    auto a = levels<false>(p), b = levels<true>(p);
    std::printf("%-40s cut=%-10g pass=%-10g exact=%d lpt=%g : main %zu  new %zu", tag, p.cutDepth, p.passDepth, p.maintainExactStep, p.lastPassThickness, a.size(), b.size());
    if (b.size() >= 2) std::printf("  new last step %.3g (pass %.3g)", b[b.size()-2]-b.back(), p.passDepth);
    if (a.size() >= 2) std::printf("  main last step %.3g", a[a.size()-2]-a.back());
    std::printf("\n");
}
int main() {
    show("card nm", {0, 1e-6, 1e-9, true, 0});
    show("card nm eq", {0, 1e-6, 1e-9, false, 0});
    // micrometre: thin real last pass merged on main?
    for (double s : {1e-9, 1e-6, 1e-3, 1.0, 1000.0, 1e4}) {
        for (bool ex : {true, false}) {
            for (int k : {1, 2, 3, 7, 8, 10, 999, 1000}) {
                double cut = k * s;
                char tag[64];
                std::snprintf(tag, 64, "s=%g k=%d exact", s, k); show(tag, {0, cut, s, ex, 0});
                std::snprintf(tag, 64, "s=%g k=%d +1e-12rel", s, k); show(tag, {0, cut*(1+1e-12), s, ex, 0});
                std::snprintf(tag, 64, "s=%g k=%d +1e-9rel", s, k); show(tag, {0, cut*(1+1e-9), s, ex, 0});
                std::snprintf(tag, 64, "s=%g k=%d +1e-7rel", s, k); show(tag, {0, cut*(1+1e-7), s, ex, 0});
                std::snprintf(tag, 64, "s=%g k=%d -1e-7rel", s, k); show(tag, {0, cut*(1-1e-7), s, ex, 0});
            }
        }
    }
    // claimed micrometre bug: cut 3e-3? find a micrometre case with thin real last pass
    show("um: cut 0.0030000005 pass 0.001", {0, 0.0030000005, 0.001, true, 0});
    show("um: cut 3.0000005e-6 pass 1e-6", {0, 3.0000005e-6, 1e-6, true, 0});
    show("um: cut 2.5e-6? pass 1e-6 lpt", {0, 3e-6, 1e-6, true, 1e-9});
    show("m: cut 3000.000001 pass 1000", {0, 3000.000001, 1000, true, 0});
    show("m: cut 3000+1e-7 pass 1000", {0, 3000.0000001, 1000, true, 0});
    show("m: cut 1000*3 pass 1000 lpt", {0, 3000, 1000, true, 0.5});
    // random fuzz realistic: diff count
    std::mt19937_64 rng(7); std::uniform_real_distribution<double> u(0,1);
    long diffs=0, N=5000000;
    for (long i=0;i<N;++i){
        double pass = std::round((0.05 + u(rng)*10)*1000)/1000;  // 3 decimals
        double cut = std::round((0.01 + u(rng)*60)*1000)/1000;
        double start = (i%3==0)? std::round(u(rng)*10*1000)/1000 : 0;
        double lpt = (i%4==0)? std::round(u(rng)*1*1000)/1000 : 0;
        PP p{start,cut,pass,(i&1)!=0,lpt};
        auto a=levels<false>(p), b=levels<true>(p);
        if (a!=b){ if(diffs<10) std::printf("DIFF start=%.17g cut=%.17g pass=%.17g ex=%d lpt=%.17g main=%zu new=%zu\n",start,cut,pass,p.maintainExactStep,lpt,a.size(),b.size()); ++diffs; }
    }
    // also inch-derived inputs
    for (long i=0;i<2000000;++i){
        double pass = (1+ (int)(u(rng)*500))/1000.0*25.4;
        double cut = (1+ (int)(u(rng)*3000))/1000.0*25.4;
        PP p{0,cut,pass,(i&1)!=0,0};
        auto a=levels<false>(p), b=levels<true>(p);
        if (a!=b){ if(diffs<20) std::printf("DIFF-inch cut=%.17g pass=%.17g ex=%d main=%zu new=%zu\n",cut,pass,p.maintainExactStep,a.size(),b.size()); ++diffs; }
    }
    // integer-ratio inputs: cut = k * pass, 3-decimal pass
    for (int pi=1; pi<=20000; ++pi) for (int k=1;k<=60;++k){ double pass=pi/1000.0; double cut=k*pass; for(bool ex:{true,false}){ PP p{0,cut,pass,ex,0}; if(levels<false>(p)!=levels<true>(p)){ if(diffs<30) std::printf("DIFF-mult pass=%.17g k=%d ex=%d\n",pass,k,ex); ++diffs;} } }
    std::printf("total diffs %ld\n", diffs);
}
