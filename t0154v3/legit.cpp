// Verifier T-0154 r3: shipped posts accept legitimate drag jobs in every transition order,
// including tool names that look like spindle words. Writes outputs for gcheck2.py.
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "post_fixture.hpp"
using namespace carvetto;
using toolpath::Toolpath;
int main(int argc, char** argv) {
  const std::string outdir = argc > 1 ? argv[1] : ".";
  const char* stems[] = {"grbl_mm","grbl_inch","linuxcnc_mm","linuxcnc_inch","mach3_mm","mach3_inch","mach4_mm","marlin_mm"};
  auto base = test::dragToolpaths();  // outline, text, profile, again
  auto D = base[0]; D.tool.name = "S12 drill M3"; D.name = "S1000 M4 drag";
  auto D2 = base[1]; D2.tool.name = "S12 drill M3"; 
  auto D5 = base[3]; D5.tool.toolNumber = 5; D5.tool.name = "Drag M3 S9";
  auto A = base[2];  A.tool.name = "M3 S12000 mill";
  auto A3 = base[2]; A3.tool.toolNumber = 3; A3.tool.name = "S12 drill M3";  // same number+name, spinning
  struct J { const char* n; std::vector<Toolpath> t; };
  std::vector<J> jobs{{"d", {D}}, {"ad", {A, D}}, {"da", {D, A}}, {"dd", {D, D2}}, {"d_d5", {D, D5}},
                      {"adad", {A, D, A, D5}}, {"d_a3", {D, A3}}, {"a3_d", {A3, D}}, {"fixture", base}};
  int bad = 0;
  for (auto s : stems) {
    std::ifstream f(std::string(CARVETTO_POSTS_DIR) + "/" + s + ".pp");
    std::string txt((std::istreambuf_iterator<char>(f)), {});
    auto p = post::PostProcessor::parse(txt);
    if (!p.ok()) { printf("%s PARSE %s\n", s, p.error().describe().c_str()); ++bad; continue; }
    for (auto& j : jobs) {
      auto r = p.value().run(j.t, test::fixtureContext());
      if (!r.ok()) { printf("%s %s REFUSED %s\n", s, j.n, r.error().describe().c_str()); ++bad; continue; }
      std::ofstream(outdir + "/" + s + "_" + j.n + ".nc") << r.value().text;
      printf("%s %s ok (%zu warnings)\n", s, j.n, r.value().warnings.size());
    }
  }
  return bad;
}
