// Verifier T-0154 r4: legitimate-looking drag tool names on the 8 shipped posts (false positives).
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "post_fixture.hpp"
using namespace carvetto;
int main() {
  const char* stems[] = {"grbl_mm","grbl_inch","linuxcnc_mm","linuxcnc_inch","mach3_mm","mach3_inch","mach4_mm","marlin_mm"};
  const char* names[] = {"S12 drill M3", "Drag 3mm 108", "Diamond 1mm 112 deg", "Drag #2 (90 deg)", "Drag tip 0.1mm 410", "Drag 120 deg M5 holder", "Drag ~ fine?!", "Drag 30mm M30"};
  for (auto s : stems) {
    std::ifstream f(std::string(CARVETTO_POSTS_DIR) + "/" + s + ".pp");
    std::string txt((std::istreambuf_iterator<char>(f)), {});
    auto p = post::PostProcessor::parse(txt);
    for (auto nm : names) {
      auto base = test::dragToolpaths();
      for (auto& tp : base) if (tp.tool.spindleOff) tp.tool.name = nm;
      auto r = p.value().run(base, test::fixtureContext());
      printf("%-14s %-24s %s\n", s, nm, r.ok() ? "ok" : r.error().describe().c_str());
    }
  }
}
