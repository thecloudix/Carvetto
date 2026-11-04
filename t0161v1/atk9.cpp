// Verifier T-0161 r1: bare pause M5, 79-char live code, N cap, tabs, shipped-post extremes.
#include <cstdio>
#include <string>
#include <vector>
#include <functional>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"
using namespace carvetto;
using toolpath::Move; using toolpath::Toolpath; using toolpath::ToolSnapshot;
static ToolSnapshot spin(int n, const char* name="Mill") { ToolSnapshot t; t.name=name; t.toolNumber=n; t.diameter=3; t.feedRate=600; t.plungeRate=200; t.spindleSpeed=12000; return t; }
static ToolSnapshot dragT(int n=3, std::string name="Drag") { ToolSnapshot t=spin(n); t.name=name; t.spindleSpeed=0; t.spindleOff=true; t.toolType=toolpath::kDiamondDragToolType; return t; }
static Toolpath cut(std::string nm, ToolSnapshot t){ Toolpath tp; tp.name=nm; tp.tool=t; tp.moves={Move::rapid({1,1,5}),Move::plunge({1,1,-0.1}),Move::feed({2,1,-0.1}),Move::retract({2,1,5})}; return tp; }
static const std::string kMoves="begin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\nbegin FEED_MOVE\n\"G1[X][Y][Z][F]\"\nend\n";
static std::string base(std::string settings, std::string header, std::string tc, std::string pause, std::string unlock, std::string vars="") {
  std::string s = settings + "VAR SPINDLE_SPEED = [S|A| S|1.0]\nVAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\nVAR FEED_RATE = [F|C| F|1.0]\n" + vars;
  s += "begin HEADER\n"+header+"end\nbegin TOOLCHANGE\n"+tc+"end\nbegin SPINDLE_ON\n\"M3[S]\"\nend\nbegin NEW_SEGMENT\n\"\"\nend\n";
  s += "begin SPINDLE_OFF_PAUSE\n"+pause+"end\nbegin SPINDLE_UNLOCK_PAUSE\n"+unlock+"end\n";
  s += "begin FOOTER\n\"M5\"\nend\n" + kMoves;
  return s;
}
static void run(const std::string& label, const std::string& pptext, const std::vector<Toolpath>& tps) {
  auto p = post::PostProcessor::parse(pptext);
  if (!p.ok()) { printf("[%s] PARSE ERROR %s\n", label.c_str(), p.error().describe().c_str()); return; }
  auto r = p.value().run(tps, test::fixtureContext());
  if (!r.ok()) { printf("[%s] REFUSED: %s\n", label.c_str(), r.error().describe().c_str()); return; }
  std::string t = r.value().text; std::string vis;
  for (char c : t) { if (c=='\r') vis += "<CR>"; else vis += c; }
  printf("[%s] ACCEPTED:\n%s----\n", label.c_str(), vis.c_str());
}

static std::string q(const std::string& s) { return "\"" + s + "\"\n"; }
static std::string readPost(const char* s) { FILE* f = fopen((std::string(CARVETTO_POSTS_DIR) + "/" + s + ".pp").c_str(), "rb"); std::string t; char b[4096]; size_t n; while ((n = fread(b,1,sizeof b,f))) t.append(b,n); fclose(f); return t; }
static void shipped(const std::string& label, const char* stem, const std::string& ppText, const std::vector<Toolpath>& tps, bool dump=false) {
  auto p = post::PostProcessor::parse(ppText);
  auto r = p.value().run(tps, test::fixtureContext());
  if (!r.ok()) { printf("[%s %s] REFUSED: %s\n", label.c_str(), stem, r.error().describe().substr(0,300).c_str()); return; }
  // longest line overall, longest live code (outside comments) and longest comment line
  size_t maxAll = 0, maxLive = 0; std::string t = r.value().text, ln, longest;
  for (size_t i = 0, j; i < t.size(); i = j + 1) { j = t.find('\n', i); if (j == std::string::npos) j = t.size(); ln = t.substr(i, j - i); if (!ln.empty() && ln.back()=='\r') ln.pop_back();
    if (ln.size() > maxAll) { maxAll = ln.size(); longest = ln; }
    size_t live = std::min(ln.find('('), ln.find(';')); live = std::min(live, ln.size()); maxLive = std::max(maxLive, live); }
  printf("[%s %s] ACCEPTED maxLine=%zu maxLive=%zu longest='%.90s'\n", label.c_str(), stem, maxAll, maxLive, longest.c_str());
  if (dump) printf("%s----\n", t.c_str());
}
int main() {
  const std::string Hp=q("(h)"), Up=q("M5")+q("(unlock)")+q("M0");
  const std::string Hs=q("; h"), Us=q("M5")+q("; unlock")+q("M0");
  const std::string NV="VAR LINE_NUMBER = [N|A|N|1.0]\n";
  std::vector<Toolpath> first{cut("D", dragT())};
  std::vector<Toolpath> spinThenDrag{cut("A", spin(1)), cut("D", dragT())};
  std::vector<Toolpath> dragThenSpin{cut("D", dragT()), cut("A", spin(1))};
  // --- B: bare M5 in pause blocks ---
  for (const char* m5 : {"M5 M0", "M0 M5", "M5 N10", "M5 T3", "M5 G4 P1", "M5 F100", "M5 M5", "M005", "N0010 M0005", "M5 (a)", "M5 ; x", "M5 X"}) {
    run(std::string("()-B pause '") + m5 + "'", base("", Hp, q("M5")+q("M0"), q(m5)+q("M0"), Up), spinThenDrag);
  }
  for (const char* m5 : {"M5 M0", "M5 G0", "M5 ; x", "M5 (a)"}) {
    run(std::string(";-B pause '") + m5 + "'", base("", Hs, q("M5")+q("M0"), q(m5)+q("M0"), Us), spinThenDrag);
    run(std::string(";-B unlock '") + m5 + "'", base("", Hs, q("M5")+q("M0"), q("M5")+q("M0"), q(m5)+q("M0")), dragThenSpin);
  }
  // order: pause before stop (T-0154 semantics; spindle still running during the pause)
  run("()-B2 pause 'M0' then 'M5'", base("", Hp, q("M5")+q("M0"), q("M0")+q("M5"), Up), spinThenDrag);
  // only bare M5 on a separate line; non-bare M5 in TOOLCHANGE (accepted by design)
  run("()-B3 TOOLCHANGE 'G0 Z5 M5' + bare pause M5", base("", Hp, q("G0 Z5 M5")+q("M0"), q("M5")+q("M0"), Up), spinThenDrag);
  // pause block with two M5 lines: one bare, one not
  run("()-B4 pause 'M5','M5 G0 Z5','M0'", base("", Hp, q("M5")+q("M0"), q("M5")+q("M5 G0 Z5")+q("M0"), Up), spinThenDrag);
  // --- L: line length ---
  std::string l79 = "G0"; while (l79.size() + 3 <= 76) l79 += " X1"; while (l79.size() < 79) l79 += (79 - l79.size() >= 4 ? " X12" : "2");
  run("()-L79 live in TOOLCHANGE (" + std::to_string(l79.size()) + ")", base("", Hp, q("M5")+q(l79)+q("M0"), q("M5")+q("M0"), Up), spinThenDrag);
  run("()-L80 live + leading space", base("", Hp, q("M5")+q(" " + l79)+q("M0"), q("M5")+q("M0"), Up), spinThenDrag);
  run("()-L comment in the middle", base("", Hp, q("M5")+q(l79.substr(0,41) + " (" + std::string(200,'c') + ") " + l79.substr(3,35))+q("M0"), q("M5")+q("M0"), Up), spinThenDrag);
  run(";-L 79 + ;comment", base("", Hs, q("M5")+q(l79 + " ; " + std::string(200,'c'))+q("M0"), q("M5")+q("M0"), Us), spinThenDrag);
  run("()-L CRLF ending with 79", base("LINE_ENDING = \"[13][10]\"\n", Hp, q("M5")+q(l79)+q("M0"), q("M5")+q("M0"), Up), spinThenDrag);
  run("()-L 80 outside drag (spinning only)", base("", q(" " + l79), q("M5")+q("M0"), q("M5")+q("M0"), Up), {cut("A", spin(1))});
  // --- N cap ---
  for (const char* nn : {"N10000000 M0", "N010000000 M0", "N09999999 M0", "N0 M0", "N00000000 M0"})
    run(std::string("()-N '") + nn + "'", base("", Hp, q("M5")+q("M0"), q("M5")+q(nn), Up), first);
  // N cap in TOOLCHANGE and moves too (all drag-section lines)
  { std::string t = base("LINE_NUMBER_START = 9999997\nLINE_NUMBER_MAXIMUM = 99999999\n", Hp, q("M5")+q("M0"), q("[N] M5")+q("[N] M0"), Up, NV);
    for (const char* g : {"\"G0[X]", "\"G1[X]"}) { auto at = t.find(g); t.insert(at + 1, "[N] "); }
    run("()-N counter past cap in moves", t, first); }
  // --- T: tabs, message rendering ---
  run("()-T tab in tool name (comment)", base("", q("(Tool [TOOLNAME])"), q("M5")+q("M0"), q("M5")+q("(T [TOOLNAME])")+q("M0"), Up), {cut("D", dragT(3, "a\tb"))});
  run("()-T multiple tabs > 64 chars", base("", Hp, q("M5")+q("M0"), q("M5\t\t\t\t" + std::string(70,'x'))+q("M0"), Up), first);
  // --- shipped posts: extremes ---
  const char* stems[] = {"grbl_mm","grbl_inch","linuxcnc_mm","linuxcnc_inch","mach3_mm","mach3_inch","mach4_mm","marlin_mm"};
  for (const char* s : stems) {
    const std::string txt = readPost(s);
    // long names: 300-char tool and toolpath names
    Toolpath d = cut(std::string(300,'p'), dragT(3, std::string(300,'n')));
    shipped("S-longnames", s, txt, {cut("A", spin(1)), d, cut("A2", spin(2))});
    // big coordinates near the machine limit (1e6 mm), arcs with large I/J, Mach3 N near wrap
    for (double c : {99999.0, 999999.0}) {
      Toolpath b = cut("Big", dragT());
      b.moves = {Move::rapid({-c,-c,5}), Move::plunge({-c,-c,-0.25}), Move::arcCW({c,-c,-0.25},{0,-c}), Move::arcCCW({-c,-c,-0.25},{0,-c}), Move::feed({-c,c,-0.25}), Move::retract({-c,c,5})};
      std::string t2 = txt; if (auto at = t2.find("LINE_NUMBER_START = 10\n"); at != std::string::npos) t2.replace(at, 23, "LINE_NUMBER_START = 999950\n");
      shipped("S-big" + std::to_string((long)c), s, t2, {cut("A", spin(1)), b, cut("A2", spin(2))});
    }
  }
  return 0;
}
