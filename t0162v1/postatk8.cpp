// Verifier T-0154 r7 (narrow check @ 77bb1c6): whitespace/separator attacks after the tab + N fix.
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
int main() {
  const std::string Hp=q("(h)"), Tp=q("M5")+q("M0"), Up=q("M5")+q("(unlock)")+q("M0");
  const std::string Hs=q("; h"), Ts=q("M5")+q("M0"), Us=q("M5")+q("; unlock")+q("M0");
  const std::string NV="VAR LINE_NUMBER = [N|A|N|1.0]\n", TV="VAR TOOL_NUMBER = [T|A||1.0]\n";
  std::vector<Toolpath> first{cut("D", dragT())};
  std::vector<Toolpath> spinThenDrag{cut("A", spin(1)), cut("D", dragT())};
  // --- ';'-style (Marlin model) ---
  std::vector<Toolpath> named{cut("D", dragT(3, "a\tb\x0b" "c\x0c" "d"))};
  auto S = [&](const char* lab, std::string pause, std::string tc = "") { run(std::string(";-") + lab, base("", Hs, tc.empty()?Ts:tc, pause, Us), first); };
  auto P = [&](const char* lab, std::string pause, std::string tc = "") { run(std::string("()-") + lab, base("", Hp, tc.empty()?Tp:tc, pause, Up), first); };
  const std::string nul(1, '\0');
  for (int pass = 0; pass < 2; ++pass) {
    std::function<void(const char*, std::string, std::string)> R = S; if (pass) R = P;
    R("W1 VT before M0", q("M5")+q("\x0bM0"), "");
    R("W2 FF before M0", q("M5")+q("\x0cM0"), "");
    R("W3 NUL before M0", q("M5")+q(nul+"M0"), "");
    R("W4 CR mid-line 'M5<CR>M0'", q("G0 Z5\rM5")+q("M0"), "");
    R("W5 CR between N and M0", q("M5")+q("N1\rM0"), "");
    R("W6 NBSP before M0", q("M5")+q("\xC2\xA0M0"), "");
    R("W7 Latin-1 NBSP before M5", q("\xA0M5")+q("M0"), "");
    R("W8 leading spaces '   M5' '   M0'", q("   M5")+q("   M0"), "");
    R("W9 trailing spaces 'M5   ' 'M0   '", q("M5   ")+q("M0   "), "");
    R("W10 VT inside word 'M\\x0b5'", q("M\x0b" "5")+q("M0"), "");
    R("W11 backspace 'G0\\x08M5' (Marlin erases)", q("M5")+q("G0 Z1\x08\x08\x08\x08M0"), "");
    R("W12 DEL 0x7F", q("M5\x7f")+q("M0"), "");
    R("W13 tab in TOOLCHANGE (drag) on this post", q("M5")+q("M0"), q("M5\t")+q("\tM0"));
  }
  // values / VAR prefixes carrying whitespace into drag lines
  run(";-V1 tool name with TAB/VT/FF in comment before M0", base("", Hs, Ts, q("M5")+q("; [TOOLNAME]")+q("M0"), Us), named);
  run("()-V2 tool name with TAB/VT/FF in comment", base("", Hp, Tp, q("M5")+q("([TOOLNAME])")+q("M0"), Up), named);
  run("()-V3 VAR X prefix with TAB on drag moves", base("", Hp, Tp, q("M5")+q("M0"), Up, "VAR X_POSITION = [X|C|\tX|1.3]\n"), first);
  run("()-W13b tab in TOOLCHANGE into drag (spin then drag)", base("", Hp, q("M5\t")+q("\tM0"), q("M5")+q("M0"), Up), spinThenDrag);
  run(";-W13b tab in TOOLCHANGE into drag (spin then drag)", base("", Hs, q("M5\t")+q("\tM0"), q("M5")+q("M0"), Us), spinThenDrag);
  run(";-W14 tab in drag move template", base("", Hs, Ts, q("M5")+q("M0"), Us) + "begin FEED_MOVE\n\"G1\t[X][Y][Z][F]\"\nend\n", first);
  run("()-W14 tab in drag move template", base("", Hp, Tp, q("M5")+q("M0"), Up) + "begin FEED_MOVE\n\"G1\t[X][Y][Z][F]\"\nend\n", first);
  run(";-W15 leading spaces before N-less M5 with ; comment", base("", Hs, Ts, q(" M5 ; x")+q("  M0  ; y"), Us), first);
  run(";-V4 VAR LINE_NUMBER on moves only (drag moves numbered)", base("", Hs, Ts, q("M5")+q("M0"), Us, NV) + "begin RAPID_MOVE\n\"[N] G0[X][Y][Z]\"\nend\n", first);
  run(";-V5 'N' in ; comment only", base("", Hs, Ts, q("M5")+q("; N1 note")+q("M0"), Us), first);
  // non-blocking class (ADR-0020: other words are the post author's responsibility)
  run("()-G1 Grbl LINE_BUFFER_SIZE 80 overflow drops the M5 line", base("", Hp, Tp, q("M5 G17 G21 G90 G94 G0 X12345678.123456 Y12345678.123456 Z12345678.123456 F12345678.123456")+q("M0"), Up), first);
  run("()-G2 Grbl 'M5 M6' (M6 unsupported -> error, M5 dropped)", base("", Hp, Tp, q("M5 M6")+q("M0"), Up), first);
  run("()-G3 Grbl 'M5 G0 G1' (modal group violation -> M5 dropped)", base("", Hp, Tp, q("M5 G0 G1")+q("M0"), Up), first);
  run(";-G4 Marlin 'G0 Z5 M5' (first word only)", base("", Hs, Ts, q("G0 Z5 M5")+q("M0"), Us), first);
  return 0;
}
