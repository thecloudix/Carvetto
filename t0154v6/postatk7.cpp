// Verifier T-0154 r6 (narrow check @ 65aae68): bare-M0 rule attacks on every post style.
#include <cstdio>
#include <string>
#include <vector>
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
  run("S1 ;-post 'M0 ; Load tool [T] then resume' (comment on the M0 line)", base("", Hs, Ts, q("M5")+q("M0 ; Load tool [T] then resume"), Us, TV), first);
  run("S2 ;-post 'M0;P5' (no space)", base("", Hs, Ts, q("M5")+q("M0;P5"), Us), first);
  run("S3 ;-post 'M0 ; [TOOLNAME]' name 'P1 S1'", base("", Hs, Ts, q("M5")+q("M0 ; [TOOLNAME]"), Us), {cut("D", dragT(3, "P1 X1"))});
  run("S4 ;-post '[N] M0' N prefix", base("", Hs, Ts, q("M5")+q("[N] M0"), Us, NV), first);
  run("S5 ;-post '[N] M0' N prefix 'N ' (space)", base("", Hs, Ts, q("M5")+q("[N] M0"), Us, "VAR LINE_NUMBER = [N|A|N |1.0]\n"), first);
  run("S6 ;-post TAB between N and M0 (Marlin skips only ' ': letter TAB -> unknown cmd)", base("", Hs, Ts, q("M5")+q("[N]\tM0"), Us, NV), first);
  run("S7 ;-post leading TAB '\\tM0' (Marlin: unknown command)", base("", Hs, Ts, q("M5")+q("\tM0"), Us), first);
  run("S8 ;-post 'M0\\tP1'", base("", Hs, Ts, q("M5")+q("M0\tP1"), Us), first);
  run("S9 ;-post 'M0\\t; x'", base("", Hs, Ts, q("M5")+q("M0\t; x"), Us), first);
  run("S10 ;-post 'M 0'", base("", Hs, Ts, q("M5")+q("M 0"), Us), first);
  run("S11 ;-post 'M1' as the only pause (Grbl ignores M1)", base("", Hs, Ts, q("M5")+q("M1"), Us), first);
  run("S12 paren-post 'M1' as the only pause (Grbl ignores M1)", base("", Hp, Tp, q("M5")+q("M1"), Up), first);
  run("S13 ;-post 'M0 N5'", base("", Hs, Ts, q("M5")+q("M0 N5"), Us), first);
  run("S14 ;-post 'N5 N6 M0'", base("", Hs, Ts, q("M5")+q("N5 N6 M0"), Us), first);
  run("S15 ;-post 'M0 M0'", base("", Hs, Ts, q("M5")+q("M0 M0"), Us), first);
  run("S16 ;-post 'N5 M1'", base("", Hs, Ts, q("M5")+q("M0")+q("N5 M1"), Us), first);
  run("S17 ;-post 'G4 P5 M0' (M0 not first)", base("", Hs, Ts, q("M5")+q("G4 P5 M0"), Us), first);
  run("S18 ;-post line before M0 'G4 P1' then 'M0'", base("", Hs, Ts, q("M5")+q("G4 P1")+q("M0"), Us), first);
  run("S19 ;-post line after M0 'P000' (realtime quick pause)", base("", Hs, Ts, q("M5")+q("M0")+q("P000"), Us), first);
  run("S20 ;-post line after M0 'M108' in comment", base("", Hs, Ts, q("M5")+q("M0")+q("; M108"), Us), first);
  run("S21 ;-post '[T]' VAR prefix 'M0 N' -> 'M0 N3' on M5 line", base("", Hs, Ts, q("M5")+q("[T] M0"), Us, "VAR TOOL_NUMBER = [T|A|N|1.0]\n"), first);
  run("S22 ;-post VAR prefix 'M0 P' on [T]", base("", Hs, Ts, q("M5")+q("M0")+q("[T]"), Us, "VAR TOOL_NUMBER = [T|A|M0 P|1.0]\n"), first);
  run("S23 ;-post 'M00' / 'M0.0' / 'M000'", base("", Hs, Ts, q("M5")+q("M00")+q("M0.0")+q("M000"), Us), first);
  run("S24 ;-post 'M0.5' (Marlin subcode)", base("", Hs, Ts, q("M5")+q("M0")+q("M0.5"), Us), first);
  run("S25 ;-post 'N123456789 M0' (9 digits)", base("", Hs, Ts, q("M5")+q("N123456789 M0"), Us), first);
  run("S26 ;-post 'N1234567890 M0' (10 digits)", base("", Hs, Ts, q("M5")+q("N1234567890 M0"), Us), first);
  run("S27 ;-post 'N-1 M0'", base("", Hs, Ts, q("M5")+q("N-1 M0"), Us), first);
  run("S28 ;-post 'N1.5 M0'", base("", Hs, Ts, q("M5")+q("N1.5 M0"), Us), first);
  // --- paren style (Grbl / LinuxCNC / Mach) ---
  run("P1 paren 'M0 (Load tool [T])'", base("", Hp, Tp, q("M5")+q("M0 (Load tool [T])"), Up, TV), first);
  run("P2 paren '(msg) M0'", base("", Hp, Tp, q("M5")+q("(msg) M0"), Up), first);
  run("P3 paren 'M0 (a)(b)'", base("", Hp, Tp, q("M5")+q("M0 (a)(b)"), Up), first);
  run("P4 paren 'M0 (a) ; P1'", base("", Hp, Tp, q("M5")+q("M0 (a) ; P1"), Up), first);
  run("P5 paren 'M0 ; P1'", base("", Hp, Tp, q("M5")+q("M0 ; P1"), Up), first);
  run("P6 paren 'N5 (x) M0'", base("", Hp, Tp, q("M5")+q("N5 (x) M0"), Up), first);
  run("P7 paren 'M0 (P1'", base("", Hp, Tp, q("M5")+q("M0 (P1"), Up), first);
  run("P8 paren 'M0 (a) b)'", base("", Hp, Tp, q("M5")+q("M0 (a) b)"), Up), first);
  run("P9 paren 'M0 (a (b)' (LinuxCNC nested-comment error)", base("", Hp, Tp, q("M5")+q("M0 (a (b)"), Up), first);
  run("P10 paren 'M0(3)' (comment glued)", base("", Hp, Tp, q("M5")+q("M0(3)"), Up), first);
  run("P11 paren 'M(x)0' (Grbl strips comments -> M0; LinuxCNC/Mach?)", base("", Hp, Tp, q("M5")+q("M(x)0"), Up), first);
  run("P12 paren 'M0 ([TOOLNAME])' name ') M3 (", base("", Hp, Tp, q("M5")+q("M0 ([TOOLNAME])"), Up), {cut("D", dragT(3, ") M3 ("))});
  run("P13 paren 'M0 ([TOOLNAME])' name ')3'", base("", Hp, Tp, q("M5")+q("M0 ([TOOLNAME])"), Up), {cut("D", dragT(3, ")3"))});
  run("P14 paren 'M0 (x)3'", base("", Hp, Tp, q("M5")+q("M0 (x)3"), Up), first);
  run("P15 paren 'M0 (x) 3'", base("", Hp, Tp, q("M5")+q("M0 (x) 3"), Up), first);
  run("P16 paren '[N] M0' N prefix", base("", Hp, Tp, q("[N] M5")+q("[N] M0"), Up, NV), first);
  run("P17 paren 'N123456789 M0' (Grbl MAX_LINE_NUMBER 10000000 -> error)", base("", Hp, Tp, q("M5")+q("N123456789 M0"), Up), first);
  run("P18 paren 'M0 (MSG, spindle off)'", base("", Hp, Tp, q("M5")+q("M0 (MSG, spindle off)"), Up), first);
  run("P19 paren '/M0' block delete", base("", Hp, Tp, q("M5")+q("M0")+q("/M0"), Up), first);
  run("P20 paren 'M0 %'", base("", Hp, Tp, q("M5")+q("M0 %"), Up), first);
  run("P21 paren 'M0 (x) (y)' via [TOOLNAME] 'x) (y'", base("", Hp, Tp, q("M5")+q("M0 ([TOOLNAME])"), Up), {cut("D", dragT(3, "x) (y"))});
  run("P22 paren TOOLCHANGE into drag 'T[T] M6 M0'", base("", Hp, q("M5")+q("T[T] M6 M0"), q("M5")+q("M0"), Up, TV), spinThenDrag);

  run("T1 ;-post TOOLCHANGE '\\tM5' + pause '\\tM5' (Marlin skips both M5)", base("", Hs, q("\tM5")+q("M0"), q("\tM5")+q("M0"), Us), spinThenDrag);
  run("T2 ;-post VAR prefix TAB on [N]: '[N] M0' with prefix 'N' value then TAB? (prefix '\\t')", base("", Hs, Ts, q("M5")+q("[N]M0"), Us, "VAR LINE_NUMBER = [N|A|\t|1.0]\n"), first);
  run("T3 ;-post '[TOOLNAME]' leading on comment line before M0 (name with TAB)", base("", Hs, Ts, q("M5")+q("; [TOOLNAME]")+q("M0"), Us), {cut("D", dragT(3, "a\tb"))});
  run("T4 ;-post vertical tab '\\vM0'", base("", Hs, Ts, q("M5")+q("\vM0"), Us), first);
  run("T5 ;-post 'M0\\t' trailing TAB", base("", Hs, Ts, q("M5")+q("M0\t"), Us), first);
  run("T6 shipped-style marlin pause with ' M0' leading space", base("", Hs, Ts, q("M5")+q(" M0"), Us), first);

  run("T1b ;-post HEADER 'M3[S]', TOOLCHANGE '\\tM5','M0', pause '\\tM5','M0' (Marlin: spindle never stopped before drag)", base("", Hs + q("M3[S]"), q("\tM5")+q("M0"), q("\tM5")+q("M0"), Us), spinThenDrag);
  return 0;
}
