// Verifier T-0154 r5 (final round).
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
int main() {
  const std::string Hp="\"(h)\"\n", Tp="\"M5\"\n\"M0\"\n", Pp="\"M5\"\n\"(pause)\"\n\"M0\"\n", Up="\"M5\"\n\"(unlock)\"\n\"M0\"\n";
  std::vector<Toolpath> first{cut("D", dragT())};
  std::vector<Toolpath> spinThenDrag{cut("A", spin(1)), cut("D", dragT())};
  // 1. LINE_ENDING carries literal text after every line (not seen by the line check)
  run("A1 LINE_ENDING ' M3 S12000[13][10]'", base("LINE_ENDING = \" M3 S12000[13][10]\"\n", Hp, Tp, Pp, Up), first);
  run("A1b LINE_ENDING '[10]M3[10]'", base("LINE_ENDING = \"[10]M3[10]\"\n", Hp, Tp, Pp, Up), first);
  const std::string Hs="\"; h\"\n", Ts="\"M5\"\n\"M0\"\n", Us="\"M5\"\n\"; unlock\"\n\"M0\"\n";
  const std::string TV="VAR TOOL_NUMBER = [T|A||1.0]\n";
  // B1 LINE_ENDING
  for (std::string le : {" M3 S12000[13][10]", "[10]M3[10]", "[10][10]", "[13][13][10]", "[10][13]", "", "[0][10]", "[9][10]", "[10] ", "[13][10]", "[13]", "[10]"})
    run("B1 LINE_ENDING '"+le+"'", base("LINE_ENDING = \""+le+"\"\n", Hp, Tp, Pp, Up), first);
  // B2 Marlin: uppercase message word ending in S/P, then a digits-only word: FASTER_GCODE_PARSER skips spaces -> S<n>/P<n>
  run("C1 ;-post pause 'M0 LOAD TOOLS [T]' tool 3", base("", Hs, Ts, "\"M5\"\n\"M0 LOAD TOOLS [T]\"\n", Us, TV), {cut("D", dragT(3))});
  run("C2 ;-post pause 'M0 SPINDLE STOP 1'", base("", Hs, Ts, "\"M5\"\n\"M0 SPINDLE STOP 1\"\n", Us), first);
  run("C3 ;-post unlock 'M0 UNLOCK UP 5'", base("", Hs, Ts, "\"M5\"\n\"M0\"\n", "\"M5\"\n\"M0 UNLOCK UP 5\"\n"), first);
  run("C4 ;-post pause 'M0 Load tools [T]' (GCODE_CASE_INSENSITIVE only)", base("", Hs, Ts, "\"M5\"\n\"M0 Load tools [T]\"\n", Us, TV), {cut("D", dragT(3))});
  run("C5 ;-post pause 'M0 [T]' with VAR prefix 'TOOLS '", base("", Hs, Ts, "\"M5\"\n\"M0 [T]\"\n", Us, "VAR TOOL_NUMBER = [T|A|TOOLS |1.0]\n"), {cut("D", dragT(3))});
  run("C6 ;-post pause 'M0 [T]' with VAR prefix 'S'", base("", Hs, Ts, "\"M5\"\n\"M0 [T]\"\n", Us, "VAR TOOL_NUMBER = [T|A|S|1.0]\n"), {cut("D", dragT(3))});
  run("C7 ;-post pause 'M0 S1'", base("", Hs, Ts, "\"M5\"\n\"M0 S1\"\n", Us), first);
  run("C8 ;-post pause 'M0 P1'", base("", Hs, Ts, "\"M5\"\n\"M0 P1\"\n", Us), first);
  run("C9 paren-post pause 'M0 P1' (Marlin with PAREN_COMMENTS: 1 ms timeout)", base("", Hp, Tp, "\"M5\"\n\"M0 P1\"\n", Up), first);
  run("C10 paren-post pause 'M0 P[T]' tool 3", base("", Hp, Tp, "\"M5\"\n\"M0 P[T]\"\n", Up, TV), {cut("D", dragT(3))});
  run("C13 ;-post pause 'M0 TOOL UP [T]' tool 3 (P after the [T] deviation)", base("", Hs, Ts, "\"M5\"\n\"M0 TOOL UP [T]\"\n", Us, TV), {cut("D", dragT(3))});
  run("C14 ;-post pause 'M0 Lock up [T]' tool 3 (GCODE_CASE_INSENSITIVE only)", base("", Hs, Ts, "\"M5\"\n\"M0 Lock up [T]\"\n", Us, TV), {cut("D", dragT(3))});
  run("C15 ;-post pause 'M0 Load tools 3 now' (GCODE_CASE_INSENSITIVE only)", base("", Hs, Ts, "\"M5\"\n\"M0 Load tools 3 now\"\n", Us), first);
  run("C16 ;-post TOOLCHANGE into drag 'M0 LOAD TOOL [T] STEP 2' (not the pause; pause follows)", base("", Hs, "\"M5\"\n\"M0 LOAD TOOL [T] STEP 2\"\n", Ts, Us, TV), spinThenDrag);
  // K: ';'-style custom post (Grbl/LinuxCNC/Mach accept ';' comments too): digits-only M0 message joins the M0 on space-ignoring controllers
  run("K1 ;-post pause 'M5','M0','M0 [T]' tool 3 (Grbl/LinuxCNC/Mach read M03)", base("", Hs, Ts, "\"M5\"\n\"M0\"\n\"M0 [T]\"\n", Us, TV), spinThenDrag);
  run("K2 ;-post TOOLCHANGE 'M5','M0 [T]' into drag tool 3", base("", Hs, "\"M5\"\n\"M0 [T]\"\n", Ts, Us, TV), spinThenDrag);
  run("K3 ;-post TOOLCHANGE 'M5','M0 [T]' into drag tool 4", base("", Hs, "\"M5\"\n\"M0 [T]\"\n", Ts, Us, TV), {cut("A", spin(1)), cut("D", dragT(4))});
  run("K4 ;-post unlock 'M5','M0','M0 [T]' tool 3", base("", Hs, Ts, Ts, "\"M5\"\n\"M0\"\n\"M0 [T]\"\n", TV), {cut("D", dragT(3))});
  run("C11 ;-post 'M0 [T]' tool 3 (fine)", base("", Hs, Ts, "\"M5\"\n\"M0 [T]\"\n", Us, TV), {cut("D", dragT(3))});
  for (std::string nm : {"M3 S18000", "x M3", "S1", "P1", "Drag S30", "( S1000)", "TOOLS 3"})
    run("C12 ;-post 'M0 [TOOLNAME]' name '"+nm+"'", base("", Hs, Ts, "\"M5\"\n\"M0 [TOOLNAME]\"\n", Us), {cut("D", dragT(3, nm))});
  // B3 VAR prefix / block delete
  run("D1 [N] prefix '/N'", base("", Hp, Tp, "\"[N] M5\"\n\"[N] M0\"\n", "\"[N] M5\"\n\"[N] M0\"\n", "VAR LINE_NUMBER = [N|A|/N|1.0]\n"), first);
  run("D2 [N] prefix ' /N'", base("", Hp, Tp, "\"[N] M5\"\n\"[N] M0\"\n", "\"[N] M5\"\n\"[N] M0\"\n", "VAR LINE_NUMBER = [N|A| /N|1.0]\n"), first);
  run("D3 [T] prefix ' M' tool 3", base("", Hp, Tp, Pp + "\"G0[T]\"\n", Up, "VAR TOOL_NUMBER = [T|A| M|1.0]\n"), {cut("D", dragT(3))});
  run("D4 [T] prefix ' M' tool 5 as the M5", base("", Hp, Tp, "\"G0[T]\"\n\"M0\"\n", Up, "VAR TOOL_NUMBER = [T|A| M|1.0]\n"), {cut("D", dragT(5))});
  run("D5 [T] prefix ' (' unclosed", base("", Hp, Tp, Pp + "\"G0[T]\"\n", Up, "VAR TOOL_NUMBER = [T|A| (|1.0]\n"), {cut("D", dragT(3))});
  run("D6 [SAFEZ] prefix ' Z' format 10.3", base("", Hp, Tp, Pp + "\"G0[SAFEZ]\"\n", Up, "VAR SAFE_Z = [SAFEZ|A| Z|10.3]\n"), first);
  // N1 written M0
  run("E1 'M0[T]' tool 5", base("", Hp, Tp, "\"M5\"\n\"M0[T]\"\n", Up, TV), {cut("D", dragT(5))});
  run("E2 'M0[T]' tool 3", base("", Hp, Tp, "\"M5\"\n\"M0[T]\"\n", Up, TV), {cut("D", dragT(3))});
  run("E3 ;-post 'M5 M0' one line", base("", Hs, Ts, "\"M5 M0\"\n", Us), first);
  run("E4 ;-post '[N] M5' '[N] M0'", base("", Hs, Ts, "\"[N] M5\"\n\"[N] M0\"\n", Us, "VAR LINE_NUMBER = [N|A|N|1.0]\n"), first);
  // N2 digit caps
  run("F1 'M000005'", base("", Hp, Tp, "\"M000005\"\n\"M0\"\n", Up), first);
  run("F2 'M00005'", base("", Hp, Tp, "\"M00005\"\n\"M0\"\n", Up), first);
  run("F3 'M00000'", base("", Hp, Tp, "\"M5\"\n\"M00000\"\n", Up), first);
  run("F4 'G0 Z000000005'", base("", Hp, Tp, Pp + "\"G0 Z000000005\"\n", Up), first);
  // N4 false positives in comments, N5 '(' on ;-post
  for (std::string nm : {"Drag 3mm 108", "Diamond 1mm 112 deg", "S12 drill M3", "Tip 0.1mm 410"})
    run("G1 paren comment name '"+nm+"'", base("", Hp, Tp, "\"M5\"\n\"(tool [TOOLNAME])\"\n\"M0\"\n", Up), {cut("D", dragT(3, nm))});
  run("G2 ;-post comment name 'M108'", base("", Hs, Ts, "\"M5\"\n\"; tool [TOOLNAME]\"\n\"M0\"\n", Us), {cut("D", dragT(3, "M108"))});
  run("G3 ;-post live '(x)'", base("", Hs, Ts, Ts + "\"G0 (x)\"\n", Us), first);
  // Marlin realtime P000 on a line of its own
  run("H1 ;-post 'P000' line (Marlin REALTIME_REPORTING_COMMANDS feed hold)", base("", Hs, Ts, Ts + "\"P000\"\n", Us), first);
  // BOM at start of the .pp
  run("I1 BOM before .pp", "\xEF\xBB\xBF" + base("", Hp, Tp, Pp, Up), first);
  // footer after unlock: literal spindle start is outside the drag section
  run("J1 footer 'M3 S12000' after drag-ending job", base("", Hp, Tp, Pp, Up), first);
}
