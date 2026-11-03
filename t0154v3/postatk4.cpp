// Verifier T-0154 r3: attacks on the strict template rule / rendered guard.
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
static std::string P="\"M5\"\n\"(pause)\"\n\"M0\"\n";
static std::string U="\"M5\"\n\"(unlock)\"\n\"M0\"\n";
static std::string mkpost(std::string header, std::string tc, std::string ns, std::string pause, std::string unlock, std::string extra="", std::string vars="") {
  std::string s = "VAR SPINDLE_SPEED = [S|A| S|1.0]\nVAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\nVAR FEED_RATE = [F|C| F|1.0]\n" + vars;
  s += "begin HEADER\n"+header+"end\nbegin TOOLCHANGE\n"+tc+"end\nbegin SPINDLE_ON\n\"M3[S]\"\nend\nbegin NEW_SEGMENT\n"+ns+"end\n";
  if (!pause.empty()) s += "begin SPINDLE_OFF_PAUSE\n"+pause+"end\n";
  if (!unlock.empty()) s += "begin SPINDLE_UNLOCK_PAUSE\n"+unlock+"end\n";
  s += "begin FOOTER\n\"M5\"\n\"M30\"\nend\n" + extra + kMoves;
  return s;
}
static void run(const std::string& label, const std::string& pptext, const std::vector<Toolpath>& tps) {
  auto p = post::PostProcessor::parse(pptext);
  if (!p.ok()) { printf("[%s] PARSE ERROR %s\n", label.c_str(), p.error().describe().c_str()); return; }
  auto r = p.value().run(tps, test::fixtureContext());
  if (!r.ok()) { printf("[%s] REFUSED: %s\n", label.c_str(), r.error().describe().c_str()); return; }
  printf("[%s] ACCEPTED:\n%s----\n", label.c_str(), r.value().text.c_str());
}
int main() {
  const std::string H="\"(h)\"\n", T="\"M5\"\n\"M6 T[T]\"\n", N="\"(seg)\"\n";
  std::vector<Toolpath> first{cut("D", dragT())};
  std::vector<Toolpath> second{cut("A", spin(1)), cut("D", dragT())};
  std::vector<Toolpath> back{cut("D", dragT()), cut("A", spin(1))};
  struct C{const char* l; std::string line;};
  std::vector<C> lits{
    // round-2 attacks
    {"r2a NBSP M\\xC2\\xA03","\"M\xC2\xA0" "3\"\n"},
    {"r2b (a (b) M3 S1000","\"(a (b) M3 S1000\"\n"},
    {"r2c #1=3 / M#1","\"#1=3\"\n\"M#1\"\n"},
    {"r2c S#1","\"S#1\"\n"},
    // whitespace / case / forms
    {"tab M<TAB>3","\"M\t3\"\n"},
    {"lower m3","\"m3\"\n"},
    {"lower s1000","\"s1000\"\n"},
    {"M03","\"M03\"\n"}, {"M3.0","\"M3.0\"\n"}, {"M+3","\"M+3\"\n"}, {"M 3","\"M 3\"\n"},
    {"S 1 000","\"S 1 000\"\n"}, {"N10M3","\"N10M3\"\n"}, {"G0X1M3","\"G0 X1 M3\"\n"},
    {"S1e3","\"S1e3\"\n"}, {"M3 in comment","\"(M3)\"\n"}, {"S5 in comment","\"(speed S5)\"\n"},
    {"comment-in-word M(c)3","\"M(c)3\"\n"},
    // fractional M values (controller rounding)
    {"M3.004 (Grbl mantissa rounds to 0)","\"M3.004\"\n"},
    {"M4.004","\"M4.004\"\n"},
    {"M3.00005 (LinuxCNC int tol 1e-4)","\"M3.00005\"\n"},
    {"M2.99995 (LinuxCNC ceil)","\"M2.99995\"\n"},
    {"M3.5 (Marlin codenum 3)","\"M3.5\"\n"},
    {"M259 (Grbl uint8 wrap)","\"M259\"\n"},
    // slash mid-word (Grbl ignores '/')
    {"M/3 (Grbl ignores /)","\"M/3\"\n"},
    {"G0 Z5 M/3","\"G0 Z5 M/3\"\n"},
    {"S/1000","\"S/1000\"\n"},
    {"M3 S/1000 after slash? M/4","\"M/4\"\n"},
    {"/M3 block delete","\"/M3\"\n"},
    {"; M3 non-marlin","\"G0 ; M3\"\n"},
    {"realtime ~ after M0","\"~\"\n"},
    {"$ cmd","\"$X\"\n"},
    {"CR via \\r","\"M5\rM3\"\n"},
    {"M-3","\"M-3\"\n"},
    {"M.3e1","\"M.3e1\"\n"},
  };
  for (auto& c : lits) {
    run(std::string("PAUSE ")+c.l, mkpost(H, T, N, P+c.line, U), first);
    run(std::string("TC ")+c.l, mkpost(H, T+c.line, N, P, U), second);
    run(std::string("UNLOCK ")+c.l, mkpost(H, T, N, P, U+c.line), back);
    run(std::string("MOVE ")+c.l, mkpost(H, T, N, P, U, "begin PLUNGE_MOVE\n\"G1[Z][F]\"\n"+c.line+"end\n"), first);
  }
  // values in live code
  std::vector<std::string> names{"M/3", "x M3.004", "M3.5", "S/9000", "a(b)M3", "a;M3", "(M3)", "M3)", "S12 drill M3", "M 3", "~", "M#1"};
  for (auto& nm : names) {
    run("LIVE TOOLNAME '"+nm+"'", mkpost(H, T, N, P, U, "begin PLUNGE_MOVE\n\"G1[Z][F] [TOOLNAME]\"\nend\n"), std::vector<Toolpath>{cut("D", dragT(3, nm))});
    run("COMMENT TOOLNAME '"+nm+"'", mkpost(H, T, N, P+"\"(tool [TOOLNAME])\"\n", U), std::vector<Toolpath>{cut("D", dragT(3, nm))});
  }
  // footer after drag: not under the rule (by design)
  run("FOOTER M3 after drag", "VAR S = [S|A| S|1.0]\nbegin HEADER\n\"(h)\"\nend\nbegin SPINDLE_OFF_PAUSE\n" + P + "end\nbegin SPINDLE_UNLOCK_PAUSE\n" + U + "end\nbegin FOOTER\n\"M3 S1000\"\nend\n" + kMoves, first);
}
