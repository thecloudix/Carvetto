// Verifier T-0162 r1: pause order (M5 before M0/M1) bypass attempts; line-length cap on all posts.
#include <cstdio>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"
using namespace carvetto;
using toolpath::Move; using toolpath::Toolpath; using toolpath::ToolSnapshot;
static ToolSnapshot spin(int n, std::string name="Mill") { ToolSnapshot t; t.name=name; t.toolNumber=n; t.diameter=3; t.feedRate=600; t.plungeRate=200; t.spindleSpeed=12000; return t; }
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
static std::string vis(const std::string& t) { std::string v; for (char c : t) { if (c=='\r') v += "<CR>"; else v += c; } return v; }
static std::string run(const std::string& label, const std::string& pptext, const std::vector<Toolpath>& tps, bool dump=true, post::PostContext ctx = test::fixtureContext()) {
  auto p = post::PostProcessor::parse(pptext);
  if (!p.ok()) { printf("[%s] PARSE ERROR %s\n", label.c_str(), p.error().describe().c_str()); return {}; }
  auto r = p.value().run(tps, ctx);
  if (!r.ok()) { printf("[%s] REFUSED: %s\n", label.c_str(), r.error().describe().substr(0,400).c_str()); return {}; }
  if (dump) printf("[%s] ACCEPTED:\n%s----\n", label.c_str(), vis(r.value().text).c_str());
  else printf("[%s] ACCEPTED\n", label.c_str());
  return r.value().text;
}
static std::string q(const std::string& s) { return "\"" + s + "\"\n"; }
static std::string readPost(const char* s) { FILE* f = fopen((std::string(CARVETTO_POSTS_DIR) + "/" + s + ".pp").c_str(), "rb"); std::string t; char b[4096]; size_t n; while ((n = fread(b,1,sizeof b,f))) t.append(b,n); fclose(f); return t; }
static std::vector<std::string> lines(const std::string& t) { std::vector<std::string> v; std::istringstream in(t); std::string l; while (std::getline(in,l)) { if (!l.empty() && l.back()=='\r') l.pop_back(); v.push_back(l);} return v; }
static std::string nameOf(size_t n, const std::string& pat) { std::string s; while (s.size() < n) s += pat; s.resize(n); return s; }
// Checks on an output: max line, non-ASCII, unbalanced parens per line, active comment keywords, N monotonic.
static int checkOut(const std::string& label, const std::string& out, bool semi, bool numbered) {
  int bad = 0; size_t maxL = 0; long long lastN = -1;
  for (const std::string& l : lines(out)) {
    maxL = std::max(maxL, l.size());
    if (l.size() > 250) { printf("  !! %s line %zu > 250: %.80s\n", label.c_str(), l.size(), l.c_str()); ++bad; }
    for (unsigned char c : l) if (c >= 0x80 || c < 0x20 || c=='~' || c=='!' || c=='?') { printf("  !! %s bad byte %02x in %.80s\n", label.c_str(), c, l.c_str()); ++bad; break; }
    if (!semi) { int depth = 0; bool err = false; size_t closes=0; for (char c : l) { if (c=='(') { if (depth) err = true; depth++; } else if (c==')') { if (!depth) err = true; else depth--; closes++; } } if (depth || err) { printf("  !! %s paren imbalance: %.120s\n", label.c_str(), l.c_str()); ++bad; } }
    // active comments: (KEYWORD, ... with spaces removed, case-insensitive
    for (size_t at = l.find(semi ? ';' : '('); at != std::string::npos; at = l.find(semi ? ';' : '(', at+1)) {
      std::string k; for (size_t i = at+1; i < l.size() && (isalnum((unsigned char)l[i]) || l[i]==' ' || l[i]==','); ++i) { if (l[i]!=' ') k += (char)toupper((unsigned char)l[i]); if (l[i]==',') break; }
      for (const char* kw : {"MSG,","DEBUG,","PRINT,","LOGOPEN,","LOGAPPEND,","LOGCLOSE","LOG,","PROBEOPEN","PROBECLOSE"})
        if (k.rfind(kw,0)==0) { printf("  !! %s active comment %s in %.100s\n", label.c_str(), kw, l.c_str()); ++bad; }
    }
    if (numbered && !l.empty() && l[0]=='N') { long long n = atoll(l.c_str()+1); if (n <= lastN && n > 100) { printf("  !! %s N not increasing %lld after %lld\n", label.c_str(), n, lastN); ++bad; } lastN = n; }
  }
  printf("  %s maxLine=%zu bad=%d\n", label.c_str(), maxL, bad);
  return bad;
}
int main() {
  const std::string Hp=q("(h [TOOLPATH_NAME])"), Up=q("M5")+q("(unlock)")+q("M0");
  const std::string Hs=q("; h [TOOLPATH_NAME]"), Us=q("M5")+q("; unlock")+q("M0");
  const std::string NV="VAR LINE_NUMBER = [N|A|N|1.0]\n";
  std::vector<Toolpath> spinThenDrag{cut("A", spin(1)), cut("D", dragT())};
  std::vector<Toolpath> dragThenSpin{cut("D", dragT()), cut("A", spin(1))};
  std::vector<Toolpath> dragOnly{cut("D", dragT())};
  printf("===== P: pause order (paren post), SPINDLE_OFF_PAUSE then SPINDLE_UNLOCK_PAUSE =====\n");
  const std::vector<std::vector<std::string>> bodies = {
    {"M0","M5"}, {"M00","M05"}, {"M000","M5"}, {"M1","M5","M0"}, {"M01","M5","M0"}, {"M0.0","M5"}, {"M1.","M5","M0"},
    {"N10 M0","N20 M5"}, {"N0010 M0","M5"}, {"m0","M5"}, {"M0","m5"}, {"/M5","M0","M5"}, {"(M5)","M0","M5"},
    {"M5 ; c","M0"}, {"M0 M5"}, {"M5 M0"}, {"M0","M5","M0"}, {"M60","M5","M0"}, {"M0 (pause)","M5"}, {"(x) M0","M5"},
    {"M0;","M5"}, {"M 0","M5"}, {"M+0","M5"}, {"M-0","M5"}, {"M0","/M5","M5"}, {"G4 P1","M0","M5"},
    // reference order OK:
    {"M5","M0"}, {"M5","M1","M0"}, {"N10 M5","N20 M0"}, {"M05","M00"}, {"M5","M0","M5","M0"} };
  for (const auto& b : bodies) {
    std::string body, label; for (auto& l : b) { body += q(l); label += "'" + l + "' "; }
    run("()-P off " + label, base("", Hp, q("M5")+q("M0"), body, Up), spinThenDrag, false);
    run("()-P unlock " + label, base("", Hp, q("M5")+q("M0"), q("M5")+q("M0"), body), dragThenSpin, false);
    run("()-P unlock-end " + label, base("", Hp, q("M5")+q("M0"), q("M5")+q("M0"), body), dragOnly, false);
  }
  printf("===== P: LINE_ENDING variants =====\n");
  for (const char* le : {"[13][10]", "[13]", "[10]"}) {
    std::string s = std::string("LINE_ENDING = \"") + le + "\"\n";
    run(std::string("()-P LE ") + le + " M0,M5", base(s, Hp, q("M5")+q("M0"), q("M0")+q("M5"), Up), spinThenDrag, false);
    run(std::string("()-P LE ") + le + " M5,M0", base(s, Hp, q("M5")+q("M0"), q("M5")+q("M0"), Up), spinThenDrag, false);
  }
  printf("===== P: ';' posts =====\n");
  for (const auto& b : std::vector<std::vector<std::string>>{{"M0","M5"},{"M1","M5","M0"},{"M5","M0"},{"; M5","M0","M5"},{"M0 ; then M5","M5"},{"M0","M5 ; x"},{"M5","M0 ; y"}}) {
    std::string body, label; for (auto& l : b) { body += q(l); label += "'" + l + "' "; }
    run(";-P off " + label, base("", Hs, q("M5")+q("M0"), body, Us), spinThenDrag, false);
    run(";-P unlock " + label, base("", Hs, q("M5")+q("M0"), q("M5")+q("M0"), body), dragThenSpin, false);
  }
  printf("===== P: values that write M words (template sees a tag as a space) =====\n");
  // [T] with prefix ' M' renders 'M1' for tool 1/3: a pause written before the M5.
  run("()-P VAR T prefix M -> written M3? (tool 3)", base("", Hp, q("M5")+q("M0"), q("[T]")+q("M5")+q("M0"), Up, "VAR TOOL_NUMBER = [T|A|M|1.0]\n"), spinThenDrag, false);
  { ToolSnapshot d = dragT(1); run("()-P VAR T prefix M -> written M1 (tool 1)", base("", Hp, q("M5")+q("M0"), q("[T]")+q("M5")+q("M0"), Up, "VAR TOOL_NUMBER = [T|A|M|1.0]\n"), {cut("A", spin(2)), cut("D", d)}, true); }
  { ToolSnapshot d = dragT(0); run("()-P VAR T prefix M -> written M0 (tool 0)", base("", Hp, q("M5")+q("M0"), q("[T]")+q("M5")+q("M0"), Up, "VAR TOOL_NUMBER = [T|A|M|1.0]\n"), {cut("A", spin(2)), cut("D", d)}, true); }
  { ToolSnapshot d = dragT(1); run("()-P N prefix ' M' (line number 1)", base("LINE_NUMBER_START = 1\n", Hp, q("M5")+q("M0"), q("[N]")+q("M5")+q("M0"), Up, "VAR LINE_NUMBER = [N|A|M|1.0]\n"), {cut("D", d)}, true); }

  printf("===== L: shipped posts, every text tag, lengths 0..6000, hostile content =====\n");
  const char* stems[] = {"grbl_mm","grbl_inch","linuxcnc_mm","linuxcnc_inch","mach3_mm","mach3_inch","mach4_mm","marlin_mm"};
  const std::vector<std::string> pats = {
    "a", "\xC3\xA9", "\xE2\x82\xAC", "\xF0\x9F\x98\x80", "x\xC3", "()", ")(", ",", "MSG,", "LOGOPEN,f ", "PROBEOPEN f", "DEBUG,#5 ",
    "\r\n", "; ", "~!?", " ", "\t", "abc)def(ghi ", "LOGCLOSE", "(MSG,hi)", "\\", "%", "[", "]"};
  const std::vector<size_t> lens = {0,1,2,3,100,200,230,233,234,235,236,237,238,240,245,246,247,248,249,250,251,252,253,254,255,256,257,300,511,512,1000,4095,6000};
  int totalBad = 0, runs = 0, refused = 0;
  std::map<std::string,std::string> sample;
  for (const char* s : stems) {
    const std::string txt = readPost(s); const bool semi = std::string(s)=="marlin_mm";
    for (const auto& pat : pats) for (size_t n : lens) {
      std::vector<Toolpath> job = test::fixtureToolpaths();
      for (const Toolpath& d : test::dragToolpaths()) job.push_back(d);
      int k = 0;
      for (Toolpath& tp : job) { tp.name = nameOf(n, std::to_string(k++) + pat); tp.tool.name = nameOf(n, pat + "T"); }
      post::PostContext ctx = test::fixtureContext(); ctx.fileName = nameOf(n, pat + "f"); ctx.product = nameOf(n, "P" + pat);
      ctx.fileNotes = nameOf(n, pat); ctx.pathName = nameOf(n, pat); ctx.fileDir = nameOf(n, pat); ctx.date = nameOf(n, pat); ctx.time = nameOf(n, pat);
      auto p = post::PostProcessor::parse(txt); auto r = p.value().run(job, ctx); ++runs;
      if (!r.ok()) { ++refused; printf("  REFUSED %s pat='%s' n=%zu: %.200s\n", s, vis(pat).c_str(), n, r.error().describe().c_str()); continue; }
      std::string lbl = std::string(s) + " pat=" + std::to_string(&pat - &pats[0]) + " n=" + std::to_string(n);
      int b = 0;
      // quiet check
      for (const std::string& l : lines(r.value().text)) {
        if (l.size() > 250) ++b;
        for (unsigned char c : l) if (c >= 0x80 || c < 0x20) { ++b; break; }
        if (!semi) { int depth = 0; bool err = false; for (char c : l) { if (c=='(') { if (depth) err=true; depth++; } else if (c==')') { if (!depth) err=true; else depth--; } } if (depth||err) ++b; }
        std::string sq; for (char c : l) if (c!=' ') sq += (char)toupper((unsigned char)c);
        for (const char* kw : {"(MSG,","(DEBUG,","(PRINT,","(LOGOPEN,","(LOGAPPEND,","(LOGCLOSE)","(PROBEOPEN","(PROBECLOSE)","(LOG,", ";MSG,", ";DEBUG,", ";PRINT,", ";LOGOPEN,", ";PROBEOPEN"})
          if (sq.find(kw) != std::string::npos) { ++b; printf("  !! %s active %s: %.120s\n", lbl.c_str(), kw, l.c_str()); }
      }
      if (b) { printf("  !! %s bad=%d\n", lbl.c_str(), b); checkOut(lbl, r.value().text, semi, false); }
      totalBad += b;
      if (n == 1000 && (pat == "()" || pat == "LOGOPEN,f ")) sample[lbl] = r.value().text;
    }
  }
  printf("L summary: runs=%d refused=%d badLines=%d\n", runs, refused, totalBad);
  for (auto& [k, v] : sample) { printf("--- sample %s (lines with '...'):\n", k.c_str()); for (auto& l : lines(v)) if (l.find("...") != std::string::npos) printf("%zu|%s\n", l.size(), l.c_str()); }

  printf("===== N/modal: Mach3 numbered lines with long names; compare against short-name run =====\n");
  for (const char* s : {"mach3_mm","mach3_inch","mach4_mm","linuxcnc_mm"}) {
    const std::string txt = readPost(s);
    auto mk = [&](size_t n) { std::vector<Toolpath> job = test::fixtureToolpaths(); for (const Toolpath& d : test::dragToolpaths()) job.push_back(d);
      for (Toolpath& tp : job) { tp.name = std::string(n, 'p'); tp.tool.name = std::string(n, 't'); } return job; };
    post::PostContext c1 = test::fixtureContext(); c1.fileName = "f";
    post::PostContext c2 = c1; c2.fileName = std::string(3000, 'f');
    auto p = post::PostProcessor::parse(txt);
    auto a = p.value().run(mk(5), c1), b = p.value().run(mk(3000), c2);
    if (!a.ok() || !b.ok()) { printf("  %s refused\n", s); continue; }
    auto la = lines(a.value().text), lb = lines(b.value().text);
    int diff = 0;
    if (la.size() != lb.size()) printf("  !! %s line count %zu vs %zu\n", s, la.size(), lb.size());
    for (size_t i = 0; i < std::min(la.size(), lb.size()); ++i) {
      auto strip = [](std::string l) { auto at = l.find('('); return at == std::string::npos ? l : l.substr(0, at); };
      if (strip(la[i]) != strip(lb[i])) { ++diff; if (diff < 5) printf("  !! %s line %zu: '%s' vs '%s'\n", s, i, la[i].c_str(), lb[i].c_str()); }
    }
    printf("  %s lines=%zu codeDiffs=%d warnings %zu vs %zu\n", s, la.size(), diff, a.value().warnings.size(), b.value().warnings.size());
  }

  printf("===== C: custom templates =====\n");
  auto hp = [&](const std::string& hdr, const std::string& extra="") { return extra + "VAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\nVAR FEED_RATE = [F|C| F|1.0]\nbegin HEADER\n" + hdr + "end\n" + kMoves; };
  std::vector<Toolpath> one{cut(std::string(400,'n'), spin(1, std::string(400,'t')))};
  // modal (Changed) text var: the cut value remembered; next line with the same var
  run("C1 Changed text var twice", hp(q("(a [TOOLPATH_NAME])")+q("(b [TOOLPATH_NAME])"), "VAR TOOLPATH_NAME = [TOOLPATH_NAME|C|]\n"), one, true);
  // N in the middle, number formats with Changed X on a comment line
  run("C2 numbered + modal X on comment line", hp(q("[N] G0[X] ([TOOLPATH_NAME])")+q("[N] G0[X] (x)"), "LINE_NUMBER_START = 999990\nLINE_NUMBER_MAXIMUM = 999999\nLINE_NUMBER_INCREMENT = 5\nVAR LINE_NUMBER = [N|A|N|1.0]\n"), one, true);
  // a text value in live code + a comment value: comment cut cannot save the live one
  run("C3 live text 260", hp(q("M117 [TOOLNAME] ([TOOLPATH_NAME])")), {cut("x", spin(1, std::string(260,'t')))}, false);
  run("C3b live text 100 + comment long", hp(q("M117 [TOOLNAME] ([TOOLPATH_NAME])")), {cut(std::string(400,'p'), spin(1, std::string(100,'t')))}, true);
  // unclosed paren comment holding the value
  run("C4 unclosed comment", hp(q("(T [TOOLNAME]")), one, true);
  // value then ')' then live code after comment
  run("C5 comment then live", hp(q("(T [TOOLNAME]) G0 X1")), one, true);
  // keyword completion: lead 'LOG' + value 'OPEN,...'
  run("C6 (LOG[TOOLNAME])", hp(q("(LOG[TOOLNAME])")), {cut("x", spin(1, "OPEN,/tmp/x" + std::string(400,'a')))}, true);
  run("C6b (PROBE[TOOLNAME])", hp(q("(PROBE[TOOLNAME])")), {cut("x", spin(1, "OPEN /tmp/x" + std::string(400,'a')))}, true);
  run("C6c ([TOOLNAME][TOOLPATH_NAME]) LOGOPEN + ,file", hp(q("([TOOLNAME][TOOLPATH_NAME])")), {cut(",/tmp/f" + std::string(400,'a'), spin(1, "LOGOPEN"))}, true);
  run("C6d ([TOOLNAME][TOOLPATH_NAME]) MSG + ,", hp(q("([TOOLNAME][TOOLPATH_NAME])")), {cut(std::string(1,',') + std::string(400,'m'), spin(1, "M S G"))}, true);
  run("C6e ([TOOLNAME] [TOOLPATH_NAME]) PROBE + OPEN", hp(q("([TOOLNAME] [TOOLPATH_NAME])")), {cut("OPEN f" + std::string(400,'m'), spin(1, "PROBE"))}, true);
  run("C6f long PROBE.. + OPEN", hp(q("([TOOLNAME][TOOLPATH_NAME])")), {cut("OPEN f" + std::string(10,'m'), spin(1, "PROBE" + std::string(400,'z')))}, true);
  // SUBSTITUTE mapping a char to '(' or ')': still sanitised after substitute
  run("C7 SUBSTITUTE", hp(q("(T [TOOLNAME])"), "SUBSTITUTE = \"a)\"\n"), one, false);
  // template exactly 250/251 with CRLF
  run("C8 template 250 CRLF", hp(q("(" + std::string(248,'x') + ")"), "LINE_ENDING = \"[13][10]\"\n"), {cut("a", spin(1))}, false);
  run("C8b template 251", hp(q("(" + std::string(249,'x') + ")")), {cut("a", spin(1))}, false);
  run("C8c 250 with leading spaces", hp(q("  (" + std::string(246,'x') + ")")), {cut("a", spin(1))}, false);
  run("C8d trailing spaces trimmed", hp(q("(" + std::string(248,'x') + ")     ")), {cut("a", spin(1))}, false);
  // TOOLS_USED (many tools)
  { std::vector<Toolpath> many; for (int i = 1; i <= 60; ++i) many.push_back(cut("p" + std::to_string(i), spin(i, "Tool number " + std::to_string(i)))); 
    std::string o = run("C9 TOOLS_USED 60 tools", hp(q("(Tools: [TOOLS_USED])")), many, false); if (!o.empty()) printf("  first: %zu|%s\n", lines(o)[0].size(), lines(o)[0].c_str()); }
  // drag section: cut comment in the spindle-off pause comment line, ';' and paren
  { std::string o = run("C10 drag comment cut", base("", Hp, q("M5")+q("M0"), q("M5")+q("(Drag [T]: [TOOLNAME])")+q("M0"), Up), {cut("A", spin(1)), cut("D", dragT(3, std::string(500, 'd')))}, true); }
  { std::string o = run("C10b drag ';' cut", base("", Hs, q("M5")+q("M0"), q("M5")+q("; Drag [T]: [TOOLNAME]")+q("M0"), Us), {cut("A", spin(1)), cut("D", dragT(3, std::string(500, 'd')))}, true); }
  // M0 line with a comment holding a long name (bare M0 + one comment)
  { std::string o = run("C11 M0 (name) drag", base("", Hp, q("M5")+q("M0"), q("M5")+q("M0 ([TOOLNAME])"), Up), {cut("A", spin(1)), cut("D", dragT(3, std::string(500, 'd')))}, true); }
  return totalBad ? 1 : 0;
}
