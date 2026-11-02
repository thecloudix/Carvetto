// T-0133 r3 verifier: AC5 fuzzer. Random custom .pp lines: [template lead][open][template text]
// [VAR-prefixed first value][1-4 adjacent text values][optional tail]. Classifies findings by
// whether the comment contains template text (literal or VAR prefix) after a value (= documented gap).
#include <cstdio>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
using namespace carvetto::post; using carvetto::toolpath::Move; using carvetto::toolpath::Toolpath;
static std::string esc(const std::string& s){std::string o;for(unsigned char c:s){if(c<0x20||c>=0x7f){char b[8];snprintf(b,8,"\\x%02x",c);o+=b;}else o+=char(c);}return o;}
static std::string low(std::string s){for(auto&c:s)c=char(std::tolower((unsigned char)c));return s;}
// controller model: comment bodies (paren and ';'); active if (after leading ws, spaces/tabs removed, lowercased)
// starts with a known keyword; WORDCOMMA if it starts with [a-z0-9_]+ ','
static void comments(const std::string& line, std::vector<std::string>& out){
  bool p=false; std::string cur;
  for(size_t i=0;i<line.size();++i){char c=line[i];
    if(p){ if(c==')'){p=false;out.push_back(cur);cur.clear();} else cur+=c; continue;}
    if(c=='('){p=true;continue;} if(c==';'){out.push_back(line.substr(i+1));return;}}
  if(p) out.push_back(cur);
}
static const char* KW[]={"msg,","debug,","print,","log,","logopen,","logappend,","logclose","pyrun,","py,","abort,","probeopen","probeclose"};
int main(int argc,char**argv){
  int iters=argc>1?atoi(argv[1]):200000; unsigned seed=argc>2?unsigned(atoi(argv[2])):1;
  std::mt19937 rng(seed);
  auto pick=[&](const std::vector<std::string>& v){return v[rng()%v.size()];};
  const std::vector<std::string> tags={"TOOLNAME","TOOLPATH_NAME","TP_DIR","TP_FILENAME","TP_EXT","PATHNAME","PRODUCT","FILE_NOTES","DATE","TIME"};
  const std::vector<std::string> opens={"(","( ","(\t",";","; ","G0 X1 (","G0 ;","(a)(","(  "};
  const std::vector<std::string> lead={"","","","L","LOG","PRO","PROBE","M","m s","py","P R O B E","_","1","Log ","ms","PROBEOP","LOG CL","AXIS"," x "};
  const std::vector<std::string> pfx={"","","","OPEN","G"," ","LOG","CLOSE","_"};
  const std::vector<std::string> tail={"","","",",","OPEN",",x","CLOSE"," ,","G,","-","x"};
  const std::vector<std::string> tok={"MSG","msg","Log","LOG","OPEN","open","APPEND","CLOSE","close","PROBE","probe","PY","py","RUN","DEBUG","PRINT","ABORT","AXIS","G","O","L","E",",",",",","," ","\t","  ","\n","\r","(",")",";","-",":","~","\xC3\x96","\xC3\xB6","\xEF\xBC\x8C","\xC0\xAC","x","/tmp/f","1","_",""};
  auto val=[&](){int n=int(rng()%5);std::string s;for(int i=0;i<n;++i)s+=pick(tok);return s;};
  std::map<std::string,std::pair<long,std::string>> agg; long runs=0,errs=0;
  for(int it=0;it<iters;++it){
    int nv=1+int(rng()%4);
    std::vector<std::string> tg; for(int i=0;i<nv;++i){ std::string t; do{t=pick(tags);}while(std::find(tg.begin(),tg.end(),t)!=tg.end()); tg.push_back(t);} 
    std::string o=pick(opens), l=pick(lead), p=pick(pfx), tl=pick(tail);
    bool gapTpl = !tl.empty() && tl!="" ; // tail directly after last value (template after value)
    // occasionally put template text between values or a prefix on a later value (gap)
    std::string between; int bi=-1; std::string laterPfx; int pi=-1;
    if(nv>1 && rng()%6==0){bi=1+int(rng()%(nv-1)); between=pick(tail);}
    if(nv>1 && rng()%6==0){pi=1+int(rng()%(nv-1)); laterPfx=pick({",","OPEN"," ,","CLOSE"});}
    std::string pp;
    if(!p.empty()) pp+="VAR V0 = ["+tg[0]+"|A|"+p+"|1.0]\n";
    if(pi>0) pp+="VAR V1 = ["+tg[pi]+"|A|"+laterPfx+"|1.0]\n";
    std::string ln=o+l;
    for(int i=0;i<nv;++i){ if(i==bi) ln+=between; ln+="["+tg[i]+"]"; }
    ln+=tl; if(o.find('(')!=std::string::npos) ln+=")";
    pp+="begin HEADER\n\""+ln+"\"\nend\nbegin RAPID_MOVE\n\"G0[Z]\"\nend\n";
    bool gap = !tl.empty() || (bi>0 && !between.empty()) || (pi>0 && !laterPfx.empty());
    auto pr=PostProcessor::parse(pp); if(!pr.ok()){agg["PARSE"].first++; agg["PARSE"].second=pp+pr.error().describe(); continue;}
    std::map<std::string,std::string> v; for(auto&t:tags) v[t]=val();
    Toolpath t; t.name=v["TOOLPATH_NAME"]; t.tool.name=v["TOOLNAME"]; t.tool.feedRate=1000; t.tool.plungeRate=300; t.tool.spindleSpeed=1e4; t.moves={Move::rapid({0,0,5})};
    PostContext c; c.fileDir=v["TP_DIR"]; c.fileName=v["TP_FILENAME"]; c.fileExt=v["TP_EXT"]; c.pathName=v["PATHNAME"]; c.product=v["PRODUCT"]; c.fileNotes=v["FILE_NOTES"]; c.date=v["DATE"]; c.time=v["TIME"];
    auto r=pr.value().run({t},c); ++runs; if(!r.ok()){++errs;continue;}
    std::string line=r.value().text.substr(0,r.value().text.find('\n'));
    // control / realtime / high bytes
    for(unsigned char ch:line) if(ch<0x20&&ch!='\t'){agg["CTRL"].first++;agg["CTRL"].second=esc(line);} else if(ch>=0x80||ch=='~'||ch=='!'||ch=='?'){agg["RT/HI"].first++;agg["RT/HI"].second=esc(line);} 
    std::vector<std::string> cs; comments(line,cs);
    // template's own text alone active? (author's text) -> render with all values empty
    for(auto& cm:cs){
      size_t k=cm.find_first_not_of(" \t"); std::string body=k==std::string::npos?"":cm.substr(k);
      std::string sq; for(char ch:body) if(ch!=' '&&ch!='\t') sq+=char(std::tolower((unsigned char)ch));
      std::string kind;
      for(auto w:KW){ if(low(body).rfind(w,0)==0 || sq.rfind(w,0)==0){kind=std::string("ACTIVE:")+w;break;} }
      if(kind.empty()){ size_t w=sq.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_"); if(w!=std::string::npos&&w>0&&sq[w]==',') kind="WORDCOMMA"; }
      if(kind.empty()) continue;
      // is it the author's own text? (template lead+prefix alone)
      std::string own=low(l+p); std::string ownsq; for(char ch:own) if(ch!=' '&&ch!='\t') ownsq+=ch;
      bool authored=false; for(auto w:KW) if(ownsq.rfind(w,0)==0) authored=true;
      std::string key=std::string(gap?"GAP ":"REAL ")+(authored?"authored ":"")+kind;
      auto& e=agg[key]; if(e.first++<1) e.second=esc(line)+"   <= "+esc(ln)+" pfx0='"+esc(p)+"' laterPfx='"+esc(laterPfx)+"' vals: "+[&]{std::string s;for(auto&x:tg)s+=x+"="+esc(v[x])+" | ";return s;}();
    }
  }
  std::cout<<"runs="<<runs<<" errors="<<errs<<"\n"; for(auto&[k,x]:agg) std::cout<<k<<" x"<<x.first<<"\n   e.g. "<<x.second<<"\n";
}
