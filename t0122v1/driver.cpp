// Verifier throwaway driver for T-0122. Not committed.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "params.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "toolpath_store.hpp"
using namespace carvetto;
namespace fs = std::filesystem;
static std::string slurp(const std::string& p){std::ifstream i(p,std::ios::binary);std::ostringstream s;s<<i.rdbuf();return s.str();}
static std::vector<geometry::Contour> contoursOf(const document::Document& doc,const std::vector<std::string>& layers){
  std::vector<geometry::Contour> out;
  for(const auto& [id,obj]:doc.objects()){const auto* v=obj.asVector(); if(!v)continue;
    if(!layers.empty()){bool ok=false; for(const auto& l:doc.layers()) for(const auto& n:layers) if(l.name==n&&l.id==obj.layer) ok=true; if(!ok)continue;}
    out.insert(out.end(),v->contours.begin(),v->contours.end());}
  return out;}
static std::vector<toolpath::Toolpath> compute(const std::string& op,const std::string& text,const fs::path& base,const document::Document& doc){
  std::vector<toolpath::Toolpath> res;
  if(op=="profile"){auto j=cli::parseProfileParams(text); auto t=cli::resolveTool(j.common.tool,base); j.params.cuttingData=t.cuttingData;
    auto r=toolpath::profileToolpath(contoursOf(doc,j.common.layers),t.tool,j.params); if(r.ok()) res.push_back(r.toolpath);}
  else {auto j=cli::parsePocketParams(text); auto t=cli::resolveTool(j.common.tool,base);
    auto r=toolpath::pocketToolpath(contoursOf(doc,j.common.layers),{toolpath::PocketTool{t.tool,t.cuttingData}},j.params);
    for(auto& tp:r.toolpaths) if(!tp.isEmpty()) res.push_back(tp);}
  return res;}
int main(int argc,char** argv){
  std::string mode=argv[1];
  if(mode=="equal"){ // equal op params in.carv out.carv
    std::string op=argv[2],pp=argv[3];
    auto in=document::loadCarv(fs::path(argv[4])); auto out=document::loadCarv(fs::path(argv[5]));
    if(!in||!out){std::cout<<"load fail\n";return 2;}
    auto before=cli::storedToolpaths(in.value().extras); auto after=cli::storedToolpaths(out.value().extras);
    auto mine=compute(op,slurp(pp),fs::path(pp).parent_path(),in.value().document);
    if(after.size()!=before.size()+mine.size()){std::cout<<"count mismatch "<<after.size()<<" vs "<<before.size()<<"+"<<mine.size()<<"\n";return 1;}
    for(size_t i=0;i<mine.size();++i){ if(!(after[before.size()+i].toolpath==mine[i])){std::cout<<"MISMATCH toolpath "<<i<<"\n";return 1;} }
    for(size_t i=0;i<before.size();++i){ if(!(after[i].toolpath==before[i].toolpath)){std::cout<<"earlier toolpath changed\n";return 1;}}
    std::cout<<"EQUAL ("<<mine.size()<<" toolpaths, moves";for(auto&m:mine)std::cout<<" "<<m.moveCount();std::cout<<")\n";return 0;}
  if(mode=="angles"){ // angles carv tools.json
    auto in=document::loadCarv(fs::path(argv[2])); const auto& doc=in.value().document;
    fs::path base=fs::path(argv[3]).parent_path();
    int fails=0;
    auto check=[&](const char* what,const std::string& op,const std::string& text,auto build){
      auto viaJson=compute(op,text,base,doc);
      auto viaRad=build(); bool eq=viaJson.size()==viaRad.size(); for(size_t i=0;eq&&i<viaRad.size();++i) eq=viaJson[i]==viaRad[i];
      std::cout<<what<<": "<<(eq&&!viaRad.empty()?"EQUAL":"DIFFER")<<" moves="<<(viaRad.empty()?0:viaRad[0].moveCount())<<"\n"; if(!eq||viaRad.empty())++fails; return viaRad;};
    tooling::ImportResult db=tooling::importToolDatabaseJson(slurp(argv[3]));
    const tooling::Tool* t3=db.database.findTool("end-mill-3mm"); const tooling::Tool* t6=db.database.findTool("end-mill-6mm");
    auto outline=contoursOf(doc,{"Outline"}); auto pockets=contoursOf(doc,{"Pockets"});
    // pocket raster 30 deg
    auto r30=check("pocket raster_angle 30","pocket",R"({"tool":{"database":"tools.json","id":"end-mill-6mm"},"layers":["Pockets"],"cut_depth":3,"strategy":"raster","raster_angle":30})",[&]{
      toolpath::PocketParams p; p.cutDepth=3; p.strategy=toolpath::PocketStrategy::Raster; p.rasterAngle=geometry::kPi/6;
      auto r=toolpath::pocketToolpath(pockets,{toolpath::PocketTool{*t6,std::nullopt}},p); std::vector<toolpath::Toolpath> v; for(auto&tp:r.toolpaths) if(!tp.isEmpty()) v.push_back(tp); return v;});
    auto r0=check("pocket raster_angle 0","pocket",R"({"tool":{"database":"tools.json","id":"end-mill-6mm"},"layers":["Pockets"],"cut_depth":3,"strategy":"raster"})",[&]{
      toolpath::PocketParams p; p.cutDepth=3; p.strategy=toolpath::PocketStrategy::Raster;
      auto r=toolpath::pocketToolpath(pockets,{toolpath::PocketTool{*t6,std::nullopt}},p); std::vector<toolpath::Toolpath> v; for(auto&tp:r.toolpaths) if(!tp.isEmpty()) v.push_back(tp); return v;});
    std::cout<<"raster 30 differs from 0: "<<(!(r30[0]==r0[0]))<<"\n";
    // profile ramp angle 10 deg, leads straight angle 30 deg, circular sweep 60
    check("profile ramp angle 10 + straight lead angle 30","profile",R"({"tool":{"database":"tools.json","id":"end-mill-3mm"},"layers":["Outline"],"passes":{"cut_depth":6,"pass_depth":3},"ramp":{"type":"zig_zag","use_angle":true,"angle":10},"leads":{"type":"straight","angle":30,"length":4}})",[&]{
      toolpath::ProfileParams p; p.passes.cutDepth=6; p.passes.passDepth=3; p.ramp.type=toolpath::RampType::ZigZag; p.ramp.useAngle=true; p.ramp.angle=10*geometry::kPi/180; p.leads.type=toolpath::LeadType::Straight; p.leads.angle=geometry::kPi/6; p.leads.length=4;
      std::vector<toolpath::Toolpath> v; auto r=toolpath::profileToolpath(outline,*t3,p); if(r.ok()) v.push_back(r.toolpath); return v;});
    check("profile circular lead sweep 60, tol angle 0.5","profile",R"({"tool":{"database":"tools.json","id":"end-mill-3mm"},"layers":["Outline"],"passes":{"cut_depth":6,"pass_depth":3},"leads":{"type":"circular","sweep":60,"radius":4},"tolerance":{"angle":0.5}})",[&]{
      toolpath::ProfileParams p; p.passes.cutDepth=6; p.passes.passDepth=3; p.leads.type=toolpath::LeadType::Circular; p.leads.sweep=geometry::kPi/3; p.leads.radius=4; p.tolerance.angle=0.5*geometry::kPi/180;
      std::vector<toolpath::Toolpath> v; auto r=toolpath::profileToolpath(outline,*t3,p); if(r.ok()) v.push_back(r.toolpath); return v;});
    // parser-level
    auto pj=cli::parseProfileParams(R"({"tool":{"id":"x"},"ramp":{"angle":90},"leads":{"angle":180,"sweep":-45}})");
    std::printf("ramp 90deg -> %.17g (pi/2=%.17g), lead 180 -> %.17g, sweep -45 -> %.17g\n",pj.params.ramp.angle,geometry::kPi/2,pj.params.leads.angle,pj.params.leads.sweep);
    auto pd=cli::parseProfileParams(R"({"tool":{"id":"x"}})");
    std::printf("defaults kept exactly: ramp %d lead %d sweep %d tol %d\n",pd.params.ramp.angle==toolpath::RampOptions{}.angle,pd.params.leads.angle==toolpath::LeadOptions{}.angle,pd.params.leads.sweep==toolpath::LeadOptions{}.sweep,pd.params.tolerance.angle==geometry::Tolerance{}.angle);
    return fails;}
  return 3;}
