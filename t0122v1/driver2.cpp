#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>
#include "carvetto/document/carv_io.hpp"
#include "toolpath_store.hpp"
using namespace carvetto;
int main(int argc,char** argv){
  int fails=0;
  for(int i=1;i<argc;++i){
    std::ifstream f(argv[i],std::ios::binary); std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(f)),{});
    auto l=document::loadCarv(bytes); if(!l){std::cout<<argv[i]<<" load fail\n";++fails;continue;}
    auto s=document::saveCarv(l.value().document,l.value().extras);
    auto l2=document::loadCarv(s.value()); auto s2=document::saveCarv(l2.value().document,l2.value().extras);
    auto tps=cli::storedToolpaths(l2.value().extras);
    // re-encode decoded toolpaths -> must equal stored bytes
    bool reenc=true; for(auto& t:tps){ auto b=cli::encodeToolpath(t); if(b!=*l2.value().extras.member(t.member)) reenc=false; }
    bool same=(s.value()==bytes)&&(s2.value()==s.value());
    std::cout<<argv[i]<<": file==resave "<<(s.value()==bytes)<<" resave==resave2 "<<(s2.value()==s.value())<<" toolpaths "<<tps.size()<<" reencode-identical "<<reenc<<"\n";
    if(!same||!reenc)++fails;
  }
  document::CarvExtras e;
  for(const char* n: {"../x","../../etc/passwd","/abs","toolpaths/../manifest.json","toolpaths/./0001.json","C:/x","C:x","a\\b","manifest.json","job.json","toolpaths/","//x",""}){
    try{ (void)e.withMember(n,{}); std::cout<<"ACCEPTED bad name '"<<n<<"'\n"; ++fails;}catch(const std::invalid_argument& x){ std::cout<<"refused '"<<n<<"': "<<x.what()<<"\n"; }
  }
  for(const char* n: {"Manifest.json","MANIFEST.JSON","job.json/x","toolpaths/0001.json"}){
    try{ (void)e.withMember(n,{'{','}'}); std::cout<<"accepted '"<<n<<"'\n";}catch(const std::invalid_argument& x){ std::cout<<"refused '"<<n<<"': "<<x.what()<<"\n"; }
  }
  return fails;}
