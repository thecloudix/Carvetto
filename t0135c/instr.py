import sys
p='/home/chris/wt-T0135/core/src/toolpath/profile.cpp'
s=open(p).read()
s=s.replace('#include <cstdio>\n','#include <cstdio>\n#include <cstdlib>\n#include <ctime>\n',1)
for call in ['matchRoughingTabs(finish, rough);','dropSweptTabs(finish, rough);','reportLoosePieces(finish, rough);','placeTabsOn(finish, rough);']:
    assert s.count('        '+call)==1, call
    s=s.replace('        '+call, '        { const std::clock_t t0 = std::clock(); '+call+' if (std::getenv("TIMING")) std::fprintf(stderr, "%s %.3f\\n", "'+call.split('(')[0]+'", double(std::clock() - t0) / CLOCKS_PER_SEC); }')
open(p,'w').write(s)
