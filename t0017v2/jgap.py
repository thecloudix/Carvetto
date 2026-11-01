import re,sys,math
exec(open('sjdump.py').read().split("for pre in ['branch','main']:")[0])
for pre in ['branch','main']:
    for i,c in enumerate(chains(pre)):
        sp=c['sp']; n=len(sp)
        js=range(n) if c['closed'] else range(n-1)
        for j in js:
            e=ends(sp[j])[1]; s=ends(sp[(j+1)%n])[0]
            g=math.dist(e,s); sc=max(1,abs(e[0]),abs(e[1]))
            if g>1e-12*sc: print(pre,'chain',i,'junction',j,'gap %.3g'%g,'kinds',sp[j][2],sp[(j+1)%n][2], 'at',e)
