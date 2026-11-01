import sys
src,dst,ln=sys.argv[1],sys.argv[2],float(sys.argv[3])
L=open(src).read().split('\n')
for i,l in enumerate(L):
    if l.startswith('P '):
        f=l.split(' '); f[15]=ln.hex(); L[i]=' '.join(f)
open(dst,'w').write('\n'.join(L))
