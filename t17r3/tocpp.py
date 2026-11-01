import re,sys
L=re.compile(r'^  L \(([^,]+),([^)]+)\)->\(([^,]+),([^)]+)\)')
A=re.compile(r'^  A c\(([^,]+),([^)]+)\) r (\S+) sa (\S+) ea (\S+) (CCW|CW)')
def read(path):
    spans=[]; started=False
    for line in open(path):
        if line.startswith('branch chain') or line.startswith('input gap'): 
            if started: break
            continue
        m=L.match(line)
        if m: spans.append(('L',m.groups())); started=True; continue
        m=A.match(line)
        if m: spans.append(('A',m.groups())); started=True; continue
        if started and not line.startswith('  '): break
    return spans
def emit(spans, name='in', closed=False, indent='    '):
    if all(k=='L' for k,_ in spans) and all(spans[i][1][2]==spans[i+1][1][0] and spans[i][1][3]==spans[i+1][1][1] for i in range(len(spans)-1)):
        pts=[(spans[0][1][0],spans[0][1][1])]+[(g[2],g[3]) for _,g in spans]
        out=indent+'%sContour %s = polyline({' % ('' ,name)
        body=(',\n'+indent+' '*(len('Contour  = polyline({')+len(name))).join('{%s, %s}'%p for p in pts)
        s=out+body+'});\n'
    else:
        s=indent+'Contour %s;\n'%name
        for k,g in spans:
            if k=='L': s+=indent+'%s.spans.emplace_back(LineSpan{{%s, %s}, {%s, %s}});\n'%(name,*g)
            else: s+=indent+'%s.spans.emplace_back(ArcSpan{{%s, %s}, %s, %s, %s, ArcDirection::%s});\n'%(name,*g)
    if closed: s+=indent+'%s.closed = true;\n'%name
    return s
if __name__=='__main__':
    sp=read(sys.argv[1]); print('// %d spans'%len(sp)); print(emit(sp, closed=len(sys.argv)>2))
