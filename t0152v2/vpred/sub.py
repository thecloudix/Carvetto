import sys
# replace each tiny nonzero coordinate (<1e-140) by sign * 2^-600 * mantissa-ish multiple, keep structure
for line in sys.stdin:
    t=line.split()
    vals=[float.fromhex(x) for x in t[1:-1]]
    out=[]
    for x in vals:
        if x!=0 and abs(x)<1e-140:
            k=round(abs(x)/5e-324); x=(1 if x>0 else -1)*k*float.fromhex(sys.argv[1])
        out.append(x.hex())
    print(t[0]," ".join(out))
