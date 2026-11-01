# T-0127 coder sweep jobs. conn lines: "conn <bin> <tag> <seed> <fam> <sep> env..."; cmp lines: "cmp <tag> <seed> <fam> <sep> <N> env..."
cfgs=[("A",0,-1,4,"ONP=4"),("C",2,-1,3,"ONP=3 TLMIN=0.3 TLMAX=4"),("E",8,-1,4,"ONP=4"),("G",3,-1,3,"ONP=4"),("I",1,-1,2,"ONP=3"),
      ("B",0,1,4,"ONP=4 TLMIN=0.3 TLMAX=4"),("D",6,-1,3,"ONP=3 TLMIN=0.3 TLMAX=4"),("F",5,-1,4,"X=1"),("H",8,1,2,"ONP=4 TLMIN=0.3 TLMAX=4"),("J",4,-1,2,"ONP=3"),("K",7,-1,2,"ONP=3")]
import sys
what=sys.argv[1]
out=[]
def envfor(env,onp1,neg):
    if onp1: env=" ".join(e for e in env.split() if not e.startswith("ONP"))+" ONP=1"
    if neg: env+=" NEGA=1"
    return env
if what=="conn":
    for setname,base,onp1,neg in [("n3",85000,False,False),("n4",86000,True,False),("g1",87000,False,True),("g2",88000,True,True)]:
        for n,f,sp,ch,env in cfgs:
            for k in range(ch):
                seed=base+ord(n)*10+k
                for b in ["conn7","conn7_old"]:
                    out.append(f"conn {b} {setname}_{n}_{seed} {seed} {f} {sp} {envfor(env,onp1,neg)}")
else:
    for setname,base,neg in [("r3",81000,False),("n1",83000,False),("n2",84000,False),("n3",85000,False),("g1",87000,True)]:
        for n,f,sp,ch,env in cfgs:
            for k in range(ch):
                seed=base+ord(n)*10+k
                for t in (0,1):
                    out.append(f"cmp {setname}_{n}_{seed}_t{t} {seed} {f} {sp} 50 TABS={t} {envfor(env,False,neg)}")
    for i,(seed,sp) in enumerate([(91001,-1),(91002,1),(91003,-1),(91004,1),(91005,-1),(91006,1),(91007,-1),(91008,1)]):
        for t in (0,1):
            out.append(f"cmp c_{seed}_t{t} {seed} 0 {sp} 120 TABS={t} ONP=4")
print("\n".join(out))
