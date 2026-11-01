cfgs=[("A",0,-1,4,"ONP=4"),("C",2,-1,3,"ONP=3 TLMIN=0.3 TLMAX=4"),("E",8,-1,4,"ONP=4"),("G",3,-1,3,"ONP=4"),("I",1,-1,2,"ONP=3"),
      ("B",0,1,4,"ONP=4 TLMIN=0.3 TLMAX=4"),("D",6,-1,3,"ONP=3 TLMIN=0.3 TLMAX=4"),("F",5,-1,4,"X=1"),("H",8,1,2,"ONP=4 TLMIN=0.3 TLMAX=4"),("J",4,-1,2,"ONP=3"),("K",7,-1,2,"ONP=3")]
import sys
out=[]
for setname,base,onp1,bins in [("r3",81000,False,["conn6s"]),("n1",83000,False,["conn6s","conn6s_old"]),("n2",84000,True,["conn6s","conn6s_old"])]:
    for n,f,sp,ch,env in cfgs:
        if onp1:
            env=" ".join(e for e in env.split() if not e.startswith("ONP"))+" ONP=1"
        for k in range(ch):
            seed=base+ord(n)*10+k
            for b in bins:
                out.append(f"{b} {setname}_{n}_{seed} {seed} {f} {sp} {env}")
print("\n".join(out))
