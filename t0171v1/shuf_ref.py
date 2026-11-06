# Independent reference of the T-0171 shuffle (exact 64-bit arithmetic in Python, no C++ library involved).
M=(1<<64)-1
def run(n):
    st=(0x0171+n)&M
    cs=[]
    for i in range(n): cs+= [2*i,2*i+1]
    for i in range(len(cs)-1,0,-1):
        st=(st+0x9E3779B97F4A7C15)&M; z=st
        z=((z^(z>>30))*0xBF58476D1CE4E5B9)&M
        z=((z^(z>>27))*0x94D049BB133111EB)&M
        z=z^(z>>31)
        j=z%(i+1); cs[i],cs[j]=cs[j],cs[i]
    h=1469598103934665603
    for v in cs: h=((h^v)*1099511628211)&M
    print("n=%d hash=%016x first=%d,%d,%d"%(n,h,cs[0],cs[1],cs[2]))
for n in (8000,16000,10000,20000): run(n)
