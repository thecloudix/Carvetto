# Sum cmp6 SUMMARY lines per family: offsets, differ, worse, better, CPU branch/main.
import re, glob, collections, sys
S = collections.defaultdict(lambda: [0]*7)
for f in glob.glob(sys.argv[1] if len(sys.argv) > 1 else '/home/chris/harness/t0128v1/logs/fz/*.log'):
    for l in open(f):
        m = re.search(r'SUMMARY fam(\d+) N=(\d+).*offsets (\d+) identical (\d+) differ (\d+).*worse (\d+) better (\d+).*cpu branch ([\d.]+) main ([\d.]+)', l)
        if m:
            a = S[int(m.group(1))]; a[0] += int(m.group(2)); a[1] += int(m.group(3)); a[2] += int(m.group(5)); a[3] += int(m.group(6)); a[4] += int(m.group(7)); a[5] += float(m.group(8)); a[6] += float(m.group(9))
for k in sorted(S):
    a = S[k]; print(f"fam{k}: inputs {a[0]} offsets {a[1]} differ {a[2]} worse {a[3]} better {a[4]} cpu branch {a[5]:.2f} main {a[6]:.2f} ratio {a[5]/max(a[6],1e-9):.3f}")
