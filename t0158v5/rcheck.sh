#!/bin/bash
# rcheck.sh scene [variants]: first-rapid start under 21 transforms (one4) per variant; prints max |S - S_id| per variant and
# plain-vs-first-variant diff; then chk4 classification (start vs mitre) for each variant. VSOUT/FLIP pass through.
B=/home/chris/wt-T0158/build-verify5-rel/bin; f=$1; V=${2:-br5 br5fma}
for v in $V; do
  base=$(timeout 300 $B/one4_$v $f 2>/dev/null | tail -1)
  mx=0; worst=id
  for t in "UP=1" "UP=2" "UP=3" "UP=4" "DN=1" "DN=2" "DN=3" "DN=4" "S=1.000000000001" "S=0.999999999999" "S=1.0000000001" "S=0.9999999999" "S=1.000000001" "S=0.999999999" "OX=3 OY=-7" "OX=0.1 OY=0.3" "OX=1000.25 OY=-999.75" "OX=3000 OY=2100"; do
    o=$(env $t timeout 300 $B/one4_$v $f 2>/dev/null | tail -1)
    d=$(python3 -c "import sys;a=sys.argv[1].split();b=sys.argv[2].split();print(((float(a[1])-float(b[1]))**2+(float(a[2])-float(b[2]))**2)**.5)" "$base" "$o")
    if python3 -c "import sys;sys.exit(0 if float('$d')>float('$mx') else 1)"; then mx=$d; worst="$t"; fi
  done
  echo "$v $(basename $f) id: $base | max move $mx ($worst)"
  timeout 300 $B/chk4_$v $f | sed "s|^.*/||; s|^|  chk4 $v |"
done
