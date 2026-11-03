#!/bin/bash
# leaf.sh perf.data binary: self time by function (perf cannot symbolise here; addr2line does)
perf report -i $1 --stdio --sort sym -g none 2>/dev/null | awk '/^ +[0-9.]+%/{gsub("%","",$1); print $1, $3}' | while read pct a; do f=$(addr2line -f -C -e $2 $(printf "0x%x" $(( a + ${OFF:-0} ))) | head -n 1 | cut -c1-110); echo "$pct|$f"; done | awk -F'|' '{s[$2]+=$1} END{for(k in s) printf "%6.2f %s\n", s[k], k}' | sort -rn | head -${3:-25}
