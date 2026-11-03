#!/bin/bash
# classify.sh seed mode envassign sc : dumps a dl3 scene and classifies mainrel/hybrel/brrel move
# differences (tabs on and off) with cmpmv.py (tiny = line moves < 1e-6 mm dropped).
S=/tmp/claude-1000/-home-chris-Carvetto/120b15af-a822-46aa-8270-71a70ec2ea48/scratchpad/v159; B=$S/bin; T=$(dirname $(readlink -f $0))
s=$1; m=$2; e=$3; sc=$4; d=$S/cls/${s}_${m}_${e}_${sc}; mkdir -p $d
env HASH=1 NOLEAD=1 $e SC=$sc HDUMP=$d $B/dl3_mainrel $s 50 $m > /dev/null
f=$d/m${m}_${s}_${sc}.scn
# FORCESEP / FORCEIOPOS are applied before HDUMP except FORCESEP (applied after): patch sep flag via env in mv? -> mv reads params as dumped
for v in mainrel hybrel brrel; do for t in tab notab; do env NOLEAD=1 $e $B/mv_$v $f $t > $d/$v.$t.txt; done; done
echo "$s $m $e $sc | tab main-hyb: $(python3 $T/cmpmv.py $d/mainrel.tab.txt $d/hybrel.tab.txt) | tab hyb-br: $(python3 $T/cmpmv.py $d/hybrel.tab.txt $d/brrel.tab.txt) | notab main-hyb: $(python3 $T/cmpmv.py $d/mainrel.notab.txt $d/hybrel.notab.txt) | notab hyb-br: $(python3 $T/cmpmv.py $d/hybrel.notab.txt $d/brrel.notab.txt)"
