#!/bin/bash
# Probes against the final build: spike repro, raw injection, phantom, rawph, R1 scans/lobes, cusp, sweeps.
cd /home/claude/vscratch/t0126/final2
./r1_spike_repro > spike.log 2>&1
for ch in 0.01 0.1 0.5; do echo "== CH $ch"; CH=$ch ./adv; done > adv.log 2>&1
timeout 2400 ./phantom 1 600 > phantom.log 2>&1
{ for f in 0 1 2 3; do ./rawph $f 1 100 0.01; ./rawph $f 1 40 0.1; done; } > rawph.log 2>&1
{
DLO=5.0 DHI=5.15 ./r1_scan 3 301 600 0.01
./r1_scan 3 301 4000 0.01
./r1_scan 3 700 4000 0.01
./r1_scan 3 606 4000 0.5
./r1_scan 3 505 4000 0.01
./r1_scan 10 101 4000 0.01
./r1_scan 6 101 4000 0.01
./r1_lobe 3 700 40 0.01
./r1_lobe 3 300 40 0.1
./r1_lobe 8 300 40 0.01
} > R1.log 2>&1
./cusp > cusp.log 2>&1
./v3src sweep3 8800100 50 > sw3.log 2>&1
echo DONE > E.done
