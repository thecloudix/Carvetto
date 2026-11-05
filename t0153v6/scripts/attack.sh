#!/bin/sh
timeout 7000 python3 -I /home/chris/harness/t0153v6/autoattack6.py /home/chris/scratch-t0153v6/bin/vc3_sab /home/chris/scratch-t0153v6/attack6 > /home/chris/harness/t0153v6/results/autoattack6.txt 2>&1
echo done > /home/chris/scratch-t0153v6/attack.done
