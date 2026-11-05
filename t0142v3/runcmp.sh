#!/bin/bash
# head-to-head branch/main alternation for the layouts where main grows > 2.5x per doubling, and the adversarial ones
cd /home/chris/harness/t0142v3
for i in 1 2 3; do for L in tgrid hex hexrev; do for w in br main; do timeout 600 ./bin/po3_$w $L 40000 1 | sed "s/^/$w /"; done; done; done > results/po3_cmp40k.txt
for i in 1 2; do for L in ycomb cring; do for w in br main; do timeout 600 ./bin/po3_$w $L 3000 1 | sed "s/^/$w /"; done; done; done > results/po3_adv3k.txt
for L in ycomb cring; do timeout 600 ./bin/po3_br $L 3000 3 0.05 1; done >> results/po3_adv3k.txt
echo done >> results/po3_adv3k.txt
