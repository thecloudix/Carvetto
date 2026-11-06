#!/bin/bash
# closed all-arc loops (T-0017 generator, 6 places incl. 3.3e5 mm), branch (I probe) vs e87f549, glibc and libm-perturbed
D=/home/chris/wt-T0128/build-verify3-drv; L=/home/chris/harness/t0128v3/logs/arcloop; mkdir -p $L; cd $D
for v in I Old; do
  ULP_K=0 timeout 600 ./arcloop$v 3000 0x7017A0C5 > $L/${v}_k0.log 2> $L/${v}_k0.err
  for k in 1 2 3 4; do for s in 1 2 3; do ULP_K=$k ULP_SEED=$s timeout 600 ./arcloop$v 3000 0x$((7000+s))A0C$k > $L/${v}_k${k}_s$s.log 2> $L/${v}_k${k}_s$s.err; done; done
done
echo done > $L/done
