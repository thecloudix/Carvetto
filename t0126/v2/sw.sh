#!/bin/bash
# $1 mode, $2 seed0, $3 chunks of 50, $4 h, $5 log
cd /tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0125v2
for ((c=0; c<$3; c++)); do ./v3 $1 $(( $2 + c*50 )) 50 $4; done >> $5 2>&1
echo SWEEPDONE >> $5
