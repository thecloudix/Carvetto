#!/bin/bash
# sigtree.sh SIG PID... : sends SIG to each PID and all its descendants (own processes only; used to pause/resume sweeps for quiet perf runs).
sig=$1; shift
walk() { local p=$1; for c in $(ps -o pid= --ppid $p); do walk $c; done; kill -$sig $p 2>/dev/null; }
for p in "$@"; do walk $p; done
