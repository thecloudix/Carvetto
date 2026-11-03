#!/bin/bash
# mkmut.sh NAME : expects NAME.cpp (mutated profile.cpp) in build-verify2; builds mut/NAME/libcarvetto_core.a
B=/home/chris/wt-T0135/build-verify2; set -e
mkdir -p $B/mut/$1
cmd=$(cat $B/profile_cmd.txt | sed "s# -o [^ ]*profile.cpp.o # -o $B/mut/$1/profile.cpp.o #; s# -MD -MT [^ ]* -MF [^ ]* # #; s# -c [^ ]*profile.cpp# -c $B/$1.cpp -I/home/chris/wt-T0135/core/src/toolpath -I/home/chris/wt-T0135/core/src#")
eval "$cmd"
cp $B/rel/core/libcarvetto_core.a $B/mut/$1/
cd $B/mut/$1 && ar r libcarvetto_core.a profile.cpp.o
