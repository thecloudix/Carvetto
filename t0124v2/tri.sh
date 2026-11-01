#!/bin/bash
# usage: tri.sh LOG scene [extra env...]
L=$1; S=$2; shift 2
case $L in
A) A="41001 400 0 -1";; B) export TLMIN=0.3 TLMAX=4; A="41002 300 0 1";; C) export TLMIN=0.3 TLMAX=4; A="41003 250 1 1";;
D) A="41004 200 4 1";; E) export TLMIN=0.3 TLMAX=4; A="41005 250 6 -1";; F) A="41006 150 5 -1";; G) A="41007 150 3 -1";;
H) A="41008 150 7 -1";; I) A="41009 150 8 -1";; J) A="41010 150 2 -1";;
esac
env "$@" ./conn_dbgtool $A $S
