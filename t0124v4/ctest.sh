#!/bin/bash
cd /home/claude/vscratch/t0124v4
V=/home/claude/vscratch/t0124v3
ctest --test-dir $V/br --output-on-failure > ctest_rel_serial.log 2>&1; echo "rel serial rc=$?" >> ctest.status
ctest --test-dir $V/br -j2 --repeat until-fail:3 --output-on-failure > ctest_rel_rep.log 2>&1; echo "rel rep rc=$?" >> ctest.status
echo done >> ctest.status
