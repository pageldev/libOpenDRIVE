#!/bin/sh

make --always-make --dry-run | grep 'g++\|gcc' | grep -w '\-c\|main.cpp' | \
    jq -nR '[inputs|{directory:".", command:., file: match(" [^ ]+$").string[1:]}]'