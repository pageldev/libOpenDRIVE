#!/usr/bin/env bash

find . -type f \
    \( -name "*.cpp" -o -name "*.c" -o -name "*.hpp" -o -name "*.h" \) \
    -not -path "./build/*" \
    -print -exec clang-format --style=file -i '{}' \;