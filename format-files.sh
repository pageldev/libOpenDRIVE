#!/usr/bin/env bash

find -E . -type f \
    -regex '.*\.(cpp|c|hpp|h)' \
    -not -path "./build/*" \
    -not -path "./Thirdparty/*" \
    -exec clang-format --style=file -i '{}' \;
