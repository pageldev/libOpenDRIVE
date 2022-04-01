#!/usr/bin/env bash

find -E . -type f \
    -regex '.*\.(cpp|c|hpp|h|js)' \
    -not -path "./build/*" \
    -not -path "./Thirdparty/*" \
    -not -name './Viewer/ModuleOpenDrive.js' \
    -not -path './Viewer/lib/*' \
    -exec clang-format --style=file -i '{}' \;
