#!/usr/bin/env bash

PROJ_ROOT=$(dirname $0)

if ! type -p clang-format >/dev/null; then
    echo "'clang-format' not found"
    exit 1
fi

CLANG_FORMAT_VERSION=$(clang-format --version 2>/dev/null)
if [[ $CLANG_FORMAT_VERSION =~ ([0-9]+)\. ]]; then
    MAJOR_VERSION=${BASH_REMATCH[1]}
    if (( MAJOR_VERSION < 13 )); then
        echo "clang-format version >= 13 required, got $MAJOR_VERSION"
        exit 1
    else
        echo "clang-format version $MAJOR_VERSION"
    fi
else
    echo "unable to determine clang-format version"
    exit 1
fi

find "${PROJ_ROOT}" -type f \
    \( -name "*.cpp" -o -name "*.c" -o -name "*.hpp" -o -name "*.h" \) \
    -not -path "${PROJ_ROOT}/build/*" \
    -print -exec clang-format-13 --style=file -i '{}' \;