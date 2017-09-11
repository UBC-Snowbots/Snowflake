#!/bin/bash

# The version of the clang executable to use
export CLANG_VERSION=4.0

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Run clang-format to see if we need to change anything
output="$($CURR_DIR/git-clang-format --binary $CURR_DIR/clang-format-$CLANG_VERSION --commit master)"

echo $output
