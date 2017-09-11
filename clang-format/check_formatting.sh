#!/bin/bash

# The version of the clang executable to use
export CLANG_VERSION=4.0

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Run clang-format to see if we need to change anything
output="$($CURR_DIR/git-clang-format --binary $CURR_DIR/clang-format-$CLANG_VERSION --commit master --diff)"

# Check the results of clang format
if [[ $output == *"no modified files to format"* ]] || [[ $output == *"clang-format did not modify any files"* ]] ; then
    # Great, we passed!
    echo "clang-format passed :D"
    exit 0
else
    # If we failed, echo the results of clang-format so we can see what we should chang
    echo "$output"
    echo "=================================================="
    echo "clang-format failed :( - please reformat your code by running fix_formatting.sh and resubmit"
    exit 1
fi
