#!/bin/bash

# The version of the clang executable to use
export CLANG_VERSION=4.0

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check that we got exactly one argument
if [ "$#" -ne 1 ]; then
    echo "This program takes 1 argument (the name of the branch to compare against)."
    exit 1
fi
TARGET_BRANCH=$1

# Fix formatting on all changes between this branch and the target branch
OUTPUT="$($CURR_DIR/git-clang-format --binary $CURR_DIR/clang-format-$CLANG_VERSION --commit $TARGET_BRANCH)"

# Print the output (the files changed), replacing spaces with newlines
printf '%s\n' $OUTPUT
