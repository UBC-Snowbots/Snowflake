#!/bin/bash

# This script can be called with no arguments (`./fix_formatting`), in which 
# case DEFAULT_BRANCH will be used as the branch to compare against to find 
# changes to format, or it can be called with a specific branch to compare 
# against (`./fix_formatting master`)

# The version of the clang executable to use
CLANG_VERSION=4.0

# The default branch to compare against
DEFAULT_BRANCH="master"

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check for arguments
if [ "$#" -eq 1 ]; then
    # If there is one arg, then that is the target branch to compare against
    # (to figure out what changes were made and need to be formatted)
    TARGET_BRANCH=$1
elif [ "$#" -eq 0 ]; then
    # If there are no args, then just compare against the default branch 
    # (to figure out what changes were made and need to be formatted)
    TARGET_BRANCH=$DEFAULT_BRANCH
else
    echo "This program takes at most one argument"
    echo "If one argument is given, it will be treated as the name of the branch to compare against to find changes to format"
    echo "If no arguments are given, then the default branch (currently: $DEFAULT_BRANCH) will be used"
    exit 1
fi

# Fix formatting on all changes between this branch and the target branch
OUTPUT="$($CURR_DIR/git-clang-format --binary $CURR_DIR/clang-format-$CLANG_VERSION --commit $TARGET_BRANCH)"

# Print the output (the files changed), replacing spaces with newlines
printf '%s\n' $OUTPUT
