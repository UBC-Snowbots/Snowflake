#!/bin/bash

#############################################################################
# This script is what will run in CI (Continuous Integration) to make sure  #
# the code base is building, tests are passing, and formatting is correct   #
#                                                                           #
# It relies on certain bash variables being set by CI before this script is #
# called. This setup is dictated in the `.travis.yml` file                  # 
#############################################################################

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# If any command exits with a non-zero value (i.e. it throws an error)
# then this will cause travis to exit. Commands encapsulated between 
# conditionals, for example, do not report their return codes. Instread,
# only the conditional, a command itself, gets polled, so if cmake or 
# python throws an error, the information is lost. set -ev ensures that
# this does not happen
set -ev

# Change to the directory this script is in
cd $CURR_DIR

CLANG_VERSION="4.0"
# Determine what we should compare this branch against to figure out what 
# files were changed
if [ "$TRAVIS_PULL_REQUEST" == "false" ] ; then
  # Not in a pull request, so compare against parent commit
  BASE_COMMIT="HEAD^"
  echo "Running clang-format against parent commit $(git rev-parse $BASE_COMMIT)"
  echo "=================================================="
else
  # In a pull request so compare against branch we're trying to merge into
  # (ex. "master")
  BASE_COMMIT="$TRAVIS_BRANCH"
  # Make sure we pull the branches we're trying to merge against
  git fetch origin $BASE_COMMIT:$BASE_COMMIT
  echo "Running clang-format against branch $BASE_COMMIT, with hash $(git rev-parse $BASE_COMMIT)"
fi

# Check if we need to change any files
output="$(./clang-format/git-clang-format --binary ./clang-format/clang-format-$CLANG_VERSION --commit $BASE_COMMIT --diff)"
if [[ $output == *"no modified files to format"* ]] || [[ $output == *"clang-format did not modify any files"* ]] ; then
    echo "clang-format passed :D"
    exit 0
else
    echo "$output"
    echo "=================================================="
    echo "clang-format failed :( - please reformat your code via the \`git clang-format\` tool and resubmit"
    exit 1
fi

echo "CI Script has finished successfully!"
exit 0
