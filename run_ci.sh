#!/bin/bash

# TODO: A nice comment here explaining what this file is

export TRAVIS_FOLD_COUNTER=1

# Display command in Travis console and fold output in dropdown section
function travis_run() {
  local command=$@

  echo -e "\e[0Ktravis_fold:start:command$TRAVIS_FOLD_COUNTER \e[34m$ $command\e[0m"
  # actually run command
  $command || exit 1 # kill build if error
  echo -e -n "\e[0Ktravis_fold:end:command$TRAVIS_FOLD_COUNTER\e[0m"

  let "TRAVIS_FOLD_COUNTER += 1"
}

# Note that we must build the codebase in order to run tests
if [ "$RUN_BUILD"=="true" || "$RUN_TESTS"=="true"]; then

    # Install all required dependecies
    travis_run ./setup_scripts/install_dependencies.sh

    # Build the codebase
    travis_run catkin_make
fi

if [ "$RUN_TESTS"=="true" ]; then

    # Run all the tests
    travis_run catkin_make run_tests

fi

if [ "$RUN_FORMATTING_CHECKS"=="true" ]; then
    # Determine what we should compare this branch against to figure out what 
    # files were changed
    if [ "$TRAVIS_PULL_REQUEST" == "false" ] ; then
      # Not in a pull request, so compare against parent commit
      base_commit="HEAD^"
      echo "Running clang-format against parent commit $(git rev-parse $base_commit)"
      echo "=================================================="
    else
      # In a pull request so compare against branch we're trying to merge into
      base_commit="$TRAVIS_BRANCH"
      # Make sure we pull the branches we're trying to merge against
      git fetch origin $base_commit:$base_commit
      echo "Running clang-format against branch $base_commit, with hash $(git rev-parse $base_commit)"
    fi
    # Check if we need to change any files
    output="$(./clang-format/git-clang-format --binary ./clang-format/clang-format-$CLANG_VERSION --commit $base_commit --diff)"
    if [[ $output == *"no modified files to format"* ]] || [[ $output == *"clang-format did not modify any files"* ]] ; then
        echo "clang-format passed :D"
        exit 0
    else
        echo "$output"
        echo "=================================================="
        echo "clang-format failed :( - please reformat your code via the \`git clang-format\` tool and resubmit"
        exit 1
    fi
fi


echo "CI Script has finished successfully!"
exit 0

