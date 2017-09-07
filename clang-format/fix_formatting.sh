#!/bin/bash

export CLANG_VERSION=4.0

# Run clang-format to see if we need to change anything
output="$(./git-clang-format --binary clang-format-$CLANG_VERSION --commit master)"

echo $output
