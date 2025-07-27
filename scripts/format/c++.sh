#!/usr/bin/env bash

# This script applies clang-format to the whole repository.

# Check version
version_string=$(clang-format --version | sed -E 's/^.* ([0-9]+\.[0-9]+)\..*$/\1/')
expected_version_string='19.1'
if [[ "$version_string" == "$expected_version_string" ]]; then
    echo "clang-format major.minor version '$version_string' matches expected '$expected_version_string'"
else
    echo "clang-format major.minor version '$version_string' doesn't match expected '$expected_version_string'"
    exit 1
fi

# Get all C++ files checked into the repo, excluding submodules
root_folder=$(git rev-parse --show-toplevel)
all_files=$( \
    git ls-tree --full-tree -r --name-only HEAD . \
    | grep "^glomap.*\(\.cc\|\.h\|\.hpp\|\.cpp\|\.cu\)$" \
    | sed "s~^~$root_folder/~")
num_files=$(echo $all_files | wc -w)
echo "Formatting ${num_files} files"

clang-format -i $all_files
