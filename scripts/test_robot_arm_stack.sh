#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
for suite in differential_screw arm robot_arm robot_arm_integration; do
  cmake -S "$root/tests/host/$suite" -B "$root/build/$suite" -G Ninja
  cmake --build "$root/build/$suite"
  ctest --test-dir "$root/build/$suite" --output-on-failure
done
