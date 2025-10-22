#!/bin/bash
set -e

colcon build --symlink-install
echo "Packages built"
source ./install/setup.bash
echo "exe sourced"

