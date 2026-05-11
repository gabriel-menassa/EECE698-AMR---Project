#!/usr/bin/env bash
set -e

conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash

cd ~/EECE698/PROJECT/amr_project_ws
gz sim src/amr_demo/worlds/world.sdf
