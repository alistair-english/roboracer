#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
source /home/autodrive_overlay/install/setup.bash

cd /home/autodrive_devkit
exec ros2 launch racer racer.launch.py
