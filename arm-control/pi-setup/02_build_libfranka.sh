#!/bin/bash
# Build libfranka (the robot library) in ~/franka_ros2_ws. Its dependencies (pinocchio, urdfdom)
# come from the ROS 2 Jazzy installation done in 01_base_ros.sh. Run as the normal user.
# libfranka >= 0.18 needs FR3 system version >= 5.9.0 (ours: 5.10.0).
set -exo pipefail
LIBFRANKA_VERSION=0.20.5
WS=~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
mkdir -p $WS/src && cd $WS
[ -d src/libfranka ] || git clone --recursive --depth 1 -b $LIBFRANKA_VERSION \
  https://github.com/frankarobotics/libfranka.git src/libfranka
rosdep install --from-paths src/libfranka --ignore-src --rosdistro jazzy -y -r
sudo apt-get clean
MAKEFLAGS=-j2 colcon build --packages-select libfranka \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
echo LIBFRANKA_DONE
