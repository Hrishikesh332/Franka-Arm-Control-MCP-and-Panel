#!/bin/bash
# Install the arm-control app on the Pi: copy it to ~/franka_teach, build franka_teach_rt and
# enable the two services (realtime process + web UI on port 8080).
# Run as the normal user from a checkout of this repository; asks for sudo for systemd.
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
APP=~/franka_teach
LIBFRANKA=~/franka_ros2_ws/install/libfranka
ROSLIB=/opt/ros/jazzy/lib/aarch64-linux-gnu
mkdir -p $APP/bin
rsync -a --delete --exclude bin --exclude test "$HERE/../app/" $APP/
g++ -O2 -std=c++17 -Wall -o $APP/bin/franka_teach_rt $APP/rt/franka_teach_rt.cpp \
  -I $LIBFRANKA/include -I /usr/include/eigen3 -L $LIBFRANKA/lib -lfranka -lpthread \
  -Wl,-rpath,$LIBFRANKA/lib:$ROSLIB -Wl,-rpath-link,$ROSLIB
sudo apt-get install -y -q python3-flask python3-requests
sudo install -m 644 "$HERE"/services/franka-teach-rt.service "$HERE"/services/franka-teach-ui.service /etc/systemd/system/
[ -f /etc/default/franka-teach ] || sudo install -m 644 "$HERE"/services/franka-teach.env /etc/default/franka-teach
sudo systemctl daemon-reload
sudo systemctl enable franka-teach-rt.service franka-teach-ui.service
sudo systemctl restart franka-teach-rt.service franka-teach-ui.service
systemctl --no-pager is-active franka-teach-rt.service franka-teach-ui.service
