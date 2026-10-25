#!/bin/bash
# Phase 1: system update, swap, ROS 2 Jazzy (only for libfranka's dependencies), realtime
# permissions. Run with sudo.
set -euxo pipefail
export DEBIAN_FRONTEND=noninteractive
APT="apt-get -o DPkg::Lock::Timeout=900 -y -q"
USER_NAME=${SUDO_USER:-anton}

# Make sure noble-updates is enabled (the Pi image ships without it)
grep -q noble-updates /etc/apt/sources.list.d/ubuntu.sources || \
  sed -i '0,/^Suites: noble$/s//Suites: noble noble-updates noble-backports/' /etc/apt/sources.list.d/ubuntu.sources

$APT update
$APT full-upgrade
$APT install build-essential cmake git curl htop python3-pip python3-venv \
  libpoco-dev libeigen3-dev libfmt-dev rt-tests cpufrequtils linux-tools-raspi iw wireless-tools

# 4 GB swap (4 GB RAM is tight for C++ ROS builds)
if [ ! -f /swapfile ]; then
  fallocate -l 4G /swapfile && chmod 600 /swapfile && mkswap /swapfile
  echo '/swapfile none swap sw 0 0' >> /etc/fstab
fi
swapon /swapfile 2>/dev/null || true

# Realtime permissions (as required by libfranka)
groupadd -f realtime
usermod -aG realtime "$USER_NAME"
cat > /etc/security/limits.d/99-realtime.conf <<'L'
@realtime soft rtprio 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard memlock unlimited
L
# CPU always at max clock
echo 'GOVERNOR="performance"' > /etc/default/cpufrequtils

# ROS 2 Jazzy
$APT install software-properties-common
add-apt-repository -y universe
VER=1.3.0
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
dpkg -i /tmp/ros2-apt-source.deb
$APT update
$APT install ros-jazzy-ros-base ros-dev-tools  # rosdep/colcon + libfranka's dependencies
[ -d /etc/ros/rosdep ] && [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
sudo -u "$USER_NAME" -E rosdep update
grep -q 'ros/jazzy' /home/$USER_NAME/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USER_NAME/.bashrc
echo PHASE1_DONE
