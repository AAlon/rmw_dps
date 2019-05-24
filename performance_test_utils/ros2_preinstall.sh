#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
WS_DIR=${TRAVIS_BUILD_DIR}/../ros2_ws

sudo sh -c 'echo "APT::Get::AllowUnauthenticated \"true\";" > /etc/apt/apt.conf.d/99myown'
#sudo sh -c 'echo "APT { Get { AllowUnauthenticated \"1\"; }; };" > /etc/apt/apt.conf.d/98myown'
# Locale
sudo apt-get update && sudo apt-get --reinstall install -qq language-pack-en
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Apt setup
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl http://repo.ros2.org/repos.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# Dependencies
sudo apt update --allow-unauthenticated && sudo apt --allow-unauthenticated install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-lark-parser \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
# install some pip packages needed for testing
sudo python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
# install Fast-RTPS dependencies
sudo apt install --allow-unauthenticated --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev


# Setup ROS2 workspace
mkdir -p ${WS_DIR}/src
cd ${WS_DIR}
## Fetch only the necessary packages using ros2_minimal.repos to avoid Xenial incompatibility issues
cp ${SCRIPT_DIR}/ros2_minimal.repos ros2_minimal.repos
vcs import src < ros2_minimal.repos
