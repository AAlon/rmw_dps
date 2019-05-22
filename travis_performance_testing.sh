#!/bin/bash
set -e
set -v

WS_DIR=${TRAVIS_BUILD_DIR}/../ros2_ws

sudo sh -c 'echo "APT::Get::AllowUnauthenticated \"true\";" > /etc/apt/apt.conf.d/99myown'
sudo sh -c 'echo "APT { Get { AllowUnauthenticated \"1\"; }; };" > /etc/apt/apt.conf.d/98myown'
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

sudo pip3 install argcomplete

# Setup workspace: ROS2, rmw_dps, ros2-performance
mkdir -p ${WS_DIR}/src
cd ${WS_DIR}
#wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
cp ${TRAVIS_BUILD_DIR}/ros2.repos ros2.repos
vcs import src < ros2.repos

## rmw_dps
mv ${TRAVIS_BUILD_DIR} ${WS_DIR}/src/ros2/
## ros2-performance
git clone https://github.com/irobot-ros/ros2-performance ${WS_DIR}/src/ros2-performance

# rosdep
sudo rosdep init
rosdep update
#rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"
rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 python3-lark-parser rti-connext-dds-5.3.1 urdfdom_headers"
sudo python3 -m pip install -U lark-parser


# Build
export RMW_IMPLEMENTATION=rmw_dps_cpp
# colcon build --symlink-install --cmake-args ' -DCMAKE_BUILD_TYPE=MinSizeRel'
colcon build --symlink-install --packages-ignore qt_gui_cpp rqt_gui_cpp --cmake-args ' -DCMAKE_BUILD_TYPE=MinSizeRel'

# Source and run
source install/local_setup.bash
./install/lib/benchmark/benchmark topology/sierra_nevada.json -t 60 --ipc on

#docker exec osrf_ros2_nightly /bin/bash -c "apt-get update && source /opt/ros/dashing/setup.bash && rosdep update && rosdep install --from-paths /shared/ros2-performance --ignore-src -r -y"
#docker exec osrf_ros2_nightly /bin/bash -c "source /opt/ros/dashing/setup.bash && source /shared/rmw_dps/install/local_setup.bash && export RMW_IMPLEMENTATION=rmw_dps_cpp && colcon build --packages-up-to benchmark"
