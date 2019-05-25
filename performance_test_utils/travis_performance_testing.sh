#!/bin/bash
set -e
set -v

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
WS_DIR=${TRAVIS_BUILD_DIR}/../ros2_ws

# Get ROS2 dependencies and source
bash ${SCRIPT_DIR}/ros2_preinstall.sh
# Complete workspace setup: rmw_dps, ros2-performance
## rmw_dps
cp -R ${TRAVIS_BUILD_DIR} ${WS_DIR}/src/ros2/rmw_dps
## ros2-performance
git clone https://github.com/irobot-ros/ros2-performance ${WS_DIR}/src/ros2-performance

# rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ${WS_DIR}/src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 python3-lark-parser rti-connext-dds-5.3.1 urdfdom_headers robot_state_publisher demo_nodes_py"
sudo python3 -m pip install -U lark-parser


# Build
export RMW_IMPLEMENTATION=rmw_dps_cpp
cd ${WS_DIR}
colcon build --symlink-install --packages-ignore launch_ros image_tools dummy_robot_bringup qt_gui_cpp rqt_gui_cpp --cmake-args ' -DCMAKE_BUILD_TYPE=MinSizeRel'

# Source and run
set +v
. install/local_setup.bash
set -v

echo "Running benchmark"
timeout 70s ./install/benchmark/lib/benchmark/benchmark install/benchmark/lib/benchmark/topology/sierra_nevada.json -t 60 --ipc on || true

ls -lh log
cat log/resources.txt
python ${SCRIPT_DIR}/perf_checker.py ${WS_DIR}/log
