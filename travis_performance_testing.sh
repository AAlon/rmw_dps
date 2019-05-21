#!/bin/bash
set -e

git clone https://github.com/irobot-ros/ros2-performance ${TRAVIS_BUILD_DIR}/../ros2-performance
docker exec osrf_ros2_nightly /bin/bash -c "apt-get update && source /opt/ros/dashing/setup.bash && rosdep update && rosdep install --from-paths /shared/ros2-performance --ignore-src -r -y"
docker exec osrf_ros2_nightly /bin/bash -c "source /opt/ros/dashing/setup.bash && source /shared/rmw_dps/install/local_setup.bash && export RMW_IMPLEMENTATION=rmw_dps_cpp && colcon build --packages-up-to benchmark"
