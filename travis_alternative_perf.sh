#!/bin/bash
set -e

git clone https://github.com/irobot-ros/ros2-performance ${TRAVIS_BUILD_DIR}/../ros2-performance

docker pull osrf/ros2:nightly
docker run -dit -v "${TRAVIS_BUILD_DIR}/..:/shared"  --network host --name=osrf_ros2_nightly --privileged osrf/ros2:nightly /bin/bash
docker exec osrf_ros2_nightly /bin/bash -c "git config --global user.name nobody"
docker exec osrf_ros2_nightly /bin/bash -c "git config --global user.email noreply@osrfoundation.org"

docker exec osrf_ros2_nightly /bin/bash -c "apt-get update && apt-get install -y python-dev && source /opt/ros/dashing/setup.bash && rosdep update && rosdep install --from-paths /shared/rmw_dps --ignore-src -r -y"
docker exec osrf_ros2_nightly /bin/bash -c "source /opt/ros/dashing/setup.bash && export RMW_IMPLEMENTATION=rmw_dps_cpp && cd /shared && colcon build"
docker exec osrf_ros2_nightly /bin/bash -c "source /shared/install/local_setup.bash && export RMW_IMPLEMENTATION=rmw_dps_cpp && ./shared/install/benchmark/lib/benchmark/benchmark /shared/install/benchmark/lib/benchmark/topology/sierra_nevada.json -t 60 --ipc on"
docker exec osrf_ros2_nightly /bin/bash -c "ls -lh /shared/log"
