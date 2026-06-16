#!/bin/bash
# ROS2 Humble + RViz2 컨테이너 실행 → Isaac Sim(sensor_drive.py)이 퍼블리시하는
# /point_cloud, /rgb, /tf 를 시각화한다.
#   ROS2 Humble: Apache-2.0 / RViz2: BSD-3 (상업적 사용 가능)
# 사용: ./run-rviz.sh
# (Isaac 컨테이너와 같은 호스트에서 --network=host로 DDS 공유)

xhost +local:root >/dev/null 2>&1

docker run --rm -it --network=host --ipc=host \
  -e DISPLAY \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/zeozeo/git/usd2usdz/fastdds_udp.xml \
  -v "$HOME/.Xauthority:/root/.Xauthority" \
  -v /home/zeozeo:/home/zeozeo \
  osrf/ros:humble-desktop \
  bash -c "source /opt/ros/humble/setup.bash && \
           rviz2 -d /home/zeozeo/git/usd2usdz/sensors.rviz"
