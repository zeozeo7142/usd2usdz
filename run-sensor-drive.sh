#!/bin/bash
# Isaac Sim 5.1 컨테이너 *안에서* 실행 → sensor_drive.py를 ROS2 환경변수와 함께 구동.
#   Jackal + PhysX LiDAR + RGB(GS) 카메라 → ROS2(rclpy) → RViz2
#
# 사용:
#   ./run-sensor-drive.sh                         # 기본(--index 1)
#   ./run-sensor-drive.sh --index 1 --lidar-hres 0.4   # 옵션은 그대로 전달됨
#
# RViz는 호스트에서 별도로: ./run-rviz.sh
# (Isaac/RViz 컨테이너 모두 docker run에 --ipc=host 필요)

set -e

# --- ROS2 브리지 환경변수 ---
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/zeozeo/git/usd2usdz/fastdds_udp.xml
export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH

SCRIPT=/home/zeozeo/git/usd2usdz/sensor_drive.py

# 인자가 하나도 없으면 기본 --index 1 사용
if [ "$#" -eq 0 ]; then
  set -- --index 1
fi

echo "[run-sensor-drive] ROS_DISTRO=$ROS_DISTRO RMW=$RMW_IMPLEMENTATION"
echo "[run-sensor-drive] sensor_drive.py $*"
exec /isaac-sim/python.sh "$SCRIPT" "$@"
