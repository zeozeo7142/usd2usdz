#!/bin/bash
# 호스트에서 Isaac Sim(GUI) 컨테이너를 띄워 teleop_test.py로 로봇을 '직접' 몰아본다.
# (헤드리스 아님 → 창이 뜨고 GS+메쉬를 보며 주행. run-isaac-sim-5.1.sh와 동일한 GUI 설정)
#
# 사용:
#   ./run-teleop.sh --env output/USDZ_TRAIN/subway_car_slab_robot.usda   # 평탄 슬랩 subway_car
#   ./run-teleop.sh --index 1                                            # ETRI1
#   (인자 없으면 --index 1)
#
# 조작: W/S 전후, A/D 회전, Space 정지, R 리셋, ESC 종료.
set -e

xhost +local:root >/dev/null 2>&1

ARGS="$@"
[ -z "$ARGS" ] && ARGS="--index 1"

docker run --rm --gpus '"device=1"' --entrypoint bash \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
  --network=host --ipc=host \
  -e DISPLAY \
  -v "$HOME/.Xauthority:/root/.Xauthority" \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  -v /home/zeozeo:/home/zeozeo \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  -lc "/isaac-sim/python.sh /home/zeozeo/git/usd2usdz/teleop_test.py $ARGS"
