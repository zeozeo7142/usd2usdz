# PortalCam USD → Isaac Sim 5.1 로드 파이프라인

> **목표**: PortalCam 스캔 결과물(`260521_ERTI 1.usd`, `260521_ERTI 1.obj`)을
> Isaac Sim 5.1에서 GS + Mesh 모두 렌더링되도록 로드
>
> **최종 결과물**: `output/USDZ_ETRI1/260521_ERTI_1_nurec_mesh.usda` (Isaac Sim 5.1에서 열기)
>
> **대상 파일** (실제 확인된 경로):
> - `USDZ/USDZ_ETRI1/260521_ERTI 1.usd` (694.7 MB) — GS 데이터 인라인 포함
> - `USDZ/USDZ_ETRI1/260521_ERTI 1.obj` (1.7 MB) — Mesh

---

## Step 1: 환경 구성

- [x] **1-1.** pip 설치 (시스템에 pip 없으므로 get-pip.py 사용)
  ```bash
  curl -sSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
  python3 /tmp/get-pip.py --user --break-system-packages
  ```

- [x] **1-2.** usd-core, numpy 설치
  ```bash
  python3 -m pip install usd-core numpy --break-system-packages
  # usd-core 26.5, numpy 2.4.6
  ```

- [x] **1-3.** Isaac Sim 5.1 Docker 이미지 pull
  ```bash
  docker pull nvcr.io/nvidia/isaac-sim:5.1.0
  ```

- [x] **1-4.** Isaac Sim 5.1 실행 스크립트 작성
  ```bash
  # ~/git/InternNav/run-isaac-sim-5.1.sh
  # /home/zeozeo 마운트 포함 (chmod o+rx /home/zeozeo 필요)
  chmod o+rx /home/zeozeo
  ```

---

## Step 2: USD 파일 구조 분석

- [x] **2-1.** `inspect_usd.py` 실행
  ```bash
  python3 inspect_usd.py "USDZ/USDZ_ETRI1/260521_ERTI 1.usd"
  ```

- [x] **2-2.** 분석 결과

  | 항목 | 값 |
  |------|----|
  | Prim 타입 | `ParticleField3DGaussianSplat` (OpenUSD 26.03 표준 스키마) |
  | GS splat 수 | 3,086,676 |
  | upAxis | Y (Isaac Sim은 Z-up) |
  | metersPerUnit | 1.0 |
  | 외부 에셋 참조 | 없음 (전부 인라인) |
  | Mesh Prim | 없음 (Mesh는 별도 OBJ 파일) |

- [x] **2-3.** 케이스 판별: **Case B** (비표준 포맷 → PLY 변환 필요)
  - `ParticleField3DGaussianSplat`은 Isaac Sim 5.1 이상에서 NuRec 렌더러로 지원
  - Isaac Sim 4.x는 미지원

---

## Step 3: GS 데이터 → 표준 3DGS PLY 변환

> `gs_to_ply.py` 수정: `ParticleField3DGaussianSplat` 전용 추출 로직 추가

- [x] **3-1.** `gs_to_ply.py` 실행
  ```bash
  python3 gs_to_ply.py \
      "USDZ/USDZ_ETRI1/260521_ERTI 1.usd" \
      "output/USDZ_ETRI1/260521_ERTI_1_gs.ply"
  # → 3,086,676 splats / 731 MB / 62 속성 (binary_little_endian)
  ```

  | 변환 항목 | 원본 | PLY 표준 |
  |-----------|------|---------|
  | 스케일 | 선형(m) | `log(scale)` |
  | 불투명도 | 0~1 선형 | `logit(opacity)` |
  | SH 계수 | `(N×16, 3)` float3 배열 | DC + f_rest (채널 우선 재배열) |
  | 쿼터니언 | USD quatf (w,x,y,z) | rot_0..3 |

---

## Step 4: 3DGRUT 설치 및 PLY → NuRec USDZ 변환

- [x] **4-1.** 3DGRUT 저장소 클론
  ```bash
  cd ~/git
  git clone --recursive https://github.com/nv-tlabs/3dgrut.git
  ```

- [x] **4-2.** uv 설치
  ```bash
  curl -LsSf https://astral.sh/uv/install.sh | sh
  ```

- [x] **4-3.** 3DGRUT Docker 이미지 빌드 (CUDA 12.8.1, A5000 호환)
  ```bash
  cd ~/git/3dgrut
  docker build --build-arg CUDA_VERSION=12.8.1 -t 3dgrut:cuda128 .
  # 소요 시간: ~30분 (CUDA 커널 컴파일 포함)
  ```

- [x] **4-4.** PLY → NuRec USDZ 변환
  ```bash
  docker run --rm --gpus '"device=1"' \
    -v /home/zeozeo/git/usd2usdz:/usd2usdz \
    3dgrut:cuda128 \
    conda run -n 3dgrut \
    python -m threedgrut.export.scripts.ply_to_usd \
      /usd2usdz/output/USDZ_ETRI1/260521_ERTI_1_gs.ply \
      --output_file /usd2usdz/output/USDZ_ETRI1/260521_ERTI_1_nurec.usdz
  # → 348 MB / OmniNuRecFieldAsset 스키마 / Z-up
  ```

  > **주의**: 마운트를 `/usd2usdz`로 해야 함.
  > `/workspace`로 마운트하면 3DGRUT 소스 코드를 덮어써 모듈 로드 실패.

---

## Step 5: Mesh + GS 통합 USDA 구성

- [x] **5-1.** OBJ를 output 폴더로 복사 (공백 없는 이름으로)
  ```bash
  cp "USDZ/USDZ_ETRI1/260521_ERTI 1.obj" \
     "output/USDZ_ETRI1/260521_ERTI_1_mesh.obj"
  ```

- [x] **5-2.** 통합 USDA 작성
  ```bash
  # output/USDZ_ETRI1/260521_ERTI_1_nurec_mesh.usda
  ```
  ```usda
  #usda 1.0
  (
      defaultPrim = "World"
      metersPerUnit = 1
      upAxis = "Z"
  )
  def Xform "World"
  {
      def Xform "GaussianSplats" (
          prepend references = @./260521_ERTI_1_nurec.usdz@
      )
      {
      }
      def Xform "Mesh" (
          prepend references = @./260521_ERTI_1_mesh.obj@
      )
      {
      }
  }
  ```

  > GS(nurec.usdz)는 Y-up 좌표 그대로 저장되어 있음.
  > Mesh(OBJ)도 동일하게 회전 없이 참조하면 두 좌표계가 일치.

---

## Step 6: Isaac Sim 5.1에서 로드

- [x] **6-1.** Isaac Sim 5.1 실행
  ```bash
  cd ~/git/InternNav && ./run-isaac-sim-5.1.sh
  # 컨테이너 bash에서:
  /isaac-sim/isaac-sim.sh
  ```

- [x] **6-2.** 파일 열기
  ```
  File > Open
  → /home/zeozeo/git/usd2usdz/output/USDZ_ETRI1/260521_ERTI_1_nurec_mesh.usda
  ```

- [x] **6-3.** 확인 항목

  | 항목 | 결과 |
  |------|------|
  | GaussianSplats 렌더링 | ✅ NuRec GS 스플래팅 정상 |
  | Mesh 렌더링 | ✅ OBJ Mesh 정상 |
  | 좌표계 일치 | ✅ GS와 Mesh 위치 일치 |

---

## Step 7: 로봇 보행용 평탄 충돌 환경 생성

> 복원 메쉬가 울퉁불퉁해 로봇 주행 불가 → 비주얼(GS+메쉬)은 그대로 두고
> 로봇이 밟는 *충돌 지오메트리*만 평탄 슬랩 + 벽 collider로 분리한다.
> 추가 패키지 없이 numpy + usd-core(pxr)만 사용.

- [x] **7-1.** `make_collision_env.py` 실행 (ETRI1)
  ```bash
  python3 make_collision_env.py --index 1 --floor-shape slab --friction 0.9
  # → output/USDZ_ETRI1/260521_ERTI_1_collision.usdc  (충돌 전용: 평탄 바닥 + 벽 collider)
  # → output/USDZ_ETRI1/260521_ERTI_1_robot.usda      (Isaac 로드용 최상위 씬)
  ```
  > ⚠️ PortalCam 데이터는 **Z-up**이다. 스크립트가 up-axis를 자동 감지하므로
  > 임의로 `rotateX=90`을 넣지 말 것(넣으면 물리 중력과 바닥이 어긋나 로봇이 옆으로 누움).

- [x] **7-2.** Isaac Sim 5.1에서 `260521_ERTI_1_robot.usda` 열기 → 검증
  | 확인 항목 | 성공 기준 |
  |-----------|-----------|
  | 바닥 수평 | 로봇이 옆으로 안 눕고 바로 섬 |
  | Physics debug view | `FloorCollider` 슬랩이 바닥 높이 수평 박스, 벽 collider가 벽을 감쌈 |
  | 비주얼 | GS + 비주얼 메쉬는 그대로(collider만 평탄) |

- [ ] **7-3.** ETRI2/3 적용 (파일 수신 후)
  ```bash
  # ETRI2: 층별 평면 + 메쉬 계단,  ETRI3: 지배적 지면만 평탄화
  python3 make_collision_env.py --index 2 --levels auto --floor-shape slab
  python3 make_collision_env.py --index 3 --levels dominant --floor-shape slab
  ```

---

## Step 8: 휠로봇(Jackal) WASD 텔레옵 주행 검증

- [x] **8-1.** Isaac Sim 5.1(GUI) 실행
  ```bash
  cd ~/git/InternNav && ./run-isaac-sim-5.1.sh
  ```

- [x] **8-2.** 컨테이너 안에서 텔레옵 스크립트 실행
  ```bash
  /isaac-sim/python.sh /home/zeozeo/git/usd2usdz/teleop_test.py --index 1
  # W/S = 전/후진, A/D = 좌/우 회전
  # → Jackal이 평탄 바닥 위를 1.5m 이상 정상 주행하면 성공
  ```
  > ⚠️ standalone 루프에서 `sim_app.update()`를 호출하지 말 것(이중 스텝 → 물리 불안정,
  > 후진이 전진보다 빠르고 들썩임). `world.step(render=True)`만 사용한다.

---

## Step 9: LiDAR / 카메라 센서 → ROS2 → RViz2 시각화

> Jackal에 **PhysX LiDAR(Ouster급)** + **RGB 카메라(GS 영상)** 부착.
> PhysX LiDAR는 충돌 collider를 레이캐스트하므로, 카메라용으로 VisualMesh를 숨겨도(=GS만 촬영) LiDAR는 정상 동작.

- [x] **9-1.** ROS2 + RViz 컨테이너 이미지 준비 (osrf 공식, BSD/Apache)
  ```bash
  docker pull osrf/ros:humble-desktop
  ```

- [x] **9-2.** UDP 전용 FastDDS 프로파일 확인 (컨테이너 간 DDS 데이터 전달 필수)
  ```bash
  cat /home/zeozeo/git/usd2usdz/fastdds_udp.xml   # UDPv4 전용, useBuiltinTransports=false
  ```
  > ⚠️ Isaac 번들 FastDDS와 osrf humble 간 공유메모리(SHM) 버전 불일치로 데이터가 안 건너온다.
  > **양쪽 컨테이너 모두** `--ipc=host` + `FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_udp.xml` 필요.
  > (`run-isaac-sim-5.1.sh`, `run-rviz.sh` 둘 다 적용됨)

- [x] **9-3.** Isaac Sim(GUI) 컨테이너에서 센서 스크립트 실행 — **런처 사용**
  ```bash
  # 런처가 ROS2 env 4종을 자동 설정하고 인자를 그대로 전달
  /home/zeozeo/git/usd2usdz/run-sensor-drive.sh --index 1
  # 발행 토픽: /clock /tf /point_cloud(PointCloud2) /rgb(Image)
  # 조작: W/S 전후, A/D 회전, ESC 종료
  ```
  > ⚠️ 이 런처는 `/isaac-sim/python.sh`를 쓰므로 **Isaac Sim 5.1 컨테이너 셸 안에서** 실행한다.
  > ⚠️ GS는 GUI 뷰포트에서만 렌더된다(헤드리스 render product에는 검게 나옴).
  > 카메라에 GS를 담으려면 GUI 세션에서 실행하고, 뷰포트에서 VisualMesh/Colliders는 끄고 GS만 켜 둔다.

- [x] **9-4.** 별도 터미널에서 RViz2 실행
  ```bash
  /home/zeozeo/git/usd2usdz/run-rviz.sh
  # → sensors.rviz 로드: PointCloud2(/point_cloud), Image(/rgb), TF, Grid
  ```
  > ⚠️ 권한 오류 시: `chmod +x /home/zeozeo/git/usd2usdz/run-rviz.sh`

- [x] **9-5.** 확인 항목
  | 항목 | 성공 기준 |
  |------|-----------|
  | /point_cloud | RViz에 LiDAR 포인트클라우드 표시, 로봇과 함께 이동 |
  | 포인트클라우드 전체 표시 | `sensors.rviz`의 PointCloud2 **Decay Time 0.5** → 1/4 깜빡임 없이 전체 누적 표시 |
  | /rgb | 카메라 영상이 **포토리얼 GS**(회색 메쉬 아님) |
  | QoS | PointCloud2/Image 모두 **Best Effort** |

- [x] **9-6.** 주행이 느릴 때 — RTF(실시간 배율) 확인 후 부하 조절
  ```bash
  # 실행한 터미널 콘솔에 60스텝마다 출력됨:
  #   [sensor] step=120 ... lidar_pts=13500 RTF=0.38 (sim 22 fps)
  # RTF<1이면 그만큼 느리게 보임 (구동 문제 아님 = 렌더/센서 부하)
  ```
  | RTF | 의미 | 조치 |
  |-----|------|------|
  | ~1.0 | 실시간 정상 | OK |
  | 0.2~0.5 | 느린 슬로모션 | 기본값(`--lidar-hres 0.8`, 13,500레이)이 균형점 |
  | <0.1 | 매우 느림 | LiDAR 더 가볍게: `--lidar-hres 1.2 --lidar-vres 1.5` |

  > LiDAR 레이캐스트가 RTF 최대 병목(레이당 ~7.7µs). GS 렌더가 RTF ~0.2 바닥을 깖.
  > 기본값: `--lin-speed 2.0 --ang-speed 2.0 --lidar-hres 0.8 --lidar-vres 1.0 --cam-skip 4`
  > 더 조밀(느림): `--lidar-hres 0.4` (27,900 pts) / Ouster OS1-128 근사: `--lidar-vfov 45 --lidar-vres 0.35 --lidar-hres 0.35 --lidar-range 120`

---

## 필요 파일 요약

```
output/USDZ_ETRI1/
├── 260521_ERTI_1_nurec_mesh.usda   ← GS+Mesh 렌더링용 (Step 6)
├── 260521_ERTI_1_nurec.usdz        ← GS (NuRec 포맷, 348 MB)
├── 260521_ERTI_1_mesh.obj          ← Mesh (1.7 MB)
├── 260521_ERTI_1_collision.usdc    ← 평탄 충돌 지오메트리 (Step 7)
└── 260521_ERTI_1_robot.usda        ← 로봇 주행/센서용 로드 파일 (Step 7~9)

# 프로젝트 루트 (로봇/센서 도구)
make_collision_env.py   teleop_test.py   sensor_drive.py
run-sensor-drive.sh   fastdds_udp.xml   run-rviz.sh   sensors.rviz
```

---

## ETRI 2, 3 처리 절차

파일 수신 후 Step 3~5를 반복:

```bash
# Step 3: PLY 변환
python3 gs_to_ply.py \
    "USDZ/USDZ_ETRI2/260521_ERTI 2.usd" \
    "output/USDZ_ETRI2/260521_ERTI_2_gs.ply"

# Step 4: NuRec USDZ 변환
docker run --rm --gpus '"device=1"' \
  -v /home/zeozeo/git/usd2usdz:/usd2usdz \
  3dgrut:cuda128 \
  conda run -n 3dgrut \
  python -m threedgrut.export.scripts.ply_to_usd \
    /usd2usdz/output/USDZ_ETRI2/260521_ERTI_2_gs.ply \
    --output_file /usd2usdz/output/USDZ_ETRI2/260521_ERTI_2_nurec.usdz

# Step 5: Mesh 복사 + USDA 생성
cp "USDZ/USDZ_ETRI2/260521_ERTI 2.obj" "output/USDZ_ETRI2/260521_ERTI_2_mesh.obj"
# → nurec_mesh.usda 작성 (경로만 ERTI2로 수정)
```
