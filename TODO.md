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

## 필요 파일 요약

```
output/USDZ_ETRI1/
├── 260521_ERTI_1_nurec_mesh.usda   ← Isaac Sim에서 열 파일
├── 260521_ERTI_1_nurec.usdz        ← GS (NuRec 포맷, 348 MB)
└── 260521_ERTI_1_mesh.obj          ← Mesh (1.7 MB)
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
