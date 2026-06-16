# PortalCam USD → Isaac Sim 5.1 변환 기술 문서

**작성일**: 2026-06-05 (최종 갱신 2026-06-08)  
**환경**: Ubuntu 24.04 / Python 3.12 / NVIDIA RTX A5000 ×4 / CUDA 13.0 (드라이버) / Isaac Sim 5.1.0 (Docker)  
**최종 결과**: Isaac Sim 5.1에서 GS+Mesh 렌더링 → 로봇 보행용 평탄 충돌 환경 생성 → Jackal 휠로봇 WASD 텔레옵 주행 → PhysX LiDAR + RGB(GS) 카메라를 부착해 ROS2로 RViz2 시각화까지 성공

---

## 1. 배경

PortalCam(XGRIDS)이 생성한 USD 파일은 Gaussian Splatting 데이터를 `ParticleField3DGaussianSplat`이라는 스키마로 저장한다. 이 스키마는 OpenUSD 26.03(2026년 3월)에서 공식 표준으로 채택되었으며, Isaac Sim 5.1(NuRec 렌더러 포함)부터 공식 지원된다.

**목표**: `260521_ERTI 1.usd`(GS)와 `260521_ERTI 1.obj`(Mesh)를 Isaac Sim 5.1에서 함께 로드하여 GS 스플래팅과 메쉬를 동시에 확인한다.

---

## 2. 입력 파일

```
USDZ/USDZ_ETRI1/
├── 260521_ERTI 1.usd   (694.7 MB)  — GS 데이터 인라인 포함
└── 260521_ERTI 1.obj   (  1.7 MB)  — Mesh
```

> ⚠️ 당초 todo.md에는 경로가 `USDZ_ERTI1/lcc-usd-result/`로 되어 있었으나,
> 실제로는 `USDZ_ETRI1/` 하위에 바로 파일이 있었다. (ETRI ≠ ERTI, 하위 폴더 없음)

---

## 3. USD 파일 구조 분석

`inspect_usd.py`로 분석한 결과:

```
/World (Xform)
└── /World/Gaussians (Xform)
    └── /World/Gaussians/gaussians (ParticleField3DGaussianSplat)
```

| 항목 | 값 |
|------|----|
| upAxis | Y (Isaac Sim 기본값 Z와 다름) |
| metersPerUnit | 1.0 |
| GS splat 수 | 3,086,676 |
| 외부 에셋 참조 | 없음 (모두 인라인) |
| Mesh Prim | 없음 (OBJ 별도 파일) |

**GS 속성** (`ParticleField3DGaussianSplat` 기준):

| 속성명 | 타입 | 비고 |
|--------|------|------|
| `positions` | point3f[3M] | 선형 좌표 |
| `orientations` | quatf[3M] | USD 쿼터니언 (w,x,y,z) |
| `scales` | float3[3M] | 선형 스케일(m), log 변환 필요 |
| `opacities` | float[3M] | 0~1 선형, logit 변환 필요 |
| `radiance:sphericalHarmonicsCoefficients` | float3[49M] | degree-3 SH, 16계수×3채널 |

---

## 4. 작업 전체 흐름 (성공/실패 모두 포함)

### 4-1. 환경 구성

**이유**: USD 파일을 Python에서 읽으려면 `usd-core` 패키지가 필요하다. 시스템에 pip조차 없었다.

**작업**:
- `get-pip.py`로 pip 설치
- `usd-core 26.5`, `numpy 2.4.6` 설치

**결과**: 설치 성공. `usdchecker` CLI는 `usd-core` Python 패키지에 미포함이므로 Step 7 유효성 검사에서 Python API로 대체.

**라이선스 검토**: `usd-core`는 TOST-1.0(Apache 2.0 기반) — 내부 연구 목적 사용 문제없음. 특허 조항 주의.

---

### 4-2. USD 구조 분석 (inspect_usd.py)

**이유**: PortalCam USD의 내부 구조를 파악해야 변환 방식을 결정할 수 있다.

**작업**: `inspect_usd.py` 실행 → docstring 백슬래시 유니코드 이스케이프 버그 수정 후 실행

**결과**: `ParticleField3DGaussianSplat` 확인, GS 데이터 속성 목록 파악.  
→ Case B (표준 PLY 변환 필요) 판별

---

### 4-3. fix_paths.py — 건너뜀

**이유(계획)**: 공백 포함 파일명을 정리하고 외부 참조 경로를 수정하려 했다.

**결과**: USD 파일이 외부 에셋 참조가 전혀 없고(GS 데이터 전부 인라인), 원본 경로로도 잘 동작하므로 불필요 → **건너뜀**.

---

### 4-4. GS → 표준 3DGS PLY 변환 (gs_to_ply.py)

**이유**: 표준 3DGS PLY 포맷으로 변환해야 3DGRUT(NuRec 변환 도구)에서 입력으로 사용할 수 있다.

**작업**: `gs_to_ply.py` 대폭 수정
- `ParticleField3DGaussianSplat` 타입 인식 추가
- 속성명 매핑 (기존 표준 primvar 이름과 다름)
- 스케일: 선형 → `log(scale)` 변환
- 불투명도: 0~1 → `logit(opacity)` 변환
- SH 계수: `(N×16, 3)` → DC + f_rest (채널 우선 재배열)
- 쿼터니언: `Gf.Quatf.real/.imaginary`로 명시 추출

**결과**: `output/USDZ_ETRI1/260521_ERTI_1_gs.ply` 생성 (731 MB, 3,086,676 splats, 62 속성)

---

### 4-5. build_combined_usd.py + package_usdz.py — 결국 미사용

**이유(계획)**: GS 원본 USD + OBJ를 묶어 USDZ로 패키징하면 Isaac Sim에서 열 수 있을 것이라 예상했다.

**작업**:
- `build_combined_usd.py` 수정: 경로, GS 타입 감지, Y-up→Z-up 회전 적용
- `package_usdz.py`로 USDZ 생성 (1,426 MB, 원본 USD + OBJ + PLY 포함)
- `UsdUtils.CreateNewUsdzPackage` 사용

**결과**:
- USDZ 자체는 생성됨 (`usdchecker` 검사: Errors/Warnings 없음)
- **Isaac Sim 4.5에서 로드 시 하위 prim이 표시되지 않거나 HydraEngine 에러 발생**
- 원인 분석: `OBJ 참조가 usd-core에서 실패 → 전체 Stage 로딩에 영향`, 캐시 문제, USDZ 내부 경로(`0/` vs `1/`) 등 복합적 원인
- PLY 제거, 파일명 변경, zipfile 직접 패키징 등 여러 시도를 했으나 모두 불안정
- 근본 원인: **Isaac Sim 4.5.0이 `ParticleField3DGaussianSplat` 스키마를 모름** (내부 USD 버전 0.24.5, 해당 스키마는 USD 26.03에서 표준화)

→ USDZ 패키징 방식 포기. `combined.usda`를 직접 여는 방식으로 전환.

---

### 4-6. combined.usda 직접 열기 — Isaac Sim 4.5에서 부분 성공

**이유**: USDZ가 불안정하므로 USDA 파일을 직접 열어본다.

**작업**: `output/USDZ_ETRI1/260521_ERTI_1_combined.usda` 직접 오픈

**결과**:
- Mesh(`/World/Mesh`): OBJ 로드 성공, 뷰포트에 Mesh 렌더링 ✅
- GaussianSplats(`/World/GaussianSplats/Gaussians/gaussians`): Stage에 로드됨, 하지만 **뷰포트에 렌더링 안 됨** ❌
- 원인: Isaac Sim 4.5.0이 `ParticleField3DGaussianSplat`을 렌더링하는 Hydra Scene Delegate가 없음

---

### 4-7. Isaac Sim 4.5에서 GS 렌더링 시도 — 모두 실패

Isaac Sim 4.5 환경에서 GS를 렌더링하기 위해 여러 방법을 시도했다.

#### 시도 1: omni.gsplat 익스텐션 설치

**이유**: Isaac Sim Extension Manager에서 `omni.gsplat.viewport`를 발견해 설치 시도.

**결과**: "Failed to solve extension dependency" 오류. 실제로는 이 익스텐션이 **Python Extension 예제 템플릿**일 뿐, 실제 GS 렌더러가 아님. Isaac Sim 4.5 컨테이너에는 GS 관련 익스텐션이 전혀 없음(확인: 두 컨테이너 모두 427개 extscache 동일, GS 관련 없음).

#### 시도 2: PLY 파일을 Isaac Sim에 Import

**이유**: PLY를 드래그 앤 드롭으로 Isaac Sim에 로드하면 GS로 렌더링될 것이라 예상.

**결과**: Isaac Sim이 PLY를 변환해 `260521_ERTI_1_gs.usd`(672 B) 생성했으나, 파일 내용은 빈 Xform만 있음 → Isaac Sim 4.5가 3DGS PLY 포맷을 인식하지 못함.

#### 시도 3: UsdGeom.Points 포인트 클라우드 변환 (gs_to_pointcloud_usd.py)

**이유**: GS 렌더링이 불가능하면 포인트 클라우드라도 보여주자.

**결과**: `260521_ERTI_1_pointcloud.usda` 생성(280 MB). Isaac Sim에서 열면 수백만 개의 거대한 구체(sphere)로 렌더링됨. 배경 GS의 scale이 커서 수 미터짜리 구로 보임. 색상이나 공간 구조를 알아볼 수 없어 **실용적 가치 없음**.

#### 시도 4: PointInstancer 근사 렌더링 검토

**이유**: PointInstancer로 billboard quad를 만들면 GS와 유사한 효과를 낼 수 있지 않을까.

**결론**: 분석 결과 포기. GS 스플래팅의 핵심 요소(매 프레임 카메라 기준 back-to-front 정렬, 2D Gaussian 투영, SH 시점 의존 색상)를 PointInstancer로 재현 불가능. 전혀 다른 렌더링 방식.

#### 결론: Isaac Sim 4.5에서 GS 렌더링은 구조적으로 불가능

| 이유 | 내용 |
|------|------|
| 내부 USD 버전 | 0.24.5 (`ParticleField3DGaussianSplat` 미지원) |
| GS Hydra Delegate | 없음 |
| omni.gsplat 공식 지원 | Isaac Sim 5.0 이상 (Kit 107.3+) |

---

### 4-8. Isaac Sim 5.1 전환

**이유**: Isaac Sim 5.1은 NuRec 렌더러를 통해 GS를 공식 지원한다는 정보를 확인.

**작업**: `nvcr.io/nvidia/isaac-sim:5.1.0` pull (별도 NGC 로그인 불필요, 이미 인증됨)

**확인 내용**: Isaac Sim 5.1 내부 USD 버전도 0.24.5로 `ParticleField3DGaussianSplat` 스키마 자체는 모름. 하지만 NuRec `OmniNuRecFieldAsset` 스키마로 변환된 USDZ는 렌더링 가능.

→ 3DGRUT 변환 도구로 PLY → NuRec USDZ 변환 필요.

---

### 4-9. 3DGRUT 설치 및 PLY → NuRec USDZ 변환

**이유**: 3DGRUT(`nv-tlabs/3dgrut`)는 NVIDIA 공식 오픈소스 도구로, 표준 3DGS PLY를 Isaac Sim 5.x에서 렌더링 가능한 NuRec USDZ 포맷으로 변환한다.

**3DGRUT 특징**:
- 출력: `OmniNuRecFieldAsset` 스키마 기반 USDZ (Isaac Sim 5.0+ 전용)
- CUDA 필수 (GPU 연산), PyTorch 의존

**작업**:
- 저장소 클론: `git clone --recursive https://github.com/nv-tlabs/3dgrut.git`
- uv 설치 (pip 대안 패키지 매니저)
- Docker 이미지 빌드 (`CUDA_VERSION=12.8.1`, A5000 sm_86 호환)
  - CUDA 13.0 드라이버지만 툴킷은 미설치 → Docker에서 CUDA 12.8.1 포함 빌드
  - 소요 시간: ~30분 (Kaolin, tiny-cuda-nn 등 CUDA 커널 컴파일)

**변환 실행**:
```bash
docker run --rm --gpus '"device=1"' \
  -v /home/zeozeo/git/usd2usdz:/usd2usdz \   # /workspace 가 아닌 별도 경로 필수
  3dgrut:cuda128 \
  conda run -n 3dgrut \
  python -m threedgrut.export.scripts.ply_to_usd \
    /usd2usdz/output/USDZ_ETRI1/260521_ERTI_1_gs.ply \
    --output_file /usd2usdz/output/USDZ_ETRI1/260521_ERTI_1_nurec.usdz
```

> ⚠️ 마운트 경로가 `/workspace`이면 3DGRUT 소스코드를 덮어써 `threedgrut` 모듈을 찾지 못함.
> 반드시 `/usd2usdz` 등 다른 경로로 마운트해야 한다.

**결과**: `260521_ERTI_1_nurec.usdz` 생성 (348 MB)
```
아카이브 내용:
  default.usda          — 루트 레이어 (upAxis=Z, OmniNuRecFieldAsset 참조)
  260521_ERTI_1_nurec.nurec  — GS 데이터 (347 MB)
  gauss.usda            — Volume prim 정의
```

**Isaac Sim 5.1에서 확인**: GS 스플래팅 렌더링 ✅

---

### 4-10. USDZ에 Mesh 추가 시도 — 실패

**이유**: nurec.usdz에 OBJ Mesh를 함께 포함해 하나의 파일로 만들려 했다.

**작업**: zipfile로 nurec.usdz에 OBJ를 추가, default.usda에 Mesh prim 참조 추가.

**결과**: Isaac Sim 5.1에서 열면 GS도, Mesh도 모두 렌더링 안 됨.

**원인**: OBJ 참조 실패가 전체 Stage 로딩을 방해하는 것으로 추정.

→ USDZ에 OBJ를 넣는 방식 포기.

---

### 4-11. USDA 파일로 GS + Mesh 통합 — 최종 성공

**이유**: USDZ 아카이브 내 경로 문제를 피하기 위해, 디스크의 USDA 파일에서 nurec.usdz(GS)와 OBJ(Mesh)를 별도로 참조하는 방식으로 전환.

**좌표계 정렬 과정**:
1. 처음에는 OBJ에 `rotateX=90` 적용 (Y-up → Z-up 변환 의도)
2. Isaac Sim 5.1에서 열면 GS와 Mesh의 좌표계가 안 맞음
3. 원인: 3DGRUT가 PLY를 변환할 때 좌표 변환을 적용하지 않고 Y-up 데이터 그대로 저장
4. → Mesh에서도 회전 제거하여 두 좌표계 일치

**최종 USDA**:
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

**Isaac Sim 5.1 실행 시 주의사항**:
- `/home/zeozeo` 디렉토리 퍼미션이 `0750`이면 컨테이너에서 접근 불가
- `chmod o+rx /home/zeozeo` 로 해결

**결과**: GS 스플래팅 + Mesh 동시 렌더링 ✅

---

## 5. 최종 파일 구조

```
output/USDZ_ETRI1/
├── 260521_ERTI_1_nurec_mesh.usda   ← Isaac Sim 5.1에서 이 파일을 열면 됨
├── 260521_ERTI_1_nurec.usdz        ← GS (NuRec 포맷, 348 MB)
└── 260521_ERTI_1_mesh.obj          ← Mesh (1.7 MB)

# 중간 산출물 (참고용)
output/USDZ_ETRI1/
├── 260521_ERTI_1_gs.ply            ← 표준 3DGS PLY (3DGRUT 입력용, 731 MB)
└── 260521_ERTI_1_combined.usda     ← Isaac Sim 4.5용 (Mesh만 렌더링, GS 불가)
```

---

## 6. 스크립트 수정 요약

| 스크립트 | 수정 내용 |
|---------|-----------|
| `inspect_usd.py` | docstring 백슬래시 유니코드 이스케이프 버그 수정 |
| `gs_to_ply.py` | `ParticleField3DGaussianSplat` 전용 추출 함수 추가, logit/log/SH/쿼터니언 변환 |
| `build_combined_usd.py` | 실제 경로 수정, GS 타입 감지 로직 추가 (결과적으로 미사용) |
| `package_usdz.py` | 수정 없이 사용 (결과적으로 미사용) |
| `gs_to_pointcloud_usd.py` | 신규 작성 (UsdGeom.Points 변환, 실용 가치 없어 미사용) |

---

## 7. 핵심 기술 발견사항

### ParticleField3DGaussianSplat 스키마

- OpenUSD 26.03(2026.03)에서 AOUSD가 공식 표준으로 채택
- PortalCam(XGRIDS)이 이 표준을 선도적으로 채택하여 USD 출력에 사용
- Isaac Sim 4.5.0 내부 USD 버전(0.24.5)은 이 스키마를 모름 → 렌더링 불가
- Isaac Sim 5.1에서도 직접 렌더링 불가, NuRec 변환 도구(`3DGRUT`)를 통해야 함

### Isaac Sim 버전별 GS 지원

| 버전 | 내부 USD | GS 지원 방식 |
|------|---------|-------------|
| 4.5.0 | 0.24.5 | ❌ 없음 |
| 5.1.0 | 0.24.5 | 🔶 NuRec USDZ 경유 (3DGRUT 변환 필요) |
| 6.0 (Early Dev) | - | ✅ NuRec Fabric Scene Delegate 내장 |

### 3DGRUT 출력 포맷

3DGRUT의 PLY → USDZ 변환 결과물은 `OmniNuRecFieldAsset` 스키마를 사용한다. 이는 `UsdVolVolume` 기반의 Omniverse 전용 스키마로, Isaac Sim 5.0 이상(Kit 107.3+)에서만 NuRec 렌더러로 렌더링 가능하다.

### USDZ vs USDA

- **USDZ**: ZIP 아카이브. 내부 경로 리졸버가 복잡하여 Isaac Sim에서 OBJ 참조 실패 시 전체 Stage 로딩이 방해받는 문제 발생.
- **USDA**: 파일시스템 직접 참조. 경로 문제 없이 안정적으로 로드됨. 최종적으로 USDA 방식 채택.

---

## 8. 환경 구성 참고

### Isaac Sim 5.1 실행 스크립트

```bash
# ~/git/InternNav/run-isaac-sim-5.1.sh
xhost +local:root
docker run --name isaac-sim-5.1 --entrypoint bash -it \
   --gpus '"device=1"' --cpus="12" --memory="60g" --shm-size="16gb" \
   -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" --rm --network=host \
   -e DISPLAY \
   -v $HOME/.Xauthority:/root/.Xauthority \
   -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
   -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
   -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
   -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
   -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
   -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
   -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
   -v ~/docker/isaac-sim/documents:/root/Documents:rw \
   -v /home/zeozeo:/home/zeozeo \   # chmod o+rx /home/zeozeo 선행 필요
   nvcr.io/nvidia/isaac-sim:5.1.0
```

### 3DGRUT Docker 실행

```bash
docker run --rm --gpus '"device=1"' \
  -v /home/zeozeo/git/usd2usdz:/usd2usdz \
  3dgrut:cuda128 \
  conda run -n 3dgrut \
  python -m threedgrut.export.scripts.ply_to_usd \
    /usd2usdz/output/USDZ_ETRI1/260521_ERTI_1_gs.ply \
    --output_file /usd2usdz/output/USDZ_ETRI1/260521_ERTI_1_nurec.usdz
```

---
---

# Part 2. 로봇 보행/주행 + 센서 시뮬레이션 (2026-06-08 추가)

GS+Mesh 렌더링 성공 이후, **복원 메쉬가 울퉁불퉁해 로봇 보행이 불가능**한 문제를 해결하고,
실제로 휠로봇을 주행시키고 LiDAR/카메라 센서 데이터를 RViz2에서 확인하기까지의 작업 기록.

---

## 9. 로봇 보행용 충돌 환경 생성 (make_collision_env.py)

### 9-1. 배경 / 핵심 원리

**이유**: GS+스캔 메쉬를 Isaac Sim에 올렸으나 복원 메쉬 표면 노이즈가 심해, Jackal(휠) / RBQ10(사족)이
바닥에서 튀거나 걸려 주행/보행 불가.

**핵심 원리**: 물리 시뮬레이션에서 로봇이 밟고 부딪히는 것은 *시각 메쉬*가 아니라 *충돌 지오메트리(collision geometry)*다.
→ 울퉁불퉁한 비주얼(GS + 스캔 메쉬)은 **그대로 두고**, 로봇이 상호작용하는 충돌 레이어만 깨끗하게 분리한다.
(사용자 결정: 충돌 범위 = 바닥 + 벽/장애물, 방식 = 평면 collision 분리, 추가 패키지 없이 numpy + usd-core만)

### 9-2. 구현 — 성공

**작업**: `make_collision_env.py` 신규 작성. 입력 OBJ(Y-up)에서 평탄 바닥 collider + 벽/장애물 collider를 분리 생성.
주요 함수:
- `parse_obj()` — `v` / `f v/vt/vn` 방어적 파싱, >3각형 fan triangulate
- `detect_levels()` — 면 법선이 수직축을 향하는(up-facing) 삼각형의 중심 높이를 면적가중 히스토그램 → 바닥 평면 검출
- `build_collision_geometry()` — 바닥면/벽면 분리, 정점 압축
- `compute_spawn()` — 바닥의 열린 지점(장애물 없는 곳) + PCA로 yaw 산출 → 로봇 스폰 지점/방향 결정
- `write_collision_usdc()` — 평평한 슬랩(slab) 바닥 + 벽 메쉬 collider, 마찰 머티리얼, `spawnPoint`/`spawnYaw` 커스텀 속성 저작

**산출물**: `260521_ERTI_1_collision.usdc`(충돌 전용) + `260521_ERTI_1_robot.usda`(Isaac 로드용 최상위 씬).

**CLI**:
```bash
python make_collision_env.py --index 1 \
  --levels {auto,dominant,lowest} \
  --floor-shape {slab,mesh} \
  --friction 0.9 --floor-angle 60
```

**층/씬별 전략 (사용자 설명 반영)**:
| 인덱스 | 데이터 | 전략 |
|--------|--------|------|
| ETRI1 | 건물 실내 한 개 층 | 단일 평면 바닥(slab) |
| ETRI2 | 실내 2층→1층(뚫린 부분/계단) + 1층 실외 | 층별 평면 + 메쉬 계단 |
| ETRI3 | 실외(계단/울타리/나무 다수) | 지배적 지면만 평탄화 |

**결과**: 평탄 슬랩 바닥 + 벽 collider 생성 성공. 로봇이 평면 위에 안착, 벽에 막힘.

### 9-3. 좌표축(up-axis) 문제 — 수정

**이유(증상)**: 생성한 `_robot.usda`를 Isaac Sim에서 열면 "축이 돌아가 있음"(로봇이 옆으로 누움).

**원인 분석**: Part 1에서는 비주얼 정렬만 보고 `rotateX=90`을 가정했으나, **실제 PortalCam 데이터는 Z-up**이었다.
무조건 회전을 넣으면 물리 중력(-Z)과 바닥이 어긋난다.

**작업**: `make_collision_env.py`에 up-axis 자동 감지(데이터 분포로 판정)를 넣어 불필요한 회전 제거.

**결과**: 바닥이 수평(중력과 정렬)으로 로드됨. 로봇이 바로 섬.

### 9-4. 잔여 노이즈 (자갈) — 수용

**증상**: 슬랩 바닥에도 일부 작은 돌출(자갈 같은 것)이 남아 Jackal이 밟고 넘어지는 경우 존재.
**결과**: slab floor로 대부분 해소. 미세 잔여물은 실용상 허용(사용자 합의).

---

## 10. Jackal 휠로봇 WASD 텔레옵 검증 (teleop_test.py)

### 10-1. 구현 — 성공

**이유**: 생성한 평탄 바닥이 실제로 로봇 주행에 적합한지 사람이 직접 조종해 검증.
**작업**: `teleop_test.py` 작성 — 키보드 WASD 텔레옵. Jackal 기본, 로봇별 프리셋(검증된 USD 경로/조인트명).
기본 로봇 `/Isaac/Robots/Clearpath/Jackal/jackal.usd`.
**결과**: GPU 헤드리스 + GUI에서 주행 검증. 1.5m 이상 직진 확인.

### 10-2. 주행 이상 (느림 / 후진이 더 빠름 / 들썩임) — 수정

**증상**: 전진이 느리고 잘 안 움직임, 후진이 전진보다 빠름, 후진키 떼면 넘어질 듯 들썩.
**원인 분석**:
1. 루프 안에서 `sim_app.update()`를 호출해 **이중 스텝**(double-step) 발생 → 물리 불안정.
2. 휠 드라이브 게인 미설정.
**작업**:
- 루프의 `sim_app.update()` 제거 (`world.step(render=True)`만 사용).
- `UsdPhysics.DriveAPI`(angular)로 휠 드라이브 설정: stiffness=0, damping=1e3, maxForce=2e3.
- 4륜 차동 구동: `vels = (lin + side*(ang*WHEEL_BASE/2))/WHEEL_R`, `WHEEL_R=0.098, WHEEL_BASE=0.37`.
**결과**: 정상 주행. 4륜(Jackal)도 문제없이 구동.

---

## 11. LiDAR / 카메라 센서 + ROS2 → RViz2 (sensor_drive.py)

가장 길고 디버깅이 많았던 작업. 목표: Jackal에 **Ouster급 LiDAR + RGB 카메라**를 달고 RViz2에서 센싱 확인.
단, **카메라에는 회색 복원 메쉬가 아니라 포토리얼 GS가 찍혀야** 함.

### 11-1. 센서 종류 선택 — RTX LiDAR → PhysX LiDAR로 전환

**이유**: 처음엔 RTX LiDAR(Ouster OS 프리셋)를 시도.
**핵심 발견(중요)**:
- **RTX LiDAR/카메라는 렌더링 visibility를 공유**한다. RTX LiDAR는 *보이는 메쉬*를 센싱.
- GS(NuRec)는 **헤드리스 오프스크린 render product에는 렌더되지 않고**, 인터랙티브 GUI 뷰포트에서만 렌더됨.
- 카메라에 GS만 찍으려면 VisualMesh를 숨겨야 하는데, 그러면 RTX LiDAR도 그 메쉬를 못 봐서 포인트가 사라짐 → visibility/투명도로 둘을 분리 불가.
**작업/결과**: → **PhysX LiDAR(`RotatingLidarPhysX`)로 전환**. PhysX는 *물리 충돌 지오메트리*를 레이캐스트하므로
렌더링 visibility와 무관 → VisualMesh를 숨겨도(카메라=GS) LiDAR는 collider를 그대로 센싱. (사용자 승인: "PhysX 라이다로 전환")

> 결론: 지금 부착된 것은 **실제 RTX Ouster가 아니라, PhysX LiDAR를 Ouster급(360°×수직30°, ~27k pts)으로 설정**한 것.
> 실제 Ouster OS1-128 사양은 `--lidar-vfov 45 --lidar-vres 0.35 --lidar-hres 0.35 --lidar-range 120`로 근사 가능(레이 수 많아 무거움).

### 11-2. 카메라에 GS 보이기 — VisualMesh invisible

**이유**: 카메라 영상에 회색 복원 메쉬가 GS 위에 겹쳐 보이는 문제.
**실패한 시도(기록)**:
- VisualMesh에 `displayOpacity=0` → 머티리얼이 없어 무효.
- 투명 `UsdPreviewSurface` 머티리얼 부여 → 헤드리스(path-traced)에선 투명, **RTX Real-Time GUI에선 불투명 렌더**(효과 없음). (이 부분을 한때 "투명 적용됨"으로 잘못 보고함 — 사용자 지적으로 정정.)
**작업(성공)**: `UsdGeom.Imageable(VisualMesh).MakeInvisible()`로 VisualMesh를 **완전히 숨김**.
**결과**: GUI 뷰포트에서 GS만 켜면 카메라(render product)에 GS가 찍힘(사용자가 캡처로 확인). PhysX LiDAR는 영향 없음.
- 참고: VisualMesh 뷰 OFF 시 RViz에 포인트클라우드가 남아 보였던 것은 **이전 프레임의 잔상(decay 잔여)**일 뿐, 실제 LiDAR는 0이었음(사용자 확인).

### 11-3. PhysX LiDAR가 0 포인트 — 로딩 방식 수정

**증상**: `open_stage(_robot.usda)`로 환경을 열면 PhysX LiDAR가 0 포인트.
**원인 분석**: `open_stage` 경로의 환경 내 physicsScene이 LiDAR 레이 쿼리를 방해.
**작업**: `World(stage_units_in_meters=1.0)` 생성 후 `add_reference_to_stage(env, "/World/Scene")`로 환경을 **레퍼런스로 로드**.
**결과**: collider 정상 레이캐스트, 1320 포인트 확인 → 이후 Ouster급 설정으로 ~27900까지.

### 11-4. 센서가 공간에 고정됨 — 마운트 위치 수정

**증상**: 로봇이 움직여도 센서가 스폰 위치에 고정.
**원인**: 센서를 아티큘레이션 루트 prim(`/World/Jackal`, 스폰 위치에서 안 움직임) 밑에 마운트.
**작업**: **움직이는 링크인 `/World/Jackal/base_link` 밑**에 LiDAR/카메라 마운트.
**결과**: 센서가 로봇과 함께 이동.

### 11-5. ROS2 토픽은 뜨는데 데이터가 안 건너옴 — FastDDS 전송 수정 (핵심)

**증상**: `ros2 topic list`엔 토픽이 보이는데 RViz/listener에 **데이터가 안 옴**. `/clock`, `/tf` 비어 있음.
**원인 분석**: Isaac 번들 FastDDS와 osrf humble FastDDS의 **공유메모리(SHM) 전송 버전 불일치** → 컨테이너 간 데이터 전달 실패.
**작업**:
- `fastdds_udp.xml` 작성 — **UDPv4 전용** 프로파일(`<useBuiltinTransports>false</useBuiltinTransports>`).
- 양쪽 컨테이너 모두 `--ipc=host` + `-e FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_udp.xml`.
- Isaac 실행 스크립트(`run-isaac-sim-5.1.sh`)에 `--ipc=host` 추가.
**결과**: talker/listener로 컨테이너 간 통신 확인. 토픽 데이터 정상 수신.

### 11-6. OmniGraph ROS2 퍼블리셔가 standalone에서 발행 안 됨 — rclpy 직접 발행

**증상**: OmniGraph ROS2 퍼블리셔 노드가 standalone 스크립트에서 메시지를 안 내보냄.
**작업**: **rclpy 직접 퍼블리시**로 재작성. `/clock`(Clock), `/tf`(TFMessage), `/point_cloud`(PointCloud2), `/rgb`(Image)를 매 스텝 직접 publish.
- `make_pc2()` — x/y/z FLOAT32, point_step 12, 비유한값 필터
- `make_img()` — rgb8
- `tf_msg()` — 쿼터니언 [w,x,y,z] → geometry_msgs [x,y,z,w]
**결과**: RViz에 LiDAR/카메라/TF 정상 표시.

### 11-7. 포인트클라우드가 1/4씩 4번에 걸쳐 보임 — RViz Decay Time

**증상**: RViz에서 포인트클라우드가 한 번에 전체가 아니라 1/4씩 나눠 깜빡이며 채워짐.
**원인 분석**: PhysX LiDAR는 **회전형**이라 매 프레임이 회전의 일부(부분 스윕). 버퍼가 프레임마다 부분/전체로 바뀜
(폭 1320 ↔ 27900 교대 관찰). "전체 스윕일 때만 publish"하는 코드측 필터(`npc >= 0.5*max_pts`)는 깔끔히 안 걸러짐.
**작업(최종 해법)**: 코드는 **매 프레임 publish**로 단순화하고, **RViz `sensors.rviz`의 PointCloud2 `Decay Time: 0.5`** 설정.
→ 회전 스윕이 0.5초간 누적돼 항상 전체 클라우드로 보임(회전 LiDAR 시각화의 정석).
**결과**: 깜빡임 해소. RViz 재시작(`./run-rviz.sh`)으로 새 설정 적용.

### 11-8. GPU 검증 운영

**작업**: 빈 GPU(0)에 `--rm` 헤드리스 Isaac 컨테이너로 RTX/PhysX 센서 실제 검증.
**결과**: 검증 후 **임시 probe 파일 정리 + GPU 반납**(16 MiB) — 사용자 표준 지침(검증 후 GPU 비우기) 준수.

---

## 12. Part 2 핵심 기술 발견사항

- **RTX vs PhysX 센서**: RTX LiDAR/카메라는 렌더 visibility(보이는 메쉬)를 센싱하고 서로 공유한다. PhysX LiDAR는 물리 collider를 레이캐스트해 visibility와 무관. → "카메라=GS, LiDAR=collider"처럼 둘을 분리하려면 PhysX LiDAR가 답.
- **GS(NuRec) 렌더 범위**: 인터랙티브 GUI 뷰포트에서만 렌더. 헤드리스 오프스크린 render product에는 안 나옴(검은 화면). 카메라로 GS를 얻으려면 GUI 세션 + 오프스크린 render product 조합.
- **VisualMesh 투명화 불가**: 투명 머티리얼은 RTX Real-Time에서 불투명 렌더됨. 숨기려면 `MakeInvisible()`(visibility=invisible)이 확실.
- **PhysX LiDAR 로딩**: `open_stage(env)`의 내장 physicsScene이 레이 쿼리를 막음. `World()` + `add_reference_to_stage()`로 로드해야 collider를 레이캐스트.
- **센서 마운트**: 아티큘레이션 루트 prim은 스폰 위치에 고정. 반드시 `base_link`(움직이는 링크) 밑에 마운트.
- **이중 스텝 금지**: standalone 루프에서 `sim_app.update()`와 `world.step()`을 같이 부르면 물리 불안정. `world.step(render=True)`만 사용.
- **컨테이너 간 ROS2(DDS)**: Isaac 번들 FastDDS ↔ osrf humble 간 SHM 버전 불일치로 데이터 전달 실패. **UDP 전용 프로파일 + 양쪽 `--ipc=host`**로 해결.
- **OmniGraph ROS2 퍼블리셔**는 standalone python 스크립트에서 발행 안 될 수 있음 → **rclpy 직접 발행**이 안정적.
- **회전 LiDAR 시각화**: 매 프레임은 부분 스윕. RViz `Decay Time`을 한 회전 주기(~0.5s)로 두면 전체 스윕이 누적돼 보인다.
- **up-axis**: PortalCam 데이터는 **Z-up**. 무조건 `rotateX=90`을 넣으면 물리에서 바닥이 어긋남 → 데이터 분포로 자동 감지.

---

## 13. Part 2 파일 구조 / 스크립트

| 파일 | 내용 |
|------|------|
| `make_collision_env.py` | OBJ → 평탄 충돌 환경 생성. `_collision.usdc`(충돌) + `_robot.usda`(로드용). slab 바닥/벽 collider, 마찰, spawnPoint/spawnYaw |
| `teleop_test.py` | WASD 키보드 텔레옵. Jackal 기본 + 로봇 프리셋 |
| `sensor_drive.py` | 센서+ROS2 통합. PhysX LiDAR(Ouster급) + RGB(GS) 카메라, World()+reference 로딩, VisualMesh invisible, rclpy 직접 발행, UDP 프로파일 |
| `fastdds_udp.xml` | UDP 전용 FastDDS 프로파일 (컨테이너 간 DDS 데이터 전달 필수) |
| `run-rviz.sh` | osrf/ros:humble-desktop 컨테이너로 RViz2 실행 (`--network=host --ipc=host`, UDP 프로파일) |
| `sensors.rviz` | RViz 설정. PointCloud2(/point_cloud, Best Effort, **Decay Time 0.5**), Image(/rgb), TF, Grid |

### sensor_drive.py 주요 상수
```python
ROBOT_PRIM="/World/Jackal"; BASE_LINK=ROBOT_PRIM+"/base_link"
SCENE_PRIM="/World/Scene"; VISUALMESH=SCENE_PRIM+"/Environment/VisualMesh"
LIDAR_OFFSET=(0,0,0.3); CAM_OFFSET=(0.2,0,0.25)
WHEEL_R=0.098; WHEEL_BASE=0.37
ROBOT_PATH="/Isaac/Robots/Clearpath/Jackal/jackal.usd"
# CLI: --index --lidar-vfov 30 --lidar-hres 0.4 --lidar-vres 1.0 --lidar-range 100 --headless --max-steps --show-visualmesh
```

### 실행 순서
```bash
# 1) Isaac (GUI) — 표준 env로 센서 스크립트 실행
/isaac-sim/python.sh /home/zeozeo/git/usd2usdz/sensor_drive.py --index 1
# 2) 별도 터미널 — RViz2 (UDP 프로파일 + ipc=host)
./run-rviz.sh
```
RViz Fixed Frame은 `world`(필요시 `lidar`). PointCloud2/Image는 Best Effort QoS.
