# PortalCam USD → Isaac Sim 5.1 변환 기술 문서

**작성일**: 2026-06-05  
**환경**: Ubuntu 24.04 / Python 3.12 / NVIDIA RTX A5000 / CUDA 13.0 (드라이버)  
**최종 결과**: Isaac Sim 5.1에서 GS 스플래팅 + Mesh 동시 렌더링 성공

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
