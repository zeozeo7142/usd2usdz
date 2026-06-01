# USD → USDZ 변환 작업 TODO

> **목표**: PortalCam 스캔 USD 파일(GS + Mesh)을 Isaac Sim 4.x에서 로드 가능한 USDZ로 변환
>
> **대상 파일**:
> - `USDZ/USDZ_ERTI1/lcc-usd-result/260521_ERTI 1.usd` (695 MB)
> - `USDZ/USDZ_ERTI2/lcc-usd-result/260521_ERTI 2.usd` (1.2 GB)
> - `USDZ/USDZ_ERTI3/lcc-usd-result/260521_ERTI 3.usd` (1.4 GB)

---

## Step 1: 환경 구성

> **Isaac Sim 불필요** — 변환 작업은 `usd-core` 단독으로 가능. Isaac Sim은 Step 7 최종 검증 시에만 필요.

- [ ] **1-1.** `usd-core` 설치 (Python 3.9+ 권장)
  ```powershell
  pip install usd-core
  ```

- [ ] **1-2.** 설치 확인
  ```powershell
  python -c "from pxr import Usd; print(Usd.GetVersion())"
  # usdcat, usdzip, usdchecker 명령어도 자동으로 설치됨
  usdchecker --help
  ```

---

## Step 2: USD 파일 내부 구조 분석

> 스크립트: `inspect_usd.py`

- [ ] **2-1.** `inspect_usd.py` 실행하여 ERTI 1 USD 구조 파악
  ```powershell
  python inspect_usd.py "USDZ\USDZ_ERTI1\lcc-usd-result\260521_ERTI 1.usd"
  ```

- [ ] **2-2.** 분석 결과에서 다음 항목 확인:
  - Prim 타입 목록 (Mesh, Points, GaussianSplats 등)
  - GS primvar 속성 존재 여부 (`primvars:positions`, `primvars:sh_coeffs`, `primvars:opacities`, `primvars:scales`, `primvars:rotations`)
  - 외부 에셋 경로 참조 (.png, .jpg, .exr, .usd 등)

- [ ] **2-3.** GS 데이터 케이스 판별:
  - **Case A**: omni.gsplat 호환 포맷 → Step 4 건너뛰고 Step 5로
  - **Case B**: 비호환 포맷 → `gs_to_ply.py`로 PLY 변환 필요
  - **Case C**: GS 없음, Point Cloud만 → 그대로 유지

- [ ] **2-4.** ERTI 2, 3도 동일하게 분석

---

## Step 3: 파일명 정리 및 참조 경로 수정

> 스크립트: `fix_paths.py`

- [ ] **3-1.** 공백 포함 파일명 복사본 생성 (원본 보존)
  ```
  260521_ERTI 1.usd  →  260521_ERTI_1.usd
  260521_ERTI 1.obj  →  260521_ERTI_1.obj
  ```

- [ ] **3-2.** `fix_paths.py` 실행하여 USD 내부 경로 공백 제거
  ```powershell
  python fix_paths.py "USDZ\USDZ_ERTI1\lcc-usd-result\260521_ERTI 1.usd"
  ```

- [ ] **3-3.** 출력 파일: `output/ERTI1/260521_ERTI_1.usd`

---

## Step 4: GS 데이터 추출/변환

> 스크립트: `gs_to_ply.py` ← **Case B인 경우에만 실행**

- [ ] **4-1.** Step 2 결과 기준으로 케이스 결정

- [ ] **4-2.** [Case B] `gs_to_ply.py` 실행하여 GS 속성을 표준 PLY로 변환
  ```powershell
  python gs_to_ply.py "output/ERTI1/260521_ERTI_1.usd" "output/ERTI1/260521_ERTI_1_gs.ply"
  ```
  - PLY 컬럼: `x, y, z, nx, ny, nz, f_dc_0~2, f_rest_0~44, opacity, scale_0~2, rot_0~3`

- [ ] **4-3.** [Case B] 변환된 PLY를 Isaac Sim에서 미리 로드 테스트

---

## Step 5: Mesh + GS 통합 USD 구성

> 스크립트: `build_combined_usd.py`

- [ ] **5-1.** `build_combined_usd.py` 실행
  ```powershell
  python build_combined_usd.py --index 1
  # 출력: output/ERTI1/260521_ERTI_1_combined.usda
  ```

- [ ] **5-2.** 생성된 combined USD 확인 항목:
  - `upAxis = "Z"`, `metersPerUnit = 1.0` 설정
  - Mesh (`/World/Mesh`) 참조 정상
  - GS (`/World/GaussianSplats`) 참조 정상
  - 좌표계 정렬 (Y-up OBJ → Z-up USD 변환 포함 여부)

- [ ] **5-3.** `usdview`로 combined USD 미리 확인 (선택)
  ```powershell
  usdview "output/ERTI1/260521_ERTI_1_combined.usda"
  ```

---

## Step 6: USDZ 패키징

> 스크립트: `package_usdz.py`

- [ ] **6-1.** `package_usdz.py` 실행하여 USDZ 생성
  ```powershell
  python package_usdz.py --index 1
  # 출력: output/ERTI1/260521_ERTI_1.usdz
  ```

- [ ] **6-2.** USDZ 아카이브 내 포함 파일 확인 (모든 참조 에셋 포함 여부)

- [ ] **6-3.** ERTI 2, 3도 동일하게 패키징

---

## Step 7: 검증

- [ ] **7-1.** `usdchecker`로 유효성 검사
  ```powershell
  usdchecker output/ERTI1/260521_ERTI_1.usdz
  # 경고/오류 없음 확인
  ```

- [ ] **7-2.** Isaac Sim 4.x에서 USDZ 로드
  - `File > Open` → `output/ERTI1/260521_ERTI_1.usdz` 선택
  - Viewport에서 Mesh 렌더링 확인
  - GS 렌더링 확인 (필요 시 `Extension Manager`에서 `omni.gsplat` 활성화)

- [ ] **7-3.** 스케일/좌표계 확인
  - 실제 크기와 일치하는지 확인 (단위: 미터)
  - 방향 정상 여부

- [ ] **7-4.** ERTI 2, 3 동일하게 검증

---

## 생성 파일 요약

| 파일 | 용도 |
|------|------|
| `inspect_usd.py` | Step 2: USD 내부 구조 분석 |
| `fix_paths.py` | Step 3: 공백 파일명 + 경로 정리 |
| `gs_to_ply.py` | Step 4: GS primvar → 표준 PLY (Case B) |
| `build_combined_usd.py` | Step 5: Mesh + GS 통합 USD 생성 |
| `package_usdz.py` | Step 6: USDZ 아카이브 패키징 |
| `output/ERTI{1,2,3}/260521_ERTI_{1,2,3}.usdz` | 최종 출력물 |

---

## Isaac Sim 환경 참고

- **GS 플러그인**: `Window > Extensions` → `omni.gsplat` 검색 후 활성화
- **지원 GS 포맷**: `.splat`, `.ply` (3DGS 표준), USD Points with primvars
- **좌표계**: Isaac Sim 기본 Z-up, 단위 미터
- **USDZ 로드**: `File > Open` 또는 Content Browser에서 드래그 앤 드롭
