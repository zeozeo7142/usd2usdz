"""
Step 2: USD 파일 내부 구조 분석 스크립트
사용법: python inspect_usd.py "<usd_file_path>"
예시:  python inspect_usd.py "USDZ/USDZ_ERTI1/lcc-usd-result/260521_ERTI 1.usd"
"""

import sys
import os

def inspect_usd(usd_path: str):
    try:
        from pxr import Usd, UsdGeom, Sdf, UsdUtils
    except ImportError:
        print("[ERROR] pxr 모듈을 찾을 수 없습니다.")
        print("  Isaac Sim Python 환경 사용: <isaac_sim_path>\\python.bat inspect_usd.py ...")
        print("  또는 standalone 설치: pip install usd-core")
        sys.exit(1)

    if not os.path.exists(usd_path):
        print(f"[ERROR] 파일 없음: {usd_path}")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"USD 분석: {usd_path}")
    print(f"파일 크기: {os.path.getsize(usd_path) / (1024**2):.1f} MB")
    print(f"{'='*60}\n")

    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print("[ERROR] USD Stage를 열 수 없습니다.")
        sys.exit(1)

    # --- 1. Stage 메타데이터 ---
    root = stage.GetRootLayer()
    print("[Stage 메타데이터]")
    print(f"  defaultPrim : {stage.GetDefaultPrim()}")
    print(f"  upAxis      : {UsdGeom.GetStageUpAxis(stage)}")
    print(f"  metersPerUnit: {UsdGeom.GetStageMetersPerUnit(stage)}")
    print(f"  startTimeCode: {stage.GetStartTimeCode()}")
    print(f"  endTimeCode  : {stage.GetEndTimeCode()}")
    print()

    # --- 2. 레이어 스택 (외부 참조) ---
    print("[레이어 스택 (외부 참조 확인)]")
    for layer in stage.GetLayerStack():
        print(f"  {layer.identifier}")
    print()

    # --- 3. Prim 타입 집계 ---
    prim_type_count: dict[str, int] = {}
    total_prims = 0
    sample_prims: dict[str, list] = {}

    for prim in stage.Traverse():
        t = prim.GetTypeName() or "(no type)"
        prim_type_count[t] = prim_type_count.get(t, 0) + 1
        if t not in sample_prims:
            sample_prims[t] = []
        if len(sample_prims[t]) < 3:
            sample_prims[t].append(str(prim.GetPath()))
        total_prims += 1

    print(f"[Prim 타입 분포] (총 {total_prims}개)")
    for t, cnt in sorted(prim_type_count.items(), key=lambda x: -x[1]):
        examples = ", ".join(sample_prims[t][:2])
        print(f"  {t:30s} : {cnt:5d}  예) {examples}")
    print()

    # --- 4. Gaussian Splatting primvar 탐색 ---
    GS_PRIMVAR_KEYWORDS = [
        "positions", "scales", "rotations", "opacities",
        "sh_coeffs", "f_dc", "f_rest", "gaussian", "splat",
        "covariance", "colors"
    ]

    print("[GS 관련 primvar 탐색]")
    gs_found = False
    for prim in stage.Traverse():
        if prim.GetTypeName() in ("Points", "GaussianSplats", "PointInstancer"):
            print(f"  [!] GS 후보 Prim: {prim.GetPath()} (타입: {prim.GetTypeName()})")
            for prop in prim.GetProperties():
                name = prop.GetName()
                if any(kw in name.lower() for kw in GS_PRIMVAR_KEYWORDS):
                    print(f"      primvar: {name}")
                    gs_found = True

    if not gs_found:
        print("  GS 관련 primvar를 찾지 못했습니다.")
        print("  → Point Cloud(UsdGeom.Points) 또는 Mesh만 포함된 파일일 수 있습니다.")
    print()

    # --- 5. 외부 에셋 경로 (텍스처 등) ---
    print("[외부 에셋 참조 경로]")
    asset_paths = set()

    def collect_assets(layer):
        for path in layer.GetExternalReferences():
            asset_paths.add(path)

    collect_assets(root)
    for layer in stage.GetLayerStack():
        collect_assets(layer)

    if asset_paths:
        for p in sorted(asset_paths):
            exists = "OK" if os.path.exists(p) else "NOT FOUND"
            print(f"  [{exists}] {p}")
    else:
        print("  외부 에셋 참조 없음 (모두 인라인 또는 참조 없음)")
    print()

    # --- 6. 결론 및 권장 케이스 ---
    print("[분석 결론]")
    has_points = "Points" in prim_type_count or "GaussianSplats" in prim_type_count
    has_mesh = "Mesh" in prim_type_count

    if has_points and gs_found:
        print("  ✓ Case A 또는 B: GS 데이터 포함 확인됨")
        print("    → Isaac Sim omni.gsplat 호환 여부를 Step 3에서 확인하세요.")
    elif has_points and not gs_found:
        print("  ✓ Case C: Point Cloud 포함 (GS primvar 없음)")
        print("    → UsdGeom.Points를 그대로 유지하고 Mesh와 함께 패키징하세요.")
    elif has_mesh:
        print("  ✓ Mesh 전용: GS/Point Cloud 없음")
        print("    → OBJ 또는 내장 Mesh와 함께 USDZ로 패키징하세요.")
    else:
        print("  ? 구조를 판별하기 어렵습니다. 위 Prim 목록을 직접 확인하세요.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("사용법: python inspect_usd.py <usd_file_path>")
        print("예시:  python inspect_usd.py \"USDZ\\USDZ_ERTI1\\lcc-usd-result\\260521_ERTI 1.usd\"")
        sys.exit(1)
    inspect_usd(sys.argv[1])
