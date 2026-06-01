"""
Step 5: Mesh(OBJ) + GS(USD/PLY) 를 하나의 USD로 통합
output/<dir>/260521_ERTI_<N>_combined.usda 생성

사용법: python build_combined_usd.py --index 1 [--output-dir output] [--gs-mode <auto|a|b|c>]
  --gs-mode auto : inspect_usd.py 결과 기반 자동 판별 (기본값)
  --gs-mode a    : Case A - 원본 USD에 GS 포함, 그대로 사용
  --gs-mode b    : Case B - PLY 파일로 변환된 GS 사용
  --gs-mode c    : Case C - GS 없음, Mesh만 포함
"""

import os
import sys
import argparse

ERTI_MAP = {
    1: {
        "usd":  r"USDZ\USDZ_ERTI1\lcc-usd-result\260521_ERTI 1.usd",
        "obj":  r"USDZ\USDZ_ERTI1\mesh-files\260521_ERTI 1.obj",
        "out_dir": "output/ERTI1",
        "base": "260521_ERTI_1",
    },
    2: {
        "usd":  r"USDZ\USDZ_ERTI2\lcc-usd-result\260521_ERTI 2.usd",
        "obj":  r"USDZ\USDZ_ERTI2\mesh-files\260521_ERTI 2.obj",
        "out_dir": "output/ERTI2",
        "base": "260521_ERTI_2",
    },
    3: {
        "usd":  r"USDZ\USDZ_ERTI3\lcc-usd-result\260521_ERTI 3.usd",
        "obj":  r"USDZ\USDZ_ERTI3\mesh-files\260521_ERTI 3.obj",
        "out_dir": "output/ERTI3",
        "base": "260521_ERTI_3",
    },
}


def detect_gs_mode(usd_path: str) -> str:
    """inspect_usd 로직을 재활용해 Case 자동 판별"""
    try:
        from pxr import Usd, UsdGeom
    except ImportError:
        print("[WARNING] pxr 없음. gs-mode를 --gs-mode 옵션으로 직접 지정하세요.")
        return "c"

    stage = Usd.Stage.Open(usd_path)
    if not stage:
        return "c"

    GS_PRIMVAR_KEYWORDS = [
        "positions", "scales", "rotations", "opacities",
        "sh_coeffs", "f_dc", "f_rest", "gaussian", "splat"
    ]

    for prim in stage.Traverse():
        if prim.GetTypeName() in ("Points", "GaussianSplats"):
            for prop in prim.GetProperties():
                if any(kw in prop.GetName().lower() for kw in GS_PRIMVAR_KEYWORDS):
                    print(f"[build_combined_usd] GS primvar 감지 → Case B (PLY 변환 권장)")
                    return "b"
            print(f"[build_combined_usd] Points 감지되나 GS primvar 없음 → Case C")
            return "c"

    print("[build_combined_usd] GS/Points 없음 → Case C (Mesh 전용)")
    return "c"


def build_combined(index: int, output_dir: str, gs_mode: str):
    try:
        from pxr import Usd, UsdGeom, Sdf
    except ImportError:
        print("[ERROR] pxr 모듈 없음.")
        sys.exit(1)

    info = ERTI_MAP[index]
    base = info["base"]
    out_dir = os.path.join(output_dir, f"ERTI{index}")
    os.makedirs(out_dir, exist_ok=True)

    # 정리된 파일 경로 (fix_paths.py 실행 후)
    clean_usd = os.path.join(out_dir, f"{base}.usd")
    clean_obj = os.path.join(out_dir, f"{base}.obj")
    ply_path  = os.path.join(out_dir, f"{base}_gs.ply")
    combined  = os.path.join(out_dir, f"{base}_combined.usda")

    # 원본 파일이 없으면 fix_paths.py 실행 안내
    if not os.path.exists(clean_usd):
        print(f"[WARNING] 정리된 USD 없음: {clean_usd}")
        print(f"  먼저 fix_paths.py를 실행하세요:")
        print(f"  python fix_paths.py \"{info['usd']}\"")
        # fix_paths 없이도 원본 경로 사용 (공백 처리)
        clean_usd = info["usd"]

    if not os.path.exists(clean_obj):
        clean_obj = info["obj"]

    # gs_mode 자동 판별
    if gs_mode == "auto":
        gs_mode = detect_gs_mode(clean_usd)

    print(f"[build_combined_usd] ERTI{index} / mode={gs_mode}")
    print(f"  USD : {clean_usd}")
    print(f"  OBJ : {clean_obj}")
    print(f"  출력: {combined}")

    # USD Stage 생성
    stage = Usd.Stage.CreateNew(combined)

    # Stage 메타데이터
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.DefinePrim("/World", "Xform"))

    world = stage.GetPrimAtPath("/World")

    # --- Mesh 레이어 ---
    if os.path.exists(clean_obj):
        obj_rel = os.path.relpath(clean_obj, out_dir).replace("\\", "/")
        mesh_prim = stage.DefinePrim("/World/Mesh", "Xform")
        mesh_prim.GetReferences().AddReference(f"./{obj_rel}")
        print(f"  [+] Mesh 참조 추가: {obj_rel}")

        # OBJ는 Y-up이므로 Z-up으로 회전
        xform = UsdGeom.Xformable(mesh_prim)
        xform.AddRotateXOp().Set(-90.0)
    else:
        print(f"  [!] OBJ 파일 없음. Mesh 레이어 건너뜀: {clean_obj}")

    # --- GS 레이어 ---
    if gs_mode == "a":
        # Case A: 원본 USD에 GS 포함 → 직접 참조
        usd_rel = os.path.relpath(clean_usd, out_dir).replace("\\", "/")
        gs_prim = stage.DefinePrim("/World/GaussianSplats", "Xform")
        gs_prim.GetReferences().AddReference(f"./{usd_rel}")
        print(f"  [+] GS 참조 추가 (Case A): {usd_rel}")

    elif gs_mode == "b":
        # Case B: PLY 파일 변환 후 참조
        if os.path.exists(ply_path):
            ply_rel = os.path.relpath(ply_path, out_dir).replace("\\", "/")
            gs_prim = stage.DefinePrim("/World/GaussianSplats", "Xform")
            # omni.gsplat이 PLY를 직접 참조하는 방식
            gs_prim.GetReferences().AddReference(f"./{ply_rel}")
            print(f"  [+] GS PLY 참조 추가 (Case B): {ply_rel}")
        else:
            print(f"  [!] PLY 파일 없음: {ply_path}")
            print(f"      먼저 gs_to_ply.py를 실행하세요:")
            print(f"      python gs_to_ply.py \"{clean_usd}\" \"{ply_path}\"")

    elif gs_mode == "c":
        # Case C: GS 없음 — Mesh만 포함 (또는 PointCloud 유지)
        usd_rel = os.path.relpath(clean_usd, out_dir).replace("\\", "/")
        pc_prim = stage.DefinePrim("/World/PointCloud", "Xform")
        pc_prim.GetReferences().AddReference(f"./{usd_rel}")
        print(f"  [+] PointCloud/Mesh 참조 추가 (Case C): {usd_rel}")

    stage.GetRootLayer().Save()
    print(f"[build_combined_usd] 저장 완료: {combined}")
    return combined


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mesh + GS 통합 USD 구성")
    parser.add_argument("--index", type=int, choices=[1, 2, 3], required=True,
                        help="ERTI 인덱스 (1, 2, 3)")
    parser.add_argument("--output-dir", default="output", help="출력 루트 디렉토리")
    parser.add_argument("--gs-mode", choices=["auto", "a", "b", "c"], default="auto",
                        help="GS 처리 모드 (auto: 자동 판별)")
    args = parser.parse_args()

    build_combined(args.index, args.output_dir, args.gs_mode)
