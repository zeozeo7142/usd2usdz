"""
Step 5: GS(USD) + Mesh(OBJ) 를 Isaac Sim 로드 가능한 통합 USDA로 구성
output/<dir>/260521_ERTI_<N>_combined.usda 생성

사용법: python build_combined_usd.py --index 1 [--output-dir output] [--gs-mode <auto|a|b|c>]
  --gs-mode auto : 원본 USD 구조 기반 자동 판별 (기본값)
  --gs-mode a    : Case A - 원본 USD에 GS 포함, 그대로 참조
  --gs-mode b    : Case B - 원본 USD 참조 + Y-up→Z-up 회전 (ParticleField3DGaussianSplat)
  --gs-mode c    : Case C - GS 없음, Mesh만 포함

Y-up → Z-up 변환: GS/Mesh Xform에 xformOp:rotateX = 90 적용
PLY 경로 (omni.gsplat 직접 로드용): output/USDZ_ETRI<N>/260521_ERTI_<N>_gs.ply
"""

import os
import sys
import argparse

ERTI_MAP = {
    1: {
        "usd":     "USDZ/USDZ_ETRI1/260521_ERTI 1.usd",
        "obj":     "USDZ/USDZ_ETRI1/260521_ERTI 1.obj",
        "out_dir": "output/USDZ_ETRI1",
        "base":    "260521_ERTI_1",
    },
    2: {
        "usd":     "USDZ/USDZ_ETRI2/260521_ERTI 2.usd",
        "obj":     "USDZ/USDZ_ETRI2/260521_ERTI 2.obj",
        "out_dir": "output/USDZ_ETRI2",
        "base":    "260521_ERTI_2",
    },
    3: {
        "usd":     "USDZ/USDZ_ETRI3/260521_ERTI 3.usd",
        "obj":     "USDZ/USDZ_ETRI3/260521_ERTI 3.obj",
        "out_dir": "output/USDZ_ETRI3",
        "base":    "260521_ERTI_3",
    },
}

PORTALCAM_TYPE = "ParticleField3DGaussianSplat"


def detect_gs_mode(usd_path: str) -> str:
    """원본 USD prim 타입 기반 Case 자동 판별."""
    try:
        from pxr import Usd
    except ImportError:
        print("[WARNING] pxr 없음. --gs-mode 옵션으로 직접 지정하세요.")
        return "c"

    if not os.path.exists(usd_path):
        print(f"[WARNING] USD 파일 없음: {usd_path}")
        return "c"

    stage = Usd.Stage.Open(usd_path)
    if not stage:
        return "c"

    GS_PRIMVAR_KEYWORDS = [
        "positions", "scales", "rotations", "opacities",
        "sh_coeffs", "f_dc", "f_rest", "gaussian", "splat",
    ]

    for prim in stage.Traverse():
        t = prim.GetTypeName()

        # PortalCam 독자 타입 → Case B
        if t == PORTALCAM_TYPE:
            print(f"[build_combined_usd] {PORTALCAM_TYPE} 감지 → Case B")
            return "b"

        # 표준 GS primvar 확인
        if t in ("Points", "GaussianSplats"):
            has_gs_primvar = any(
                kw in prop.GetName().lower()
                for prop in prim.GetProperties()
                for kw in GS_PRIMVAR_KEYWORDS
            )
            if has_gs_primvar:
                print("[build_combined_usd] GS primvar 감지 → Case B")
                return "b"
            print("[build_combined_usd] Points 감지, GS primvar 없음 → Case C")
            return "c"

    print("[build_combined_usd] GS/Points 없음 → Case C (Mesh 전용)")
    return "c"


def build_combined(index: int, output_dir: str, gs_mode: str):
    try:
        from pxr import Usd, UsdGeom, Sdf
    except ImportError:
        print("[ERROR] pxr 모듈 없음.")
        sys.exit(1)

    info     = ERTI_MAP[index]
    base     = info["base"]
    out_dir  = info["out_dir"]
    os.makedirs(out_dir, exist_ok=True)

    usd_path = info["usd"]
    obj_path = info["obj"]
    ply_path = os.path.join(out_dir, f"{base}_gs.ply")
    combined = os.path.join(out_dir, f"{base}_combined.usda")

    if gs_mode == "auto":
        gs_mode = detect_gs_mode(usd_path)

    print(f"[build_combined_usd] ERTI{index} / mode={gs_mode}")
    print(f"  원본 USD : {usd_path}")
    print(f"  OBJ      : {obj_path}")
    print(f"  PLY      : {ply_path}")
    print(f"  출력     : {combined}")

    # --- Stage 생성 ---
    stage = Usd.Stage.CreateNew(combined)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world)

    def rel_path(target: str) -> str:
        return os.path.relpath(os.path.abspath(target), os.path.abspath(out_dir)).replace("\\", "/")

    # --- Mesh 레이어 (OBJ) ---
    if obj_path and os.path.exists(obj_path):
        mesh_prim = stage.DefinePrim("/World/Mesh", "Xform")
        mesh_prim.GetReferences().AddReference(f"./{rel_path(obj_path)}")
        # OBJ는 Y-up → Z-up 회전
        UsdGeom.Xformable(mesh_prim).AddRotateXOp().Set(90.0)
        print(f"  [+] Mesh 참조: {rel_path(obj_path)}")
    else:
        print(f"  [!] OBJ 없음: {obj_path}")

    # --- GS 레이어 ---
    if gs_mode in ("a", "b"):
        if not os.path.exists(usd_path):
            print(f"  [ERROR] 원본 USD 없음: {usd_path}")
            sys.exit(1)

        gs_xform = stage.DefinePrim("/World/GaussianSplats", "Xform")
        gs_xform.GetReferences().AddReference(f"./{rel_path(usd_path)}")

        # Y-up → Z-up 회전 (X축 +90°)
        UsdGeom.Xformable(gs_xform).AddRotateXOp().Set(90.0)

        # PLY 경로를 커스텀 속성으로 기록 (omni.gsplat 직접 로드용 참고)
        if os.path.exists(ply_path):
            ply_attr = gs_xform.CreateAttribute(
                "userProperties:gsplatPlyPath", Sdf.ValueTypeNames.Asset
            )
            ply_attr.Set(Sdf.AssetPath(rel_path(ply_path)))
            print(f"  [+] GS 원본 USD 참조: {rel_path(usd_path)}  (Y-up→Z-up 회전)")
            print(f"  [+] omni.gsplat PLY 경로 기록: {rel_path(ply_path)}")
        else:
            print(f"  [+] GS 원본 USD 참조: {rel_path(usd_path)}  (Y-up→Z-up 회전)")
            print(f"  [!] PLY 없음. gs_to_ply.py를 먼저 실행하세요.")

    elif gs_mode == "c":
        if os.path.exists(usd_path):
            pc_prim = stage.DefinePrim("/World/PointCloud", "Xform")
            pc_prim.GetReferences().AddReference(f"./{rel_path(usd_path)}")
            UsdGeom.Xformable(pc_prim).AddRotateXOp().Set(90.0)
            print(f"  [+] PointCloud 참조: {rel_path(usd_path)}")

    stage.GetRootLayer().Save()
    print(f"[build_combined_usd] 저장 완료: {combined}")
    return combined


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GS + Mesh 통합 USDA 구성")
    parser.add_argument("--index", type=int, choices=[1, 2, 3], required=True,
                        help="ERTI 인덱스 (1, 2, 3)")
    parser.add_argument("--output-dir", default="output", help="출력 루트 디렉토리")
    parser.add_argument("--gs-mode", choices=["auto", "a", "b", "c"], default="auto",
                        help="GS 처리 모드 (auto: 자동 판별)")
    args = parser.parse_args()

    build_combined(args.index, args.output_dir, args.gs_mode)
