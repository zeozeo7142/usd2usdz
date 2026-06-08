"""
GS PLY → UsdGeom.Points USD 변환 (포인트 클라우드 시각화용)

Isaac Sim에 GS 렌더러가 없을 때 대안으로 사용.
SH DC 항으로 색상 복원, scale 평균값으로 point width 설정.

사용법: python3 gs_to_pointcloud_usd.py <input.ply> <output.usda>
예시:   python3 gs_to_pointcloud_usd.py \
            output/USDZ_ETRI1/260521_ERTI_1_gs.ply \
            output/USDZ_ETRI1/260521_ERTI_1_pointcloud.usda
"""

import sys
import os
import numpy as np

SH_C0 = 0.28209479177387814


def read_ply(ply_path: str):
    with open(ply_path, "rb") as f:
        props = []
        n_verts = 0
        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("element vertex"):
                n_verts = int(line.split()[-1])
            elif line.startswith("property float"):
                props.append(line.split()[-1])
            elif line == "end_header":
                break

        print(f"[gs_to_pointcloud] {n_verts:,} splats, {len(props)} 속성 읽는 중...")
        data = np.frombuffer(f.read(n_verts * len(props) * 4), dtype=np.float32)
        data = data.reshape(n_verts, len(props))

    idx = {name: i for i, name in enumerate(props)}

    positions = data[:, [idx["x"], idx["y"], idx["z"]]]

    # SH DC → 선형 RGB
    f_dc = data[:, [idx["f_dc_0"], idx["f_dc_1"], idx["f_dc_2"]]]
    colors = np.clip(SH_C0 * f_dc + 0.5, 0.0, 1.0).astype(np.float32)

    # opacity: logit → sigmoid
    opacities = (1.0 / (1.0 + np.exp(-data[:, idx["opacity"]]))).astype(np.float32)

    # scale: log → linear, 평균 → width
    scales = np.exp(data[:, [idx["scale_0"], idx["scale_1"], idx["scale_2"]]])
    widths = (scales.mean(axis=1) * 2.0).astype(np.float32)

    return positions, colors, opacities, widths


def write_pointcloud_usd(out_path: str, positions, colors, opacities, widths):
    try:
        from pxr import Usd, UsdGeom, Vt, Gf, Sdf
    except ImportError:
        print("[ERROR] pxr 모듈 없음.")
        sys.exit(1)

    n = len(positions)
    print(f"[gs_to_pointcloud] USD 생성 중... ({n:,} points)")

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world)
    # Y-up PLY → Z-up USD
    UsdGeom.Xformable(world).AddRotateXOp().Set(90.0)

    pts = UsdGeom.Points.Define(stage, "/World/GaussianSplatPoints")

    # 위치 (numpy → Vt.Vec3fArray)
    pts.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(positions))

    # 포인트 크기
    pts.GetWidthsAttr().Set(Vt.FloatArray.FromNumpy(widths))

    # displayColor
    color_pv = UsdGeom.PrimvarsAPI(pts).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex
    )
    color_pv.Set(Vt.Vec3fArray.FromNumpy(colors))

    # displayOpacity
    opacity_pv = UsdGeom.PrimvarsAPI(pts).CreatePrimvar(
        "displayOpacity", Sdf.ValueTypeNames.FloatArray, UsdGeom.Tokens.vertex
    )
    opacity_pv.Set(Vt.FloatArray.FromNumpy(opacities))

    stage.GetRootLayer().Save()
    size_mb = os.path.getsize(out_path) / (1024**2)
    print(f"[gs_to_pointcloud] 완료: {out_path} ({size_mb:.1f} MB)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("사용법: python3 gs_to_pointcloud_usd.py <input.ply> <output.usda>")
        sys.exit(1)

    ply_path, out_path = sys.argv[1], sys.argv[2]

    if not os.path.exists(ply_path):
        print(f"[ERROR] PLY 없음: {ply_path}")
        sys.exit(1)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    positions, colors, opacities, widths = read_ply(ply_path)
    write_pointcloud_usd(out_path, positions, colors, opacities, widths)
