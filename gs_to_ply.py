"""
Step 4: USD 내 GS(Gaussian Splatting) 데이터를 표준 3DGS PLY 포맷으로 변환
Case B (비호환 포맷)일 때만 실행합니다.

사용법: python gs_to_ply.py "<usd_file>" "<output.ply>"
예시:  python gs_to_ply.py "USDZ/USDZ_ETRI1/260521_ERTI 1.usd" "output/USDZ_ETRI1/260521_ERTI_1_gs.ply"

지원 prim 타입:
  - ParticleField3DGaussianSplat  (PortalCam 독자 스키마)
  - Points, GaussianSplats         (표준 primvar 기반)

표준 3DGS PLY 컬럼:
  x, y, z, nx, ny, nz,
  f_dc_0, f_dc_1, f_dc_2,
  f_rest_0 ~ f_rest_44 (degree-3 SH, 45개),
  opacity,
  scale_0, scale_1, scale_2,
  rot_0, rot_1, rot_2, rot_3
"""

import sys
import os
import struct
import argparse
import numpy as np

PLY_PROPS = (
    ["x", "y", "z", "nx", "ny", "nz"]
    + [f"f_dc_{i}" for i in range(3)]
    + [f"f_rest_{i}" for i in range(45)]
    + ["opacity"]
    + [f"scale_{i}" for i in range(3)]
    + [f"rot_{i}" for i in range(4)]
)  # 총 62 속성


def write_ply(path: str, data: np.ndarray):
    """data: (N, 62) float32 배열"""
    n = data.shape[0]
    header = "ply\n"
    header += "format binary_little_endian 1.0\n"
    header += f"element vertex {n}\n"
    for prop in PLY_PROPS:
        header += f"property float {prop}\n"
    header += "end_header\n"

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(data.astype(np.float32).tobytes())
    print(f"[gs_to_ply] PLY 저장: {path} ({n:,} splats)")


def extract_portalcam_gs(prim) -> np.ndarray:
    """
    ParticleField3DGaussianSplat prim에서 데이터 추출.

    속성 레이아웃:
      positions       : point3f[N]   — 선형 좌표 (미터)
      orientations    : quatf[N]     — USD 쿼터니언 (w, x, y, z)
      scales          : float3[N]    — 선형 스케일 (미터) → log 변환 필요
      opacities       : float[N]     — 0~1 선형 → logit 변환 필요
      radiance:sphericalHarmonicsCoefficients : float3[N*16]
                                     — (N, 16, 3) 으로 reshape 후
                                       DC = [:, 0, :]
                                       rest = [:, 1:, :].transpose(0,2,1).reshape(N, 45)
    """
    from pxr import Vt

    def get(name):
        attr = prim.GetAttribute(name)
        return attr.Get() if (attr and attr.HasValue()) else None

    # --- 위치 ---
    raw_pos = get("positions")
    if raw_pos is None:
        print("[ERROR] 'positions' 속성 없음.")
        sys.exit(1)
    positions = np.array(raw_pos, dtype=np.float32).reshape(-1, 3)
    N = len(positions)
    print(f"[gs_to_ply] splat 수: {N:,}")

    # --- 쿼터니언 (w, x, y, z) ---
    raw_ori = get("orientations")
    if raw_ori is not None:
        # USD Gf.Quatf: .real = w, .imaginary = Vec3f(x, y, z)
        qw = np.array([q.real          for q in raw_ori], dtype=np.float32)
        qx = np.array([q.imaginary[0]  for q in raw_ori], dtype=np.float32)
        qy = np.array([q.imaginary[1]  for q in raw_ori], dtype=np.float32)
        qz = np.array([q.imaginary[2]  for q in raw_ori], dtype=np.float32)
        rotations = np.stack([qw, qx, qy, qz], axis=1)  # (N, 4)
    else:
        print("[WARNING] orientations 없음. 단위 쿼터니언으로 채웁니다.")
        rotations = np.tile([1.0, 0.0, 0.0, 0.0], (N, 1)).astype(np.float32)

    # --- 스케일: 선형 → log ---
    raw_scales = get("scales")
    if raw_scales is not None:
        scales_linear = np.array(raw_scales, dtype=np.float32).reshape(N, 3)
        scales_linear = np.clip(scales_linear, 1e-8, None)
        scales = np.log(scales_linear)
    else:
        print("[WARNING] scales 없음. log(0.01)로 채웁니다.")
        scales = np.full((N, 3), np.log(0.01), dtype=np.float32)

    # --- 불투명도: 선형 → logit ---
    raw_opacity = get("opacities")
    if raw_opacity is not None:
        alpha = np.array(raw_opacity, dtype=np.float32).reshape(N)
        alpha = np.clip(alpha, 1e-6, 1 - 1e-6)
        opacities = np.log(alpha / (1.0 - alpha))  # logit
    else:
        print("[WARNING] opacities 없음. logit(0.5)=0으로 채웁니다.")
        opacities = np.zeros(N, dtype=np.float32)

    # --- SH 계수 ---
    raw_sh = get("radiance:sphericalHarmonicsCoefficients")
    if raw_sh is not None:
        sh_degree = get("radiance:sphericalHarmonicsDegree") or 3
        n_coeffs = (sh_degree + 1) ** 2  # degree 3 → 16

        sh_flat = np.array(raw_sh, dtype=np.float32)           # (N*n_coeffs, 3)
        sh = sh_flat.reshape(N, n_coeffs, 3)                   # (N, 16, 3)

        f_dc   = sh[:, 0, :]                                   # (N, 3)
        # PLY 순서: [R계수1..15, G계수1..15, B계수1..15]
        f_rest = sh[:, 1:, :].transpose(0, 2, 1).reshape(N, -1)  # (N, 45)
        # degree < 3이면 45열로 패딩
        if f_rest.shape[1] < 45:
            pad = np.zeros((N, 45 - f_rest.shape[1]), dtype=np.float32)
            f_rest = np.concatenate([f_rest, pad], axis=1)
    else:
        print("[WARNING] SH 계수 없음. 0으로 채웁니다.")
        f_dc   = np.zeros((N, 3),  dtype=np.float32)
        f_rest = np.zeros((N, 45), dtype=np.float32)

    normals = np.zeros((N, 3), dtype=np.float32)

    data = np.concatenate([
        positions,              # 3
        normals,                # 3
        f_dc,                   # 3
        f_rest,                 # 45
        opacities.reshape(N, 1),# 1
        scales,                 # 3
        rotations,              # 4
    ], axis=1)                  # 총 62

    return data


def extract_standard_gs(gs_prim) -> np.ndarray:
    """Points / GaussianSplats prim (표준 primvar 기반)에서 데이터 추출."""
    from pxr import UsdGeom

    def get_attr(prim, *names):
        for name in names:
            attr = prim.GetAttribute(name)
            if attr and attr.HasValue():
                val = attr.Get()
                if val is not None:
                    return np.array(val, dtype=np.float32)
        return None

    positions = get_attr(gs_prim, "primvars:positions", "points", "primvars:points")
    if positions is None:
        pts = UsdGeom.Points(gs_prim)
        raw = pts.GetPointsAttr().Get()
        if raw:
            positions = np.array(raw, dtype=np.float32)
    if positions is None:
        print("[ERROR] 위치 데이터를 찾지 못했습니다.")
        sys.exit(1)

    N = len(positions)
    print(f"[gs_to_ply] splat 수: {N:,}")

    scales = get_attr(gs_prim, "primvars:scales", "primvars:scale", "widths")
    if scales is None:
        scales = np.zeros((N, 3), dtype=np.float32)
        print("[WARNING] scale 없음. 0으로 채웁니다.")
    elif scales.ndim == 1:
        scales = np.column_stack([scales, scales, scales])

    rotations = get_attr(gs_prim, "primvars:rotations", "primvars:orientations", "orientations")
    if rotations is None:
        rotations = np.tile([1, 0, 0, 0], (N, 1)).astype(np.float32)
        print("[WARNING] rotation 없음. 단위 쿼터니언으로 채웁니다.")

    opacities = get_attr(gs_prim, "primvars:opacities", "primvars:opacity", "opacities")
    if opacities is None:
        opacities = np.ones(N, dtype=np.float32) * 0.5
        print("[WARNING] opacity 없음. 0.5로 채웁니다.")
    if opacities.ndim > 1:
        opacities = opacities[:, 0]

    f_dc = get_attr(gs_prim, "primvars:sh_coeffs", "primvars:f_dc", "primvars:colors")
    if f_dc is None:
        colors = get_attr(gs_prim, "primvars:displayColor", "primvars:color")
        if colors is not None:
            C0 = 0.28209479177387814
            f_dc = (np.array(colors, dtype=np.float32)[:, :3] - 0.5) / C0
        else:
            f_dc = np.zeros((N, 3), dtype=np.float32)
            print("[WARNING] SH/색상 없음. 0으로 채웁니다.")

    f_rest_raw = get_attr(gs_prim, "primvars:f_rest", "primvars:sh_rest")
    if f_rest_raw is None:
        f_rest = np.zeros((N, 45), dtype=np.float32)
    else:
        f_rest = f_rest_raw.reshape(N, -1) if f_rest_raw.ndim == 1 else f_rest_raw
        if f_rest.shape[1] < 45:
            pad = np.zeros((N, 45 - f_rest.shape[1]), dtype=np.float32)
            f_rest = np.concatenate([f_rest, pad], axis=1)

    normals = np.zeros((N, 3), dtype=np.float32)
    positions  = positions[:N].reshape(N, 3)
    scales     = scales[:N].reshape(N, 3)
    rotations  = rotations[:N].reshape(N, 4)
    opacities  = opacities[:N].reshape(N, 1)
    f_dc       = f_dc[:N].reshape(N, 3)
    f_rest     = f_rest[:N].reshape(N, 45)

    data = np.concatenate([
        positions, normals, f_dc, f_rest, opacities, scales, rotations,
    ], axis=1)
    return data


def extract_gs_from_usd(usd_path: str) -> np.ndarray:
    try:
        from pxr import Usd, UsdGeom, Vt
    except ImportError:
        print("[ERROR] pxr 모듈 없음.")
        sys.exit(1)

    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"[ERROR] USD 열기 실패: {usd_path}")
        sys.exit(1)

    PORTACAM_TYPE = "ParticleField3DGaussianSplat"
    STANDARD_TYPES = ("Points", "GaussianSplats")

    gs_prim = None
    gs_type = None
    for prim in stage.Traverse():
        t = prim.GetTypeName()
        if t == PORTACAM_TYPE:
            gs_prim = prim
            gs_type = PORTACAM_TYPE
            break
        if t in STANDARD_TYPES and gs_prim is None:
            gs_prim = prim
            gs_type = t

    if gs_prim is None:
        print("[ERROR] GS prim을 찾지 못했습니다. inspect_usd.py로 구조를 먼저 확인하세요.")
        sys.exit(1)

    print(f"[gs_to_ply] GS prim: {gs_prim.GetPath()} (타입: {gs_type})")

    if gs_type == PORTACAM_TYPE:
        return extract_portalcam_gs(gs_prim)
    else:
        return extract_standard_gs(gs_prim)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="USD GS 데이터 → 표준 3DGS PLY 변환")
    parser.add_argument("usd_path", help="입력 USD 파일 경로")
    parser.add_argument("ply_path", help="출력 PLY 파일 경로")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.ply_path)), exist_ok=True)
    data = extract_gs_from_usd(args.usd_path)
    write_ply(args.ply_path, data)
    print("[gs_to_ply] 완료")
