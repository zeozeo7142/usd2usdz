"""
Step 4: USD 내 GS(Gaussian Splatting) primvar 데이터를 표준 3DGS PLY 포맷으로 변환
Case B (비호환 포맷)일 때만 실행합니다.

사용법: python gs_to_ply.py "<usd_file>" "<output.ply>"
예시:  python gs_to_ply.py "output/ERTI1/260521_ERTI_1.usd" "output/ERTI1/260521_ERTI_1_gs.ply"

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

# PLY 헤더 생성 (바이너리 리틀엔디언)
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

    # GS prim 탐색 (Points 또는 GaussianSplats 타입)
    gs_prim = None
    for prim in stage.Traverse():
        if prim.GetTypeName() in ("Points", "GaussianSplats"):
            gs_prim = prim
            print(f"[gs_to_ply] GS prim 발견: {prim.GetPath()} ({prim.GetTypeName()})")
            break

    if gs_prim is None:
        print("[ERROR] GS prim을 찾지 못했습니다. inspect_usd.py로 구조를 먼저 확인하세요.")
        sys.exit(1)

    def get_attr(prim, *names):
        """여러 후보 속성명 중 존재하는 것을 반환"""
        for name in names:
            attr = prim.GetAttribute(name)
            if attr and attr.HasValue():
                val = attr.Get()
                if val is not None:
                    return np.array(val, dtype=np.float32)
        return None

    # 위치
    positions = get_attr(gs_prim,
        "primvars:positions", "points", "primvars:points")
    if positions is None:
        # UsdGeom.Points 표준 속성
        pts = UsdGeom.Points(gs_prim)
        raw = pts.GetPointsAttr().Get()
        if raw:
            positions = np.array(raw, dtype=np.float32)
    if positions is None:
        print("[ERROR] 위치 데이터를 찾지 못했습니다.")
        sys.exit(1)

    N = len(positions)
    print(f"[gs_to_ply] splat 수: {N:,}")

    # 스케일 (log 공간으로 저장되는 경우가 많음)
    scales = get_attr(gs_prim,
        "primvars:scales", "primvars:scale", "widths")
    if scales is None:
        scales = np.zeros((N, 3), dtype=np.float32)
        print("[WARNING] scale 데이터 없음. 0으로 채웁니다.")
    elif scales.ndim == 1:
        scales = np.column_stack([scales, scales, scales])

    # 회전 (quaternion: w, x, y, z 또는 x, y, z, w)
    rotations = get_attr(gs_prim,
        "primvars:rotations", "primvars:orientations", "orientations")
    if rotations is None:
        rotations = np.tile([1, 0, 0, 0], (N, 1)).astype(np.float32)
        print("[WARNING] rotation 데이터 없음. 단위 쿼터니언으로 채웁니다.")

    # 불투명도 (sigmoid 전 logit 공간인 경우도 있음)
    opacities = get_attr(gs_prim,
        "primvars:opacities", "primvars:opacity", "opacities")
    if opacities is None:
        opacities = np.ones(N, dtype=np.float32) * 0.5
        print("[WARNING] opacity 데이터 없음. 0.5로 채웁니다.")
    if opacities.ndim > 1:
        opacities = opacities[:, 0]

    # SH 계수 (f_dc: degree-0, f_rest: degree 1~3)
    f_dc = get_attr(gs_prim,
        "primvars:sh_coeffs", "primvars:f_dc", "primvars:colors")
    if f_dc is None:
        # 색상에서 추정
        colors = get_attr(gs_prim, "primvars:displayColor", "primvars:color")
        if colors is not None:
            # RGB → SH DC 계수 변환 (C0 = 0.28209479177...)
            C0 = 0.28209479177387814
            f_dc = (np.array(colors, dtype=np.float32)[:, :3] - 0.5) / C0
        else:
            f_dc = np.zeros((N, 3), dtype=np.float32)
            print("[WARNING] SH/색상 데이터 없음. 0으로 채웁니다.")

    f_rest_raw = get_attr(gs_prim, "primvars:f_rest", "primvars:sh_rest")
    if f_rest_raw is None:
        f_rest = np.zeros((N, 45), dtype=np.float32)
    else:
        f_rest = np.array(f_rest_raw, dtype=np.float32)
        if f_rest.ndim == 1:
            # interleaved 형태 처리
            expected = N * 45
            if len(f_rest) == expected:
                f_rest = f_rest.reshape(N, 45)
            else:
                # 맞지 않으면 0으로 패딩
                f_rest_new = np.zeros((N, 45), dtype=np.float32)
                cols = min(f_rest.shape[-1] if f_rest.ndim > 1 else 1, 45)
                f_rest_new[:, :cols] = f_rest[:N].reshape(N, -1)[:, :cols]
                f_rest = f_rest_new

    normals = np.zeros((N, 3), dtype=np.float32)

    # 배열 정렬
    positions = positions[:N].reshape(N, 3)
    scales = scales[:N].reshape(N, 3)
    rotations = rotations[:N].reshape(N, 4)
    opacities = opacities[:N].reshape(N, 1)
    f_dc = f_dc[:N].reshape(N, 3)
    f_rest = f_rest[:N].reshape(N, 45)

    # 최종 결합: (N, 62)
    data = np.concatenate([
        positions,   # 3
        normals,     # 3
        f_dc,        # 3
        f_rest,      # 45
        opacities,   # 1
        scales,      # 3
        rotations,   # 4
    ], axis=1)       # 총 62

    return data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="USD GS primvar → 3DGS PLY 변환")
    parser.add_argument("usd_path", help="입력 USD 파일 경로")
    parser.add_argument("ply_path", help="출력 PLY 파일 경로")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.ply_path)), exist_ok=True)
    data = extract_gs_from_usd(args.usd_path)
    write_ply(args.ply_path, data)
    print("[gs_to_ply] 완료")
