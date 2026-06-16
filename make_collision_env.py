"""
Step 7: 로봇 보행/주행용 충돌(collision) 환경 생성

복원된 스캔 메쉬는 표면 노이즈가 많아 울퉁불퉁하다. 로봇(Jackal, RBQ10)이
보행하려면 평평한 바닥이 필요한데, 시뮬레이션에서 로봇이 실제로 밟는 것은
'시각 메쉬'가 아니라 '충돌 지오메트리'다.

따라서 GS + 스캔 메쉬는 비주얼로 그대로 두고, 충돌 레이어만 분리한다:
  - 평평한 바닥 collider (로봇이 이 위에 안착)
  - 벽/가구 collider (스캔 메쉬에서 바닥면만 제거한 삼각형 메쉬)

좌표계: PortalCam OBJ/NuRec는 이미 Z-up이다(수평면 법선이 Z축에 몰림).
따라서 회전 없이 stage upAxis=Z, 중력 -Z 그대로 물리가 올바르게 작동한다.

생성물 (out_dir 예: output/USDZ_ETRI1/):
  260521_ERTI_<N>_collision.usdc  : 충돌 전용 지오메트리 (Z-up, crate 바이너리)
  260521_ERTI_<N>_robot.usda      : Isaac Sim 로드용 Z-up 씬 (GS+메쉬+충돌 참조)

사용법:
  python make_collision_env.py --index 1
  python make_collision_env.py --index 1 --floor-band 0.08
  python make_collision_env.py --index 1 --rotate-x 90   # 다른 좌표계일 때만

추가 패키지 설치 없음 — numpy + usd-core(pxr)만 사용.
기존 작동 파일(_nurec_mesh.usda, _nurec.usdz, _mesh.obj)은 수정하지 않는다.
"""

import os
import sys
import math
import array
import argparse

import numpy as np

from build_combined_usd import ERTI_MAP

# 위쪽 축 인덱스. PortalCam OBJ/NuRec는 실제로 Z-up이다
# (수평면 법선이 Z축에 압도적으로 몰려 있고, Z extent=층고).
# 원본 _nurec_mesh.usda가 회전 없이 올바르게 렌더된 것도 이 때문.
UP = 2  # Z

HORIZ_COS = math.cos(math.radians(20.0))  # 수평면 판정 임계값 (법선·Y > cos20°)


# --------------------------------------------------------------------------- #
# A. OBJ 파싱 (numpy only)
# --------------------------------------------------------------------------- #
def parse_obj(path: str):
    """OBJ → (points (N,3) float32, faces (M,3) int32).

    'v ' 정점, 'f ' 면만 읽는다. 면 토큰은 'v', 'v/vt', 'v/vt/vn', 'v//vn'
    모두 지원 (첫 '/' 앞 정수, 1-based → 0-based). 4각형 이상은 fan triangulate.
    array.array로 누적해 대용량(ETRI3 765k face) 피크 메모리를 줄인다.
    """
    vx = array.array("f")
    fi = array.array("i")  # 평탄화된 삼각형 인덱스 (0-based)

    with open(path, "r") as fh:
        for line in fh:
            if line.startswith("v "):
                _, x, y, z = line.split()[:4]
                vx.append(float(x)); vx.append(float(y)); vx.append(float(z))
            elif line.startswith("f "):
                toks = line.split()[1:]
                idx = [int(t.split("/", 1)[0]) - 1 for t in toks]
                # fan triangulation (삼각형이면 그대로)
                for k in range(1, len(idx) - 1):
                    fi.append(idx[0]); fi.append(idx[k]); fi.append(idx[k + 1])

    points = np.frombuffer(vx, dtype=np.float32).reshape(-1, 3).copy()
    faces = np.frombuffer(fi, dtype=np.int32).reshape(-1, 3).copy()
    print(f"[parse_obj] {os.path.basename(path)}: "
          f"verts={len(points):,}, faces={len(faces):,}")
    return points, faces


# --------------------------------------------------------------------------- #
# 면 법선/면적/중심 (B, C 공용)
# --------------------------------------------------------------------------- #
def face_geometry(points: np.ndarray, faces: np.ndarray):
    """면별 (단위법선, 면적, 중심) 반환."""
    tri = points[faces]                       # (M, 3, 3)
    e1 = tri[:, 1] - tri[:, 0]
    e2 = tri[:, 2] - tri[:, 0]
    cross = np.cross(e1, e2)                   # (M, 3)
    norm = np.linalg.norm(cross, axis=1)
    area = 0.5 * norm
    safe = np.where(norm > 1e-12, norm, 1.0)
    normals = cross / safe[:, None]
    centroids = tri.mean(axis=1)              # (M, 3)
    return normals, area, centroids


# --------------------------------------------------------------------------- #
# B. 바닥 레벨 검출 (다중 층 지원)
# --------------------------------------------------------------------------- #
def detect_levels(points, normals, area, centroids, faces,
                  mode="auto", band_override=None, thresh_frac=0.12,
                  levels_z=None, min_area=10.0):
    """위를 향한(up-facing, 법선 +UP) 수평면 클러스터로 바닥 레벨을 검출.

    천장은 아래를 향하므로(-UP) 자동 배제된다. 계단 tread는 면적이 작고
    레벨 사이 높이라 클러스터를 이루지 않아 obstacle 메쉬에 남는다.

    mode:
      auto     : 임계 이상인 모든 레벨 (ETRI1 단층=1개, ETRI2 다층=N개)
      dominant : 면적이 가장 큰 레벨 1개만 (ETRI3 지배적 지면)
      lowest   : 가장 낮은 레벨 1개만

    반환: levels = [{z, t_below, t_above, bbox_min, bbox_max, area}], up_mask
    """
    cu = centroids[:, UP]
    up_mask = normals[:, UP] > HORIZ_COS       # 위를 향한 면만 = 바닥 후보
    zmin, zmax = float(cu.min()), float(cu.max())
    extent = max(zmax - zmin, 1e-6)

    def refine(z_lo, z_hi):
        ref = up_mask & (cu >= z_lo) & (cu <= z_hi)
        if ref.sum() == 0:
            return None
        w = area[ref]
        z = float(np.average(cu[ref], weights=w))
        # 적응적 밴드 위쪽 두께
        if band_override is not None:
            t_above = float(band_override)
        else:
            sheet = up_mask & (cu >= z - 0.10) & (cu <= z + 1.0)
            t_above = (float(np.percentile(cu[sheet] - z, 90.0))
                       if sheet.sum() >= 5 else 0.20)
            t_above = float(min(max(t_above, 0.15), 0.50))
        # 이 레벨에 속하는 바닥면의 XY footprint
        lvl = up_mask & (cu >= z - 0.15) & (cu <= z + t_above)
        vid = np.unique(faces[lvl].reshape(-1))
        bbmin = points[vid].min(0) if len(vid) else points.min(0)
        bbmax = points[vid].max(0) if len(vid) else points.max(0)
        return {"z": z, "t_below": 0.15, "t_above": t_above,
                "bbox_min": bbmin, "bbox_max": bbmax, "area": float(w.sum())}

    # 수동 지정 우선
    if levels_z:
        levels = []
        for z in levels_z:
            lv = refine(z - 0.25, z + 0.25)
            if lv:
                lv["z"] = float(z)
                levels.append(lv)
        print(f"[detect_levels] 수동 레벨 {len(levels)}개: "
              f"{[round(l['z'],2) for l in levels]}")
        return levels, up_mask

    if up_mask.sum() < 3 or area[up_mask].sum() <= 0:
        z = float(np.percentile(cu, 2.0))
        print(f"[detect_levels] up-facing 클러스터 없음 → 2퍼센타일 z={z:.3f}")
        return [refine(z - 0.25, z + 0.25)], up_mask

    # 면적가중 히스토그램에서 연속 bin 묶음 = 레벨
    nbins = max(10, int(extent / 0.05))
    hist, edges = np.histogram(cu[up_mask], bins=nbins,
                               range=(zmin, zmax), weights=area[up_mask])
    above = hist >= thresh_frac * hist.max()
    cand = []
    i = 0
    while i < nbins:
        if above[i]:
            j = i
            while j < nbins and above[j]:
                j += 1
            lv = refine(edges[i], edges[j])
            if lv:
                cand.append(lv)
            i = j
        else:
            i += 1

    if mode == "dominant":
        levels = [max(cand, key=lambda l: l["area"])]
    elif mode == "lowest":
        levels = [min(cand, key=lambda l: l["z"])]
    else:  # auto: 최소 면적 이상인 레벨만 (작은 천장 조각/노이즈 제외)
        big = [l for l in cand if l["area"] >= min_area]
        if not big:                                  # 전부 작으면 최대 1개라도
            big = [max(cand, key=lambda l: l["area"])]
        levels = sorted(big, key=lambda l: l["z"])

    print(f"[detect_levels] mode={mode} → 레벨 {len(levels)}개:")
    for l in levels:
        print(f"    z={l['z']:7.3f}  면적 {l['area']:6.1f}㎡  "
              f"밴드[-{l['t_below']:.2f},+{l['t_above']:.2f}]")
    return levels, up_mask


# --------------------------------------------------------------------------- #
# C. 충돌 지오메트리 구성
#    - 장애물 메쉬: 바닥면을 제외한 전부 (벽·계단·나무·울타리)
#    - 레벨별 평탄 바닥 메쉬: 그 레벨 바닥면을 level_z로 스냅 (footprint 보존)
# --------------------------------------------------------------------------- #
def _compact(points, sel_faces):
    used, inverse = np.unique(sel_faces.reshape(-1), return_inverse=True)
    return points[used].astype(np.float32), inverse.reshape(-1, 3).astype(np.int32)


def build_collision_geometry(points, faces, centroids, floor_face_mask, levels):
    """floor_face_mask: 바닥으로 간주해 제거할 후보(밴드와 무관, 방향 기준).
    반환: (obstacle_pts, obstacle_faces, floor_meshes)
    floor_meshes = [(pts, faces, z), ...]  (각 pts는 Z가 level_z로 평탄화됨)
    """
    cu = centroids[:, UP]
    floor_mask = np.zeros(len(faces), dtype=bool)
    floor_meshes = []

    for l in levels:
        mask_L = floor_face_mask & (cu >= l["z"] - l["t_below"]) & \
                 (cu <= l["z"] + l["t_above"])
        floor_mask |= mask_L
        pts_L, faces_L = _compact(points, faces[mask_L])
        pts_L[:, UP] = float(l["z"])                 # 평탄화 (footprint 유지)
        floor_meshes.append((pts_L, faces_L, float(l["z"])))

    obstacle_pts, obstacle_faces = _compact(points, faces[~floor_mask])
    dropped, total = int(floor_mask.sum()), len(faces)

    print(f"[build_geometry] 바닥면 {dropped:,}/{total:,} "
          f"({100.0 * dropped / max(total, 1):.1f}%) → 평탄 바닥 메쉬 "
          f"{len(floor_meshes)}개, 장애물 메쉬 faces={len(obstacle_faces):,}")
    for i, (p, f, z) in enumerate(floor_meshes):
        print(f"    바닥{i}: z={z:.3f} verts={len(p):,} faces={len(f):,}")
    return obstacle_pts, obstacle_faces, floor_meshes


def compute_spawn(floor_meshes, obstacle_pts):
    """가장 낮은 레벨 바닥에서 '장애물이 위에 없는 가장 열린 지점'을 고른다.
    로봇이 가구/벽에 박혀 물리 폭발하는 것을 막기 위함. 반환: (x, y, z)."""
    lm = min(floor_meshes, key=lambda fm: fm[2])
    fpts, fz = lm[0], lm[2]
    ax = [k for k in range(3) if k != UP]              # 수평 두 축
    fxy = fpts[:, ax]
    if len(fxy) == 0:
        return None
    # 바닥 위 로봇 높이(0.1~1.2m) 안에 있는 장애물 정점만 (천장/먼 벽 제외)
    oz = obstacle_pts[:, UP]
    env = obstacle_pts[(oz > fz + 0.1) & (oz < fz + 1.2)]
    center = [0.0, 0.0, 0.0]
    if len(env) == 0:
        c = fxy.mean(0)
    else:
        oxy = env[:, ax]
        fs = fxy[:: max(1, len(fxy) // 1500)]
        os_ = oxy[:: max(1, len(oxy) // 3000)]
        d = np.array([np.min(np.sum((os_ - q) ** 2, axis=1)) for q in fs])
        c = fs[int(np.argmax(d))]
        print(f"[compute_spawn] 열린 바닥 지점 장애물거리 {d.max()**0.5:.2f}m")
    center[ax[0]], center[ax[1]], center[UP] = float(c[0]), float(c[1]), float(fz)

    # 스폰 방향(yaw): 바닥 footprint 주축 = 복도 길이방향 → 로봇이 벽이 아니라
    # 열린 방향을 보게 한다. ax[0]를 X로 간주한 평면 내 yaw(rad).
    fc = fxy - fxy.mean(0)
    eigval, eigvec = np.linalg.eigh(fc.T @ fc)
    axis_v = eigvec[:, int(np.argmax(eigval))]
    yaw = float(np.arctan2(axis_v[1], axis_v[0]))
    return (center[0], center[1], center[2], yaw)


# --------------------------------------------------------------------------- #
# D. 충돌 에셋 _collision.usdc 저작 (Y-up, crate)
# --------------------------------------------------------------------------- #
def _author_mesh(stage, path, pts, faces, approx):
    from pxr import UsdGeom, UsdPhysics, Vt
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
    m.CreateFaceVertexCountsAttr(
        Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
    m.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(
        np.array([pts.min(0), pts.max(0)], dtype=np.float32)))
    mp = m.GetPrim()
    UsdPhysics.CollisionAPI.Apply(mp)
    UsdPhysics.MeshCollisionAPI.Apply(mp).CreateApproximationAttr(approx)
    UsdGeom.Imageable(mp).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    return mp


def _author_floor_slab(stage, path, level):
    """레벨 footprint bbox를 덮는 구멍 없는 얇은 박스 슬랩 (UP축 2cm)."""
    from pxr import UsdGeom, UsdPhysics, Gf
    bmin, bmax = level["bbox_min"], level["bbox_max"]
    center = [0.5 * (bmin[k] + bmax[k]) for k in range(3)]
    center[UP] = float(level["z"])
    scale = [(bmax[k] - bmin[k]) * 1.10 for k in range(3)]
    scale[UP] = 0.02
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
    xf.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in scale]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdGeom.Imageable(cube.GetPrim()).CreateVisibilityAttr(
        UsdGeom.Tokens.invisible)
    return cube.GetPrim()


def _make_physics_material(stage, path, static_f, dynamic_f):
    """마찰 물리 물성 생성. 바닥/벽에 바인딩하면 바퀴가 헛돌지 않는다."""
    from pxr import UsdShade, UsdPhysics
    mat = UsdShade.Material.Define(stage, path)
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(float(static_f))
    pm.CreateDynamicFrictionAttr(float(dynamic_f))
    pm.CreateRestitutionAttr(0.0)
    return mat


def _bind_physics_material(prim, mat):
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        mat, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")


def write_collision_usdc(out_path, obstacle_pts, obstacle_faces,
                         floor_meshes, levels, floor_shape, approx,
                         spawn=None, friction=0.9):
    from pxr import Usd, UsdGeom, Sdf, Gf

    stage = Usd.Stage.CreateNew(out_path)           # .usdc → crate 자동
    up_tok = UsdGeom.Tokens.z if UP == 2 else UsdGeom.Tokens.y
    UsdGeom.SetStageUpAxis(stage, up_tok)            # OBJ와 동일 (Z-up)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Colliders")
    stage.SetDefaultPrim(root.GetPrim())

    # 권장 로봇 스폰 지점/방향 (열린 바닥, 복도 방향) — teleop_test.py가 읽는다
    if spawn is not None:
        root.GetPrim().CreateAttribute(
            "usd2usdz:spawnPoint", Sdf.ValueTypeNames.Double3
        ).Set(Gf.Vec3d(spawn[0], spawn[1], spawn[2]))
        if len(spawn) > 3:
            root.GetPrim().CreateAttribute(
                "usd2usdz:spawnYaw", Sdf.ValueTypeNames.Double
            ).Set(float(spawn[3]))

    # 마찰 물성 (바퀴 슬립 방지)
    mat = _make_physics_material(stage, "/Colliders/PhysicsMaterial",
                                 static_f=friction, dynamic_f=friction * 0.9)
    collider_prims = []

    # 벽/계단/나무/울타리 등 장애물 (원본 형상 유지, approximation=none)
    collider_prims.append(_author_mesh(stage, "/Colliders/Obstacles",
                          obstacle_pts, obstacle_faces, approx))

    # 레벨별 바닥 collider
    #   slab : footprint bbox를 덮는 구멍 없는 박스 (로봇 추락 방지, 기본값)
    #   mesh : 바닥면을 level_z로 스냅한 메쉬 (footprint·구멍 그대로 보존)
    if floor_shape == "slab":
        for i, l in enumerate(levels):
            collider_prims.append(
                _author_floor_slab(stage, f"/Colliders/Floor_{i}", l))
    else:
        for i, (pts, faces, _z) in enumerate(floor_meshes):
            collider_prims.append(
                _author_mesh(stage, f"/Colliders/Floor_{i}", pts, faces, approx))

    for prim in collider_prims:
        _bind_physics_material(prim, mat)

    stage.GetRootLayer().Save()
    print(f"[write_collision_usdc] 저장: {out_path} "
          f"(장애물 1 + 바닥 {len(levels)} [{floor_shape}], 마찰 {friction})")


# --------------------------------------------------------------------------- #
# E. 최상위 _robot.usda 저작 (Z-up, 로드 대상)
# --------------------------------------------------------------------------- #
def rel_path(target: str, base_dir: str) -> str:
    return os.path.relpath(os.path.abspath(target),
                           os.path.abspath(base_dir)).replace("\\", "/")


def write_robot_usda(out_path, out_dir, nurec, mesh_obj, collision,
                     rotate_x, physics_scene):
    from pxr import Usd, UsdGeom, UsdPhysics, Gf

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # 데이터가 이미 Z-up이므로 기본 회전 없음. 원본 _nurec_mesh.usda와 동일 정렬.
    env = UsdGeom.Xform.Define(stage, "/World/Environment")
    if abs(rotate_x) > 1e-6:
        UsdGeom.Xformable(env).AddRotateXOp().Set(float(rotate_x))

    refs = [
        ("GaussianSplats", nurec),
        ("VisualMesh", mesh_obj),
        ("Colliders", collision),
    ]
    for name, target in refs:
        prim = stage.DefinePrim(f"/World/Environment/{name}", "Xform")
        prim.GetReferences().AddReference(f"./{rel_path(target, out_dir)}")

    if physics_scene:
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))  # Z-up
        scene.CreateGravityMagnitudeAttr(9.81)

    stage.GetRootLayer().Save()
    print(f"[write_robot_usda] 저장: {out_path} (rotate_x={rotate_x})")


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="로봇 보행용 충돌 환경 생성")
    ap.add_argument("--index", type=int, required=True, choices=[1, 2, 3])
    ap.add_argument("--output-dir", default="output")
    ap.add_argument("--floor-band", type=float, default=None,
                    help="바닥 밴드 위쪽 두께(m) 오버라이드")
    ap.add_argument("--levels", choices=["auto", "dominant", "lowest"],
                    default="auto",
                    help="auto: 모든 유의미 레벨(ETRI1 단층/ETRI2 다층) / "
                         "dominant: 면적 최대 지면 1개(ETRI3) / lowest: 최저 1개")
    ap.add_argument("--levels-z", default=None,
                    help="바닥 레벨 높이 수동 지정 (쉼표 구분, 예: '-0.2,3.1')")
    ap.add_argument("--level-thresh", type=float, default=0.12,
                    help="레벨 검출 임계 (최대 bin 면적 대비 비율)")
    ap.add_argument("--min-level-area", type=float, default=10.0,
                    help="auto 모드에서 바닥 레벨로 인정할 최소 면적(㎡)")
    ap.add_argument("--floor-shape", choices=["slab", "mesh"], default="slab",
                    help="slab: 구멍 없는 박스 바닥(추락 방지, 기본) / "
                         "mesh: 바닥 footprint·구멍 그대로 보존")
    ap.add_argument("--friction", type=float, default=0.9,
                    help="바닥/벽 collider 정지마찰 계수 (바퀴 슬립 방지)")
    ap.add_argument("--floor-angle", type=float, default=60.0,
                    help="바닥으로 간주해 제거할 면의 최대 기울기(도). 크게 하면 "
                         "기울어진 바닥 범프까지 제거(벽은 보존), 작으면 수평면만")
    ap.add_argument("--rotate-x", type=float, default=0.0,
                    help="환경 X축 회전(도). 데이터가 Z-up이므로 기본 0 "
                         "(회전 불필요). 다른 좌표계일 때만 사용.")
    ap.add_argument("--approx", default="none",
                    help="MeshCollisionAPI approximation (정적 환경은 none 권장)")
    ap.add_argument("--physics-scene", dest="physics_scene",
                    action="store_true", default=True)
    ap.add_argument("--no-physics-scene", dest="physics_scene",
                    action="store_false")
    args = ap.parse_args()

    try:
        from pxr import UsdPhysics  # noqa: F401
    except ImportError:
        print("[ERROR] pxr(usd-core) 모듈 없음.")
        sys.exit(1)

    info = ERTI_MAP[args.index]
    out_dir = info["out_dir"]
    base = info["base"]
    os.makedirs(out_dir, exist_ok=True)

    mesh_obj = os.path.join(out_dir, f"{base}_mesh.obj")
    nurec = os.path.join(out_dir, f"{base}_nurec.usdz")
    collision = os.path.join(out_dir, f"{base}_collision.usdc")
    robot = os.path.join(out_dir, f"{base}_robot.usda")

    # OBJ 소스: out_dir의 _mesh.obj 우선, 없으면 원본
    if not os.path.exists(mesh_obj):
        mesh_obj = info["obj"]
    if not os.path.exists(mesh_obj):
        print(f"[ERROR] OBJ 없음: {mesh_obj}")
        sys.exit(1)
    if not os.path.exists(nurec):
        print(f"[WARNING] NuRec USDZ 없음: {nurec} (참조 경로만 기록됨)")

    print(f"=== ERTI{args.index} 충돌 환경 생성 "
          f"(levels={args.levels}, rotate_x={args.rotate_x}) ===")

    levels_z = None
    if args.levels_z:
        levels_z = [float(s) for s in args.levels_z.split(",") if s.strip()]

    # A. 파싱
    points, faces = parse_obj(mesh_obj)
    # B/C 공용 기하
    normals, area, centroids = face_geometry(points, faces)
    # B. 바닥 레벨 검출 (다중 층)
    levels, up_mask = detect_levels(
        points, normals, area, centroids, faces,
        mode=args.levels, band_override=args.floor_band,
        thresh_frac=args.level_thresh, levels_z=levels_z,
        min_area=args.min_level_area)
    # C. 충돌 지오메트리 (장애물 메쉬 + 레벨별 평탄 바닥 메쉬)
    #    제거 마스크: 밴드 내에서 기울기 ≤ floor_angle인 면(기울어진 범프 포함).
    #    거의 수직인 벽은 보존되어 충돌이 유지된다.
    floor_cos = math.cos(math.radians(args.floor_angle))
    floor_face_mask = np.abs(normals[:, UP]) > floor_cos
    obstacle_pts, obstacle_faces, floor_meshes = build_collision_geometry(
        points, faces, centroids, floor_face_mask, levels)
    spawn = compute_spawn(floor_meshes, obstacle_pts)
    print(f"[main] 권장 스폰 지점: {tuple(round(v, 2) for v in spawn)}")

    # D. 충돌 에셋
    write_collision_usdc(collision, obstacle_pts, obstacle_faces,
                         floor_meshes, levels, args.floor_shape, args.approx,
                         spawn=spawn, friction=args.friction)
    # E. 최상위 씬
    write_robot_usda(robot, out_dir, nurec, mesh_obj, collision,
                     args.rotate_x, args.physics_scene)

    print(f"=== 완료 ===")
    print(f"  충돌: {collision}")
    print(f"  로드: {robot}")
    print(f"  Isaac Sim 5.1에서 {os.path.basename(robot)} 을 열어 검증하세요.")


if __name__ == "__main__":
    main()
