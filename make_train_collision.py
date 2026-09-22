"""
subway_car(TRAIN) — ETRI식 '평탄 슬랩' 충돌 버전 추가 생성.

subway_car.usdz는 이미 NuRec GS + 메쉬 + (울퉁불퉁 원본 그대로의) 콜라이더가
묶인 완성 패키지다. 이 스크립트는 '평탄 주행'용 대안 버전을 만든다:
  - 같은 메쉬로 ETRI식 평탄 슬랩 바닥 + 장애물 콜라이더 생성(make_collision_env 재사용)
  - 번들 메쉬의 콜라이더는 비활성화(시각용으로만 유지)
  - GS + 비주얼 메쉬 + 평탄 슬랩 + physicsScene 를 참조하는 _slab_robot.usda

원본 usdz는 수정하지 않고 output/USDZ_TRAIN/ 아래에만 생성.
사용: python3 make_train_collision.py   (GPU 불필요, numpy + usd-core만)
"""
import os
import sys
import math
import zipfile

import numpy as np

import make_collision_env as mce  # 레벨검출/슬랩/스폰 로직 재사용

SRC_USDZ = "USDZ/USDZ_TRAIN/subway_car.usdz"
OUT_DIR = "output/USDZ_TRAIN"
EXTRACT = os.path.join(OUT_DIR, "subway_car")
BASE = "subway_car"
FLOOR_ANGLE = 60.0
FRICTION = 0.9
# GUI에서 확인한 양호한 스폰 위치(x, y, z). None이면 자동(가장 넓은 층) 검출.
# 주의: teleop_test는 이 z를 '바닥높이'로 보고 +0.3m 위에 로봇을 놓는다.
FIXED_SPAWN = (3.3, -12.0, 0.3)


def load_usd_mesh(path):
    """USD Mesh에서 정점/삼각형 추출 (다각형은 fan triangulate)."""
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(path)
    mesh = None
    for p in stage.Traverse():
        if p.GetTypeName() == "Mesh":
            mesh = UsdGeom.Mesh(p)
            break
    if mesh is None:
        raise RuntimeError(f"Mesh prim 없음: {path}")
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float32)
    counts = np.array(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    idx = np.array(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    tris = []
    o = 0
    for c in counts:
        for k in range(1, c - 1):
            tris.append((idx[o], idx[o + k], idx[o + k + 1]))
        o += c
    return pts, np.array(tris, dtype=np.int32)


def main():
    from pxr import Usd, UsdGeom, UsdPhysics, Gf

    os.makedirs(EXTRACT, exist_ok=True)
    # 1) usdz 추출 (GS default.usda 참조 + mesh.usd 확보)
    zipfile.ZipFile(SRC_USDZ).extractall(EXTRACT)
    print(f"[train] 추출: {EXTRACT}")

    # 2) 번들 메쉬 로드
    pts, faces = load_usd_mesh(os.path.join(EXTRACT, "mesh.usd"))
    print(f"[train] mesh verts={len(pts):,} tris={len(faces):,}")

    # 3) 평탄 슬랩 충돌 생성 (make_collision_env 재사용, UP=Z)
    normals, area, centroids = mce.face_geometry(pts, faces)
    levels, _ = mce.detect_levels(
        pts, normals, area, centroids, faces,
        mode="auto", band_override=None, thresh_frac=0.12,
        levels_z=None, min_area=10.0)
    floor_cos = math.cos(math.radians(FLOOR_ANGLE))
    floor_face_mask = np.abs(normals[:, mce.UP]) > floor_cos
    obstacle_pts, obstacle_faces, floor_meshes = mce.build_collision_geometry(
        pts, faces, centroids, floor_face_mask, levels)
    if FIXED_SPAWN is not None:
        # 사용자가 GUI에서 확인한 고정 스폰 사용 (yaw=0)
        spawn = (FIXED_SPAWN[0], FIXED_SPAWN[1], FIXED_SPAWN[2], 0.0)
        print(f"[train] 고정 스폰 사용: {spawn}")
    else:
        # 자동: compute_spawn은 최저층을 고르므로, 가장 넓은 층(승강장)만 넘긴다.
        top_fm = max(floor_meshes, key=lambda fm: len(fm[1]))  # face 수 ∝ 면적
        print(f"[train] 검출 층 (z, faces): "
              f"{[(round(fm[2], 3), len(fm[1])) for fm in floor_meshes]} "
              f"→ 가장 넓은 층 z={top_fm[2]:.3f} 에 스폰")
        spawn = mce.compute_spawn([top_fm], obstacle_pts)
        print(f"[train] 스폰 지점: {tuple(round(v, 2) for v in spawn)}")

    coll = os.path.join(OUT_DIR, f"{BASE}_slab_collision.usdc")
    mce.write_collision_usdc(
        coll, obstacle_pts, obstacle_faces, floor_meshes, levels,
        "slab", "none", spawn=spawn, friction=FRICTION, full_mesh=(pts, faces))

    # 4) 로봇 씬: GS+비주얼메쉬(default.usda) + 번들 콜라이더 off + 평탄 슬랩 + 중력
    robot = os.path.join(OUT_DIR, f"{BASE}_slab_robot.usda")
    st = Usd.Stage.CreateNew(robot)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    world = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(world.GetPrim())

    sc = st.DefinePrim("/World/SubwayCar", "Xform")
    sc.GetReferences().AddReference("./subway_car/default.usda")
    # 번들 메쉬는 시각용으로 유지하되 콜라이더는 끔(평탄 슬랩으로 대체)
    mprim = st.OverridePrim("/World/SubwayCar/gauss/mesh")
    UsdPhysics.CollisionAPI.Apply(mprim).CreateCollisionEnabledAttr(False)

    colp = st.DefinePrim("/World/Colliders", "Xform")
    colp.GetReferences().AddReference(f"./{BASE}_slab_collision.usdc")

    scene = UsdPhysics.Scene.Define(st, "/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    st.GetRootLayer().Save()

    print(f"=== 완료 ===\n  충돌: {coll}\n  로드: {robot}")

    # 5) (선택) 단일 완성품 usdz로 패키징 — GS+메쉬+평탄슬랩+physicsScene 한 파일
    #    subway_car.usdz처럼 그냥 열면 됨. nurec<2GiB일 때만 안전(오프셋 버그).
    if "--usdz" in sys.argv:
        from pxr import UsdUtils
        pkg = os.path.join(OUT_DIR, f"{BASE}_slab.usdz")
        if os.path.exists(pkg):
            os.remove(pkg)
        if UsdUtils.CreateNewUsdzPackage(robot, pkg):
            mb = os.path.getsize(pkg) / 1e6
            print(f"  단일 usdz: {pkg} ({mb:.0f} MB)")
        else:
            print("  [경고] usdz 패키징 실패")


if __name__ == "__main__":
    main()
