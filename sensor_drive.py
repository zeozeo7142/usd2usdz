"""
sensor_drive.py — Jackal + PhysX LiDAR + RGB(GS) 카메라, WASD 주행,
ROS2(rclpy 직접 퍼블리시)로 PointCloud2 / Image / TF / Clock → RViz2.

설계 (GPU 실측으로 검증된 사실 기반):
  - RTX 라이다는 카메라와 visibility 공유 → "GS 카메라 + 라이다" 동시 불가.
  - 그래서 LiDAR를 PhysX(물리 충돌 레이캐스트, visibility 무관)로 사용:
      · VisualMesh 숨김  → 카메라 = 순수 GS (GUI에서 NuRec 렌더)
      · PhysX 라이다     → 안 보이는 Colliders(평평바닥+벽)를 센싱
  - 환경은 open_stage가 아니라 World()+reference로 로드해야 PhysX 라이다가
    콜라이더를 레이캐스트한다 (open_stage는 env physicsScene 때문에 라이다 0).

실행 (Isaac Sim 5.1 컨테이너, ROS2 환경변수 필수):
  export ROS_DISTRO=humble
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/zeozeo/git/usd2usdz/fastdds_udp.xml
  export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
  /isaac-sim/python.sh /home/zeozeo/git/usd2usdz/sensor_drive.py --index 1
  ※ Isaac/RViz 컨테이너 모두 docker run에 --ipc=host.

조작: W/S 전후, A/D 회전, Space 정지, ESC 종료.
RViz: 호스트에서 ./run-rviz.sh  (Fixed Frame: world)
"""
import argparse
import sys

parser = argparse.ArgumentParser(description="Jackal + PhysX LiDAR + GS Camera → ROS2/RViz")
parser.add_argument("--index", type=int, choices=[1, 2, 3])
parser.add_argument("--env", default=None)
parser.add_argument("--robot-usd", default=None)
parser.add_argument("--lin-speed", type=float, default=1.5)
parser.add_argument("--ang-speed", type=float, default=2.0)
parser.add_argument("--ros-distro", default="humble")
parser.add_argument("--cam-w", type=int, default=640)
parser.add_argument("--cam-h", type=int, default=480)
parser.add_argument("--lidar-vfov", type=float, default=30.0, help="라이다 수직 FOV(도)")
parser.add_argument("--lidar-hres", type=float, default=0.4, help="수평 해상도(도/샘플)")
parser.add_argument("--lidar-vres", type=float, default=1.0, help="수직 해상도(도/채널)")
parser.add_argument("--lidar-range", type=float, default=100.0, help="최대 거리(m)")
parser.add_argument("--show-visualmesh", action="store_true",
                    help="VisualMesh를 보이게(디버그). 기본은 숨김(카메라=GS)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--max-steps", type=int, default=400)
parser.add_argument("--spawn", default=None)
args = parser.parse_args()

from isaacsim import SimulationApp                       # noqa: E402
sim_app = SimulationApp({"headless": args.headless})

import os                                                # noqa: E402
import carb                                              # noqa: E402
import carb.input                                        # noqa: E402
import numpy as np                                       # noqa: E402
import omni.appwindow                                    # noqa: E402
import omni.usd                                          # noqa: E402
import omni.replicator.core as rep                       # noqa: E402
from pxr import Usd, UsdGeom, Gf                         # noqa: E402

carb.settings.get_settings().set_string(
    "/exts/isaacsim.ros2.bridge/ros_distro", args.ros_distro)
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")
sim_app.update()

from isaacsim.core.api import World                      # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from isaacsim.robot.wheeled_robots.robots import WheeledRobot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.sensors.physx import RotatingLidarPhysX    # noqa: E402

import rclpy                                             # noqa: E402
from rclpy.qos import qos_profile_sensor_data            # noqa: E402
from builtin_interfaces.msg import Time                  # noqa: E402
from rosgraph_msgs.msg import Clock                      # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField, Image  # noqa: E402
from tf2_msgs.msg import TFMessage                       # noqa: E402
from geometry_msgs.msg import TransformStamped           # noqa: E402

import functools                                         # noqa: E402
print = functools.partial(print, flush=True)

ERTI_OUT = {1: ("output/USDZ_ETRI1", "260521_ERTI_1"),
            2: ("output/USDZ_ETRI2", "260521_ERTI_2"),
            3: ("output/USDZ_ETRI3", "260521_ERTI_3")}
ROBOT_PATH = "/Isaac/Robots/Clearpath/Jackal/jackal.usd"
WHEELS = ["front_left_wheel_joint", "rear_left_wheel_joint",
          "front_right_wheel_joint", "rear_right_wheel_joint"]
WHEEL_R, WHEEL_BASE = 0.098, 0.37
ROBOT_PRIM = "/World/Jackal"
BASE_LINK = ROBOT_PRIM + "/base_link"
SCENE_PRIM = "/World/Scene"          # env를 reference로 붙이는 위치
VISUALMESH = SCENE_PRIM + "/Environment/VisualMesh"
LIDAR_OFFSET = (0.0, 0.0, 0.3)
CAM_OFFSET = (0.2, 0.0, 0.25)


def resolve_paths():
    if args.env:
        env = os.path.abspath(args.env)
        d, base = os.path.dirname(env), os.path.basename(env).replace("_robot.usda", "")
    elif args.index:
        od, base = ERTI_OUT[args.index]
        if not os.path.isabs(od):
            od = os.path.join(os.path.dirname(os.path.abspath(__file__)), od)
        env, d = os.path.abspath(os.path.join(od, f"{base}_robot.usda")), od
    else:
        carb.log_error("--index 또는 --env 필요"); sim_app.close(); sys.exit(1)
    return env, os.path.join(d, f"{base}_collision.usdc")


def detect_spawn(coll):
    if args.spawn:
        x, y, z = [float(v) for v in args.spawn.split(",")]
        return np.array([x, y, z]), 0.0
    st = Usd.Stage.Open(coll)
    root = st.GetPrimAtPath("/Colliders")
    sp = root.GetAttribute("usd2usdz:spawnPoint").Get()
    ya = root.GetAttribute("usd2usdz:spawnYaw")
    yaw = float(ya.Get()) if (ya and ya.HasValue()) else 0.0
    return np.array([float(sp[0]), float(sp[1]), float(sp[2])]), yaw


def set_wheel_drive(stage, damping=1.0e3, max_force=2.0e3):
    from pxr import UsdPhysics
    for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
        if p.GetName() in WHEELS:
            d = UsdPhysics.DriveAPI.Get(p, "angular") or \
                UsdPhysics.DriveAPI.Apply(p, "angular")
            d.CreateStiffnessAttr(0.0)
            d.CreateDampingAttr(damping)
            d.CreateMaxForceAttr(max_force)


def to_time(t):
    s = int(t)
    return Time(sec=s, nanosec=int((t - s) * 1e9))


PC_FIELDS = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
             PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
             PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]


def make_pc2(stamp, frame, pts):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]              # 미히트(inf) 제거
    m = PointCloud2()
    m.header.stamp = stamp
    m.header.frame_id = frame
    m.height = 1
    m.width = len(pts)
    m.fields = PC_FIELDS
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * len(pts)
    m.is_dense = True
    m.data = np.ascontiguousarray(pts).tobytes()
    return m


def make_img(stamp, frame, rgb):
    rgb = np.asarray(rgb)
    if rgb.ndim == 3 and rgb.shape[2] == 4:
        rgb = rgb[:, :, :3]
    rgb = np.ascontiguousarray(rgb.astype(np.uint8))
    m = Image()
    m.header.stamp = stamp
    m.header.frame_id = frame
    m.height, m.width = rgb.shape[0], rgb.shape[1]
    m.encoding = "rgb8"
    m.is_bigendian = 0
    m.step = m.width * 3
    m.data = rgb.tobytes()
    return m


def tf_msg(stamp, parent, child, t, q):
    ts = TransformStamped()
    ts.header.stamp = stamp
    ts.header.frame_id = parent
    ts.child_frame_id = child
    ts.transform.translation.x = float(t[0])
    ts.transform.translation.y = float(t[1])
    ts.transform.translation.z = float(t[2])
    ts.transform.rotation.w = float(q[0])
    ts.transform.rotation.x = float(q[1])
    ts.transform.rotation.y = float(q[2])
    ts.transform.rotation.z = float(q[3])
    return ts


def main():
    env, coll = resolve_paths()
    if not os.path.exists(env):
        carb.log_error(f"환경 없음: {env}"); sim_app.close(); sys.exit(1)
    root = get_assets_root_path()
    robot_usd = args.robot_usd if (args.robot_usd and os.path.isabs(args.robot_usd)) \
        else (root + ROBOT_PATH if root else None)
    if robot_usd is None:
        carb.log_error("자산 서버 못 찾음 → --robot-usd"); sim_app.close(); sys.exit(1)

    # World 생성 후 env를 reference (open_stage 금지 — PhysX 라이다가 콜라이더 못 봄)
    world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(env, SCENE_PRIM)
    stage = omni.usd.get_context().get_stage()

    # VisualMesh 숨김 → 카메라 = 순수 GS (PhysX 라이다는 콜라이더를 보므로 무관)
    vm = stage.GetPrimAtPath(VISUALMESH)
    if vm and vm.IsValid() and not args.show_visualmesh:
        UsdGeom.Imageable(vm).MakeInvisible()
        print("[sensor] VisualMesh 숨김 → 카메라=GS")

    spawn, yaw = detect_spawn(coll)
    spawn = spawn.copy(); spawn[2] += 0.4
    quat = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    robot = WheeledRobot(prim_path=ROBOT_PRIM, name="jackal", wheel_dof_names=WHEELS,
                         create_robot=True, usd_path=robot_usd,
                         position=spawn, orientation=quat)
    world.scene.add(robot)
    set_wheel_drive(stage)

    # PhysX LiDAR (base_link 하위 → 로봇과 함께 이동, 충돌 지오메트리 센싱)
    # Ouster급 설정: 360°×수직FOV, rotation_frequency=0 → 매 프레임 360° 전체 스캔
    # (회전형이면 매 프레임 일부만 → RViz에 1/4씩 보임)
    lidar = world.scene.add(RotatingLidarPhysX(
        prim_path=BASE_LINK + "/lidar", name="lidar",
        translation=np.array(LIDAR_OFFSET),
        rotation_frequency=0,
        fov=(360.0, args.lidar_vfov),
        resolution=(args.lidar_hres, args.lidar_vres),
        valid_range=(0.3, args.lidar_range)))

    # 카메라 prim + rgb 어노테이터 (GUI에서 GS 렌더)
    cam_path = BASE_LINK + "/camera"
    cam = UsdGeom.Camera(stage.DefinePrim(cam_path, "Camera"))
    cxf = UsdGeom.XformCommonAPI(cam)
    cxf.SetTranslate(Gf.Vec3d(*CAM_OFFSET))
    cxf.SetRotate((90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    cam.GetClippingRangeAttr().Set((0.05, 1000.0))
    cam_rp = rep.create.render_product(cam_path, (args.cam_w, args.cam_h), name="cam_rp")
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([cam_rp])

    world.reset()
    lidar.add_depth_data_to_frame()
    lidar.add_point_cloud_data_to_frame()
    world.play()

    rclpy.init()
    node = rclpy.create_node("isaac_sensor_drive")
    pc_pub = node.create_publisher(PointCloud2, "/point_cloud", qos_profile_sensor_data)
    img_pub = node.create_publisher(Image, "/rgb", qos_profile_sensor_data)
    clk_pub = node.create_publisher(Clock, "/clock", 10)
    tf_pub = node.create_publisher(TFMessage, "/tf", 10)

    print(f"[sensor] dof={list(robot.dof_names)} 스폰={spawn.tolist()} yaw={np.degrees(yaw):.0f}°")
    print("[sensor] 준비 완료. W/S 전후, A/D 회전, ESC 종료. (RViz Fixed Frame: world)")

    pressed, quit_f = set(), {"v": False}
    K = carb.input.KeyboardInput

    def on_key(e, *_):
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            pressed.add(e.input)
            if e.input == K.ESCAPE:
                quit_f["v"] = True
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(e.input)
        return True
    sub = in_if = kb = None
    try:
        aw = omni.appwindow.get_default_app_window()
        in_if = carb.input.acquire_input_interface()
        kb = aw.get_keyboard()
        sub = in_if.subscribe_to_keyboard_events(kb, on_key)
    except Exception as e:  # noqa: BLE001
        carb.log_warn(f"키보드 구독 실패({e}) — 자동 전진")

    side = np.array([-1.0 if "left" in j else 1.0 for j in WHEELS])

    def command():
        lin = ang = 0.0
        if K.W in pressed: lin += args.lin_speed
        if K.S in pressed: lin -= args.lin_speed
        if K.A in pressed: ang += args.ang_speed
        if K.D in pressed: ang -= args.ang_speed
        if (args.headless or sub is None) and not pressed: lin = args.lin_speed
        return lin, ang

    step = 0
    while sim_app.is_running() and not quit_f["v"]:
        lin, ang = command()
        vels = (lin + side * (ang * WHEEL_BASE / 2.0)) / WHEEL_R
        robot.apply_wheel_actions(ArticulationAction(joint_velocities=vels))
        world.step(render=True)

        stamp = to_time(world.current_time)
        clk_pub.publish(Clock(clock=stamp))

        pos, oq = robot.get_world_pose()
        tfm = TFMessage()
        tfm.transforms.append(tf_msg(stamp, "world", "base_link", pos, oq))
        tfm.transforms.append(tf_msg(stamp, "base_link", "lidar", LIDAR_OFFSET, [1, 0, 0, 0]))
        tfm.transforms.append(tf_msg(stamp, "base_link", "camera", CAM_OFFSET, [1, 0, 0, 0]))
        tf_pub.publish(tfm)

        # 저수준 인터페이스로 포인트클라우드 버퍼를 읽어 매 프레임 publish.
        # 회전 라이다라 프레임마다 부분 스윕이 나오므로, RViz의 Decay Time(>0)으로
        # 한 회전 분량을 누적시키면 전체 클라우드가 보인다(sensors.rviz에 설정됨).
        pc = lidar._lidar_sensor_interface.get_point_cloud_data(BASE_LINK + "/lidar")
        npc = 0
        if pc is not None:
            arr = np.asarray(pc, dtype=np.float32).reshape(-1, 3)
            arr = arr[np.isfinite(arr).all(axis=1)]
            npc = len(arr)
            if npc:
                pc_pub.publish(make_pc2(stamp, "lidar", arr))

        try:
            img = rgb_annot.get_data()
            if img is not None and np.asarray(img).size:
                img_pub.publish(make_img(stamp, "camera", img))
        except Exception:  # noqa: BLE001
            pass

        rclpy.spin_once(node, timeout_sec=0.0)
        step += 1
        if step % 60 == 0:
            print(f"[sensor] step={step} playing={world.is_playing()} "
                  f"pos=({pos[0]:.1f},{pos[1]:.1f}) lidar_pts={npc}")
        if args.headless and step > args.max_steps:
            break

    if sub is not None:
        in_if.unsubscribe_to_keyboard_events(kb, sub)
    node.destroy_node()
    rclpy.shutdown()
    sim_app.close()


if __name__ == "__main__":
    main()
