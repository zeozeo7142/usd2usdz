"""
teleop_test.py — Isaac Sim 5.1에서 휠로봇을 스폰해 평탄 바닥 collision 검증 (WASD 텔레옵)

make_collision_env.py가 만든 _robot.usda 환경을 열고, Isaac 내장 휠로봇을
검출된 바닥 위에 스폰한 뒤 키보드(WASD)로 주행시킨다. 로봇이 평탄 바닥에
안착하는지, 바닥 노이즈에 걸려 튀지 않는지, 벽/계단에 막히는지 눈과 수치로
확인할 수 있다.

실행 (Isaac Sim 5.1 컨테이너 내부, GUI):
  cd /home/zeozeo/git/usd2usdz
  /isaac-sim/python.sh teleop_test.py --index 1
  /isaac-sim/python.sh teleop_test.py --index 2 --robot nova_carter
  /isaac-sim/python.sh teleop_test.py --env output/USDZ_ETRI1/260521_ERTI_1_robot.usda \
                                      --robot-usd /home/zeozeo/robots/jackal/jackal.usd \
                                      --wheel-joints front_left_wheel,front_right_wheel \
                                      --wheel-radius 0.098 --wheel-base 0.37

조작:
  W / S : 전진 / 후진
  A / D : 좌회전 / 우회전
  Space : 정지
  R     : 스폰 위치로 리셋
  ESC   : 종료

주의:
  - 내장 로봇 에셋은 Isaac 자산 서버(인터넷/Nucleus)에서 받는다. 회사망에서
    막히면 --robot-usd 로 로컬 로봇 USD를 직접 지정한다.
  - 조인트 이름이 다르면 apply_wheel_actions가 실패한다. 스크립트가 로봇의
    실제 dof 이름을 출력하므로, 어긋나면 --wheel-joints 로 맞춘다.
"""

import argparse
import sys

# --------------------------------------------------------------------------- #
# 1) 인자 파싱 — SimulationApp 생성 '전'에 끝내야 한다
# --------------------------------------------------------------------------- #
parser = argparse.ArgumentParser(description="휠로봇 WASD 텔레옵 바닥 검증")
parser.add_argument("--index", type=int, choices=[1, 2, 3],
                    help="ETRI 인덱스 (output/USDZ_ETRI<N>/..._robot.usda 자동 경로)")
parser.add_argument("--env", default=None,
                    help="_robot.usda 경로 직접 지정 (--index 대신)")
parser.add_argument("--output-dir", default="output")
parser.add_argument("--robot", default="carter",
                    choices=["jetbot", "carter", "nova_carter"],
                    help="내장 휠로봇 프리셋 (기본 carter)")
parser.add_argument("--robot-usd", default=None,
                    help="로봇 USD 오버라이드 (절대경로 또는 자산서버 상대경로)")
parser.add_argument("--wheel-joints", default=None,
                    help="구동 바퀴 조인트 이름 (쉼표 구분). 프리셋 기본값 오버라이드")
parser.add_argument("--wheel-radius", type=float, default=None)
parser.add_argument("--wheel-base", type=float, default=None)
parser.add_argument("--spawn", default=None,
                    help="스폰 위치 'x,y,z' (기본: 검출 바닥 중앙 위)")
parser.add_argument("--lin-speed", type=float, default=1.0,
                    help="전진 속도 m/s")
parser.add_argument("--ang-speed", type=float, default=1.5,
                    help="회전 속도 rad/s")
parser.add_argument("--headless", action="store_true",
                    help="GUI 없이 실행 (자동 주행 + 로깅만)")
parser.add_argument("--no-visual", action="store_true",
                    help="충돌 지오메트리만 로드 (GS/메쉬 생략, 빠른 물리 테스트)")
args = parser.parse_args()

# --------------------------------------------------------------------------- #
# 2) SimulationApp 부팅 (다른 isaac/omni import보다 반드시 먼저)
# --------------------------------------------------------------------------- #
from isaacsim import SimulationApp                      # noqa: E402

sim_app = SimulationApp({"headless": args.headless})

# --------------------------------------------------------------------------- #
# 3) 부팅 후 모듈 import
# --------------------------------------------------------------------------- #
import os                                               # noqa: E402
import numpy as np                                      # noqa: E402
import carb                                             # noqa: E402
import carb.input                                       # noqa: E402
import omni.appwindow                                   # noqa: E402
from pxr import Usd, UsdGeom                            # noqa: E402

from isaacsim.core.api import World                     # noqa: E402
from isaacsim.core.utils.stage import open_stage, \
    add_reference_to_stage                              # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from isaacsim.robot.wheeled_robots.robots import WheeledRobot  # noqa: E402
from isaacsim.robot.wheeled_robots.controllers.differential_controller \
    import DifferentialController                       # noqa: E402

import functools                                         # noqa: E402
print = functools.partial(print, flush=True)            # fastShutdown 시 로그 유실 방지

# --------------------------------------------------------------------------- #
# 로봇 프리셋 (자산서버 상대경로 / 조인트 / 바퀴 파라미터)
# 경로·조인트가 버전에 따라 다를 수 있으니 어긋나면 CLI로 교체한다.
# --------------------------------------------------------------------------- #
# 경로/조인트는 Isaac Sim 5.1 자산 서버에서 실측 확인됨 (verify2.py)
ROBOTS = {
    "jetbot": {
        "path": "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
        "joints": ["left_wheel_joint", "right_wheel_joint"],
        "wheel_radius": 0.0325, "wheel_base": 0.1125,
    },
    "carter": {   # 기본값 — 건물 스케일 적합, 검증 완료
        "path": "/Isaac/Robots/NVIDIA/Carter/carter_v1.usd",
        "joints": ["left_wheel", "right_wheel"],
        "wheel_radius": 0.24, "wheel_base": 0.413,
    },
    "nova_carter": {
        "path": "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
        "joints": ["joint_wheel_left", "joint_wheel_right"],
        "wheel_radius": 0.14, "wheel_base": 0.413,
    },
    "jackal": {   # 스키드스티어 4륜 — DifferentialController는 앞 2륜만 구동
        "path": "/Isaac/Robots/Clearpath/Jackal/jackal.usd",
        "joints": ["front_left_wheel_joint", "front_right_wheel_joint"],
        "wheel_radius": 0.098, "wheel_base": 0.37,
    },
}

# 출력 디렉토리 매핑 (make_collision_env.py / build_combined_usd.py와 일치)
ERTI_OUT = {
    1: ("output/USDZ_ETRI1", "260521_ERTI_1"),
    2: ("output/USDZ_ETRI2", "260521_ERTI_2"),
    3: ("output/USDZ_ETRI3", "260521_ERTI_3"),
}


def resolve_paths():
    """--index 또는 --env 로부터 env(.usda)와 collision(.usdc) 경로를 만든다."""
    if args.env:
        env_path = os.path.abspath(args.env)
        d = os.path.dirname(env_path)
        base = os.path.basename(env_path).replace("_robot.usda", "")
        coll = os.path.join(d, f"{base}_collision.usdc")
    elif args.index:
        out_dir, base = ERTI_OUT[args.index]
        # 상대경로는 스크립트 위치 기준으로 앵커 (컨테이너 cwd가 /isaac-sim이어도 동작)
        if not os.path.isabs(out_dir):
            out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   out_dir)
        env_path = os.path.abspath(os.path.join(out_dir, f"{base}_robot.usda"))
        coll = os.path.abspath(os.path.join(out_dir, f"{base}_collision.usdc"))
    else:
        carb.log_error("--index 또는 --env 중 하나가 필요합니다.")
        sim_app.close(); sys.exit(1)
    if not os.path.exists(env_path):
        carb.log_error(f"환경 파일 없음: {env_path}")
        sim_app.close(); sys.exit(1)
    return env_path, coll


def detect_spawn(coll_path):
    """make_collision_env.py가 collision.usdc에 저장한 '열린 바닥' 스폰 지점을 읽는다.
    (없으면 가장 낮은 바닥 슬랩/메쉬 중심으로 폴백)"""
    if args.spawn:
        x, y, z = [float(v) for v in args.spawn.split(",")]
        return np.array([x, y, z]), z
    try:
        st = Usd.Stage.Open(coll_path)
        root = st.GetPrimAtPath("/Colliders")
        attr = root.GetAttribute("usd2usdz:spawnPoint") if root else None
        if attr and attr.HasValue():
            sp = attr.Get()
            return np.array([float(sp[0]), float(sp[1]), float(sp[2])]), float(sp[2])
        # 폴백: 가장 낮은 Floor(슬랩 Cube 또는 메쉬) 중심
        cands = []
        for p in st.Traverse():
            if not p.GetName().startswith("Floor"):
                continue
            if p.GetTypeName() == "Cube":
                for op in UsdGeom.Xformable(p).GetOrderedXformOps():
                    if "translate" in op.GetOpName():
                        t = op.Get()
                        cands.append((float(t[0]), float(t[1]), float(t[2])))
            elif p.GetTypeName() == "Mesh":
                a = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
                cands.append((float(a[:, 0].mean()), float(a[:, 1].mean()),
                              float(a[:, 2].mean())))
        cands.sort(key=lambda c: c[2])
        cx, cy, fz = cands[0]
        return np.array([cx, cy, fz]), fz
    except Exception as e:                              # noqa: BLE001
        carb.log_warn(f"바닥 자동검출 실패({e}) → 원점 위 1m 스폰")
        return np.array([0.0, 0.0, 1.0]), 0.0


def main():
    env_path, coll_path = resolve_paths()
    preset = dict(ROBOTS[args.robot])
    if args.wheel_joints:
        preset["joints"] = [s for s in args.wheel_joints.split(",") if s]
    if args.wheel_radius is not None:
        preset["wheel_radius"] = args.wheel_radius
    if args.wheel_base is not None:
        preset["wheel_base"] = args.wheel_base

    # 로봇 USD 경로 확정
    if args.robot_usd and os.path.isabs(args.robot_usd):
        robot_usd = args.robot_usd
    else:
        root = get_assets_root_path()
        if root is None:
            carb.log_error(
                "Isaac 자산 서버 경로를 찾지 못했습니다. 인터넷/Nucleus가 "
                "막혀 있으면 --robot-usd 로 로컬 로봇 USD를 지정하세요.")
            sim_app.close(); sys.exit(1)
        robot_usd = root + (args.robot_usd or preset["path"])
    print(f"[teleop] 환경 : {env_path}")
    print(f"[teleop] 로봇 : {robot_usd}")
    print(f"[teleop] 조인트: {preset['joints']}  "
          f"r={preset['wheel_radius']} base={preset['wheel_base']}")

    # 환경 로드 + World
    #   기본: 전체 _robot.usda (GS + 메쉬 + 충돌) 로드
    #   --no-visual: 충돌(collision.usdc)만 로드 — GS(366MB) 생략, 빠른 물리 테스트
    if args.no_visual:
        world = World(stage_units_in_meters=1.0)
        add_reference_to_stage(coll_path, "/World/Colliders")
        print("[teleop] --no-visual: 충돌 지오메트리만 로드 (GS/메쉬 생략)")
    else:
        open_stage(env_path)
        world = World(stage_units_in_meters=1.0)

    spawn_pos, floor_z = detect_spawn(coll_path)
    spawn_pos = spawn_pos.copy()
    spawn_pos[2] = floor_z + max(0.3, preset["wheel_radius"] * 2.0)  # 살짝 위
    print(f"[teleop] 스폰 : {spawn_pos.tolist()} (floor_z={floor_z:.3f})")

    robot = WheeledRobot(
        prim_path="/World/TeleopRobot",
        name="teleop_robot",
        wheel_dof_names=preset["joints"],
        create_robot=True,
        usd_path=robot_usd,
        position=spawn_pos,
    )
    world.scene.add(robot)
    world.reset()
    world.play()                       # standalone에서 타임라인 시작(없으면 안 움직임)

    print(f"[teleop] 로봇 dof 이름: {list(robot.dof_names)}")
    controller = DifferentialController(
        name="diff", wheel_radius=preset["wheel_radius"],
        wheel_base=preset["wheel_base"])

    # ----------------------------------------------------------------- #
    # 키보드 입력 (눌린 키 집합 유지)
    # ----------------------------------------------------------------- #
    pressed = set()
    want_reset = {"v": False}
    want_quit = {"v": False}
    K = carb.input.KeyboardInput

    def on_key(event, *_):
        et = event.type
        if et == carb.input.KeyboardEventType.KEY_PRESS:
            pressed.add(event.input)
            if event.input == K.R:
                want_reset["v"] = True
            elif event.input == K.ESCAPE:
                want_quit["v"] = True
        elif et == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(event.input)
        return True

    in_iface = kb = sub = None
    try:
        app_window = omni.appwindow.get_default_app_window()
        in_iface = carb.input.acquire_input_interface()
        kb = app_window.get_keyboard()
        sub = in_iface.subscribe_to_keyboard_events(kb, on_key)
    except Exception as e:                              # noqa: BLE001
        carb.log_warn(f"키보드 구독 실패({e}) — 자동 주행 모드로 진행")

    def command():
        lin = ang = 0.0
        if K.W in pressed:
            lin += args.lin_speed
        if K.S in pressed:
            lin -= args.lin_speed
        if K.A in pressed:
            ang += args.ang_speed
        if K.D in pressed:
            ang -= args.ang_speed
        if K.SPACE in pressed:
            lin = ang = 0.0
        # headless/키보드없음이면 자동 전진 (스모크 테스트용)
        if args.headless or sub is None:
            lin = args.lin_speed
        return lin, ang

    print("[teleop] 준비 완료. W/S 전후, A/D 회전, Space 정지, R 리셋, ESC 종료")

    step = 0
    while sim_app.is_running() and not want_quit["v"]:
        world.step(render=not args.headless)

        if want_reset["v"]:
            try:
                robot.set_world_pose(position=spawn_pos)
                for fn in ("set_linear_velocity", "set_angular_velocity"):
                    if hasattr(robot, fn):
                        getattr(robot, fn)(np.zeros(3))
            except Exception as e:                      # noqa: BLE001
                carb.log_warn(f"리셋 실패: {e}")
            want_reset["v"] = False

        lin, ang = command()
        robot.apply_wheel_actions(controller.forward([lin, ang]))

        # 약 1초마다 로봇 높이 로깅 (바닥 안착/관통/튐 확인)
        step += 1
        if step % 60 == 0:
            pos, _ = robot.get_world_pose()
            dz = pos[2] - floor_z
            flag = ""
            if pos[2] < floor_z - 0.5:
                flag = "  ⚠️ 바닥 관통(fell through)!"
            elif dz > 1.5:
                flag = "  ⚠️ 떠 있음/튐?"
            print(f"  t={step/60:5.1f}s  pos=({pos[0]:.2f},{pos[1]:.2f},"
                  f"{pos[2]:.2f})  바닥+{dz:.2f}m{flag}")

        if args.headless and step > 60 * 20:    # headless는 20초 후 종료
            break

    if sub is not None:
        in_iface.unsubscribe_to_keyboard_events(kb, sub)
    sim_app.close()


if __name__ == "__main__":
    main()
