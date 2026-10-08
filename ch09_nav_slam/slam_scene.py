"""책 9.3 (b04·b05) SLAM 용 Isaac Sim 씬 — 동작 검증본.

SLAM Toolbox 가 요구하는 것은 셋이다. 하나라도 빠지면 시작조차 못 한다.

    /scan            2D LaserScan
    /odom            오도메트리
    odom → base_link TF

원문과의 차이:
  [ERR-049] b04 는 `/opt/ros/humble` 을 소싱하라고 한다. 같은 장 b02 는 jazzy 다.
            Ubuntu 24.04 에는 humble 이 없다.
  [ERR-055] IsaacComputeOdometry 는 core.nodes 소속이다 (wheeled_robots 아님).
  [ERR-058] 원고는 3D 라이다(Ouster OS)를 쓰면서 LaserScan 을 기대한다.
            LaserScan 은 **2D 형식**이다. Isaac Sim 에는 2D 라이다 설정이
            여럿 있으니(RPLIDAR_S2E, LMS4000, microScan3 …) 그중 하나를 쓴다.

## RTX 라이다 ROS2 발행에서 실제로 막혔던 것

발행자는 등록되는데 메시지가 한 건도 안 나오는 상태를 오래 겪었다.
동작하는 구성은 다음과 같다.

  · 렌더 프로덕트를 **그래프 안에서** IsaacCreateRenderProduct 로 만든다
    (`rep.create.render_product()` 로 파이썬에서 만들면 안 된다)
  · 사슬은 OnPlaybackTick → OgnIsaacRunOneSimulationFrame
                          → IsaacCreateRenderProduct → ROS2RtxLidarHelper
  · `Lidar.create()` 가 만드는 것은 Xform 이다. 실제 OmniLidar prim 은
    그 아래에 있으므로 **타입으로 찾아** cameraPrim 에 준다
  · 라이다와 그래프를 **world.reset() 전에** 구성한다

실행:
    conda activate isaac_env
    source ../ch08_ros2/setup_ros2_env.sh
    python slam_scene.py

다른 터미널 — SLAM 실행:
    source /opt/ros/jazzy/setup.bash        # [ERR-049] humble 아님
    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ros2 launch slam_toolbox online_async_launch.py \\
        slam_params_file:=$PWD/slam_params.yaml use_sim_time:=true

확인:
    ros2 topic hz /map
    ros2 run nav2_map_server map_saver_cli -f ~/maps/room
"""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "1") == "1"})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
app.update()

import omni.graph.core as og  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationActions  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, Vt  # noqa: E402

import omni.usd  # noqa: E402

World.clear_instance()
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
nucleus = nucleus_utils.get_assets_root_path()

print("\n===SLAMSCENE===", flush=True)

# ── 지도를 그릴 방 ───────────────────────────────────────────────
ROOM_X, ROOM_Y, WALL_T = 12.0, 9.0, 0.3
stage.DefinePrim("/World/Room", "Xform")


def box(name, cx, cy, sx, sy, col):
    c = UsdGeom.Cube.Define(stage, f"/World/Room/{name}")
    c.GetSizeAttr().Set(1.0)
    c.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*col)]))
    a = UsdGeom.XformCommonAPI(c.GetPrim())
    a.SetTranslate(Gf.Vec3d(cx, cy, 1.0))
    a.SetScale(Gf.Vec3f(sx, sy, 2.0))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())


W = (0.72, 0.74, 0.78)
box("N", 0, ROOM_Y / 2, ROOM_X, WALL_T, W)
box("S", 0, -ROOM_Y / 2, ROOM_X, WALL_T, W)
box("E", ROOM_X / 2, 0, WALL_T, ROOM_Y, W)
box("W", -ROOM_X / 2, 0, WALL_T, ROOM_Y, W)
# 안쪽 구조물 — 지도에 특징이 있어야 루프 클로저가 걸린다
box("P1", -3.0, 1.5, 1.0, 1.0, (0.8, 0.35, 0.3))
box("P2", 3.0, -1.5, 1.0, 1.0, (0.3, 0.5, 0.8))
box("P3", 0.0, 2.8, 2.5, 0.4, (0.85, 0.7, 0.3))

# ── 로봇 ─────────────────────────────────────────────────────────
CARTER = "/World/Carter"
stage_utils.add_reference_to_stage(
    usd_path=nucleus + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
    prim_path=CARTER)

# 섀시 링크를 먼저 찾는다 — 라이다를 여기에 **달아야** 한다.
CHASSIS = next(
    (str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(CARTER))
     if "chassis" in p.GetName().lower()), CARTER)

# ── [ERR-058] 2D 라이다 — LaserScan 은 2D 형식이다 ───────────────
# ★ 라이다를 **로봇 섀시 아래에** 만든다.
# 월드에 따로 두면 로봇은 움직이는데 스캔은 제자리에서 나온다.
# 그러면 SLAM 이 스캔과 오도메트리를 맞추지 못해 지도가 여러 각도로 겹쳐 그려진다.
#
# ── [ERR-069] 높이를 눈대중으로 정하면 라이다가 로봇 몸통 안에 묻힌다 ──
# z=0.30 으로 두었더니 3200 빔 중 451 개만 반사가 왔고, 정면 0° 구간은
# 반사가 **한 개도** 없었다. 몸통이 빔을 막고 두 틈으로만 새어 나온 것이다.
# 앞이 안 보이니 Nav2 는 장애물을 못 찍고 그대로 들이받는다.
# 눈대중 대신 로봇 **바운딩박스를 재서** 그 위에 올린다.
_bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
_rng = _bbox.ComputeWorldBound(stage.GetPrimAtPath(CARTER)).ComputeAlignedRange()
_chassis_z = UsdGeom.Xformable(stage.GetPrimAtPath(CHASSIS)) \
    .ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2]
LIDAR_Z = _rng.GetMax()[2] - _chassis_z + 0.05      # 몸통 꼭대기보다 5 cm 위
print(f"로봇 높이 {_rng.GetMin()[2]:.3f}~{_rng.GetMax()[2]:.3f} m, "
      f"섀시 z={_chassis_z:.3f} → 라이다 z={LIDAR_Z:+.3f} (섀시 기준)", flush=True)

LIDAR_ROOT = f"{CHASSIS}/Lidar"
Lidar.create(LIDAR_ROOT, config="RPLIDAR_S2E")
UsdGeom.XformCommonAPI(stage.GetPrimAtPath(LIDAR_ROOT)).SetTranslate(
    Gf.Vec3d(0.0, 0.0, LIDAR_Z))
# Lidar.create 는 Xform 을 만든다. 실제 OmniLidar 는 그 아래에 있다.
OMNI_LIDAR = next(
    (str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(LIDAR_ROOT))
     if p.GetTypeName() == "OmniLidar"), None)
assert OMNI_LIDAR, "OmniLidar prim 을 찾지 못했다"
print(f"라이다 RPLIDAR_S2E (2D)  OmniLidar: {OMNI_LIDAR}", flush=True)

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr().Set(1200.0)

keys = og.Controller.Keys

# ── /scan — 공식 6노드 구조 ──────────────────────────────────────
og.Controller.edit(
    {"graph_path": "/World/ScanGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            # 이 노드만 등록 이름에 Ogn 접두사가 포함된다
            ("run", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("rp", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("scan_pub", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ],
        keys.SET_VALUES: [
            ("rp.inputs:cameraPrim", [OMNI_LIDAR]),
            ("scan_pub.inputs:topicName", "/scan"),
            ("scan_pub.inputs:frameId", "base_link"),   # SLAM 이 TF 로 찾는 프레임
            ("scan_pub.inputs:type", "laser_scan"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "run.inputs:execIn"),
            ("run.outputs:step", "rp.inputs:execIn"),
            ("rp.outputs:execOut", "scan_pub.inputs:execIn"),
            ("rp.outputs:renderProductPath", "scan_pub.inputs:renderProductPath"),
            ("ctx.outputs:context", "scan_pub.inputs:context"),
        ],
    })

# ── /odom + odom→base_link TF + /clock ───────────────────────────
og.Controller.edit(
    {"graph_path": "/World/OdomGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            # [ERR-055] core.nodes 소속이다 (wheeled_robots 아님)
            ("odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("odom_pub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("tf_pub", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            # use_sim_time:=true 를 쓰려면 /clock 이 있어야 한다.
            # 원고는 use_sim_time 을 켜라고만 하고 /clock 발행은 말하지 않는다.
            ("clock_pub", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ],
        keys.SET_VALUES: [
            ("odom.inputs:chassisPrim", [CHASSIS]),
            ("odom_pub.inputs:topicName", "/odom"),
            ("odom_pub.inputs:odomFrameId", "odom"),
            ("odom_pub.inputs:chassisFrameId", "base_link"),
            ("tf_pub.inputs:parentFrameId", "odom"),
            ("tf_pub.inputs:childFrameId", "base_link"),
            ("clock_pub.inputs:topicName", "/clock"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "odom.inputs:execIn"),
            ("tick.outputs:tick", "odom_pub.inputs:execIn"),
            ("tick.outputs:tick", "tf_pub.inputs:execIn"),
            ("tick.outputs:tick", "clock_pub.inputs:execIn"),
            ("odom.outputs:angularVelocity", "odom_pub.inputs:angularVelocity"),
            ("odom.outputs:linearVelocity", "odom_pub.inputs:linearVelocity"),
            ("odom.outputs:position", "odom_pub.inputs:position"),
            ("odom.outputs:orientation", "odom_pub.inputs:orientation"),
            ("odom.outputs:position", "tf_pub.inputs:translation"),
            ("odom.outputs:orientation", "tf_pub.inputs:rotation"),
            ("sim_time.outputs:simulationTime", "clock_pub.inputs:timeStamp"),
            # ★ TF 와 오도메트리에도 **같은 시뮬레이션 시각**을 넣어야 한다.
            # 빠뜨리면 SLAM 이 스캔과 TF 의 시각을 맞추지 못해 스캔을 버린다:
            #   Message Filter dropping message: frame 'base_link' ...
            #   'discarding message because the queue is full'
            # 원고도 이 연결이 없다.
            ("sim_time.outputs:simulationTime", "tf_pub.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "odom_pub.inputs:timeStamp"),
        ],
    })

# ── /cmd_vel 구독 → 차동 구동 (책 b04 의 teleop 실습) ───────────
# 원고 b04 는 teleop_twist_keyboard 로 로봇을 몰라고 한다.
# 그러려면 /cmd_vel 을 받아 바퀴를 돌리는 사슬이 있어야 한다.
# [ERR-052] 노드 이름은 DifferentialController 다
#           (원고 8장의 IsaacComputeDifferentialDriveCommand 는 없다).
WHEEL_JOINTS = ["joint_wheel_left", "joint_wheel_right"]

og.Controller.edit(
    {"graph_path": "/World/CmdVelGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("cmd_sub", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            # [ERR-059] ROS2SubscribeTwist 는 **vectord[3]** 을 내는데
            # DifferentialController 는 **double** 을 받는다. 직접 이으면
            # 연결은 되지만 값이 넘어가지 않아 wheelCmd 가 [0, 0] 으로 남는다.
            # BreakVector3 로 성분을 뽑아 이어야 한다.
            ("lin_xyz", "omni.graph.nodes.BreakVector3"),
            ("ang_xyz", "omni.graph.nodes.BreakVector3"),
            ("diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("art", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("cmd_sub.inputs:topicName", "/cmd_vel"),
            # NovaCarter 실측값
            ("diff.inputs:wheelRadius", 0.14),
            ("diff.inputs:wheelDistance", 0.41),
            ("diff.inputs:maxLinearSpeed", 1.0),
            ("diff.inputs:maxAngularSpeed", 1.5),
            ("art.inputs:targetPrim", [CARTER]),
            ("art.inputs:jointNames", WHEEL_JOINTS),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "cmd_sub.inputs:execIn"),
            ("tick.outputs:tick", "diff.inputs:execIn"),
            # ★ dt 를 이어 주지 않으면 기본값 0.0 이라
            # velocityCommand 가 [0.0, 0.0] 로 나와 바퀴가 돌지 않는다.
            # 명령은 도달하는데 로봇만 안 움직여서 원인을 찾기 어렵다.
            ("tick.outputs:deltaSeconds", "diff.inputs:dt"),
            ("tick.outputs:tick", "art.inputs:execIn"),
            # 벡터 → 스칼라: 전진은 x 성분, 회전은 z 성분
            ("cmd_sub.outputs:linearVelocity", "lin_xyz.inputs:tuple"),
            ("cmd_sub.outputs:angularVelocity", "ang_xyz.inputs:tuple"),
            ("lin_xyz.outputs:x", "diff.inputs:linearVelocity"),
            ("ang_xyz.outputs:z", "diff.inputs:angularVelocity"),
            ("diff.outputs:velocityCommand", "art.inputs:velocityCommand"),
        ],
    })

# 뷰포트를 방 전체가 보이게 잡는다.
# 기본 카메라는 고정이라 로봇이 움직이면 곧 화면 밖으로 나간다.
# 자율주행을 눈으로 확인하거나 녹화할 때 이게 없으면 아무것도 안 보인다.
if os.environ.get("HEADLESS", "1") != "1":
    from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
    set_camera_view(eye=np.array([0.0, -13.0, 14.0]),
                    target=np.array([0.0, 0.0, 0.0]))

# ─────────────── 움직이는 장애물 — 원고 b10 마지막 체크리스트용
#
#   "✅ 이동 중 장애물(다른 박스)을 추가해도 경로를 재계획한다"
#
# 실행 중인 씬에 물체를 넣으려면 **밖에서 조종할 수단**이 있어야 한다.
# ROS 토픽으로 위치를 받아 옮기는 상자를 하나 둔다. 처음에는 방 밖에 둔다.
#
#   ros2 topic pub --once /obstacle_pose geometry_msgs/msg/Twist \
#     "{linear: {x: 2.0, y: 1.2}}"      # 로봇 앞을 막는다
OBSTACLE = "/World/MovableObstacle"
_ob = UsdGeom.Cube.Define(stage, OBSTACLE)
_ob.GetSizeAttr().Set(1.0)
_oba = UsdGeom.XformCommonAPI(_ob.GetPrim())
_oba.SetTranslate(Gf.Vec3d(0.0, 0.0, -50.0))      # 처음엔 바닥 아래(안 보임)
_oba.SetScale(Gf.Vec3f(0.8, 0.8, 1.2))
_ob.GetDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.2, 0.2)])
UsdPhysics.CollisionAPI.Apply(_ob.GetPrim())

og.Controller.edit(
    {"graph_path": "/World/ObstacleGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("ob_sub", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ],
        keys.SET_VALUES: [("ob_sub.inputs:topicName", "obstacle_pose")],
        keys.CONNECT: [
            ("tick.outputs:tick", "ob_sub.inputs:execIn"),
            ("ctx.outputs:context", "ob_sub.inputs:context"),
        ],
    })
OB_XY = og.Controller.attribute("/World/ObstacleGraph/ob_sub.outputs:linearVelocity")

world.reset()
app_utils.play()

carter = Articulation(prim_paths_expr=CARTER, name="carter")
carter.initialize()
names = list(carter.dof_names)
wheels = [i for i, n in enumerate(names) if "wheel" in n.lower()]

print(f"\n발행 시작 — /scan · /odom · /tf · /clock", flush=True)
print(f"방 {ROOM_X}×{ROOM_Y} m, 내부 구조물 3개, 바퀴 {len(wheels)}개", flush=True)
print(f"\nSLAM 실행 (다른 터미널):", flush=True)
print(f"  ros2 launch slam_toolbox online_async_launch.py \\", flush=True)
print(f"      slam_params_file:=$PWD/slam_params.yaml use_sim_time:=true", flush=True)
print(f"\n로봇 조종 (또 다른 터미널):", flush=True)
print(f"  ros2 run teleop_twist_keyboard teleop_twist_keyboard", flush=True)
print(f"  i=앞  ,=뒤  j=좌회전  l=우회전  k=정지  q/z=속도조절", flush=True)

s = 0
try:
    _ob_last = None
    while True:
        world.step(render=True)
        s += 1

        # /obstacle_pose 로 장애물을 옮긴다 (b10 재계획 시험용).
        # (0,0) 은 "치운다" 로 보고 바닥 아래로 내린다.
        _xy = OB_XY.get()
        _cur = (round(float(_xy[0]), 2), round(float(_xy[1]), 2))
        if _cur != _ob_last:
            _ob_last = _cur
            _z = 0.6 if (abs(_cur[0]) > 0.01 or abs(_cur[1]) > 0.01) else -50.0
            UsdGeom.XformCommonAPI(stage.GetPrimAtPath(OBSTACLE)).SetTranslate(
                Gf.Vec3d(_cur[0], _cur[1], _z))
            print(f"  [장애물] ({_cur[0]:+.2f}, {_cur[1]:+.2f}) "
                  f"{'배치' if _z > 0 else '치움'}", flush=True)
        if s % 180 == 0 and os.environ.get("DEBUG_CMDVEL") == "1":
            def _g(attr):
                try:
                    return og.Controller.get(og.Controller.attribute(attr))
                except Exception as e:
                    return f"<{type(e).__name__}>"
            print(f"  [{s:>5}] cmd_vel lin={_g('/World/CmdVelGraph/cmd_sub.outputs:linearVelocity')} "
                  f"ang={_g('/World/CmdVelGraph/cmd_sub.outputs:angularVelocity')} | "
                  f"dt={_g('/World/CmdVelGraph/diff.inputs:dt')} "
                  f"wheelCmd={_g('/World/CmdVelGraph/diff.outputs:velocityCommand')}", flush=True)
        elif s % 900 == 0:
            print(f"  ... {s} 스텝  (teleop 으로 로봇을 모세요)", flush=True)
except KeyboardInterrupt:
    print(f"\n종료 ({s} 스텝)", flush=True)
print("===SLAMSCENE_END===", flush=True)
app.close()
