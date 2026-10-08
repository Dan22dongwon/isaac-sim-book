"""책 8.4~8.6 (b07~b10) 동작 검증본 — LiDAR · 관절상태 · TF2 · cmd_vel.

원문과의 차이:
  [ERR-050] b07: ROS2RtxLidarHelper 에 `lidarPrimPath` 입력이 **없다.**
            카메라 헬퍼와 같이 `renderProductPath` 를 쓴다.
  [ERR-051] b08: IsaacArticulationController 에 `usePath` 입력이 **없다.**
            대상 지정은 `targetPrim` 또는 `robotPath` 로 한다.
  [ERR-052] b10: `isaacsim.robot.wheeled_robots.IsaacComputeDifferentialDriveCommand`
            라는 노드가 **없다.** 확장 이름도 `wheeled_robots.nodes` 로 바뀌었다.
            실제 노드는 `isaacsim.robot.wheeled_robots.DifferentialController`.
  [ERR-048] sim_time 노드·timestamp 연결 문제는 8.3 과 동일 (b08 에도 있다).

실행:
    conda activate isaac_env
    source setup_ros2_env.sh
    python ros2_full_bridge.py

확인 (다른 터미널):
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ros2 topic list
    ros2 topic echo /joint_states --once
    ros2 topic echo /tf --once
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "1") == "1"})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
app_utils.enable_extension("isaacsim.robot.wheeled_robots.nodes")
app.update()

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

World.clear_instance()
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
nucleus = nucleus_utils.get_assets_root_path()

RESULT = []


def step(tag, fn):
    try:
        note = fn()
        RESULT.append(("✅", tag))
        print(f"  ✅ {tag}\n      {note}", flush=True)
    except Exception as e:
        RESULT.append(("❌", tag))
        print(f"  ❌ {tag}\n      {type(e).__name__}: {str(e)[:170]}", flush=True)


print("\n===FULLBRIDGE===", flush=True)

# ── 로봇: Franka (관절상태·TF 용) ────────────────────────────────
FRANKA = "/World/Franka"
stage_utils.add_reference_to_stage(
    usd_path=nucleus + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    prim_path=FRANKA)

# LiDAR 가 맞출 벽 — 허공이면 반사가 없어 스캔 데이터가 비어 보인다.
# 화면으로 볼 때도 로봇이 방 안에 있는 편이 알아보기 쉽다.
from pxr import Vt  # noqa: E402

for i, (x, y, c) in enumerate([(3.5, 0, (0.75, 0.3, 0.3)),
                               (-3.5, 0, (0.3, 0.55, 0.8)),
                               (0, 3.5, (0.85, 0.7, 0.25)),
                               (0, -3.5, (0.35, 0.65, 0.4))]):
    w_ = UsdGeom.Cube.Define(stage, f"/World/Wall{i}")
    w_.GetSizeAttr().Set(2.0)
    w_.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*c)]))
    UsdGeom.XformCommonAPI(w_.GetPrim()).SetTranslate(Gf.Vec3d(x, y, 1.0))

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr().Set(1200.0)
key = UsdLux.RectLight.Define(stage, "/World/Key")
key.CreateIntensityAttr().Set(15000.0)
key.CreateWidthAttr().Set(8.0)
key.CreateHeightAttr().Set(8.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(2.0, -3.0, 5.0))

world.reset()

if os.environ.get("HEADLESS", "1") == "0":
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[4.5, -5.5, 3.5], target=[0.0, 0.0, 0.6])
    except Exception as e:
        print(f"(뷰포트 이동 실패: {e})", flush=True)

keys = og.Controller.Keys


# ── b07: LiDAR ROS2 발행 ─────────────────────────────────────────
def _b07():
    from pxr import Usd
    from isaacsim.sensors.experimental.rtx import Lidar
    # [ERR-034] config 이름에 제조사 접두사를 붙이지 않는다 (7장에서 확인)
    Lidar.create("/World/Lidar", config="OS1")
    UsdGeom.XformCommonAPI(
        stage.GetPrimAtPath("/World/Lidar")).SetTranslate(Gf.Vec3d(0, 0, 0.5))

    # ⚠️ Lidar.create 가 만드는 것은 **Xform** 이고, 실제 OmniLidar prim 은
    # 그 아래에 따로 있다 (/World/Lidar/sensor). 하우징 메시·재질과 함께
    # 계층으로 구성되기 때문이다. Xform 에 렌더 프로덕트를 붙이면:
    #   Warning: Render product ... not attached to RTX Lidar
    #            (OmniLidar prim is required)
    # 이름을 가정하지 말고 **타입으로 찾는다.**
    omni_lidar = next(
        (str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Lidar"))
         if p.GetTypeName() == "OmniLidar"), None)
    assert omni_lidar, "OmniLidar prim 을 찾지 못했다"

    # [ERR-054] ★ 렌더 프로덕트를 **파이썬에서 만들면 안 된다.**
    # rep.create.render_product() 로 만들어 renderProductPath 에 넣으면
    # 발행자는 등록되지만 **데이터가 한 건도 흐르지 않는다.**
    # 공식 그래프(Tools > Robotics > ROS 2 OmniGraphs > RTX Lidar)는
    # 그래프 **안에서** IsaacCreateRenderProduct 노드로 만든다.
    #   OnPlaybackTick → IsaacRunOneSimulationFrame → IsaacCreateRenderProduct
    #                  → ROS2RtxLidarHelper
    # 원고 b07 은 OnPlaybackTick + 헬퍼 두 개뿐이라 이 사슬이 통째로 빠져 있다.
    og.Controller.edit(
        {"graph_path": "/World/LiDARROS2Graph", "evaluator_name": "execution"},
        {
            # [ERR-053] 원고는 노드 하나로 "LaserScan + PointCloud2 동시 발행" 이라
            # 하지만, `type` 은 laser_scan **또는** point_cloud 둘 중 하나다.
            # 둘 다 원하면 **헬퍼 노드를 두 개** 만들어야 한다.
            # [ERR-057] 헬퍼 **하나당 렌더 프로덕트 하나**를 줘야 한다.
            # 헬퍼 둘이 같은 렌더 프로덕트를 공유하면 발행자는 등록되지만
            # **메시지가 한 건도 나오지 않는다.** (원인 확정에 오래 걸렸다)
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
                # 주의: 이 노드만 **Ogn 접두사가 등록 이름에 포함**된다.
                # 다른 노드(IsaacCreateRenderProduct 등)는 접두사가 없다.
                ("run_frame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("rp_scan", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("scan_pub", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("rp_scan.inputs:cameraPrim", [omni_lidar]),
                ("scan_pub.inputs:topicName", "/scan"),
                ("scan_pub.inputs:frameId", "lidar_frame"),
                ("scan_pub.inputs:type", "laser_scan"),
                # showDebugView 를 켜면 라이다 광선이 씬에 그려진다.
                # 스캔이 실제로 도는지 **눈으로** 확인할 수 있는 진단 수단이다.
                ("scan_pub.inputs:showDebugView",
                 os.environ.get("LIDAR_DEBUG", "0") == "1"),
                # [ERR-053] fullScan 은 문서에 "this setting is ignored" 라고
                # 명시돼 있다. RTX Lidar 는 항상 전체 스캔을 낸다. 설정해도 소용없다.
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "run_frame.inputs:execIn"),
                ("run_frame.outputs:step", "rp_scan.inputs:execIn"),
                ("rp_scan.outputs:execOut", "scan_pub.inputs:execIn"),
                # 각자의 렌더 프로덕트를 각자의 헬퍼에 이어 준다
                ("rp_scan.outputs:renderProductPath",
                 "scan_pub.inputs:renderProductPath"),
                ("ctx.outputs:context", "scan_pub.inputs:context"),
            ],
        })
    return f"/scan 발행 그래프 (OmniLidar: {omni_lidar})"


step("b07 LiDAR → /scan", _b07)


# ── b08: 관절 상태 발행/구독 ─────────────────────────────────────
def _b08():
    og.Controller.edit(
        {"graph_path": "/World/JointGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                ("js_pub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("js_sub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("art_ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
                # 시뮬레이션 시간은 core 확장 소속이다 [ERR-048]
                ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            keys.SET_VALUES: [
                ("js_pub.inputs:topicName", "/joint_states"),
                ("js_pub.inputs:targetPrim", [FRANKA]),
                ("js_sub.inputs:topicName", "/joint_command"),
                # [ERR-051] usePath 는 없다. targetPrim 만으로 지정한다.
                ("art_ctrl.inputs:targetPrim", [FRANKA]),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "js_pub.inputs:execIn"),
                ("on_tick.outputs:tick", "js_sub.inputs:execIn"),
                ("on_tick.outputs:tick", "art_ctrl.inputs:execIn"),
                # ROS2PublishJointState 는 timeStamp 입력이 **있다** (카메라와 다름)
                ("sim_time.outputs:simulationTime", "js_pub.inputs:timeStamp"),
                ("js_sub.outputs:jointNames", "art_ctrl.inputs:jointNames"),
                ("js_sub.outputs:positionCommand", "art_ctrl.inputs:positionCommand"),
                ("js_sub.outputs:velocityCommand", "art_ctrl.inputs:velocityCommand"),
            ],
        })
    return "/joint_states 발행 + /joint_command 구독"


step("b08 관절 상태", _b08)


# ── b09: TF2 발행 ────────────────────────────────────────────────
def _b09():
    og.Controller.edit(
        {"graph_path": "/World/TFGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                ("tf_pub", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("tf_pub.inputs:targetPrims", [FRANKA]),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "tf_pub.inputs:execIn"),
            ],
        })
    return "/tf 발행 (Franka 링크 트리)"


step("b09 TF2 → /tf", _b09)


# ── b10: cmd_vel 구독 → 차동 구동 ────────────────────────────────
def _b10():
    og.Controller.edit(
        {"graph_path": "/World/CmdVelGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                ("cmd_sub", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                # [ERR-052] IsaacComputeDifferentialDriveCommand 는 없다.
                # 확장 이름도 wheeled_robots.nodes 로 바뀌었지만
                # **등록 네임스페이스는 wheeled_robots** 그대로다.
                ("diff_ctrl", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ],
            keys.SET_VALUES: [
                ("cmd_sub.inputs:topicName", "/cmd_vel"),
                ("diff_ctrl.inputs:wheelRadius", 0.125),
                ("diff_ctrl.inputs:wheelDistance", 0.46),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "cmd_sub.inputs:execIn"),
                ("on_tick.outputs:tick", "diff_ctrl.inputs:execIn"),
                ("cmd_sub.outputs:linearVelocity", "diff_ctrl.inputs:linearVelocity"),
                ("cmd_sub.outputs:angularVelocity", "diff_ctrl.inputs:angularVelocity"),
            ],
        })
    return "/cmd_vel 구독 → DifferentialController"


step("b10 cmd_vel 구독", _b10)

# ── 발행 시작 ────────────────────────────────────────────────────
app_utils.play()
ok = sum(1 for m, _ in RESULT if m == "✅")
print(f"\n{ok}/{len(RESULT)} 그래프 생성 성공", flush=True)
print("play() — 토픽 발행 시작\n", flush=True)
print("확인:  ros2 topic list", flush=True)
print("       ros2 topic echo /joint_states --once", flush=True)
print("       ros2 topic hz /tf", flush=True)

s = 0
try:
    while True:
        world.step(render=True)
        s += 1
        if s % 600 == 0:
            print(f"  ... {s} 스텝 발행 중", flush=True)
except KeyboardInterrupt:
    print(f"\n종료 ({s} 스텝)", flush=True)
print("===FULLBRIDGE_END===", flush=True)
app.close()
