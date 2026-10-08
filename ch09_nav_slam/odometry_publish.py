"""책 9.2 (b03) 오도메트리 발행 — 동작 검증본.

SLAM 과 Nav2 는 `/odom` 과 `odom → base_link` TF 가 있어야 시작조차 못 한다.
이 절이 9장의 토대다.

원문과의 차이:
  [ERR-055] `isaacsim.robot.wheeled_robots.IsaacComputeOdometry` 라는 노드가 없다.
            이름은 실재하지만 **네임스페이스가 다르다.**
              책:   isaacsim.robot.wheeled_robots.IsaacComputeOdometry
              실제: isaacsim.core.nodes.IsaacComputeOdometry
            그대로 쓰면 OmniGraphError: unrecognized type 이 난다.
  [ERR-045] pip 설치 Isaac Sim 은 ROS 2 환경변수가 필요하다.
            ch08_ros2/setup_ros2_env.sh 를 먼저 source 할 것.

실행:
    conda activate isaac_env
    source ../ch08_ros2/setup_ros2_env.sh
    python odometry_publish.py

확인 (다른 터미널):
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ros2 topic echo /odom --once
    ros2 run tf2_ros tf2_echo odom base_link
"""
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "1") == "1"})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
app.update()

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationActions  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

World.clear_instance()
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
nucleus = nucleus_utils.get_assets_root_path()

# ── 바퀴 로봇 (Carter) ───────────────────────────────────────────
# [ERR-022] Carter 경로는 제조사별로 재편됐다 (5장에서 확인)
CARTER = "/World/Carter"
stage_utils.add_reference_to_stage(
    usd_path=nucleus + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
    prim_path=CARTER)

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr().Set(1200.0)
world.reset()

# 섀시 링크를 이름이 아니라 **실재로** 찾는다. 에셋마다 이름이 다르다.
from pxr import Usd  # noqa: E402

CHASSIS = next(
    (str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(CARTER))
     if "chassis" in p.GetName().lower()), None)
if CHASSIS is None:
    CHASSIS = CARTER

print("\n===ODOM===", flush=True)
print(f"섀시 링크: {CHASSIS}", flush=True)

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/OdomGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            # [ERR-055] 네임스페이스가 core.nodes 다 (책의 wheeled_robots 가 아니다)
            ("compute_odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("odom_pub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("raw_tf_pub", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ],
        keys.SET_VALUES: [
            ("compute_odom.inputs:chassisPrim", [CHASSIS]),
            ("odom_pub.inputs:topicName", "/odom"),
            ("odom_pub.inputs:odomFrameId", "odom"),
            ("odom_pub.inputs:chassisFrameId", "base_link"),
            # odom → base_link TF — SLAM 이 이걸 요구한다
            ("raw_tf_pub.inputs:parentFrameId", "odom"),
            ("raw_tf_pub.inputs:childFrameId", "base_link"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "compute_odom.inputs:execIn"),
            ("tick.outputs:tick", "odom_pub.inputs:execIn"),
            ("tick.outputs:tick", "raw_tf_pub.inputs:execIn"),
            ("compute_odom.outputs:angularVelocity", "odom_pub.inputs:angularVelocity"),
            ("compute_odom.outputs:linearVelocity", "odom_pub.inputs:linearVelocity"),
            ("compute_odom.outputs:position", "odom_pub.inputs:position"),
            ("compute_odom.outputs:orientation", "odom_pub.inputs:orientation"),
            # TF 도 같은 오도메트리 값을 받아야 /odom 과 어긋나지 않는다
            ("compute_odom.outputs:position", "raw_tf_pub.inputs:translation"),
            ("compute_odom.outputs:orientation", "raw_tf_pub.inputs:rotation"),
        ],
    })
print("[b03] 오도메트리 그래프 생성 — /odom + odom→base_link TF", flush=True)

app_utils.play()

# 로봇을 실제로 굴린다. 가만히 있으면 오도메트리가 0이라 검증이 안 된다.
arm = Articulation(prim_paths_expr=CARTER, name="carter")
arm.initialize()
names = list(arm.dof_names)
wheels = [i for i, n in enumerate(names) if "wheel" in n.lower()]
print(f"바퀴 관절 {len(wheels)}개: {[names[i] for i in wheels][:4]}", flush=True)

print(f"\n확인:  ros2 topic echo /odom --once", flush=True)
print(f"       ros2 run tf2_ros tf2_echo odom base_link", flush=True)

s = 0
try:
    while True:
        if wheels:
            # 앞으로 + 살짝 회전 — 오도메트리가 변해야 검증이 된다
            cmd = np.full((1, len(wheels)), 6.0)
            cmd[0, ::2] = 5.0
            arm.apply_action(ArticulationActions(
                joint_velocities=cmd, joint_indices=np.array(wheels)))
        world.step(render=True)
        s += 1
        if s % 600 == 0:
            print(f"  ... {s} 스텝 주행 중", flush=True)
except KeyboardInterrupt:
    print(f"\n종료 ({s} 스텝)", flush=True)
print("===ODOM_END===", flush=True)
app.close()
