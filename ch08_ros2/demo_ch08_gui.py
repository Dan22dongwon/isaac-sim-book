"""8장 실습을 **화면으로 직접 보는** 데모.

Isaac Sim 창(왼쪽)과 RViz2 창(오른쪽)을 나란히 띄운다.
Isaac Sim 이 발행한 카메라 영상이 RViz2 에 실시간으로 뜨는 것을 눈으로 확인한다.

장면에 **움직이는 물체**를 둔다. 정지 화면이면 영상이 살아 있는지 알 수 없다.
빨간 공이 원을 그리며 돌고, 그 모습이 RViz2 Image 패널에 그대로 나타나야 한다.

  [ERR-045] pip 설치 Isaac Sim 은 ROS 2 환경변수 3개가 필요하다.
            반드시 setup_ros2_env.sh 를 먼저 source 할 것.
  [ERR-048] 원고의 sim_time 노드·timestamp 연결은 둘 다 존재하지 않는다. 삭제했다.

실행:
    conda activate isaac_env
    source setup_ros2_env.sh
    python demo_ch08_gui.py            # 기본 3분
    MINUTES=5 python demo_ch08_gui.py

다른 터미널에서:
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    rviz2
    # Add > By topic > /camera/image_raw > Image
"""
import math
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "0") == "1"})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
app.update()

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, Vt  # noqa: E402

import omni.usd  # noqa: E402

MINUTES = float(os.environ.get("MINUTES", "3"))

World.clear_instance()
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

# ── 장면: 가운데 흰 기둥 + 그 둘레를 도는 빨간 공 ────────────────
post = UsdGeom.Cylinder.Define(stage, "/World/Post")
post.GetRadiusAttr().Set(0.12)
post.GetHeightAttr().Set(0.8)
post.GetAxisAttr().Set("Z")
UsdGeom.XformCommonAPI(post.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.4))

ball = UsdGeom.Sphere.Define(stage, "/World/Ball")
ball.GetRadiusAttr().Set(0.16)
ball.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.15, 0.15)]))
ball_api = UsdGeom.XformCommonAPI(ball.GetPrim())

# 색이 다른 블록 셋 — 영상에서 방향을 알 수 있게
for i, (x, y, col) in enumerate([(1.2, 0.0, (0.2, 0.4, 0.9)),
                                 (-1.2, 0.0, (0.95, 0.75, 0.1)),
                                 (0.0, 1.2, (0.2, 0.7, 0.3))]):
    c = UsdGeom.Cube.Define(stage, f"/World/Marker{i}")
    c.GetSizeAttr().Set(0.3)
    c.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*col)]))
    UsdGeom.XformCommonAPI(c.GetPrim()).SetTranslate(Gf.Vec3d(x, y, 0.15))

# ── 카메라 ───────────────────────────────────────────────────────
CAM = "/World/Camera"
cam = UsdGeom.Camera.Define(stage, CAM)
# 기둥에 바짝 붙이면 기둥만 화면을 채워 **움직이는 공이 안 보인다.**
# 영상이 살아 있다는 것을 보여 주는 게 이 데모의 핵심이므로 넉넉히 물러난다.
UsdGeom.XformCommonAPI(cam.GetPrim()).SetTranslate(Gf.Vec3d(0.0, -4.6, 2.4))
UsdGeom.XformCommonAPI(cam.GetPrim()).SetRotate(Gf.Vec3f(68.0, 0.0, 0.0))

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr().Set(1200.0)
key = UsdLux.RectLight.Define(stage, "/World/Key")
key.CreateIntensityAttr().Set(14000.0)
key.CreateWidthAttr().Set(6.0)
key.CreateHeightAttr().Set(6.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(1.5, -2.5, 3.5))

world.reset()


# ── ROS2 발행 그래프 ─────────────────────────────────────────────
def setup_camera_ros2_publisher(camera_prim_path,
                                rgb_topic="/camera/image_raw",
                                depth_topic="/camera/depth/image_raw",
                                camera_info_topic="/camera/camera_info",
                                frame_id="camera_frame"):
    render_product = rep.create.render_product(camera_prim_path, (640, 480))
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/CameraROS2Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                # [ERR-048] 원고의 sim_time 노드는 존재하지 않고,
                # ROS2CameraHelper 에 timestamp 입력도 없다. 통째로 삭제.
                ("rgb_pub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("depth_pub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cam_info_pub", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("rgb_pub.inputs:topicName", rgb_topic),
                ("rgb_pub.inputs:frameId", frame_id),
                ("rgb_pub.inputs:type", "rgb"),
                ("rgb_pub.inputs:renderProductPath", render_product.path),
                ("depth_pub.inputs:topicName", depth_topic),
                ("depth_pub.inputs:frameId", frame_id),
                ("depth_pub.inputs:type", "depth"),
                ("depth_pub.inputs:renderProductPath", render_product.path),
                ("cam_info_pub.inputs:topicName", camera_info_topic),
                ("cam_info_pub.inputs:frameId", frame_id),
                ("cam_info_pub.inputs:renderProductPath", render_product.path),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "rgb_pub.inputs:execIn"),
                ("on_tick.outputs:tick", "depth_pub.inputs:execIn"),
                ("on_tick.outputs:tick", "cam_info_pub.inputs:execIn"),
            ],
        })


setup_camera_ros2_publisher(CAM)

# 뷰포트를 카메라와 비슷한 각도로 옮겨 둔다 (Isaac Sim 창에서 보기 좋게)
try:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[2.6, -3.4, 2.0], target=[0.0, 0.0, 0.4])
except Exception as e:
    print(f"(뷰포트 이동 실패: {e})", flush=True)

# OnPlaybackTick 그래프는 play 상태에서만 발행한다
app_utils.play()

print("\n" + "=" * 68, flush=True)
print("  8장 데모 — 창 두 개를 보세요", flush=True)
print("=" * 68, flush=True)
print("  왼쪽  Isaac Sim  빨간 공이 기둥 둘레를 돕니다", flush=True)
print("  오른쪽 RViz2     같은 장면이 ROS 2 토픽으로 전달돼 표시됩니다", flush=True)
print("\n  RViz2 를 아직 안 띄웠다면 다른 터미널에서:", flush=True)
print("     source /opt/ros/jazzy/setup.bash", flush=True)
print("     export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp", flush=True)
print("     rviz2", flush=True)
print("     → Add > By topic > /camera/image_raw > Image", flush=True)
print(f"\n  {MINUTES:.0f}분 뒤 자동 종료. 먼저 닫으려면 Ctrl+C.", flush=True)
print("=" * 68 + "\n", flush=True)

steps = int(MINUTES * 60 * 60)
R = 0.9
try:
    for s in range(steps):
        # 빨간 공을 원 궤도로 돌린다 — 영상이 살아 있는지 한눈에 보인다
        t = s / 60.0
        ball_api.SetTranslate(Gf.Vec3d(R * math.cos(t * 1.2),
                                       R * math.sin(t * 1.2), 0.16))
        world.step(render=True)
        if s % (60 * 15) == 0:
            print(f"  [{s/60:5.0f}초] 발행 중 — RViz2 Image 패널에 "
                  f"빨간 공이 움직여야 합니다", flush=True)
except KeyboardInterrupt:
    print("\n  사용자가 중단했습니다.", flush=True)

print("\n  데모 종료", flush=True)
app.close()
