"""책 8장 b03·b05 검증 — ROS2 Bridge 활성화 + 카메라 토픽 발행 (동작본).

[ERR 후보 A] b03 의 활성화 방식이 standalone 에서 부실하다.
    원고:  manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    이 방식은 True 를 돌려주지만, standalone Python 에서는 OmniGraph **노드
    팩토리가 등록되지 않아** 곧바로 그래프를 만들면 다음 오류가 난다:
      OmniGraphError: Could not create node using unrecognized type
      'isaacsim.ros2.bridge.ROS2ReadSimulationTime'
    Isaac Sim 6.1 공식 예제(isaac-sim-ros2-bridge skill)는 헬퍼를 쓴다:
      import isaacsim.core.experimental.utils.app as app_utils
      app_utils.enable_extension("isaacsim.ros2.bridge")
      simulation_app.update()

[ERR 후보 B] b05 마지막 줄 경로 앞에 **쉼표 오타**:
      setup_camera_ros2_publisher(",/World/Carter/chassis_link/Camera")

[핵심] 그래프는 OnPlaybackTick 트리거라 **play() 전에는 토픽을 발행하지 않는다.**

실행:
    source /opt/ros/jazzy/setup.bash
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python ch8_ros2_publish.py
"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

# ── b03 (수정): 공식 헬퍼로 bridge 활성화 ────────────────────────
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
app.update()
print("\n===ROS2PUB===", flush=True)
print("[b03] app_utils.enable_extension + update — bridge 활성화", flush=True)

import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.GetSizeAttr().Set(0.5)
UsdGeom.XformCommonAPI(cube.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.5))
cam = UsdGeom.Camera.Define(stage, "/World/Camera")
UsdGeom.XformCommonAPI(cam.GetPrim()).SetTranslate(Gf.Vec3d(2.0, 0.0, 1.0))
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr().Set(1000.0)


# ── b05 (쉼표 오타 수정): 카메라 ROS2 발행 그래프 ────────────────
def setup_camera_ros2_publisher(camera_prim_path,
                                rgb_topic="/camera/image_raw",
                                depth_topic="/camera/depth/image_raw",
                                camera_info_topic="/camera/camera_info",
                                frame_id="camera_frame"):
    render_product = rep.create.render_product(camera_prim_path, (640, 480))
    keys = og.Controller.Keys
    (graph, _, _, _) = og.Controller.edit(
        {"graph_path": "/World/CameraROS2Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnPlaybackTick"),
                # [ERR-048] 원고는 sim_time 노드를 만들어 timestamp 에 연결하지만,
                # ① isaacsim.ros2.bridge.ROS2ReadSimulationTime 이라는 노드가 없고
                # ② ROS2CameraHelper 에 timestamp 입력 자체가 없다.
                # 타임스탬프는 노드가 내부에서 처리한다
                # (resetSimulationTimeOnStop / useSystemTime 로 조절).
                # → sim_time 노드와 그 연결을 **통째로 삭제**한다.
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
    print(f"[b05] 카메라 ROS2 발행 그래프 생성됨", flush=True)
    print(f"       {rgb_topic} / {depth_topic} / {camera_info_topic}", flush=True)
    return graph


world.reset()

# 공식 예제 패턴: stage 로딩이 끝날 때까지 update 를 돌린다.
# 이 대기 시간 동안 bridge 의 rclpy 로드·노드 팩토리 등록이 완료된다.
from isaacsim.core.experimental.utils.stage import is_stage_loading  # noqa: E402

for _ in range(5):
    app.update()
while is_stage_loading():
    app.update()

# 노드 등록이 프레임 단위로 지연되므로, **가벼운 probe 노드**로 준비를 확인한다.
# 주의: 실제 카메라 그래프로 재시도하면 첫 실패가 그래프 껍데기를 남겨
#   "A graph already exists at this path" 로 이후 재시도가 전부 막힌다.
# 그래서 probe 는 **매번 다른 경로**를 쓰고, 준비되면 카메라 그래프를 **한 번만** 만든다.
# 중요: 반복 횟수가 아니라 **시간**으로 기다려야 한다.
# app.update() 300회는 수백 ms 만에 끝나는데, 번들 rclpy 로드는 앱 시작 후
# 약 10초에야 일어난다. 횟수 기준으로 돌면 로딩 전에 소진돼 버린다.
import time  # noqa: E402

# [ERR-048] 책의 'isaacsim.ros2.bridge.ROS2ReadSimulationTime' 은 **존재하지 않는다.**
# isaacsim.ros2.nodes 확장이 제공하는 35개 노드 목록에 그 이름이 없다.
# 시뮬레이션 시간 노드는 ROS2 쪽이 아니라 core 쪽에 있다:
#   isaacsim.core.nodes.IsaacReadSimulationTime
# probe 는 확실히 존재하는 ROS2 노드로 한다.
PROBE = "isaacsim.ros2.bridge.ROS2CameraHelper"
ready = False
t0 = time.time()
attempt = 0
while time.time() - t0 < 60.0:
    try:
        og.Controller.edit(
            {"graph_path": f"/World/_probe_{attempt}", "evaluator_name": "execution"},
            {og.Controller.Keys.CREATE_NODES: [("p", PROBE)]})
        ready = True
        break
    except og.OmniGraphError:
        attempt += 1
        for _ in range(10):
            app.update()
        time.sleep(0.2)
if not ready:
    print(f"[b05] ❌ ROS2 노드 타입 미등록 ({time.time()-t0:.0f}초 대기, "
          f"probe {attempt}회)", flush=True)
    app.close()
    raise SystemExit(1)
print(f"[b05] ROS2 노드 팩토리 준비 확인 ({time.time()-t0:.1f}초, probe {attempt}회)",
      flush=True)

# [ERR-B] 원문은 ",/World/Carter/..." — 쉼표 오타. 올바른 경로로.
setup_camera_ros2_publisher("/World/Camera")

# OnPlaybackTick 그래프는 play 상태에서만 발행한다
app_utils.play()
print("[핵심] play() — 이제부터 토픽 발행 시작", flush=True)

print(f"\n확인:  ros2 topic list   /   ros2 topic hz /camera/image_raw", flush=True)
print(f"(600스텝마다 알림. 종료: 이 프로세스를 kill)", flush=True)

step = 0
try:
    while True:
        world.step(render=True)
        step += 1
        if step % 600 == 0:
            print(f"  ... {step} 스텝 발행 중", flush=True)
except KeyboardInterrupt:
    print(f"\n종료 (총 {step} 스텝)", flush=True)
print("===ROS2PUB_END===", flush=True)
app.close()
