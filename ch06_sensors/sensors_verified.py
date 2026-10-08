"""Chapter 6 센서 — Isaac Sim 6.1에서 동작이 검증된 코드 모음.

책 6.2~6.5의 센서 API를 실제로 실행해 확인한 형태로 정리했다.
원문과 다른 부분은 각 함수의 주석에 표시했다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python sensors_verified.py
"""
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402

root = nucleus_utils.get_assets_root_path()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.scene.add(
    DynamicCuboid(
        prim_path="/World/Target",
        name="target",
        position=np.array([0.0, 0.0, 0.5]),
        scale=np.array([0.3, 0.3, 0.3]),
        color=np.array([0.9, 0.2, 0.2]),
    )
)
stage_utils.add_reference_to_stage(
    usd_path=root + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    prim_path="/World/UR5e",
)
world.reset()


# ── 6.2 카메라 ────────────────────────────────────────────────────
def camera_demo():
    """RTX 카메라로 RGB·깊이를 얻는다.

    원문과의 차이:
      - [ERR-024] 원문은 뒤 블록에서 `camera.prim_path`로 render_product를 또 만든다.
        CameraSensor는 이미 render product를 갖고 있으므로 불필요하고,
        `prim_path` 속성 자체가 없어 AttributeError가 난다.
      - [ERR-025] get_data()는 **warp 배열**을 반환한다. `.numpy()`로 변환해야 한다.
        `np.array()`를 쓰면 RuntimeError: Invalid indexing in slice 가 난다.
    """
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

    rtx_camera = RtxCamera("/World/Cam", tick_rate=30.0, translations=[[0.0, -3.0, 1.5]])
    camera = CameraSensor(
        rtx_camera,
        resolution=(480, 640),  # (height, width) — OpenCV/NumPy 관례
        annotators=["rgb", "distance_to_image_plane"],
    )
    app_utils.play(commit=True)
    for _ in range(60):
        world.step(render=True)

    rgb, _ = camera.get_data("rgb")
    depth, _ = camera.get_data("distance_to_image_plane")
    rgb_np = rgb.numpy()          # ← warp → numpy
    depth_np = depth.numpy()

    print(f"  RGB   : shape={rgb_np.shape} dtype={rgb_np.dtype} 평균={rgb_np.mean():.1f}")
    print(f"  Depth : shape={depth_np.shape} 유한값={np.isfinite(depth_np).mean() * 100:.0f}%")
    print(f"  경로   : rtx_camera.paths = {rtx_camera.paths}")
    return rgb_np


# ── 6.3 RTX LiDAR ─────────────────────────────────────────────────
def lidar_demo():
    """RTX LiDAR 스캔 데이터를 얻는다. 원문 그대로 동작한다.

    annotator 이름은 'generic-model-output' (하이픈, 소문자)이다.
    """
    from isaacsim.sensors.experimental.rtx import (
        Lidar,
        LidarSensor,
        parse_generic_model_output_data,
    )

    lidar = Lidar.create(
        "/World/Sensors/LiDAR_01",
        config="Example_Rotary",
        translations=[[0.0, 0.0, 0.5]],
    )
    sensor = LidarSensor(lidar, annotators=["generic-model-output"])
    app_utils.play(commit=True)
    for _ in range(40):
        world.step(render=True)

    data, info = sensor.get_data("generic-model-output")
    gmo = parse_generic_model_output_data(data)
    print(f"  LiDAR : 파싱 성공, 필드 예시={[f for f in dir(gmo) if not f.startswith('_')][:5]}")
    return gmo


# ── 6.4 IMU ───────────────────────────────────────────────────────
def imu_demo():
    """IMU를 부착하고 읽는다. 원문 그대로 동작한다."""
    from isaacsim.sensors.experimental.physics import IMU, IMUSensor

    imu = IMUSensor(IMU.create("/World/UR5e/base_link/IMU"))
    world.reset()
    for _ in range(30):
        world.step(render=False)
    d = imu.get_data()
    print(f"  IMU   : keys={list(d.keys())}")
    return d


# ── 6.5 접촉 센서 ─────────────────────────────────────────────────
def contact_demo():
    """접촉 센서를 부착하고 읽는다.

    원문과의 차이:
      - [ERR-013] import 경로가 `isaacsim.sensors.physics` → ImportError.
        `isaacsim.sensors.experimental.physics` 가 맞다.
    """
    from isaacsim.sensors.experimental.physics import Contact, ContactSensor

    cs = ContactSensor(
        Contact.create(
            "/World/Target/ContactSensor",
            min_threshold=0.0,
            max_threshold=100.0,
            radius=0.05,
        )
    )
    world.reset()
    for _ in range(60):
        world.step(render=False)
    d = cs.get_data()
    print(f"  Contact: keys={list(d.keys())}")
    return d


# ── 6.9.2 기성품 센서 자산 ────────────────────────────────────────
def realsense_demo():
    """RealSense 깊이 카메라 자산을 씬에 붙인다.

    원문과의 차이:
      - [ERR-023] 원문의 D435(`rsd435.usd`)는 에셋 서버에 존재하지 않는다.
        제공되는 것은 D455 / D457 뿐이다.
    """
    from pxr import Usd

    import omni.usd

    stage_utils.add_reference_to_stage(
        usd_path=root + "/Isaac/Sensors/RealSense/D455/rsd455.usd",
        prim_path="/World/RealSense",
    )
    prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/RealSense")
    n = sum(1 for _ in Usd.PrimRange(prim))
    print(f"  D455  : prim {n}개 로드")
    return n


# ── 6.10.1 힘/토크 근사 ───────────────────────────────────────────
def force_torque_demo():
    """관절 힘을 읽는다.

    원문과의 차이:
      - [ERR-009] `ArticulationView`는 Isaac Sim에서 삭제되었다.
        `isaacsim.core.prims.Articulation`이 그 역할을 흡수했고,
        `prim_paths_expr` 인자는 그대로 쓴다.
    """
    from isaacsim.core.prims import Articulation

    view = Articulation(prim_paths_expr="/World/UR5e", name="ur5e_view")
    world.scene.add(view)
    world.reset()
    for _ in range(20):
        world.step(render=False)
    forces = view.get_measured_joint_forces()
    a = np.asarray(forces)
    print(f"  F/T   : get_measured_joint_forces() shape={a.shape}")
    return a


if __name__ == "__main__":
    print("\n=== Chapter 6 센서 검증 ===")
    for name, fn in [
        ("6.2 카메라", camera_demo),
        ("6.3 LiDAR", lidar_demo),
        ("6.4 IMU", imu_demo),
        ("6.5 접촉", contact_demo),
        ("6.9 RealSense", realsense_demo),
        ("6.10 힘/토크", force_torque_demo),
    ]:
        print(f"\n[{name}]")
        try:
            fn()
        except Exception as e:
            print(f"  ❌ {type(e).__name__}: {str(e)[:120]}")
    app.close()
