"""9.7 확장 — **4족보행 로봇**으로 창고를 3D 재구성한다.

Implements: 9.7.5 확장 (책 밖)

`cuvslam_scene.py` 는 바퀴 로봇(NovaCarter)이다. 이 파일은 같은 파이프라인을
**4족보행 로봇**에 붙인 것이다. 바뀌는 곳이 세 군데다.

1. **구동 방식** — 차동구동이 아니라 **보행 정책(locomotion policy)** 이다.
   `DifferentialController` 대신 강화학습으로 학습된 정책이 관절을 움직인다.

2. **카메라가 없다** — NovaCarter 는 Hawk 스테레오가 내장이지만
   Spot·ANYmal·Go2 자산에는 **카메라가 0대**다. 직접 달아야 한다.

       Spot       관절 16  카메라 0  크기 0.87×0.44×0.78 m
       ANYmal-D   관절 16  카메라 0  크기 1.01×0.68×0.91 m
       Go1-sensor 관절 16  카메라 0

3. **오도메트리 기준 링크** — 바퀴 로봇의 `chassis_link` 대신 **몸통(body)** 이다.

## ⚠️ 정책 API 가 7.0.0 에서 바뀌었다

```python
SpotFlatTerrainPolicy(...)        # ← 호출하면 RuntimeError 를 던지는 안내용 껍데기
```

```python
from isaacsim.robot.policy.examples import RobotPolicyRunner, get_spot_spec
runner = RobotPolicyRunner(get_spot_spec(), prim_path=..., position=...)
runner.initialize()
runner.step(dt, [vx, vy, wz])     # base 기준 평면 속도
```

`step()` 의 `command` 가 `[vx, vy, wz]` 라 `/cmd_vel` 과 그대로 대응된다.
바퀴 로봇과 달리 **옆걸음(vy)** 도 된다.

사용법:
    ROBOT=spot SCENE=warehouse python quadruped_scene.py
    ROBOT=anymal ...   ROBOT=go2 ...
"""
import os

from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
app = SimulationApp({"headless": HEADLESS})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
app_utils.enable_extension("isaacsim.robot.policy.examples")
app.update()

import numpy as np                                        # noqa: E402
import omni.graph.core as og                              # noqa: E402
import omni.usd                                           # noqa: E402
from isaacsim.core.api import World                       # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.robot.policy.examples import (              # noqa: E402
    RobotPolicyRunner, get_anymal_spec, get_go2_spec, get_spot_spec)
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux                  # noqa: E402

# ─────────────────────────────────────────────────────────── 설정
ROBOT = os.environ.get("ROBOT", "spot")
SCENE = os.environ.get("SCENE", "warehouse")
SPECS = {"spot": get_spot_spec, "anymal": get_anymal_spec, "go2": get_go2_spec}
if ROBOT not in SPECS:
    raise SystemExit(f"ROBOT 은 {list(SPECS)} 중 하나 — 받은 값: {ROBOT}")

WAREHOUSES = {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "forklift": "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "shelves": "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
}

# ★ 보행 정책은 **물리 500 Hz / 렌더 50 Hz** 로 돌아야 한다.
#
#   물리 10스텝당 렌더 1회다. 정책은 매 물리 스텝마다 갱신돼야 자세를 유지한다.
#   렌더 루프에서 runner.step() 을 1회씩만 부르면 **정책이 1/10 속도로 돌아**
#   로봇이 자세를 못 잡고 넘어진다. 물리 콜백에서 불러야 한다.
PHYSICS_HZ, DECIM = 500.0, 10
World.clear_instance()
world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYSICS_HZ,
              rendering_dt=DECIM / PHYSICS_HZ)
stage = omni.usd.get_context().get_stage()
nucleus = get_assets_root_path()

if SCENE in WAREHOUSES:
    add_reference_to_stage(nucleus + WAREHOUSES[SCENE], "/World/Env")
    print(f"\n환경: {WAREHOUSES[SCENE].split('/')[-1]}", flush=True)
else:
    world.scene.add_default_ground_plane()
    print("\n환경: 기본 지면", flush=True)

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr().Set(1200.0)

# ───────────────────────────────── 보행 로봇 + 정책
ROBOT_PATH = "/World/Robot"
START = (-6.0, 0.0, 0.8)          # 4족은 서 있는 높이에서 떨어뜨린다

runner = RobotPolicyRunner(SPECS[ROBOT](), prim_path=ROBOT_PATH,
                           position=np.array(START))

# ★ spawn() 을 불러야 로봇이 **실제로 스테이지에 만들어진다.**
#   생성자만 부르고 넘어가면 프림이 없어 이렇게 죽는다.
#
#       RuntimeError: RobotPolicyRunner.initialize requires spawn() to ...
#       [omni.graph] The prim at path '/World/Robot' ...
#
#   그래서 **몸통 찾기·카메라 장착은 spawn() 뒤에** 와야 한다.
runner.spawn()

# 정책이 학습된 주기에 맞춘다 — 임의로 정하면 보행이 깨진다.
DT = runner.physics_dt
world.set_simulation_dt(physics_dt=DT, rendering_dt=DT * runner.decimation)
print(f"로봇: {ROBOT}  (정책 {SPECS[ROBOT]().name})", flush=True)
print(f"물리 {1 / DT:.0f} Hz · 렌더 {1 / (DT * runner.decimation):.0f} Hz"
      f"  (decimation {runner.decimation})", flush=True)

# ───────────────────────────────── 몸통 링크 찾기 (하드코딩하지 않는다)
def find_body():
    """오도메트리·카메라를 붙일 몸통 링크를 찾는다.

    자산마다 이름이 다르다: Spot=body, ANYmal=base, Go2=base.
    이름 후보를 순서대로 보고, 없으면 루트 바로 아래 첫 Xform 을 쓴다.
    """
    root = stage.GetPrimAtPath(ROBOT_PATH)
    for want in ("body", "base", "trunk", "torso"):
        for p in Usd.PrimRange(root):
            if p.GetName().lower() == want:
                return str(p.GetPath())
    for p in Usd.PrimRange(root):
        if p.GetTypeName() == "Xform" and str(p.GetPath()) != ROBOT_PATH:
            return str(p.GetPath())
    return ROBOT_PATH


BODY = find_body()
print(f"몸통: {BODY}", flush=True)

# ───────────────────────────────── 스테레오 카메라를 **직접 단다**
#
# 4족 자산에는 카메라가 없다. 몸통 앞쪽에 15 cm 간격으로 두 대를 만든다.
# NovaCarter 의 Hawk 와 같은 베이스라인으로 맞춰 비교가 되게 한다.
BASELINE = 0.15
# Spot 몸통 길이가 0.87 m 라 앞쪽 0.28 m 는 **몸통 안**이다.
# 몸통 밖(0.44 m 초과)으로 빼야 자기 몸이 화면을 가리지 않는다.
CAM_FWD, CAM_UP = 0.46, 0.12      # 몸통 기준 앞·위 오프셋

# 640x480 은 DLSS 내부 렌더가 320x240 이 되어 최소치 미만 → 크래시한다
WIDTH, HEIGHT = 1280, 720


def make_camera(name, y):
    """몸통에 붙는 카메라 한 대.

    ★ 오일러각으로 돌리면 **영상이 90° 기울어진다.**

    USD 카메라는 `-Z` 가 정면, `+Y` 가 위다. 로봇 정면(+X)을 보게 하려고
    Y축 -90° 만 주면 시선은 맞지만 **카메라의 "위" 가 월드 +Y(옆)** 로 간다.
    영상이 옆으로 누워 나오고, TF 와 어긋나 cuVSLAM 이 추적을 못 한다.

    오일러각 순서를 고민하지 말고 **기저 벡터로 행렬을 직접 만든다.**

        카메라 -Z(시선) → 월드 +X      즉  +Z → -X
        카메라 +Y(위)   → 월드 +Z
        카메라 +X(오른쪽) = Y × Z      = (0,-1,0)

    Gf.Matrix4d 는 행벡터 규약이라 **각 행이 기저 벡터**다.
    """
    path = f"{BODY}/{name}"
    cam = UsdGeom.Camera.Define(stage, path)
    m = Gf.Matrix4d(
        0.0, -1.0, 0.0, 0.0,      # 카메라 +X → 월드 -Y
        0.0, 0.0, 1.0, 0.0,       # 카메라 +Y → 월드 +Z  (위가 위)
        -1.0, 0.0, 0.0, 0.0,      # 카메라 +Z → 월드 -X  (시선 -Z 가 +X)
        CAM_FWD, y, CAM_UP, 1.0)  # 위치
    cam.MakeMatrixXform().Set(m)
    cam.GetFocalLengthAttr().Set(2.87)           # Hawk 과 동일한 화각(약 90°)
    cam.GetHorizontalApertureAttr().Set(5.76)
    # ★ 세로 조리개를 렌더 종횡비에 맞춘다.
    #   빼먹으면 fy 가 fx 와 크게 달라지고, Isaac 이 이렇게 경고하며 강제로 맞춘다.
    #       [Warning] camera_info_utils: Forcing fy to fx (135.14 != 637.78)
    #   그 상태의 camera_info 는 실제 영상 기하와 어긋나 추적이 틀어진다.
    cam.GetVerticalApertureAttr().Set(5.76 * HEIGHT / WIDTH)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
    return path


CAM_L = make_camera("cam_stereo_left", +BASELINE / 2)
CAM_R = make_camera("cam_stereo_right", -BASELINE / 2)
print(f"카메라: 직접 장착 — 베이스라인 {BASELINE * 100:.1f} cm", flush=True)

# 외부 파라미터 — 몸통(base_link) 기준 광학 프레임
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
T_base = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(BODY))
FLIP = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0))


def extrinsics(cam_path):
    T_cam = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(cam_path))
    T = FLIP * (T_cam * T_base.GetInverse())
    t, q = T.ExtractTranslation(), T.ExtractRotationQuat()
    i = q.GetImaginary()
    return ([float(t[0]), float(t[1]), float(t[2])],
            [float(i[0]), float(i[1]), float(i[2]), float(q.GetReal())])


T_L, Q_L = extrinsics(CAM_L)
T_R, Q_R = extrinsics(CAM_R)

# ─────────────────────────────────────────────────────────── 그래프
keys = og.Controller.Keys

og.Controller.edit(
    {"graph_path": "/World/StereoGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("run", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("rp_l", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("rp_r", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("img_l", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("img_r", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("raw_l", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("depth_l", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ],
        keys.SET_VALUES: [
            ("rp_l.inputs:cameraPrim", [CAM_L]),
            ("rp_l.inputs:width", WIDTH), ("rp_l.inputs:height", HEIGHT),
            ("rp_r.inputs:cameraPrim", [CAM_R]),
            ("rp_r.inputs:width", WIDTH), ("rp_r.inputs:height", HEIGHT),
            ("img_l.inputs:topicName", "front_stereo_camera/left/image_rect_color"),
            ("img_l.inputs:type", "rgb"),
            ("img_l.inputs:frameId", "front_stereo_camera_left"),
            ("img_l.inputs:useSystemTime", False),
            ("img_r.inputs:topicName", "front_stereo_camera/right/image_rect_color"),
            ("img_r.inputs:type", "rgb"),
            ("img_r.inputs:frameId", "front_stereo_camera_right"),
            ("img_r.inputs:useSystemTime", False),
            ("raw_l.inputs:topicName", "front_stereo_camera/left/image_raw"),
            ("raw_l.inputs:type", "rgb"),
            ("raw_l.inputs:frameId", "front_stereo_camera_left"),
            ("raw_l.inputs:useSystemTime", False),
            ("depth_l.inputs:topicName", "front_stereo_camera/depth/ground_truth"),
            ("depth_l.inputs:type", "depth"),
            ("depth_l.inputs:frameId", "front_stereo_camera_left"),
            ("depth_l.inputs:useSystemTime", False),
            # 한 노드가 좌·우를 함께 받아야 P[3] 에 베이스라인이 들어간다
            ("info.inputs:topicName", "front_stereo_camera/left/camera_info"),
            ("info.inputs:frameId", "front_stereo_camera_left"),
            ("info.inputs:topicNameRight", "front_stereo_camera/right/camera_info"),
            ("info.inputs:frameIdRight", "front_stereo_camera_right"),
            ("info.inputs:useSystemTime", False),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "run.inputs:execIn"),
            ("run.outputs:step", "rp_l.inputs:execIn"),
            ("run.outputs:step", "rp_r.inputs:execIn"),
            ("rp_l.outputs:execOut", "img_l.inputs:execIn"),
            ("rp_l.outputs:execOut", "raw_l.inputs:execIn"),
            ("rp_l.outputs:execOut", "depth_l.inputs:execIn"),
            ("rp_r.outputs:execOut", "img_r.inputs:execIn"),
            ("rp_r.outputs:execOut", "info.inputs:execIn"),
            ("rp_l.outputs:renderProductPath", "img_l.inputs:renderProductPath"),
            ("rp_l.outputs:renderProductPath", "raw_l.inputs:renderProductPath"),
            ("rp_l.outputs:renderProductPath", "depth_l.inputs:renderProductPath"),
            ("rp_r.outputs:renderProductPath", "img_r.inputs:renderProductPath"),
            ("rp_l.outputs:renderProductPath", "info.inputs:renderProductPath"),
            ("rp_r.outputs:renderProductPath", "info.inputs:renderProductPathRight"),
            ("ctx.outputs:context", "img_l.inputs:context"),
            ("ctx.outputs:context", "img_r.inputs:context"),
            ("ctx.outputs:context", "raw_l.inputs:context"),
            ("ctx.outputs:context", "depth_l.inputs:context"),
            ("ctx.outputs:context", "info.inputs:context"),
        ],
    })

# TF · 오도메트리 · /clock — staticPublisher 는 끈다(VOLATILE 이라야 컨테이너로 간다)
og.Controller.edit(
    {"graph_path": "/World/TfGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("odom_pub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("tf_odom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("tf_l", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("tf_r", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        keys.SET_VALUES: [
            ("odom.inputs:chassisPrim", [BODY]),     # 바퀴 로봇의 섀시 = 4족의 몸통
            ("odom_pub.inputs:topicName", "odom"),
            ("odom_pub.inputs:odomFrameId", "odom"),
            ("odom_pub.inputs:chassisFrameId", "base_link"),
            ("tf_odom.inputs:parentFrameId", "odom"),
            ("tf_odom.inputs:childFrameId", "base_link"),
            ("tf_odom.inputs:staticPublisher", False),
            ("tf_l.inputs:parentFrameId", "base_link"),
            ("tf_l.inputs:childFrameId", "front_stereo_camera_left"),
            ("tf_l.inputs:translation", T_L), ("tf_l.inputs:rotation", Q_L),
            ("tf_l.inputs:staticPublisher", False),
            ("tf_r.inputs:parentFrameId", "base_link"),
            ("tf_r.inputs:childFrameId", "front_stereo_camera_right"),
            ("tf_r.inputs:translation", T_R), ("tf_r.inputs:rotation", Q_R),
            ("tf_r.inputs:staticPublisher", False),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "odom.inputs:execIn"),
            ("tick.outputs:tick", "odom_pub.inputs:execIn"),
            ("tick.outputs:tick", "tf_odom.inputs:execIn"),
            ("tick.outputs:tick", "tf_l.inputs:execIn"),
            ("tick.outputs:tick", "tf_r.inputs:execIn"),
            ("tick.outputs:tick", "clock.inputs:execIn"),
            ("ctx.outputs:context", "odom_pub.inputs:context"),
            ("ctx.outputs:context", "tf_odom.inputs:context"),
            ("ctx.outputs:context", "tf_l.inputs:context"),
            ("ctx.outputs:context", "tf_r.inputs:context"),
            ("ctx.outputs:context", "clock.inputs:context"),
            ("odom.outputs:position", "tf_odom.inputs:translation"),
            ("odom.outputs:orientation", "tf_odom.inputs:rotation"),
            ("odom.outputs:position", "odom_pub.inputs:position"),
            ("odom.outputs:orientation", "odom_pub.inputs:orientation"),
            ("odom.outputs:linearVelocity", "odom_pub.inputs:linearVelocity"),
            ("odom.outputs:angularVelocity", "odom_pub.inputs:angularVelocity"),
            ("sim_time.outputs:simulationTime", "odom_pub.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "tf_odom.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "tf_l.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "tf_r.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "clock.inputs:timeStamp"),
        ],
    })

# /cmd_vel — 바퀴 로봇과 달리 **파이썬에서 읽어** 정책에 넘긴다.
# DifferentialController 는 바퀴용이라 쓸 수 없다.
og.Controller.edit(
    {"graph_path": "/World/CmdVelGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("cmd_sub", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ],
        keys.SET_VALUES: [("cmd_sub.inputs:topicName", "cmd_vel")],
        keys.CONNECT: [
            ("tick.outputs:tick", "cmd_sub.inputs:execIn"),
            ("ctx.outputs:context", "cmd_sub.inputs:context"),
        ],
    })

# ───────────────────────── 연결 검사 — 값·실행 포트를 모두 본다
CRITICAL = [
    ("/World/StereoGraph", "img_l.inputs:renderProductPath"),
    ("/World/StereoGraph", "img_r.inputs:renderProductPath"),
    ("/World/StereoGraph", "raw_l.inputs:renderProductPath"),
    ("/World/StereoGraph", "depth_l.inputs:renderProductPath"),
    ("/World/StereoGraph", "info.inputs:renderProductPath"),
    ("/World/StereoGraph", "info.inputs:renderProductPathRight"),
    ("/World/TfGraph", "tf_odom.inputs:translation"),
    ("/World/TfGraph", "tf_odom.inputs:rotation"),
    ("/World/TfGraph", "odom.inputs:execIn"),
    ("/World/TfGraph", "clock.inputs:timeStamp"),
    ("/World/CmdVelGraph", "cmd_sub.inputs:execIn"),
]
print("\n═══ 연결 검사 ═══", flush=True)
bad = [a for _, a in CRITICAL]
bad = []
for g, attr in CRITICAL:
    n = len(og.Controller.attribute(f"{g}/{attr}").get_upstream_connections())
    print(f"  {'✅' if n else '❌'} {attr:<36} 상류 {n}", flush=True)
    if n == 0:
        bad.append(attr)
if bad:
    raise RuntimeError(f"연결 안 됨: {bad}")

# ─────────────────────────────────────────────────────────── 실행
LIN = og.Controller.attribute("/World/CmdVelGraph/cmd_sub.outputs:linearVelocity")
ANG = og.Controller.attribute("/World/CmdVelGraph/cmd_sub.outputs:angularVelocity")


def on_physics(step_size):
    """★ 물리 콜백 — 매 물리 스텝(500 Hz)마다 정책을 갱신한다.

    렌더 루프(50 Hz)에서 부르면 정책이 1/10 속도로 돌아 로봇이 넘어진다.
    """
    lin, ang = LIN.get(), ANG.get()
    runner.step(step_size, [float(lin[0]), float(lin[1]), float(ang[2])])


world.add_physics_callback("locomotion_policy", on_physics)
world.reset()
runner.initialize()
app_utils.play()
print(f"""
═══ {ROBOT} 보행 + 스테레오 발행 시작 ═══
  front_stereo_camera/left|right/image_rect_color · camera_info
  front_stereo_camera/left/image_raw · depth/ground_truth
  /clock · /odom · /tf · /cmd_vel(구독)

  해상도 {WIDTH}x{HEIGHT}   베이스라인 {BASELINE * 100:.1f} cm

  ★ 4족은 **옆걸음(linear.y)** 도 된다 — 바퀴 로봇에는 없는 자유도다
""", flush=True)

s = 0
try:
    while True:
        # 정책 갱신은 물리 콜백이 맡는다. 여기서는 렌더만 돌린다.
        world.step(render=True)
        s += 1
        if s % 250 == 0:
            xf = UsdGeom.XformCache(Usd.TimeCode.Default())
            p = xf.GetLocalToWorldTransform(
                stage.GetPrimAtPath(BODY)).ExtractTranslation()
            lin, ang = LIN.get(), ANG.get()
            print(f"  [{s:7d}] 위치 x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:+.2f}"
                  f"  명령 vx={lin[0]:+.2f} vy={lin[1]:+.2f} wz={ang[2]:+.2f}",
                  flush=True)
except KeyboardInterrupt:
    print(f"\n종료 ({s} 스텝)", flush=True)
app.close()
