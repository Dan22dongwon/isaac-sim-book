"""9.7.5 수정본 — Isaac Sim 스테레오 카메라 → cuVSLAM.

Implements: 9.7.5 / ERR-060, ERR-061

원고와 다른 점 세 가지.

1. [ERR-061] 카메라를 **새로 만들지 않는다.** NovaCarter 자산에 스테레오가
   이미 4쌍 들어 있다. front_hawk 쌍에 발행 그래프만 붙인다.

       /nova_carter/chassis_link/sensors/front_hawk/left/camera_left
       /nova_carter/chassis_link/sensors/front_hawk/right/camera_right

2. [ERR-061] 토픽 이름이 원고와 다르다. 런치 파일이 기다리는 이름에 맞춘다.

       원고   /left/image_raw
       실제   front_stereo_camera/left/image_rect_color  +  .../camera_info

3. [ERR-061] `camera_info` 가 **필수**다. cuVSLAM 은 여기서 초점거리·주점과
   스테레오 베이스라인을 읽는다. 없으면 영상이 와도 거리를 계산하지 못한다.

8·9장에서 얻은 교훈도 그대로 적용한다.

- [ERR-054] 렌더 프로덕트는 **그래프 안에서** IsaacCreateRenderProduct 로 만든다
- [ERR-057] 타임스탬프를 연결하지 않으면 수신 측이 프레임을 전부 버린다
- [ERR-059] 포트 타입이 다르면 경고만 뜨고 값이 0 이다 — 연결 수를 확인한다
"""
import os

from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
app = SimulationApp({"headless": HEADLESS})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.bridge")
app.update()

import numpy as np                                        # noqa: E402
import omni.graph.core as og                              # noqa: E402
import omni.usd                                           # noqa: E402
from isaacsim.core.api import World                       # noqa: E402
from isaacsim.core.prims import Articulation              # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics      # noqa: E402

# ─────────────────────────────────────────────────────────── 씬
#
# SCENE=warehouse  → NVIDIA 공식 창고 자산 (기본, 권장)
# SCENE=room       → 직접 만든 빈 방 (대조용)
#
# 왜 창고가 기본인가 — **비주얼 SLAM 은 무늬를 먹고 산다.**
# 직접 만든 방(민무늬 벽 + 단색 상자)에서는 벽을 정면으로 마주 보는 순간
# 특징점이 사라져 추적이 끊긴다. 실제로 그렇게 끊겼다.
#
#     [ERROR] Can't select enough new tracks to reach FAILED_ACTIVE_TRACK_COUNT limit
#     [ERROR] Failed to track on the PnP stage
#
# 창고 자산은 선반·팔레트·바닥표시·기둥이 있어 어느 방향을 봐도 특징점이 있다.
# 2D 라이다(9.1~9.6)는 거리만 재므로 민무늬라도 상관없었다. 여기서 갈린다.
SCENE = os.environ.get("SCENE", "warehouse")

World.clear_instance()
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60)
stage = omni.usd.get_context().get_stage()
nucleus = get_assets_root_path()

WAREHOUSES = {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "forklift": "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "shelves": "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
    "twin": "/Isaac/Environments/Digital_Twin_Warehouse/small_warehouse_digital_twin.usd",
}

if SCENE in WAREHOUSES:
    add_reference_to_stage(usd_path=nucleus + WAREHOUSES[SCENE],
                           prim_path="/World/Env")
    START = (-6.0, 0.0, 0.2)
    print(f"\n환경: {WAREHOUSES[SCENE].split('/')[-1]}", flush=True)
else:
    # 대조용 빈 방 — 특징점이 부족해 추적이 끊기는 것을 보여 주는 용도
    world.scene.add_default_ground_plane()
    ROOM_X, ROOM_Y, WALL_T, WALL_H = 12.0, 9.0, 0.3, 2.0
    for i, (x, y, sx, sy) in enumerate([
            (0, ROOM_Y / 2, ROOM_X, WALL_T), (0, -ROOM_Y / 2, ROOM_X, WALL_T),
            (ROOM_X / 2, 0, WALL_T, ROOM_Y), (-ROOM_X / 2, 0, WALL_T, ROOM_Y)]):
        c = UsdGeom.Cube.Define(stage, f"/World/Room/Wall{i}")
        c.GetSizeAttr().Set(1.0)
        a = UsdGeom.XformCommonAPI(c.GetPrim())
        a.SetTranslate(Gf.Vec3d(x, y, WALL_H / 2))
        a.SetScale(Gf.Vec3f(sx, sy, WALL_H))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    for i, (x, y, sx, sy, sz) in enumerate([
            (-3.0, 2.0, 0.8, 0.8, 1.6), (2.5, -2.0, 1.2, 0.6, 1.0),
            (3.5, 2.5, 0.6, 1.6, 1.4), (-2.0, -2.8, 1.0, 1.0, 0.8),
            (0.5, 3.0, 1.4, 0.5, 1.2), (-4.5, -1.0, 0.7, 1.2, 1.8)]):
        c = UsdGeom.Cube.Define(stage, f"/World/Props/Box{i}")
        c.GetSizeAttr().Set(1.0)
        a = UsdGeom.XformCommonAPI(c.GetPrim())
        a.SetTranslate(Gf.Vec3d(x, y, sz / 2))
        a.SetScale(Gf.Vec3f(sx, sy, sz))
        c.GetDisplayColorAttr().Set([Gf.Vec3f(*[(0.25 + 0.12 * i) % 1.0,
                                                (0.70 - 0.11 * i) % 1.0,
                                                (0.45 + 0.09 * i) % 1.0])])
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    START = (-4.0, -3.0, 0.1)
    print("\n환경: 직접 만든 빈 방 (대조용 — 특징점 부족)", flush=True)

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr().Set(1200.0)

CARTER = "/World/Carter"
add_reference_to_stage(
    usd_path=nucleus + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
    prim_path=CARTER)
UsdGeom.XformCommonAPI(stage.GetPrimAtPath(CARTER)).SetTranslate(Gf.Vec3d(*START))

# ───────────────────────────────────── [ERR-061] 이미 있는 스테레오 쌍을 찾는다
CHASSIS = next(
    (str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(CARTER))
     if "chassis" in p.GetName().lower()), CARTER)


def find_camera(*name_parts):
    """이름 조각을 모두 포함하는 Camera 프림 경로를 찾는다.

    경로를 하드코딩하지 않는다 — 자산 버전마다 계층이 달라진다(9장 교훈).
    """
    for p in Usd.PrimRange(stage.GetPrimAtPath(CARTER)):
        if p.GetTypeName() != "Camera":
            continue
        path = str(p.GetPath()).lower()
        if all(part in path for part in name_parts):
            return str(p.GetPath())
    return None


CAM_L = find_camera("front_hawk", "left")
CAM_R = find_camera("front_hawk", "right")

print("\n═══ 스테레오 카메라 ═══", flush=True)
print(f"  섀시   {CHASSIS}", flush=True)
print(f"  왼쪽   {CAM_L}", flush=True)
print(f"  오른쪽 {CAM_R}", flush=True)
if not (CAM_L and CAM_R):
    raise RuntimeError("front_hawk 스테레오 쌍을 찾지 못했다 — 자산 구조 확인 필요")

# ★ Hawk 카메라는 **어안 150°** 다 — 핀홀로 바꾸지 않으면 추적이 틀어진다
#
#   자산 기본값:
#       cameraProjectionType    = fisheyePolynomial
#       fthetaMaxFov            = 150.0
#       physicalDistortionModel = rational_polynomial
#
#   그런데 cuVSLAM 런치 파일은 rectified_images: True 이고,
#   Isaac Sim 의 ROS2CameraInfoHelper 가 내보내는 camera_info 는 **핀홀**이다
#   (fx=319.52, cx=320, cy=240 — 150° 어안을 설명하지 못한다).
#
#   즉 **영상과 camera_info 가 서로 다른 렌즈를 말한다.** 영상은 휘어 있는데
#   수신 측은 곧다고 믿으므로 특징점 기하가 어긋난다. 토픽 이름이
#   image_rect_color 라고 해서 왜곡이 펴지는 것이 아니다.
#
#   렌더를 핀홀로 되돌려 camera_info 와 맞춘다.
def force_pinhole(cam_path):
    """어안 투영을 끄고 핀홀로 렌더하게 한다."""
    prim = stage.GetPrimAtPath(cam_path)
    a = prim.GetAttribute("cameraProjectionType")
    before = a.Get() if a and a.IsValid() else None
    if a and a.IsValid():
        a.Set("pinhole")
    return before


for _c in (CAM_L, CAM_R):
    _b = force_pinhole(_c)
print(f"  투영 {_b} → pinhole (어안이면 camera_info 와 어긋난다)", flush=True)

# 베이스라인 실측 — camera_info 의 스케일이 여기서 나온다
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
pl = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(CAM_L)).ExtractTranslation()
pr = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(CAM_R)).ExtractTranslation()
baseline = float(np.linalg.norm(np.array(pl) - np.array(pr)))
print(f"  베이스라인 {baseline * 100:.2f} cm", flush=True)

# ───────────────────────────── 카메라 외부 파라미터 (base_link 기준)
#
# cuVSLAM 은 base_link → 카메라 TF 가 **없으면 죽는다.** 경고가 아니라 중단이다.
#
#     Invalid frame ID "base_link" ... frame does not exist
#     terminate called: Could not find the requested transform!
#     process has died [exit code -6]
#
# 카메라가 로봇 어디에 붙어 있는지 알아야 카메라 포즈를 **로봇 포즈**로
# 바꿔 줄 수 있기 때문이다. 원고에는 이 요구사항이 한 줄도 없다.
#
# USD 카메라는 **-Z 가 정면, +Y 가 위**다. ROS 광학 프레임은 **+Z 가 정면,
# +Y 가 아래**다. 그래서 X 축 180° 회전을 한 번 걸어 준다.
xc2 = UsdGeom.XformCache(Usd.TimeCode.Default())
T_base = xc2.GetLocalToWorldTransform(stage.GetPrimAtPath(CHASSIS))
FLIP = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0))


def extrinsics(cam_path):
    """base_link(섀시) 기준 카메라 광학 프레임의 위치·자세를 낸다."""
    T_cam = xc2.GetLocalToWorldTransform(stage.GetPrimAtPath(cam_path))
    T_rel = FLIP * (T_cam * T_base.GetInverse())
    t = T_rel.ExtractTranslation()
    q = T_rel.ExtractRotationQuat()          # Gf.Quatd
    i = q.GetImaginary()
    # quatd[4] 는 (I, J, K, R) 순서다 — w 를 뒤에 둔다
    return [float(t[0]), float(t[1]), float(t[2])], \
           [float(i[0]), float(i[1]), float(i[2]), float(q.GetReal())]


T_L, Q_L = extrinsics(CAM_L)
T_R, Q_R = extrinsics(CAM_R)
print(f"  왼쪽  base_link 기준 {[round(v, 4) for v in T_L]}", flush=True)
print(f"  오른쪽 base_link 기준 {[round(v, 4) for v in T_R]}", flush=True)

# ─────────────────────────────────────────────────────────── 그래프
keys = og.Controller.Keys

# ★ 640x480 으로 두면 무거운 씬에서 **렌더 파이프라인이 크래시**한다.
#
#     [omni.rtx] DLSS increasing input dimensions:
#                Render resolution of (320, 240) is below minimum
#     [crash] A crash has occurred.
#
#   DLSS 는 내부적으로 **절반 해상도**(320x240)로 렌더한 뒤 확대하는데,
#   그 값이 DLSS 최소치 미만이라 깨진다. 가벼운 씬에서는 DLSS 경로를
#   타지 않아 드러나지 않다가, 창고처럼 무거운 씬에서 터진다.
#
#   cuVSLAM 문서도 "VGA 이상 권장" 이라고 한다. 720p 로 올린다.
WIDTH, HEIGHT = 1280, 720


# ★ 스테레오 camera_info 는 **노드 하나가 두 카메라를 함께** 받아야 한다.
#
#   ROS2CameraInfoHelper 에는 오른쪽 전용 입력이 따로 있다.
#       inputs:renderProductPathRight  ·  inputs:topicNameRight  ·  inputs:frameIdRight
#
#   왼쪽·오른쪽에 CameraInfoHelper 를 **하나씩** 두면 각자 "나는 단안" 이라고
#   발행해서 **P[3] = 0** 이 된다. P[3] = -fx × 베이스라인 이므로 0 이면
#   cuVSLAM 이 베이스라인을 0 으로 읽고 **거리(scale)를 계산하지 못한다.**
#   영상은 정상적으로 흐르기 때문에 증상만 보고는 원인을 찾기 어렵다.
og.Controller.edit(
    {"graph_path": "/World/StereoGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("run", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            # [ERR-054] 렌더 프로덕트는 그래프 **안에서** 만든다
            ("rp_l", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("rp_r", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("img_l", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("img_r", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),  # 하나뿐
            # ── 9.7.6 nvblox 용 ──────────────────────────────
            # nvblox 는 cuVSLAM 과 **다른 토픽 이름**을 기다린다.
            #   색   front_stereo_camera/left/image_raw        (rect 가 아니다)
            #   깊이 front_stereo_camera/depth/ground_truth
            # 같은 렌더 프로덕트에서 함께 뽑는다.
            ("raw_l", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("depth_l", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        keys.SET_VALUES: [
            ("rp_l.inputs:cameraPrim", [CAM_L]),
            ("rp_l.inputs:width", WIDTH), ("rp_l.inputs:height", HEIGHT),
            ("rp_r.inputs:cameraPrim", [CAM_R]),
            ("rp_r.inputs:width", WIDTH), ("rp_r.inputs:height", HEIGHT),
            # [ERR-061] 원고의 image_raw 가 아니라 image_rect_color 다
            ("img_l.inputs:topicName", "front_stereo_camera/left/image_rect_color"),
            ("img_l.inputs:type", "rgb"),
            ("img_l.inputs:frameId", "front_stereo_camera_left"),
            ("img_l.inputs:useSystemTime", False),
            ("img_r.inputs:topicName", "front_stereo_camera/right/image_rect_color"),
            ("img_r.inputs:type", "rgb"),
            ("img_r.inputs:frameId", "front_stereo_camera_right"),
            ("img_r.inputs:useSystemTime", False),
            # 한 노드가 좌·우를 모두 발행한다 → P[3] 에 베이스라인이 들어간다
            ("info.inputs:topicName", "front_stereo_camera/left/camera_info"),
            ("info.inputs:frameId", "front_stereo_camera_left"),
            ("info.inputs:topicNameRight", "front_stereo_camera/right/camera_info"),
            ("info.inputs:frameIdRight", "front_stereo_camera_right"),
            ("info.inputs:useSystemTime", False),
            # nvblox 용 — 이름만 다르고 소스는 같은 렌더 프로덕트다
            ("raw_l.inputs:topicName", "front_stereo_camera/left/image_raw"),
            ("raw_l.inputs:type", "rgb"),
            ("raw_l.inputs:frameId", "front_stereo_camera_left"),
            ("raw_l.inputs:useSystemTime", False),
            ("depth_l.inputs:topicName", "front_stereo_camera/depth/ground_truth"),
            ("depth_l.inputs:type", "depth"),
            ("depth_l.inputs:frameId", "front_stereo_camera_left"),
            ("depth_l.inputs:useSystemTime", False),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "run.inputs:execIn"),
            ("run.outputs:step", "rp_l.inputs:execIn"),
            ("run.outputs:step", "rp_r.inputs:execIn"),
            ("rp_l.outputs:execOut", "img_l.inputs:execIn"),
            ("rp_r.outputs:execOut", "img_r.inputs:execIn"),
            ("rp_r.outputs:execOut", "info.inputs:execIn"),
            ("rp_l.outputs:renderProductPath", "img_l.inputs:renderProductPath"),
            ("rp_r.outputs:renderProductPath", "img_r.inputs:renderProductPath"),
            # ★ 좌·우 렌더 프로덕트를 **같은 노드**에 넣는다
            ("rp_l.outputs:renderProductPath", "info.inputs:renderProductPath"),
            ("rp_r.outputs:renderProductPath", "info.inputs:renderProductPathRight"),
            ("ctx.outputs:context", "img_l.inputs:context"),
            ("ctx.outputs:context", "img_r.inputs:context"),
            ("ctx.outputs:context", "info.inputs:context"),
            # nvblox 용 — 왼쪽 렌더 프로덕트를 공유한다
            ("rp_l.outputs:execOut", "raw_l.inputs:execIn"),
            ("rp_l.outputs:execOut", "depth_l.inputs:execIn"),
            ("rp_l.outputs:renderProductPath", "raw_l.inputs:renderProductPath"),
            ("rp_l.outputs:renderProductPath", "depth_l.inputs:renderProductPath"),
            ("ctx.outputs:context", "raw_l.inputs:context"),
            ("ctx.outputs:context", "depth_l.inputs:context"),
        ],
    })

# TF 그래프 — odom → base_link → 카메라
#
# ★ staticPublisher 를 **끈다.** True 로 두면 QoS 가 TRANSIENT_LOCAL 이 되는데,
#   그 상태에서는 컨테이너(다른 ROS 배포판) 쪽으로 데이터가 넘어가지 않았다.
#   영상·클록은 VOLATILE 이라 잘 건너간다. TF 도 같은 조건으로 맞춘다.
#
#       staticPublisher=True   → TRANSIENT_LOCAL → 컨테이너 수신 ❌
#       staticPublisher=False  → VOLATILE        → 컨테이너 수신 ✅
#
#   카메라는 고정이지만 매 틱 내보내도 부담이 없다. 어차피 odom→base_link 는
#   동적이라 계속 내보내야 한다.
og.Controller.edit(
    {"graph_path": "/World/TfGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            # [ERR-055] wheeled_robots 가 아니라 core.nodes 다
            ("odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("odom_pub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("tf_odom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("tf_l", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("tf_r", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ],
        keys.SET_VALUES: [
            # ★ 루트(CARTER)가 아니라 **섀시 링크**를 줘야 한다.
            #   루트 Xform 을 주면 값이 계속 0 으로 나온다. 문서에는
            #   "articulation root 또는 rigid body prim" 이라고 돼 있지만,
            #   실제로 움직이는 값을 얻으려면 리지드바디인 섀시를 지정한다.
            ("odom.inputs:chassisPrim", [CHASSIS]),
            ("odom_pub.inputs:topicName", "odom"),
            ("odom_pub.inputs:odomFrameId", "odom"),
            ("odom_pub.inputs:chassisFrameId", "base_link"),
            # odom → base_link (동적) — nvblox 가 로봇 위치를 읽는 경로다
            ("tf_odom.inputs:parentFrameId", "odom"),
            ("tf_odom.inputs:childFrameId", "base_link"),
            ("tf_odom.inputs:staticPublisher", False),
            # base_link → 카메라 (고정이지만 VOLATILE 로 매 틱)
            ("tf_l.inputs:parentFrameId", "base_link"),
            ("tf_l.inputs:childFrameId", "front_stereo_camera_left"),
            ("tf_l.inputs:translation", T_L),
            ("tf_l.inputs:rotation", Q_L),
            ("tf_l.inputs:staticPublisher", False),
            ("tf_r.inputs:parentFrameId", "base_link"),
            ("tf_r.inputs:childFrameId", "front_stereo_camera_right"),
            ("tf_r.inputs:translation", T_R),
            ("tf_r.inputs:rotation", Q_R),
            ("tf_r.inputs:staticPublisher", False),
        ],
        keys.CONNECT: [
            # 실행 포트는 전부 tick 에서 직접 끈다.
            # odom.outputs:execOut 로 연쇄시켰더니 /odom 과 odom→base_link TF 가
            # 둘 다 죽었다. 노드 간 exec 연쇄는 이 버전에서 신뢰하기 어렵다.
            ("tick.outputs:tick", "odom.inputs:execIn"),
            ("tick.outputs:tick", "odom_pub.inputs:execIn"),
            ("tick.outputs:tick", "tf_odom.inputs:execIn"),
            ("tick.outputs:tick", "tf_l.inputs:execIn"),
            ("tick.outputs:tick", "tf_r.inputs:execIn"),
            ("ctx.outputs:context", "odom_pub.inputs:context"),
            ("ctx.outputs:context", "tf_odom.inputs:context"),
            ("ctx.outputs:context", "tf_l.inputs:context"),
            ("ctx.outputs:context", "tf_r.inputs:context"),
            # 오도메트리 값을 TF 로 흘린다 (원고가 9.2 에서 빠뜨린 연결)
            ("odom.outputs:position", "tf_odom.inputs:translation"),
            ("odom.outputs:orientation", "tf_odom.inputs:rotation"),
            ("odom.outputs:position", "odom_pub.inputs:position"),
            ("odom.outputs:orientation", "odom_pub.inputs:orientation"),
            ("odom.outputs:linearVelocity", "odom_pub.inputs:linearVelocity"),
            ("odom.outputs:angularVelocity", "odom_pub.inputs:angularVelocity"),
            # [ERR-057] 타임스탬프 — 없으면 수신 측이 전부 버린다
            ("sim_time.outputs:simulationTime", "odom_pub.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "tf_odom.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "tf_l.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "tf_r.inputs:timeStamp"),
        ],
    })

# /clock — [ERR-057] use_sim_time:True 인 cuVSLAM 에는 필수다
og.Controller.edit(
    {"graph_path": "/World/ClockGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "clock.inputs:execIn"),
            ("ctx.outputs:context", "clock.inputs:context"),
            ("sim_time.outputs:simulationTime", "clock.inputs:timeStamp"),
        ],
    })

# /cmd_vel → 바퀴 — [ERR-059] BreakVector3 경유 + dt 연결
WHEEL_RADIUS, WHEEL_BASE = 0.14, 0.413
og.Controller.edit(
    {"graph_path": "/World/CmdVelGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("cmd_sub", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("lin_xyz", "omni.graph.nodes.BreakVector3"),
            ("ang_xyz", "omni.graph.nodes.BreakVector3"),
            ("diff", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("art", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            ("cmd_sub.inputs:topicName", "cmd_vel"),
            ("diff.inputs:wheelRadius", WHEEL_RADIUS),
            ("diff.inputs:wheelDistance", WHEEL_BASE),
            ("art.inputs:targetPrim", [CARTER]),
            ("art.inputs:jointNames", ["joint_wheel_left", "joint_wheel_right"]),
        ],
        keys.CONNECT: [
            # DifferentialController 의 출력은 velocityCommand **하나뿐**이다.
            # execOut 이 없으므로 diff·art 를 tick 에서 각각 직접 구동한다.
            ("tick.outputs:tick", "cmd_sub.inputs:execIn"),
            ("tick.outputs:tick", "diff.inputs:execIn"),
            ("tick.outputs:tick", "art.inputs:execIn"),
            ("ctx.outputs:context", "cmd_sub.inputs:context"),
            # [ERR-059] vectord[3] → double 직결은 조용히 끊긴다
            ("cmd_sub.outputs:linearVelocity", "lin_xyz.inputs:tuple"),
            ("cmd_sub.outputs:angularVelocity", "ang_xyz.inputs:tuple"),
            ("lin_xyz.outputs:x", "diff.inputs:linearVelocity"),
            ("ang_xyz.outputs:z", "diff.inputs:angularVelocity"),
            ("tick.outputs:deltaSeconds", "diff.inputs:dt"),
            ("diff.outputs:velocityCommand", "art.inputs:velocityCommand"),
        ],
    })

# ───────────────────────── [ERR-059] 연결이 실제로 걸렸는지 확인한다
CRITICAL = [
    ("/World/CmdVelGraph", "diff.inputs:linearVelocity"),
    ("/World/CmdVelGraph", "diff.inputs:angularVelocity"),
    ("/World/CmdVelGraph", "diff.inputs:dt"),
    ("/World/StereoGraph", "img_l.inputs:renderProductPath"),
    ("/World/StereoGraph", "img_r.inputs:renderProductPath"),
    ("/World/StereoGraph", "info.inputs:renderProductPath"),
    # ★ 이게 0 이면 P[3]=0 이 되어 cuVSLAM 이 거리를 못 잡는다
    ("/World/StereoGraph", "info.inputs:renderProductPathRight"),
    ("/World/StereoGraph", "raw_l.inputs:renderProductPath"),
    ("/World/StereoGraph", "depth_l.inputs:renderProductPath"),
    ("/World/ClockGraph", "clock.inputs:timeStamp"),
    # ★ 이게 없으면 cuVSLAM 이 경고가 아니라 **중단**된다
    ("/World/TfGraph", "tf_l.inputs:timeStamp"),
    ("/World/TfGraph", "tf_r.inputs:timeStamp"),
    # ★ nvblox 가 로봇 위치를 읽는 경로
    ("/World/TfGraph", "tf_odom.inputs:translation"),
    ("/World/TfGraph", "tf_odom.inputs:rotation"),
    ("/World/TfGraph", "odom_pub.inputs:position"),
    # ★ 실행 포트도 검사한다 — 여기를 빼 놨다가 /odom 이 죽은 걸 늦게 알았다.
    #   값 연결만 확인하면 "노드가 아예 안 돈다" 를 놓친다.
    ("/World/TfGraph", "odom.inputs:execIn"),
    ("/World/TfGraph", "odom_pub.inputs:execIn"),
    ("/World/TfGraph", "tf_odom.inputs:execIn"),
    ("/World/TfGraph", "tf_l.inputs:execIn"),
    ("/World/TfGraph", "tf_r.inputs:execIn"),
    ("/World/StereoGraph", "img_l.inputs:execIn"),
    ("/World/StereoGraph", "depth_l.inputs:execIn"),
    ("/World/CmdVelGraph", "art.inputs:execIn"),
]
print("\n═══ 연결 검사 ═══", flush=True)
bad = []
for g, attr in CRITICAL:
    n = len(og.Controller.attribute(f"{g}/{attr}").get_upstream_connections())
    mark = "✅" if n else "❌"
    print(f"  {mark} {attr:<34} 상류 {n}", flush=True)
    if n == 0:
        bad.append(attr)
if bad:
    raise RuntimeError(f"연결 안 됨(타입 불일치 의심): {bad}")

world.reset()
carter = Articulation(prim_paths_expr=CARTER, name="carter")
carter.initialize()
app_utils.play()

print(f"""
═══ cuVSLAM 용 스테레오 발행 시작 ═══
  [cuVSLAM 9.7.5]
    front_stereo_camera/left|right/image_rect_color
    front_stereo_camera/left|right/camera_info
  [nvblox 9.7.6]
    front_stereo_camera/left/image_raw
    front_stereo_camera/depth/ground_truth
  [공통]  /clock · /odom · /tf(odom→base_link→카메라) · /cmd_vel(구독)

  해상도 {WIDTH}x{HEIGHT}   베이스라인 {baseline * 100:.2f} cm

컨테이너에서:
  ros2 launch isaac_ros_cuvslam isaac_ros_visual_slam_isaac_sim.launch.py
""", flush=True)

s = 0
try:
    while True:
        world.step(render=True)
        s += 1
        if s % 600 == 0:
            p, _ = carter.get_world_poses()
            print(f"  [{s:6d}] 로봇 x={p[0][0]:+.2f} y={p[0][1]:+.2f}", flush=True)
except KeyboardInterrupt:
    print(f"\n종료 ({s} 스텝)", flush=True)
app.close()
