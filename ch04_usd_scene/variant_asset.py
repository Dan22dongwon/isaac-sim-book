from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd
from isaacsim.storage import native as nucleus_utils
from pxr import Usd, UsdGeom, Sdf
stage = omni.usd.get_context().get_stage()
nucleus = nucleus_utils.get_assets_root_path()

# ── 1. 베이스 로봇 (Reference — 항상 로드) ────────────
robot_prim = stage.DefinePrim("/World/Robots/UR5e_Configurable", "Xform")
robot_prim.GetReferences().AddReference(
    nucleus + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"
)

# ── 2. 그리퍼 Variant Set 구성 ─────────────────────────
gripper_vs = robot_prim.GetVariantSets().AddVariantSet("gripper_config")

for variant_name, usd_suffix in [
    ("robotiq_2f85", "/Isaac/Robots/Robotiq/2F85/2f85_instanceable.usd"),
    ("robotiq_hand_e", "/Isaac/Robots/Robotiq/HandE/hand_e.usd"),
]:
    gripper_vs.AddVariant(variant_name)
    gripper_vs.SetVariantSelection(variant_name)
    with gripper_vs.GetVariantEditContext():
        gp = stage.DefinePrim(
            f"/World/Robots/UR5e_Configurable/tool0/Gripper", "Xform"
        )
        gp.GetReferences().AddReference(nucleus + usd_suffix)

# 기본 선택값 지정
gripper_vs.SetVariantSelection("robotiq_2f85")

# ── 3. 센서를 Payload로 분리 (선택적 로드) ─────────────
sensor_prim = stage.DefinePrim(
    "/World/Robots/UR5e_Configurable/tool0/Gripper/RealSense_D435",
    "Xform"
)
sensor_prim.GetPayloads().AddPayload(
    nucleus + "/Isaac/Sensors/Intel/RealSense/rsd435.usd"
)
# 기본적으로 로드 안 함 — 필요할 때만 .Load() 호출

# ── 4. 최종 저장 (재사용 가능한 자산으로) ─────────────
stage.GetRootLayer().Export("./ur5e_configurable.usd")
print("✅ Variant + Payload 최적화 자산 저장 완료")
print("   그리퍼 변형: robotiq_2f85, robotiq_hand_e")
print("   센서: Payload로 분리 (기본 미로드)")

app.close()
