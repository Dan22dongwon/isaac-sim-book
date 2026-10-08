"""책 4.5 조명 설정 — 동작 검증본.

원문과의 차이 (ERR-011):
    원문 : from isaacsim.core.utils.prims import set_prim_translation, set_prim_rotation
           → ImportError. 이 함수들은 Isaac Sim 6.1에 존재하지 않는다.
    수정 : UsdGeom.XformCommonAPI 로 이동/회전한다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python lighting.py
"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402

stage = omni.usd.get_context().get_stage()

# ── 1. 천장 형광등 (RectLight) ─────────────────────────────────────
rect_light = UsdLux.RectLight.Define(stage, "/World/Lights/CeilingLight_01")
rect_light.CreateIntensityAttr().Set(5000.0)
rect_light.CreateWidthAttr().Set(2.0)   # 2m 폭
rect_light.CreateHeightAttr().Set(0.5)  # 0.5m 높이
rect_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.95, 0.8))  # 따뜻한 백색

# 조명 위치 설정 (5m 높이)
# [수정 ERR-011] set_prim_translation / set_prim_rotation 은 6.1에 없다.
xform_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/Lights/CeilingLight_01"))
xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 5.0))
# 회전이 필요하면 SetRotate 를 쓴다. 단, RectLight는 회전 없이도 -Z(아래)를 비춘다.
# xform_api.SetRotate(Gf.Vec3f(0.0, 0.0, 0.0))

# ── 2. 돔 조명 (HDRI 환경 조명) ────────────────────────────────────
dome_light = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
dome_light.CreateIntensityAttr().Set(500.0)
# HDRI 텍스처 적용 (선택사항)
# dome_light.CreateTextureFileAttr().Set("path/to/hdri.hdr")

# ── 확인 ──────────────────────────────────────────────────────────
prim = stage.GetPrimAtPath("/World/Lights/CeilingLight_01")
pos = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default()
).ExtractTranslation()

print("\n✅ 조명 배치 완료")
print(f"  RectLight  위치       : {tuple(round(v, 3) for v in pos)}")
print(f"  RectLight  속성 이름  : {rect_light.GetIntensityAttr().GetName()}")
print(f"  RectLight  intensity  : {rect_light.GetIntensityAttr().Get()}")
print(f"  DomeLight  intensity  : {dome_light.GetIntensityAttr().Get()}")

app.close()
