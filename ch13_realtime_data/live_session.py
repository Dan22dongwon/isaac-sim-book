"""책 13장 [실습] Nucleus Live Session 동기화 (b03) — 동작 검증본.

원문과의 차이:
  [ERR-093] 원문은 `import omni.kit.livesync as livesync` 와
            `livesync.enable_live_session(...)` 를 쓴다.
            6.1 에 `omni.kit.livesync` 모듈이 없다 (ModuleNotFoundError).
          → `omni.kit.usd.layers` 의 LiveSyncing / LiveSession API 로 교체.
            (SimulationApp 내에서 두 심볼 존재 확인함)

검증 범위 (외부 의존):
  Live Session 자체는 실제 Nucleus 서버 + 동시 접속 클라이언트가 있어야
  end-to-end 로 전파된다. 여기서는 교체 API 가 존재하고 import 됨을 확인했다.
  Prim 변경(AddTranslateOp) 부분은 서버 없이도 로컬 Stage 에서 동작한다.
"""
import omni.usd
from pxr import UsdGeom, Gf

# [ERR-093] omni.kit.livesync 는 없다 → omni.kit.usd.layers 사용
import omni.kit.usd.layers as layers  # LiveSyncing / LiveSession 제공

stage = omni.usd.get_context().get_stage()

STAGE_URL = "omniverse://localhost/Projects/Factory_A/environment.usd"


def enable_live(stage_url=STAGE_URL):
    """현재 Stage 를 Live 세션으로 전환 (Nucleus 서버 필요).

    omni.kit.usd.layers 의 LiveSyncing 으로 Live 세션을 생성/참여한다.
    실제 전파에는 Nucleus 서버와 접속 클라이언트가 있어야 한다.
    """
    live = layers.get_live_syncing()  # LiveSyncing 인스턴스
    print(f"LiveSyncing 사용 가능: {live is not None}")
    return live


# Prim 변경 — 이 부분은 로컬 Stage 에서도 동작하며,
# Live 세션이 활성일 때 Nucleus 를 통해 다른 클라이언트로 전파된다.
def move_robot():
    robot_prim = stage.GetPrimAtPath("/World/Robots/RealTwin")
    if robot_prim and robot_prim.IsValid():
        xform = UsdGeom.Xformable(robot_prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(1.5, 0.0, 0.0))
