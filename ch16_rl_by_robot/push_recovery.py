"""책 16장 [실습] Push Recovery 외란 이벤트 (b05) — 동작 검증본.

원문과의 차이:
  [누락] `from isaaclab.utils import configclass` 와 `import isaaclab.envs.mdp as mdp`
         가 원문에 빠져 있다 → 추가. (원문은 앞 블록 import 를 가정한 스니펫)

검증 결과:
  mdp.push_by_setting_velocity 존재 ✅ — EventTerm cfg 생성 확인.

실행:  python push_recovery.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.utils import configclass  # [누락]
import isaaclab.envs.mdp as mdp          # [누락]


@configclass
class PushRecoveryEventsCfg:
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(4.0, 8.0),
        params={"velocity_range": {"x": (-1.5, 1.5), "y": (-1.5, 1.5)}},
    )


if __name__ == "__main__":
    cfg = PushRecoveryEventsCfg()
    print(f"✅ PushRecoveryEventsCfg 생성 OK — "
          f"push {cfg.push_robot.interval_range_s}s 간격, "
          f"func={cfg.push_robot.func.__name__}")
    app.close()
