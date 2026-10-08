"""책 16장 [실습] 매니퓰레이터 Lift 보상 구성 (b02) — 동작 검증본.

원문과의 차이:
  [ERR-096] import `isaaclab_tasks.manager_based.manipulation.lift.mdp` 없음
          → `isaaclab_tasks.core.lift.mdp`.
  [누락]    원문에 `from isaaclab.utils import configclass` import 가 빠져 있다 → 추가.
  [ERR-097] rigid 객체용 보상 함수 2개가 제거됨:
            `object_is_lifted`  → 직접 대응 없음. core.lift.mdp 는 `cable_lifting`/
                                  `deformable_lifting` (변형체 전용) 만 제공.
                                  rigid lift 보상은 사용자 정의가 필요하다.
            `object_goal_distance` → `object_reached_goal` (단, dense→sparse 로 의미 다름).

검증 결과 (SimulationApp 내):
  core.lift.mdp: object_ee_distance·action_rate_l2·joint_vel_l2·object_reached_goal 존재 ✅
                 object_is_lifted·object_goal_distance 없음 ❌ (제거 확인)

실행:  python lift_rewards.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass  # [누락] 원문에 없던 import
import isaaclab_tasks.core.lift.mdp as mdp  # [ERR-096]


@configclass
class LiftRewardsCfg:
    reaching_object = RewTerm(
        func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0,
    )
    # [ERR-097] object_is_lifted 제거됨 — rigid 객체용 직접 대응이 없다.
    #   변형체면 cable_lifting/deformable_lifting, rigid 면 사용자 정의 필요.
    # [ERR-097] object_goal_distance → object_reached_goal (sparse)
    object_goal_tracking = RewTerm(
        func=mdp.object_reached_goal, params={"threshold": 0.04}, weight=16.0,
    )
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-1e-4)


if __name__ == "__main__":
    cfg = LiftRewardsCfg()
    terms = [t for t in vars(cfg) if not t.startswith("_")]
    print(f"✅ LiftRewardsCfg 생성 OK — {len(terms)}항목: {terms}")
    print("   (object_is_lifted 는 제거됨 — rigid lift 보상은 사용자 정의 필요)")
    app.close()
