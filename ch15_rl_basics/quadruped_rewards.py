"""책 15장 [실습] 4족 보행 보상 함수 구성 (b01) — 동작 검증본.

원문과의 차이:
  [ERR-096] 원문 import `isaaclab_tasks.manager_based.locomotion.mdp` 경로가 없다.
          → `isaaclab_tasks.core.velocity.mdp` 로 교체. 이 경로에 b01 이 쓰는
            여섯 보상 함수가 **전부** 들어 있다.
  [ERR-097 정정] 이전 검증에서 `feet_air_time_positive_biped` 가 "없다"고 보고했으나
          이는 **검증자 오류**였다 — 틀린 모듈(envs.mdp/locomotion.mdp)에서 찾았다.
          `core.velocity.mdp` 에 실제로 존재한다. 원문 함수명은 옳았다.

검증 결과 (SimulationApp 내):
  core.velocity.mdp 의 6개 함수 모두 존재 →
  track_lin_vel_xy_exp·track_ang_vel_z_exp·lin_vel_z_l2·joint_torques_l2·
  action_rate_l2·feet_air_time_positive_biped. QuadrupedRewardsCfg 생성 OK.

실행:  python quadruped_rewards.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass
# [ERR-096] manager_based.locomotion.mdp → core.velocity.mdp
import isaaclab_tasks.core.velocity.mdp as mdp


@configclass
class QuadrupedRewardsCfg:
    """4족 보행 보상 함수 구성 — 여러 항목의 가중 합."""

    track_lin_vel_xy = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=1.5,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    track_ang_vel_z = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=0.75,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    lin_vel_z_penalty = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    joint_torque_penalty = RewTerm(func=mdp.joint_torques_l2, weight=-0.00002)
    action_rate_penalty = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    # [ERR-097 정정] feet_air_time_positive_biped 는 core.velocity.mdp 에 존재한다
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped, weight=0.25,
        params={"command_name": "base_velocity", "threshold": 0.5},
    )


if __name__ == "__main__":
    cfg = QuadrupedRewardsCfg()
    terms = [t for t in vars(cfg) if not t.startswith("_")]
    print(f"✅ QuadrupedRewardsCfg 생성 OK — 보상 항목 {len(terms)}개: {terms}")
    for t in terms:
        fn = getattr(cfg, t).func
        print(f"   {t}: {fn.__module__}.{fn.__name__}")
    app.close()
