"""책 16장 [실습] 비대칭 Actor/Critic 관측 (b07) — 동작 검증본.

원문과의 차이:
  [누락]   원문에 configclass·ObsGroup·ObsTerm·mdp import 가 빠져 있다 → 추가.
  [ERR-097] Critic 의 `mdp.terrain_height` 함수가 없다 → 지형 높이 관측은
            `mdp.height_scan` (레이캐스터 기반) 을 쓴다. 여기서는 해당 항목을
            height_scan 으로 교체(파라미터는 센서 cfg 필요하므로 주석 처리).

검증 결과:
  joint_pos_rel·joint_vel_rel·base_ang_vel·projected_gravity·base_lin_vel·
  contact_forces 존재 ✅ / terrain_height 없음 ❌ → height_scan 사용.

실행:  python asymmetric_obs.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils import configclass
import isaaclab.envs.mdp as mdp


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """Actor 관측 — 실제 로봇 센서로 얻을 수 있는 정보만."""
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)

    @configclass
    class CriticCfg(ObsGroup):
        """Critic 관측 — 시뮬레이션 특권 정보 추가 활용."""
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)      # 실제 로봇은 측정 어려움
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        contact_forces = ObsTerm(func=mdp.contact_forces)  # 특권 정보
        # [ERR-097] terrain_height 없음 → height_scan (센서 cfg params 필요):
        #   terrain_height = ObsTerm(func=mdp.height_scan,
        #                            params={"sensor_cfg": SceneEntityCfg("height_scanner")})

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


if __name__ == "__main__":
    cfg = ObservationsCfg()
    pol = [t for t in vars(cfg.policy) if not t.startswith("_")]
    cri = [t for t in vars(cfg.critic) if not t.startswith("_")]
    print(f"✅ 비대칭 관측 생성 OK — Actor {len(pol)}항목, Critic {len(cri)}항목")
    print(f"   Critic 전용 특권정보: {set(cri) - set(pol)}")
    app.close()
