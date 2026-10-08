"""책 15장 [실습] 도메인 랜덤화 설정 (b02) — 동작 검증본.

원문과의 차이:
  [ERR-097] 원문의 관측 노이즈를 EventTerm(func=mdp.add_gaussian_noise) 로 건다.
            `isaaclab.envs.mdp` 에 add_gaussian_noise 함수가 없다.
            Isaac Lab 3.0 에서 관측 노이즈는 **이벤트가 아니라 관측항의 noise 필드**로
            건다 → ObsTerm(..., noise=GaussianNoiseCfg(mean=0, std=0.02)).
          → add_obs_noise EventTerm 을 제거하고, 올바른 방식 예시를 주석/함수로 제공.

  나머지 세 이벤트(randomize_rigid_body_mass·material, push_by_setting_velocity)는
  `isaaclab.envs.mdp` 에 존재한다 — 원문 그대로 동작.

검증 결과 (SimulationApp 내):
  randomize_rigid_body_mass·randomize_rigid_body_material·push_by_setting_velocity 존재 ✅
  add_gaussian_noise 없음 ❌ → GaussianNoiseCfg import 로 대체 방식 확인.

실행:  python domain_randomization.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.utils import configclass
from isaaclab.utils.noise import GaussianNoiseCfg
import isaaclab.envs.mdp as mdp


@configclass
class DomainRandomizationCfg:
    """물리 파라미터 무작위화 설정 (이벤트)."""

    randomize_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass, mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "mass_distribution_params": (0.5, 1.5),
            "operation": "scale",
        },
    )
    randomize_friction = EventTerm(
        func=mdp.randomize_rigid_body_material, mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.7, 1.3),
            "dynamic_friction_range": (0.7, 1.3),
        },
    )
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity, mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )
    # [ERR-097] 관측 노이즈는 EventTerm 이 아니다. 아래처럼 관측항에 직접 건다:
    #   joint_pos = ObsTerm(func=mdp.joint_pos_rel,
    #                       noise=GaussianNoiseCfg(mean=0.0, std=0.02))


if __name__ == "__main__":
    cfg = DomainRandomizationCfg()
    events = [t for t in vars(cfg) if not t.startswith("_")]
    print(f"✅ DomainRandomizationCfg 생성 OK — 이벤트 {len(events)}개: {events}")
    # ERR-097 대체 방식이 유효한지: GaussianNoiseCfg 로 관측항 생성
    noisy = ObsTerm(func=mdp.joint_pos_rel,
                    noise=GaussianNoiseCfg(mean=0.0, std=0.02))
    print(f"✅ 관측 노이즈 올바른 방식 OK — ObsTerm(noise=GaussianNoiseCfg) : "
          f"{type(noisy.noise).__name__}")
    app.close()
