"""책 14장 [실습] 환경 설정 구조 예시 (b02·b03·b08) — import 검증본.

이 블록들은 '구조를 보여 주는' 설정/뼈대 코드다. 실제 학습 가능한 환경이 되려면
자산(Articulation)·실제 reward 함수·씬 cfg 가 더 필요하다(책 뒤 장에서 완성).
여기서는 **Isaac Lab 3.0 에서 해당 클래스/심볼이 전부 존재함**을 import 로 검증한다.

원문과의 차이:
  없음 — b02(ManagerBasedRLEnvCfg)·b03(DirectRLEnv)·b08(TiledCameraCfg) 구조 그대로.
  단, 실행 시 Warp 우회를 위해 `import warp` 를 SimulationApp 보다 먼저 둔다(ENV-001).

실행:  python env_cfg_examples.py   → 모든 import 성공 여부 출력
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

import torch

# ── b02: Manager-based 환경 설정 구조 ──────────────────────
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass


@configclass
class MyRobotEnvCfg(ManagerBasedRLEnvCfg):
    @configclass
    class ObservationsCfg:
        @configclass
        class PolicyCfg(ObsGroup):
            joint_pos = ObsTerm(func="joint_pos_rel")
            joint_vel = ObsTerm(func="joint_vel_rel")
        policy: PolicyCfg = PolicyCfg()

    @configclass
    class RewardsCfg:
        alive = RewTerm(func="is_alive", weight=1.0)

    @configclass
    class TerminationsCfg:
        time_out = DoneTerm(func="time_out", time_out=True)

    observations: ObservationsCfg = ObservationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()


# ── b03: Direct 워크플로우 환경 구조 ───────────────────────
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg  # noqa


class MyRobotDirectEnv(DirectRLEnv):
    def _get_observations(self) -> dict:
        obs = torch.cat(
            [self.robot.data.joint_pos, self.robot.data.joint_vel], dim=-1
        )
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        pole = self.robot.data.joint_pos[:, self.pole_dof_idx]
        return 1.0 - torch.abs(pole)


# ── b08: Tiled Rendering 카메라 설정 ───────────────────────
from isaaclab.sensors import TiledCameraCfg

camera_cfg = TiledCameraCfg(
    prim_path="/World/envs/env_.*/Robot/Camera",
    width=80, height=80,
    data_types=["rgb", "distance_to_image_plane"],
    update_period=0.1,
)

print("✅ b02 ManagerBasedRLEnvCfg 서브클래스 생성 OK")
print("✅ b03 DirectRLEnv 서브클래스 정의 OK")
print(f"✅ b08 TiledCameraCfg 생성 OK ({camera_cfg.width}x{camera_cfg.height})")
app.close()
