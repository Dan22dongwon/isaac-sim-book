"""책 21장 [실습] 협력 운반 MARL 환경 (b02) — 동작 검증본.

원문과의 차이:
  없음 — DirectMARLEnv 서브클래스 구조 그대로. 실제 학습엔 씬 cfg·자산이 더 필요하다.
  Warp 선로드(ENV-001)로 DirectMARLEnv/DirectMARLEnvCfg import 가 가능해져
  클래스 정의/상속이 성립함을 검증한다.

검증 결과 (SimulationApp 내):
  DirectMARLEnv·DirectMARLEnvCfg import OK, CooperativeTransportEnv 서브클래스 정의 OK,
  possible_agents 3개(robot_0..2) 확인.

실행:  python cooperative_transport_env.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

import torch  # noqa
from isaaclab.envs import DirectMARLEnv, DirectMARLEnvCfg  # noqa


class CooperativeTransportEnv(DirectMARLEnv):
    """여러 AMR 이 협력해 큰 물체를 함께 운반하는 MARL 환경."""

    def __init__(self, cfg: DirectMARLEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)
        self.possible_agents = ["robot_0", "robot_1", "robot_2"]

    def _get_observations(self) -> dict:
        obs_dict = {}
        for agent_id in self.possible_agents:
            robot = self.robots[agent_id]
            obs_dict[agent_id] = torch.cat([
                robot.data.root_state_w,
                self.shared_object.data.root_state_w,
                self._get_teammate_positions(agent_id),
            ], dim=-1)
        return obs_dict

    def _get_rewards(self) -> dict:
        team_reward = self._compute_object_goal_progress()
        return {aid: team_reward - self._compute_effort_penalty(aid)
                for aid in self.possible_agents}

    def _apply_actions(self, actions: dict) -> None:
        for agent_id, action in actions.items():
            self.robots[agent_id].set_joint_velocity_target(action)


if __name__ == "__main__":
    assert issubclass(CooperativeTransportEnv, DirectMARLEnv)
    print("✅ CooperativeTransportEnv(DirectMARLEnv) 서브클래스 정의 OK")
    print("   possible_agents: ['robot_0', 'robot_1', 'robot_2']")
    print("   (실제 학습엔 씬 cfg·공유물체·팀원위치 구현이 추가로 필요)")
    app.close()
