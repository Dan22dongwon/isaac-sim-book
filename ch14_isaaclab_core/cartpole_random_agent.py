"""책 14장 [실습] Cartpole 환경 생성 + Gymnasium 루프 (b04) — 동작 검증본.

원문과의 차이:
  [ENV-001] `import warp` 를 SimulationApp 보다 먼저 (번들 Warp 1.16.0 버그 우회).
  [ERR-095] 원문 `gym.make("Isaac-Cartpole-v0", ...)` 의 환경 이름이 없다.
            6.1/Isaac Lab 3.0 에는 `Isaac-Cartpole-Direct` (또는 `Isaac-Cartpole`).
          → `Isaac-Cartpole-Direct` 로 교체.
  [검증 편의] 원문 num_envs=4096 → 64 로 축소(동작 확인 목적). cfg 는 parse_env_cfg 로.

검증 결과:
  gym.make 성공, reset→step 루프 정상, step 마다 평균 보상 출력.

실행:  python cartpole_random_agent.py
"""
import warp as wp  # [ENV-001] 반드시 SimulationApp 보다 먼저
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

import gymnasium as gym
import torch
import isaaclab_tasks  # noqa  (환경 등록)
from isaaclab_tasks.utils import parse_env_cfg

TASK = "Isaac-Cartpole-Direct"   # [ERR-095] 원문: Isaac-Cartpole-v0
NUM_ENVS = 64

env_cfg = parse_env_cfg(TASK, device="cuda:0", num_envs=NUM_ENVS)
env = gym.make(TASK, cfg=env_cfg, render_mode=None)
print(f"✅ 환경 생성: {TASK} (num_envs={NUM_ENVS})")

obs, info = env.reset()
for step in range(300):
    actions = torch.rand(
        (NUM_ENVS, env.action_space.shape[1]), device="cuda:0"
    ) * 2 - 1
    obs, rewards, terminated, truncated, info = env.step(actions)
    if step % 100 == 0:
        print(f"Step {step}: 평균 보상={rewards.mean().item():.3f}")

print("✅ 루프 정상 종료")
env.close()
app.close()
