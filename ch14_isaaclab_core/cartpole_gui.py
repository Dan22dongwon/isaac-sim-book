"""Isaac Lab Cartpole GUI 실행본 — Ch14.

핵심 2가지:
  [ENV-001] `import warp` 를 SimulationApp 보다 먼저 → 번들 Warp 1.16.0(버그)
            대신 pip 1.17.0 이 sys.modules 에 자리잡아 isaaclab_tasks 가 import 된다.
  [GUI]     env_cfg.sim.use_fabric=False + 루프에서 app.update() 호출해야
            에디터 뷰포트에 카트폴이 렌더된다 (기본 Fabric 이면 화면이 빈다).

실행:  conda activate isaac_env && python cartpole_gui.py
"""
import time
t0 = time.time()
def log(m):
    print(f"[{time.time()-t0:7.2f}s] {m}", flush=True)

log("START")
import warp as wp        # ← 반드시 SimulationApp 보다 먼저 (ENV-001)
wp.config.enable_backward = False

import isaacsim
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": False, "width": 1600, "height": 1000})
log("GUI 준비")

import gymnasium as gym
import torch
import isaaclab_tasks  # noqa
from isaaclab_tasks.utils import parse_env_cfg

TASK = "Isaac-Cartpole-Direct"
NUM_ENVS = 16
env_cfg = parse_env_cfg(TASK, device="cuda:0", num_envs=NUM_ENVS)
env_cfg.sim.use_fabric = False          # USD 저작 강제
env = gym.make(TASK, cfg=env_cfg, render_mode=None)
log("gym.make 완료")
obs, _ = env.reset()
log("reset 완료")

sim = env.unwrapped.sim
sim.set_camera_view(eye=[9.0, 9.0, 6.0], target=[0.0, 0.0, 1.0])

# 에디터 뷰포트를 그리기 위해 app.update() 를 여러 번 호출
for _ in range(120):
    app.update()
log("초기 app.update 완료 — 뷰포트 펌프")

step = 0
while app.is_running():
    with torch.inference_mode():
        action = torch.rand((NUM_ENVS, env.action_space.shape[1]),
                            device="cuda:0") * 2 - 1
        env.step(action)
    app.update()                        # 에디터 뷰포트 갱신
    step += 1
    if step % 30 == 0:
        log(f"step {step}")
env.close()
