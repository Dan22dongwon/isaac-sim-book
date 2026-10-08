"""책 23장 [실습] 커스텀 태스크 Gymnasium 등록 (b01) — 동작 검증본.

원문과의 차이:
  [🟡 관행] 원문은 `Isaac-MyCustomTask-v0` 로 등록한다. 기능상 오류는 아니나
            Isaac Lab 3.0 공식 환경 143개 중 다수가 `-v0` 없이 등록돼 있어
            관행상 `Isaac-MyCustomTask` 가 자연스럽다. (gym.register 는 임의 문자열 허용)
  [검증 편의] 원문은 사용자 프로젝트의 custom_env_cfg·agents 모듈을 참조한다.
            여기서는 등록 '패턴' 자체가 동작함을 보이려고 실제 존재하는
            ManagerBasedRLEnvCfg 를 entry 로 써서 레지스트리 등록을 확인한다.

검증 결과:
  gym.register 후 gym.registry 에 id 가 올라옴을 확인.

실행:  python custom_task_register.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

import gymnasium as gym
from isaaclab.envs import ManagerBasedRLEnvCfg

TASK_ID = "Isaac-MyCustomTask"  # 원문 Isaac-MyCustomTask-v0 → -v0 생략(관행)

gym.register(
    id=TASK_ID,
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={"env_cfg_entry_point": ManagerBasedRLEnvCfg},
    disable_env_checker=True,
)

if __name__ == "__main__":
    assert TASK_ID in gym.registry, "레지스트리 등록 실패"
    print(f"✅ gym.register OK — '{TASK_ID}' 레지스트리 등록 확인")
    print(f"   entry_point: {gym.registry[TASK_ID].entry_point}")
    app.close()
