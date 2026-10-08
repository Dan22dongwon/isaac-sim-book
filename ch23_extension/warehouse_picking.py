"""책 23장 [실습] 창고 피킹 종합 프로젝트 (b12) — 동작 검증본.

원문과의 차이:
  [🟡 관행] `Isaac-WarehousePicking-v0` → 관행상 `-v0` 생략 가능(b01 참고).
  [검증 편의] 원문의 _create_shelf_cfg 등은 사용자가 구현할 메서드다(미구현 pass).
            여기서는 ManagerBasedRLEnvCfg 서브클래스 정의와 gym.register 패턴이
            성립함을 검증한다. 실제 학습엔 씬/보상 cfg 구현이 필요.

검증 결과:
  WarehousePickingEnvCfg(ManagerBasedRLEnvCfg) 서브클래스 정의 OK,
  gym.register 로 레지스트리 등록 확인.

실행:  python warehouse_picking.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

import gymnasium as gym
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils import configclass


@configclass
class WarehousePickingEnvCfg(ManagerBasedRLEnvCfg):
    """창고 피킹 태스크 — Ch4 씬 + Ch16 Pick 정책 결합 (골격)."""

    def __post_init__(self):
        super().__post_init__()
        # 실제 구현 시: Ch4 창고 씬 + Ch16 lift 보상 재사용
        # self.scene.warehouse_shelf = self._create_shelf_cfg()
        # self.rewards = self._create_lift_rewards_cfg()


TASK_ID = "Isaac-WarehousePicking"  # 원문 -v0 → 관행상 생략

gym.register(
    id=TASK_ID,
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={"env_cfg_entry_point": WarehousePickingEnvCfg},
    disable_env_checker=True,
)

if __name__ == "__main__":
    assert issubclass(WarehousePickingEnvCfg, ManagerBasedRLEnvCfg)
    assert TASK_ID in gym.registry
    print(f"✅ WarehousePickingEnvCfg 정의 + '{TASK_ID}' 등록 OK")
    print("   (씬/보상 cfg 구현은 사용자 몫 — 종합 프로젝트 골격)")
    app.close()
