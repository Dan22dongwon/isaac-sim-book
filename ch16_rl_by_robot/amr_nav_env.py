"""책 16장 [실습] LiDAR 기반 AMR 내비게이션 환경 구조 (b03) — 동작 검증본.

원문과의 차이:
  [누락]    `from isaaclab.utils import configclass` import 추가.
  [ERR-099] 학습 커맨드의 `Isaac-Navigation-Flat-v0` 환경이 없다
            → `IsaacContrib-Navigation-Flat-AnymalC` (contrib 네임스페이스).

검증 결과:
  RayCasterCfg + patterns.LidarPatternCfg 존재 ✅ — cfg 생성 확인.

실행:  python amr_nav_env.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

from isaaclab.envs import ManagerBasedRLEnvCfg  # noqa
from isaaclab.sensors import RayCasterCfg, patterns
from isaaclab.utils import configclass  # [누락]


@configclass
class AMRNavigationEnvCfg(ManagerBasedRLEnvCfg):
    lidar = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0, 0),
            horizontal_fov_range=(-180, 180),
            horizontal_res=2.0,  # 2도 간격 = 180개 빔
        ),
        max_distance=10.0,
    )


# 학습 (원문 Isaac-Navigation-Flat-v0 → IsaacContrib-Navigation-Flat-AnymalC):
#   uv run train --rl_library rsl_rl \
#       --task=IsaacContrib-Navigation-Flat-AnymalC --num_envs=2048

if __name__ == "__main__":
    cfg = AMRNavigationEnvCfg()
    n_beams = int((180 - (-180)) / cfg.lidar.pattern_cfg.horizontal_res)
    print(f"✅ AMRNavigationEnvCfg 생성 OK — LiDAR {n_beams}빔, "
          f"max {cfg.lidar.max_distance}m")
    app.close()
