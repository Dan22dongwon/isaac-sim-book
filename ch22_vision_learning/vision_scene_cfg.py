"""책 22장 [실습] 비전 정책용 TiledCamera 씬 (b04) — 동작 검증본.

원문과의 차이:
  [누락] 원문 스니펫에 `from isaaclab.utils import configclass` 가 빠져 있다 → 추가.

검증 결과 (SimulationApp 내):
  TiledCameraCfg + PinholeCameraCfg(spawn) + OffsetCfg 로 depth 카메라 cfg 생성 OK
  (80x45, focal 12mm). Warp 선로드(ENV-001) 적용.

실행:  python vision_scene_cfg.py
"""
import warp as wp  # [ENV-001]
wp.config.enable_backward = False

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": True})

from isaaclab.sensors import TiledCameraCfg
import isaaclab.sim as sim_utils
from isaaclab.utils import configclass  # [누락]


@configclass
class VisionPolicySceneCfg:
    """비전 기반 학생 정책을 위한 씬 구성."""

    tiled_camera = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base/front_cam",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.3, 0.0, 0.1), rot=(0.5, -0.5, 0.5, -0.5)
        ),
        data_types=["distance_to_image_plane"],  # Depth (RGB 보다 가벼움)
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=12.0, clipping_range=(0.05, 10.0)
        ),
        width=80, height=45,
    )


if __name__ == "__main__":
    cfg = VisionPolicySceneCfg()
    c = cfg.tiled_camera
    print(f"✅ VisionPolicySceneCfg 생성 OK — {c.width}x{c.height} depth, "
          f"focal {c.spawn.focal_length}mm, data={c.data_types}")
    app.close()
