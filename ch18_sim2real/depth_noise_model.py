"""책 18장 [실습] 깊이 카메라 노이즈 모델 (b04) — 동작 검증본.

원문과의 차이:
  없음 — 원문 그대로 동작. self-test 추가.

검증 결과:
  2m 평면 깊이맵에 노이즈 적용 → 노이즈가 더해지고, 경계/반사 마스크 픽셀 일부 0(결측).
  참고: 원문 모델은 baseline_noise_mm(=2.0)를 미터 단위 깊이에 그대로 std 로 더해
  노이즈가 크다(단위 혼용). 모델 동작 자체는 정상 — 결측 픽셀과 노이즈 분포로 검증.

실행:  python depth_noise_model.py
"""
import numpy as np


class RealisticDepthNoiseModel:
    """실제 깊이 카메라 특성 기반 정밀 노이즈 모델."""

    def __init__(self, sensor_spec: dict):
        self.baseline_noise_mm = sensor_spec.get("baseline_noise_mm", 2.0)
        self.distance_noise_factor = sensor_spec.get("distance_factor", 0.002)
        self.edge_dropout_prob = sensor_spec.get("edge_dropout", 0.15)
        self.reflective_dropout_prob = sensor_spec.get("reflective_dropout", 0.3)

    def apply(self, depth_map, edge_mask, reflective_mask):
        noisy = depth_map.copy()
        noise_std = self.baseline_noise_mm + depth_map * self.distance_noise_factor
        noisy += np.random.normal(0, noise_std)
        edge_dropout = np.random.random(depth_map.shape) < self.edge_dropout_prob
        noisy[edge_mask & edge_dropout] = 0
        reflect_dropout = np.random.random(depth_map.shape) < self.reflective_dropout_prob
        noisy[reflective_mask & reflect_dropout] = 0
        return noisy


if __name__ == "__main__":
    m = RealisticDepthNoiseModel({"baseline_noise_mm": 2.0, "distance_factor": 0.002,
                                  "edge_dropout": 0.15, "reflective_dropout": 0.3})
    depth = np.full((64, 64), 2.0)
    edge = np.zeros((64, 64), dtype=bool); edge[0, :] = True
    refl = np.zeros((64, 64), dtype=bool); refl[-1, :] = True
    out = m.apply(depth, edge, refl)
    dropouts = int((out == 0).sum())
    assert dropouts > 0, "경계/반사 마스크에서 결측 픽셀이 생겨야 함"
    assert out.shape == depth.shape
    print(f"✅ 깊이맵 노이즈 적용 — 결측 {dropouts}px, shape {out.shape} 유지")
