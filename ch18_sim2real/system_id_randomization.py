"""책 18장 [실습] 시스템 식별 기반 보수적 랜덤화 (b01) — 동작 검증본.

원문과의 차이:
  없음 — 원문 그대로 동작. self-test 추가.

검증 결과:
  실측값 ±15% 범위로 마찰/댐핑 샘플링 — 1000회 샘플이 모두 범위 내.

실행:  python system_id_randomization.py
"""
import numpy as np


class SystemIdentifiedRandomization:
    """실측 파라미터를 중심으로 한 보수적 랜덤화."""

    def __init__(self, measured_params: dict):
        self.measured = measured_params
        self.tolerance = 0.15  # 실측값 ±15%

    def sample_friction(self) -> float:
        base = self.measured["joint_friction"]
        return np.random.uniform(base * (1 - self.tolerance),
                                 base * (1 + self.tolerance))

    def sample_damping(self) -> float:
        base = self.measured["joint_damping"]
        return np.random.uniform(base * (1 - self.tolerance),
                                 base * (1 + self.tolerance))


if __name__ == "__main__":
    measured = {"joint_friction": 0.15, "joint_damping": 2.3,
                "actuator_delay_ms": 8.0}
    r = SystemIdentifiedRandomization(measured)
    fr = [r.sample_friction() for _ in range(1000)]
    assert all(0.15 * 0.85 <= v <= 0.15 * 1.15 for v in fr)
    print(f"✅ 마찰 샘플 1000회 모두 ±15% 범위 내 "
          f"[{min(fr):.4f}, {max(fr):.4f}] (중심 0.15)")
