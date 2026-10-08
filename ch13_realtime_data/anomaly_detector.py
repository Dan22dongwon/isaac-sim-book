"""책 13장 [실습] 이동평균/표준편차 기반 이상 감지 — 동작 검증본.

원문(b04)과의 차이:
  없음 — AnomalyDetector 클래스는 원문 그대로 동작한다.
  원문의 '사용 예'는 get_latest_vibration_reading()·set_equipment_warning_indicator()
  같은 사용자 구현 함수를 호출하므로 그대로는 못 돌린다. 여기서는 합성 데이터로
  클래스 자체를 검증하는 self-test 를 붙였다.

검증 결과 (합성 데이터):
  정상 50개(mean=5.0, std=0.5) → 이상 감지 0건 (기대 0)
  이상값 15.0 삽입 → is_anomaly=True, z_score≈6.7 (threshold=3.0)

실행:  python anomaly_detector.py
"""
import numpy as np
from collections import deque


class AnomalyDetector:
    """이동 평균/표준편차 기반 간이 이상 감지 (원문 b04 그대로)."""

    def __init__(self, window_size=100, z_threshold=3.0):
        self.window_size = window_size
        self.z_threshold = z_threshold
        self.history = deque(maxlen=window_size)

    def update(self, value: float) -> dict:
        self.history.append(value)

        if len(self.history) < 30:  # 초기 데이터 부족
            return {"is_anomaly": False, "z_score": 0.0}

        arr = np.array(self.history)
        mean = arr.mean()
        std = arr.std()

        if std == 0:
            return {"is_anomaly": False, "z_score": 0.0}

        z_score = (value - mean) / std
        is_anomaly = abs(z_score) > self.z_threshold

        return {
            "is_anomaly": is_anomaly,
            "z_score": z_score,
            "mean": mean,
            "std": std,
        }


def _self_test():
    rng = np.random.default_rng(42)
    det = AnomalyDetector(window_size=200, z_threshold=3.0)

    false_alarms = 0
    for _ in range(50):
        v = rng.normal(5.0, 0.5)
        if det.update(v)["is_anomaly"]:
            false_alarms += 1
    print(f"정상 50개 → 오탐 {false_alarms}건 (기대 0)")

    r = det.update(15.0)  # 명백한 이상값
    print(f"이상값 15.0 → is_anomaly={r['is_anomaly']}, "
          f"z_score={r['z_score']:.2f} (mean={r['mean']:.2f}, std={r['std']:.2f})")
    assert false_alarms == 0 and r["is_anomaly"], "self-test 실패"
    print("✅ self-test 통과")


if __name__ == "__main__":
    _self_test()
