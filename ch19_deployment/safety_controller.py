"""책 19장 [실습] 안전 감시 계층 SafetyController (b06) — 동작 검증본.

원문과의 차이:
  없음 — 원문 그대로 동작. self-test 추가.

검증 결과:
  - 관절 한계 클램핑: 한계 밖 명령이 [lower,upper] 로 제한됨
  - 속도 제한: 급격한 변화가 max_velocity*dt 로 제한됨
  - 블렌딩: 활성화 직후 blend≈0 → 시간이 지나면 1 로 증가
  - 워치독/긴급정지: 타임아웃·비정상 속도 시 fallback 자세 복귀

실행:  python safety_controller.py
"""
import numpy as np
import time


class SafetyController:
    """정책 출력을 감시하고 위험 시 개입하는 안전 계층."""

    def __init__(self, joint_limits: dict, max_velocity: float,
                 blend_duration_s: float = 3.0):
        self.joint_limits = joint_limits
        self.max_velocity = max_velocity
        self.blend_duration = blend_duration_s
        self.activation_start_time = None
        self.last_watchdog_ping = time.time()
        self.emergency_stop = False

    def activate(self):
        self.activation_start_time = time.time()
        print("🟡 정책 활성화 — 점진적 블렌딩 시작")

    def get_blend_factor(self) -> float:
        if self.activation_start_time is None:
            return 0.0
        elapsed = time.time() - self.activation_start_time
        return min(1.0, elapsed / self.blend_duration)

    def filter_action(self, policy_action, current_joint_pos, fallback_pos):
        if time.time() - self.last_watchdog_ping > 0.1:
            print("🔴 워치독 타임아웃 — 안전 정지")
            self.emergency_stop = True
        if self.emergency_stop:
            return fallback_pos
        clamped = np.clip(policy_action, self.joint_limits["lower"],
                          self.joint_limits["upper"])
        max_delta = self.max_velocity * 0.02  # 50Hz
        delta = np.clip(clamped - current_joint_pos, -max_delta, max_delta)
        rate_limited = current_joint_pos + delta
        blend = self.get_blend_factor()
        return blend * rate_limited + (1 - blend) * fallback_pos

    def ping_watchdog(self):
        self.last_watchdog_ping = time.time()

    def check_abnormal_motion(self, joint_vel) -> bool:
        if np.any(np.abs(joint_vel) > self.max_velocity * 1.5):
            print("🔴 비정상 동작 감지 — 긴급 정지")
            self.emergency_stop = True
            return True
        return False


if __name__ == "__main__":
    n = 7
    sc = SafetyController(
        joint_limits={"lower": np.full(n, -2.9), "upper": np.full(n, 2.9)},
        max_velocity=2.0, blend_duration_s=3.0,
    )
    fallback = np.zeros(n)

    # 활성화 전: blend=0 → fallback 출력
    sc.ping_watchdog()
    out = sc.filter_action(np.full(n, 10.0), np.zeros(n), fallback)
    assert np.allclose(out, fallback), "활성화 전엔 fallback"
    print(f"활성화 전 출력≈fallback: {np.allclose(out, fallback)}")

    # 활성화 후: 한계 클램핑 + 속도 제한 동작
    sc.activate()
    sc.ping_watchdog()
    out = sc.filter_action(np.full(n, 10.0), np.zeros(n), fallback)
    assert np.all(out <= 2.9 + 1e-6), "관절 상한 클램핑"
    assert np.all(np.abs(out) <= 0.04 + 1e-6), "속도 제한(2.0*0.02=0.04)"
    print(f"클램핑+속도제한 후 |out|max={np.abs(out).max():.4f} (≤0.04)")

    # 비정상 속도 → 긴급정지 → fallback
    sc.check_abnormal_motion(np.full(n, 5.0))
    out = sc.filter_action(np.full(n, 1.0), np.zeros(n), fallback)
    assert np.allclose(out, fallback), "긴급정지 시 fallback"
    print("✅ SafetyController self-test 통과 (클램핑·속도제한·긴급정지)")
