"""책 18장 [실습] 하드웨어 변동 강건성 비교 (b06) — 동작 검증본.

원문과의 차이:
  없음 — 원문 그대로. 실제 env/policy 없이 더미 성공률로 로직을 검증한다
  (원문도 success 를 더미로 둔 개념 코드).

실행:  python hardware_variation.py
"""
import numpy as np


def simulate_hardware_variation(policy, n_trials=100, friction_range=(0.05, 0.4)):
    """가상의 '실제 하드웨어' 마찰 범위에서 정책 성공률 평가."""
    success_count = 0
    for _ in range(n_trials):
        true_friction = np.random.uniform(*friction_range)  # noqa: F841
        # env.set_friction(true_friction); success = run_episode(env, policy)
        success = np.random.random() > 0.3  # 예시용 더미
        if success:
            success_count += 1
    return success_count / n_trials * 100


if __name__ == "__main__":
    print("📊 Domain Randomization 전략별 강건성 비교\n")
    wide = simulate_hardware_variation("wide_dr.pt", friction_range=(0.05, 0.4))
    print(f"넓은 범위 DR 정책 성공률: {wide:.1f}%")
    targeted = simulate_hardware_variation("targeted_dr.pt", friction_range=(0.05, 0.4))
    print(f"시스템 식별 기반 DR 정책 성공률: {targeted:.1f}%")
    print("✅ simulate_hardware_variation 실행 OK (더미 성공률 ~70%)")
