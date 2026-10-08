"""책 21장 [실습] MARL 에이전트 기여도 분석 (b07) — 동작 검증본.

원문과의 차이:
  없음 — 원문 그대로. self-test 로 균형/무임승차 두 경우를 확인.

실행:  python marl_contribution.py
"""
import numpy as np


def analyze_marl_contribution(agent_rewards: dict, n_episodes=100):
    """각 에이전트의 기여도 균형 분석 — 무임승차 감지."""
    agent_ids = list(agent_rewards.keys())
    mean_rewards = {aid: np.mean(agent_rewards[aid]) for aid in agent_ids}

    print("📊 에이전트별 평균 보상:")
    for aid, r in mean_rewards.items():
        print(f"  {aid}: {r:.3f}")

    values = list(mean_rewards.values())
    balance_score = 1 - (np.std(values) / (np.mean(values) + 1e-6))
    print(f"균형 점수: {balance_score:.2f} (1.0에 가까울수록 균등 기여)")
    if balance_score < 0.7:
        print("⚠️ 무임승차 의심 — 보상 구조 재검토 필요")
    else:
        print("✅ 에이전트 간 균형 잡힌 기여")
    return balance_score


if __name__ == "__main__":
    rng = np.random.default_rng(0)
    balanced = {f"robot_{i}": rng.normal(10.0, 0.5, 100) for i in range(3)}
    s1 = analyze_marl_contribution(balanced)
    print()
    freerider = {"robot_0": rng.normal(15, 0.5, 100),
                 "robot_1": rng.normal(15, 0.5, 100),
                 "robot_2": rng.normal(2, 0.5, 100)}
    s2 = analyze_marl_contribution(freerider)
    assert s1 > 0.7 and s2 < 0.7, "균형/무임승차 구분 실패"
    print(f"\n✅ self-test 통과 — 균형 {s1:.2f} vs 무임승차 {s2:.2f}")
