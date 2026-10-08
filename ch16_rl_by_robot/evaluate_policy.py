"""책 16장 [실습] 정책 평가 스크립트 (b10) — 동작 검증본.

원문과의 차이:
  없음 — 평가 로직은 원문 그대로다. 실제 env·policy 가 있어야 돌지만,
  로직 자체를 검증하도록 **목(mock) env/policy** 로 self-test 를 붙였다.

검증 결과 (mock):
  n_episodes 반복 → 평균/표준편차/성공률(오차<0.2) 집계가 올바르게 동작.

실행:  python evaluate_policy.py
"""
import numpy as np


def evaluate_policy(env, policy, n_episodes=50):
    """학습된 정책의 속도 추적 정확도 평가 (원문 b10 그대로)."""
    tracking_errors = []
    for ep in range(n_episodes):
        obs, _ = env.reset()
        episode_errors = []
        for step in range(500):
            action = policy(obs)
            obs, reward, terminated, truncated, info = env.step(action)
            cmd_vel = info["command"]
            actual_vel = info["base_lin_vel"]
            error = np.linalg.norm(cmd_vel - actual_vel)
            episode_errors.append(error)
            if terminated.any():
                break
        tracking_errors.append(np.mean(episode_errors))

    print(f"평균 속도 추적 오차: {np.mean(tracking_errors):.3f} m/s")
    print(f"표준편차: {np.std(tracking_errors):.3f} m/s")
    print(f"성공률(오차<0.2m/s): "
          f"{np.mean(np.array(tracking_errors) < 0.2)*100:.1f}%")
    return tracking_errors


class _MockEnv:
    """로직 검증용 간이 env — 명령 속도에 노이즈를 섞어 실제 속도를 흉내낸다."""
    def __init__(self, rng):
        self.rng = rng

    def reset(self):
        self._cmd = np.array([1.0, 0.0])
        self._t = 0
        return {}, {}

    def step(self, action):
        self._t += 1
        actual = self._cmd + self.rng.normal(0, 0.1, 2)
        info = {"command": self._cmd, "base_lin_vel": actual}
        terminated = np.array([self._t >= 50])
        return {}, 0.0, terminated, np.array([False]), info


if __name__ == "__main__":
    rng = np.random.default_rng(0)
    env = _MockEnv(rng)
    errors = evaluate_policy(env, policy=lambda obs: 0, n_episodes=20)
    assert len(errors) == 20
    print("✅ evaluate_policy self-test 통과")
