"""책 18장 [실습] 실제 액추에이터 지연·포화 모델 (b02) — 동작 검증본.

원문과의 차이:
  [ERR-100] 원문은 `np.radians(backlash_deg)` 를 쓰면서 `import numpy as np` 가 없다
            (torch 만 import). 인스턴스화 시 NameError.
          → `import numpy as np` 추가.

검증 결과:
  delay_steps=2 → 처음 2스텝은 0 토크, 이후 지연된 명령 출력.
  torque_limit=50 → ±50 으로 포화. self-test 통과.

실행:  python actuator_model.py
"""
import numpy as np   # [ERR-100] 원문 누락
import torch


class RealisticActuatorModel:
    """실제 액추에이터 특성을 반영한 지연·포화 모델."""

    def __init__(self, delay_steps=2, torque_limit=50.0, backlash_deg=0.5):
        self.delay_steps = delay_steps
        self.torque_limit = torque_limit
        self.backlash = np.radians(backlash_deg)  # ERR-100: np 필요
        self.command_buffer = []

    def apply(self, commanded_torque: torch.Tensor) -> torch.Tensor:
        self.command_buffer.append(commanded_torque)
        if len(self.command_buffer) > self.delay_steps:
            delayed_torque = self.command_buffer.pop(0)
        else:
            delayed_torque = torch.zeros_like(commanded_torque)
        return torch.clamp(delayed_torque, -self.torque_limit, self.torque_limit)


if __name__ == "__main__":
    m = RealisticActuatorModel(delay_steps=2, torque_limit=50.0)
    outs = [m.apply(torch.tensor([100.0, -80.0])) for _ in range(4)]
    print("스텝별 출력:", [o.tolist() for o in outs])
    assert torch.all(outs[0] == 0) and torch.all(outs[1] == 0), "지연 2스텝"
    assert outs[2].tolist() == [50.0, -50.0], "토크 포화 ±50"
    print("✅ 지연 2스텝 + 토크 포화 ±50 확인")
