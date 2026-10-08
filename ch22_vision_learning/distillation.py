"""책 22장 [실습] 교사-학생 증류 손실 (b03) — 동작 검증본.

원문과의 차이:
  없음 — 원문 그대로. self-test 추가.

검증 결과:
  행동만 정합 시 action MSE 와 동일, 잠재 정합 추가 시 latent_weight 만큼 증가.
  teacher_latent 는 detach 되어 grad 가 student 로만 흐름.

실행:  python distillation.py
"""
import torch
import torch.nn.functional as F


def distillation_loss(student_action, teacher_action,
                      student_latent=None, teacher_latent=None,
                      latent_weight=0.5):
    """학생-교사 행동 및 잠재표현 정합 손실."""
    action_loss = F.mse_loss(student_action, teacher_action)
    total_loss = action_loss
    if student_latent is not None and teacher_latent is not None:
        latent_loss = F.mse_loss(student_latent, teacher_latent.detach())
        total_loss = total_loss + latent_weight * latent_loss
    return total_loss


if __name__ == "__main__":
    sa = torch.tensor([[1.0, 2.0]], requires_grad=True)
    ta = torch.tensor([[1.5, 2.5]])
    l1 = distillation_loss(sa, ta)
    expected = F.mse_loss(sa, ta)
    assert torch.allclose(l1, expected), "행동 손실 불일치"
    print(f"행동 정합 손실: {l1.item():.4f}")

    sl = torch.tensor([[0.0, 0.0]], requires_grad=True)
    tl = torch.tensor([[1.0, 1.0]], requires_grad=True)
    l2 = distillation_loss(sa, ta, sl, tl, latent_weight=0.5)
    assert l2.item() > l1.item(), "잠재 손실이 더해져야 함"
    l2.backward()
    assert tl.grad is None, "teacher_latent 는 detach 되어 grad 없어야 함"
    print(f"행동+잠재 손실: {l2.item():.4f} (teacher_latent grad={tl.grad})")
    print("✅ distillation_loss self-test 통과")
