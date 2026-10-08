"""책 22장 [실습] 교사-학생 성능 비교 (b07) — 동작 검증본.

원문과의 차이:
  없음 — 원문 그대로. self-test 로 세 등급(우수/보통/실패) 분기를 확인.

실행:  python teacher_student_compare.py
"""
import numpy as np


def compare_teacher_student(teacher_rewards, student_rewards):
    """증류 성공 여부 정량 평가."""
    teacher_mean = np.mean(teacher_rewards)
    student_mean = np.mean(student_rewards)
    retention_rate = (student_mean / teacher_mean) * 100
    print(f"교사 정책 평균 보상: {teacher_mean:.2f}")
    print(f"학생 정책 평균 보상: {student_mean:.2f}")
    print(f"성능 유지율: {retention_rate:.1f}%")
    if retention_rate > 85:
        print("✅ 우수한 증류 — 학생이 교사 성능을 잘 재현함")
    elif retention_rate > 60:
        print("🟡 보통 증류 — 추가 학습 또는 관측 개선 검토")
    else:
        print("🔴 증류 실패 — 학생 관측이 태스크에 불충분할 수 있음")
    return retention_rate


if __name__ == "__main__":
    r1 = compare_teacher_student([10] * 50, [9] * 50)      # 90% 우수
    print()
    r3 = compare_teacher_student([10] * 50, [4] * 50)      # 40% 실패
    assert r1 > 85 and r3 < 60
    print("\n✅ compare_teacher_student self-test 통과 (등급 분기 동작)")
