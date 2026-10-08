"""책 21장 [실습] 헝가리안 기반 Fleet 작업 할당 (b04) — 동작 검증본.

원문과의 차이:
  없음 — 원문 그대로 동작. self-test 추가.

검증 결과:
  로봇3·작업3 → 최적 할당 {0:0,1:1,2:2}, 총 이동거리가 완전탐색 최소와 일치.

실행:  python fleet_task_allocator.py
"""
import numpy as np
from scipy.optimize import linear_sum_assignment


class FleetTaskAllocator:
    """헝가리안 알고리즘 기반 최적 작업 할당."""

    def allocate_tasks(self, robot_positions, task_positions) -> dict:
        n_robots, n_tasks = len(robot_positions), len(task_positions)
        cost_matrix = np.zeros((n_robots, n_tasks))
        for i, rp in enumerate(robot_positions):
            for j, tp in enumerate(task_positions):
                cost_matrix[i, j] = np.linalg.norm(rp - tp)
        r_idx, t_idx = linear_sum_assignment(cost_matrix)
        return {int(r): int(t) for r, t in zip(r_idx, t_idx)}


if __name__ == "__main__":
    alloc = FleetTaskAllocator()
    robot_pos = np.array([[0, 0], [5, 0], [10, 0]])
    task_pos = np.array([[2, 3], [8, 1], [12, 5]])
    result = alloc.allocate_tasks(robot_pos, task_pos)
    print(f"작업 할당 결과: {result}")

    # 완전탐색으로 최적성 검증
    import itertools
    def total(a):
        return sum(np.linalg.norm(robot_pos[r] - task_pos[t]) for r, t in a.items())
    best = min(total({i: p[i] for i in range(3)})
               for p in itertools.permutations(range(3)))
    assert abs(total(result) - best) < 1e-9, "헝가리안 결과가 최소비용과 불일치"
    print(f"✅ 최적 할당 확인 — 총 이동거리 {total(result):.3f} (완전탐색 최소와 일치)")
