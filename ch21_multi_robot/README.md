# Chapter 21 — 다중 로봇 협업

블록 3개 (원고 b02, b04, b07). MARL 환경, 작업 할당, 기여도 분석.

## 이 장에서 찾은 오류

없음.

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b02 | DirectMARLEnv 협력 운반 | ✅ `DirectMARLEnv`, `DirectMARLEnvCfg` 존재 |
| b04 | 헝가리안 알고리즘 작업 할당 | ✅ scipy 정상, 실행 확인 |
| b07 | 에이전트 기여도 분석 | ✅ 순수 Python 개념 코드 |

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `isaaclab.envs.DirectMARLEnv` | ✅ |
| `isaaclab.envs.DirectMARLEnvCfg` | ✅ |
| `scipy.optimize.linear_sum_assignment` | ✅ |

## 실습 파일

| 파일 | 책 블록 | 상태 |
|---|---|---|
| [`fleet_task_allocator.py`](fleet_task_allocator.py) | b04 | ✅ 실행 — 헝가리안 할당이 완전탐색 최소와 일치 |
| [`marl_contribution.py`](marl_contribution.py) | b07 | ✅ 실행 — 균형(1.00)/무임승차(0.43) 구분 |
| [`cooperative_transport_env.py`](cooperative_transport_env.py) | b02 | ✅ 서브클래스 정의 — DirectMARLEnv 상속 성립 |

## 검증 범위

- **런타임**: b04·b07 순수 Python 실행 확인 ✅, b02 는 Warp 해제(ENV-001)로
  `DirectMARLEnv` 서브클래스 정의·상속 확인 ✅ (실제 학습엔 씬 cfg 추가 필요)

## 캡처

cfg/알고리즘 코드라 GUI 캡처 대상 없음. Isaac Lab 실행 캡처는 [Ch14](../ch14_isaaclab_core/shots/) 참조.
