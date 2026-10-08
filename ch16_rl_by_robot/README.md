# Chapter 16 — 로봇별 강화학습

블록 6개 (원고 b02~b10) + ch16.7 b02.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-096** 🔴 | b02 | `isaaclab_tasks.manager_based.manipulation.lift.mdp` → `isaaclab_tasks.core.lift.mdp` |
| ERR-097 🟠 | b02 | `object_is_lifted` 제거(rigid 대응 없음), `object_goal_distance` → `object_reached_goal`(sparse) |
| ERR-097 🟠 | b07 | `terrain_height` 관측 함수 없음 → `height_scan` |
| ERR-098 🟡 | ch16.7 b02 | `isaaclab_arena` 모듈이 존재하지 않는다 |
| ERR-099 🟠 | b03 | `Isaac-Navigation-Flat-v0` 없음 → `IsaacContrib-Navigation-Flat-AnymalC` |

> **누락 import**: b02·b03·b05·b07 은 원문 스니펫에 `configclass`/`mdp`/`ObsGroup` import 가
> 빠져 있다(앞 블록 가정). 검증본엔 모두 추가했다.

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b02 | Lift 보상 설계 | ✅ **cfg 생성** — ERR-096 경로 수정, object_is_lifted 제거분 제외 4항목 |
| b03 | AMR 내비게이션 RL 환경 | ✅ **cfg 생성** — RayCaster/Lidar 180빔. ERR-099 환경명은 학습커맨드 주석 |
| b05 | Push Recovery 외란 설정 | ✅ **cfg 생성** — push_by_setting_velocity |
| b07 | 비대칭 관측 (Actor/Critic) | ✅ **cfg 생성** — Actor 10 / Critic 12항목 (terrain_height만 교체) |
| b10 | 정책 평가 스크립트 | ✅ **실행** — mock env self-test (성공률 집계 동작) |
| ch16.7 b02 | **Isaac Arena** | ❌ ERR-098 — `isaaclab_arena` 없음 (미구현 개념) |

## 실습 파일

| 파일 | 책 블록 | 상태 |
|---|---|---|
| [`lift_rewards.py`](lift_rewards.py) | b02 | ✅ cfg 생성 (ERR-096·097) |
| [`amr_nav_env.py`](amr_nav_env.py) | b03 | ✅ cfg 생성 (ERR-099 주석) |
| [`push_recovery.py`](push_recovery.py) | b05 | ✅ cfg 생성 |
| [`asymmetric_obs.py`](asymmetric_obs.py) | b07 | ✅ cfg 생성 (ERR-097 terrain_height) |
| [`evaluate_policy.py`](evaluate_policy.py) | b10 | ✅ 실행 (mock self-test) |

## 캡처 — Franka 매니퓰레이션 RL 환경

ch16 의 로봇별 RL 중 매니퓰레이션 예로 `Isaac-Open-Drawer-Franka-Direct` 를
4개 병렬 환경으로 띄워 GUI 렌더(use_fabric=False + app.update()).

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch16/ch16_franka_drawer_gui.png`](../_docs/images/book/ch16/ch16_franka_drawer_gui.png) | Isaac Lab GUI — Franka 4대 + 캐비닛 |
| [`../_docs/images/book/ch16/ch16_franka_drawer_viewport.png`](../_docs/images/book/ch16/ch16_franka_drawer_viewport.png) | 뷰포트 — 4개 병렬 환경 RTX 렌더 |
| [`../_docs/images/book/ch16/ch16_franka_drawer.mp4`](../_docs/images/book/ch16/ch16_franka_drawer.mp4) | **영상** — Franka 4대 랜덤 관절 동작 (15초) |
| [`../_docs/images/book/ch16/ch16_franka_drawer_viewport.mp4`](../_docs/images/book/ch16/ch16_franka_drawer_viewport.mp4) | **영상(뷰포트)** — 매니퓰레이터 움직임 (15초) |

> **보행(locomotion) 환경 캡처 보류 (ENV-002)**: AnymalC 등 `joint_acc` 를 쓰는 환경은
> warp 1.17.0 의 `ProxyArray` 커널 타입 불일치로 `env.reset()` 에서 터진다.
> Cartpole·Franka(joint_acc 미사용)는 정상. 자세한 내용은 수정사항 ENV-002.

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `isaaclab_tasks.core.lift.mdp` | ✅ (원고: `manager_based.manipulation.lift.mdp`) |
| `object_ee_distance` | ✅ |
| `object_is_lifted` | ❌ → `deformable_lifting` / `cable_lifting` |
| `object_goal_distance` | ❌ → `object_reached_goal` |
| `RayCasterCfg` | ✅ |
| `patterns.LidarPatternCfg` | ✅ |
| Isaac 환경 총 수 | 143개 |
| Franka 관련 | 35개 |
| Reach 관련 | 10개 |
| Velocity 관련 | 23개 |
| Navigation 관련 | 2개 (contrib 만) |

## 검증 범위

- **문법**: 5개 `.py` 블록 전부 `ast.parse` 통과 ✅
- **런타임**: Warp 문제 **해결됨** (ENV-001, Ch14 참조) — `import warp` 선로드로
  Isaac Lab 환경 실행 가능. b02·b03 의 import 경로/환경명 오류(ERR-096·097·099)는
  여전히 유효 (Warp 와 무관한 별개 문제).

## 캡처

Isaac Lab 실행 캡처는 [Ch14](../ch14_isaaclab_core/shots/) 참조 (동일 Warp 우회 적용).
