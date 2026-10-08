# Chapter 15 — 강화학습 기초

블록 2개 (원고 b01~b02). RL 보상 설계와 도메인 랜덤화 설정.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-096** 🔴 | b01 | `isaaclab_tasks.manager_based.locomotion.mdp` → `isaaclab_tasks.core.velocity.mdp` (6개 보상 함수가 전부 여기 있음) |
| ERR-097 🟠 | b02 | `add_gaussian_noise` 이벤트 함수 없음 → 관측 노이즈는 `ObsTerm(noise=GaussianNoiseCfg(...))` 로 건다 |

> **정정**: 이전 판에서 b01 의 `feet_air_time_positive_biped` 를 "함수 없음(ERR-097)"으로
> 보고했으나 이는 **검증자 오류**였다. 틀린 모듈(`envs.mdp`/`locomotion.mdp`)에서 찾았고,
> 실제로는 `isaaclab_tasks.core.velocity.mdp` 에 존재한다. **원문 함수명은 옳았다.**
> ERR-096 의 올바른 경로도 `core.locomotion` 이 아니라 `core.velocity` 다.

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b01 | 보상 함수 구성 (QuadrupedRewardsCfg) | ✅ **생성 확인** — ERR-096 경로만 고치면 6개 함수 전부 동작 |
| b02 | 도메인 랜덤화 (DomainRandomizationCfg) | ✅ **생성 확인** — 이벤트 3개 OK, 노이즈만 ERR-097 방식 교체 |

## 실습 파일

| 파일 | 책 블록 | 상태 |
|---|---|---|
| [`quadruped_rewards.py`](quadruped_rewards.py) | b01 | ✅ cfg 생성 — 보상 6항목, func 경로 출력 |
| [`domain_randomization.py`](domain_randomization.py) | b02 | ✅ cfg 생성 — 이벤트 3 + GaussianNoiseCfg 예시 |

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `isaaclab_tasks.core.locomotion.mdp` | ✅ (원고: `manager_based.locomotion.mdp`) |
| `track_lin_vel_xy_exp` | ✅ |
| `track_ang_vel_z_exp` | ✅ |
| `lin_vel_z_l2` | ✅ |
| `joint_torques_l2` | ✅ |
| `action_rate_l2` | ✅ |
| `feet_air_time_positive_biped` | ❌ |
| `randomize_rigid_body_mass` | ✅ |
| `randomize_rigid_body_material` | ✅ |
| `push_by_setting_velocity` | ✅ |
| `add_gaussian_noise` | ❌ |
| RL 라이브러리: sb3 | ✅ 2.9.0 |
| RL 라이브러리: rl_games | ✅ |
| RL 라이브러리: rsl_rl | ✅ |
| RL 라이브러리: skrl | ✅ 2.1.0 |
| `isaaclab_rl.skrl` | ✅ |
| `isaaclab_rl.sb3` | ✅ |
| `isaaclab_rl.rl_games` | ✅ |
| `isaaclab_rl.rsl_rl` | ✅ |

## 검증 범위

- **문법**: 2개 `.py` 블록 전부 `ast.parse` 통과 ✅
- **런타임**: Warp 문제 **해결됨** (ENV-001, Ch14 참조) — `import warp` 선로드로
  Isaac Lab 환경 실행 가능. b01·b02 의 보상/랜덤화 cfg 는 개념 코드이나
  이제 실제 환경에 붙여 돌려 볼 수 있다.

## 캡처

이 장의 4족 보행 보상(b01)은 **보행 환경 캡처가 보류**다 — AnymalC 등 보행 환경은
warp 1.17.0 `ProxyArray` 문제(ENV-002)로 `reset` 에서 터진다(`joint_acc` 경로).
실행 가능한 Isaac Lab 캡처는 [Ch14 Cartpole](../_docs/images/book/ch14/)·
[Ch16 Franka](../_docs/images/book/ch16/) 참조.
