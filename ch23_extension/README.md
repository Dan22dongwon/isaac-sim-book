# Chapter 23 — 커스텀 확장 개발

블록 3개 (원고 b01, b09, b12). 환경 등록, Extension, 종합 프로젝트.

## 이 장에서 찾은 오류

없음 (아래 참고 사항 있음).

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b01 | 커스텀 태스크 gym.register | ✅ 개념 코드 (🟡 `-v0` 참고) |
| b09 | omni.ext.IExt Extension | ✅ `IExt` 클래스 존재 |
| b12 | 창고 피킹 종합 프로젝트 | ✅ 개념 코드 (🟡 `-v0` 참고) |

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `omni.ext.IExt` | ✅ |
| `isaaclab.envs.ManagerBasedRLEnv` | ✅ |
| `isaaclab.envs.ManagerBasedRLEnvCfg` | ✅ |
| `isaaclab.utils.configclass` | ✅ |

## 참고: `-v0` 환경 이름 관행

원고 b01/b12에서 커스텀 환경을 `Isaac-MyCustomTask-v0` 형식으로 등록한다.
이것은 기능상 오류는 아니나 (gym.register는 아무 문자열이나 받음),
Isaac Lab 3.0의 공식 환경 143개 중 138개가 `-v0` 없이 등록되어 있으므로
관행에 맞추면 `-v0` 없이 `Isaac-MyCustomTask` 가 낫다.

## 실습 파일

| 파일 | 책 블록 | 상태 |
|---|---|---|
| [`custom_task_register.py`](custom_task_register.py) | b01 | ✅ 실행 — gym.register 레지스트리 등록 확인 |
| [`custom_extension.py`](custom_extension.py) | b09 | ✅ 실행 — IExt 서브클래스 수명주기 호출 |
| [`warehouse_picking.py`](warehouse_picking.py) | b12 | ✅ 실행 — 종합 cfg 정의 + 등록 |

## 검증 범위

- **런타임**: 세 블록 모두 Warp 선로드(ENV-001) 로 실제 실행 — 등록/서브클래스/수명주기 확인 ✅
- `-v0` 환경명은 관행상 생략 가능(아래 참고), 기능상 오류 아님

## 캡처

등록/확장 골격 코드라 GUI 캡처 대상 없음. Isaac Lab 실행 캡처는 [Ch14](../ch14_isaaclab_core/shots/) 참조.
