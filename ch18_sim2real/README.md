# Chapter 18 — Sim-to-Real 전이

블록 5개 (원고 b01~b06). 도메인 랜덤화, 액추에이터 모델, 깊이 노이즈.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| ERR-100 🟠 | b02 | `import numpy as np` 누락 — `np.radians()` 사용하는데 `torch`만 import |

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b01 | SystemIdentifiedRandomization | ✅ 순수 Python 개념 코드 |
| b02 | RealisticActuatorModel | ⚠️ ERR-100 — `np` 미정의 `NameError` |
| b03 | 렌더링 랜덤화 (Replicator) | ✅ rep API 모두 존재 |
| b04 | RealisticDepthNoiseModel | ✅ 순수 Python 개념 코드 |
| b06 | 하드웨어 변동 시뮬레이션 | ✅ 순수 Python 개념 코드 |

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `torch` | ✅ 2.12.0+cu130 |
| `numpy` | ✅ |
| `omni.replicator.core` | ✅ (b03) |
| b01 `SystemIdentifiedRandomization` 실행 | ✅ sample=0.17 |
| b02 `RealisticActuatorModel()` 인스턴스화 | ❌ `NameError: name 'np' is not defined` |
| b04 `RealisticDepthNoiseModel` 실행 | ✅ mean=2.01 |
| b06 `simulate_hardware_variation` 실행 | ✅ ~70% |

## 실습 파일

| 파일 | 책 블록 | 상태 |
|---|---|---|
| [`system_id_randomization.py`](system_id_randomization.py) | b01 | ✅ 실행 — ±15% 샘플 1000회 범위 내 |
| [`actuator_model.py`](actuator_model.py) | b02 | ✅ 실행 — ERR-100 수정, 지연2+포화±50 확인 |
| [`rendering_randomization.py`](rendering_randomization.py) | b03 | ⚠️ rep 씬 의존 — 구조 검증 |
| [`depth_noise_model.py`](depth_noise_model.py) | b04 | ✅ 실행 — 결측 픽셀 생성 확인 |
| [`hardware_variation.py`](hardware_variation.py) | b06 | ✅ 실행 — 성공률 집계 동작 |

## 검증 범위

- **런타임**: b01·b02·b04·b06 순수 Python 실행 확인 ✅ (b02 는 ERR-100 수정 후 동작)
- **b03**: Replicator 랜덤화 — rep 씬 의존, API·구조 검증

## 캡처

순수 Python 개념 코드 위주라 GUI 씬 캡처 대상 없음.
