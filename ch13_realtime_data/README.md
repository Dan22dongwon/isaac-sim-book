# Chapter 13 — 실시간 데이터 연동

블록 7개 (원고 b01~b07). 대부분 외부 서비스(OPC-UA, MQTT, Nucleus Live)
연동 패턴이라 실행에는 해당 서버가 필요하다.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-093** 🔴 | b03 | `omni.kit.livesync` 모듈이 없다 → `omni.kit.usd.layers.LiveSession` 사용 |
| ERR-094 🟠 | b07 | `DynamicCuboid.set_color()` 메서드 없다 → `apply_visual_material()` 사용 |

## 블록별 상태

| 블록 | 내용 | 상태 | 검증 방법 |
|---|---|---|---|
| b01 | OPC-UA Bridge 클래스 | ✅ 문법 OK | `ast.parse` + 패턴 확인. `opcua` 미설치 (원고 주석에 설치 안내 있음) |
| b02 | MQTT 구독자 | ✅ 문법 OK | `ast.parse` + 패턴 확인. `paho-mqtt` 미설치 (원고 주석에 설치 안내 있음) |
| b03 | **Live Session** | ❌ ERR-093 | SimulationApp 내 `import omni.kit.livesync` → `ModuleNotFoundError` |
| **b04** | **이상 감지기** | ✅ **실행 확인** | 합성 데이터 51개로 테스트 — 정상 50개 중 0건 오탐, 이상값 z=6.72 검출 |
| b05 | — | — | 빈 블록 |
| b06 | — | — | 빈 블록 |
| b07 | MQTT 모니터링 통합 | ⚠️ ERR-094 | SimulationApp 내 `DynamicCuboid.set_color` → `False` |

## 검증 결과

### b04 AnomalyDetector 실측

```
정상 데이터 50개 (mean=5.0, std=0.5):
  이상 감지 0건 ✅ (기대: 0)

이상값 15.0 삽입:
  is_anomaly=True, z_score=6.72 ✅ (threshold=3.0)
  mean=5.09, std=1.47
```

### ERR-093 SimulationApp 실행 확인

```
import omni.kit.livesync → No module named 'omni.kit.livesync'

대안:
  omni.kit.usd.layers.LiveSession  ✅
  omni.kit.usd.layers.LiveSyncing  ✅
```

### ERR-094 SimulationApp 실행 확인

```
DynamicCuboid.set_color        → False (없음)
DynamicCuboid.apply_visual_material → True  (대안)
```

## 실습 파일

| 파일 | 책 블록 | 상태 | 비고 |
|---|---|---|---|
| [`anomaly_detector.py`](anomaly_detector.py) | b04 | ✅ 실행 | self-test 포함 — 오탐 0, 이상값 z≈6.8 |
| [`mqtt_monitor.py`](mqtt_monitor.py) | b07 | ✅ 실행 | ERR-094 수정, 합성 진동으로 색 전환 캡처 |
| [`live_session.py`](live_session.py) | b03 | ⚠️ API 검증 | ERR-093 수정 (livesync→usd.layers), 전파는 Nucleus 필요 |
| [`opcua_bridge.py`](opcua_bridge.py) | b01 | ⚠️ 구조 검증 | OPC-UA 서버 + `opcua` 필요 |
| [`mqtt_subscriber.py`](mqtt_subscriber.py) | b02 | ⚠️ 구조 검증 | MQTT 브로커 + `paho-mqtt` 필요 |

## 캡처

b07(모니터링 통합)을 MQTT 대신 **합성 진동 신호**로 실제 실행해 상태 색 전환을 확인했다
(ERR-094 `apply_visual_material` 수정이 end-to-end 로 동작함을 증명).

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch13/ch13_monitor_normal.png`](../_docs/images/book/ch13/ch13_monitor_normal.png) | 정상 — 컨베이어 인디케이터 초록 |
| [`../_docs/images/book/ch13/ch13_monitor_anomaly.png`](../_docs/images/book/ch13/ch13_monitor_anomaly.png) | 이상(z=7.3) — 빨강 전환 |

나머지 b01·b02·b03 은 외부 서비스(OPC-UA·MQTT·Nucleus) 의존이라 독립 GUI 캡처 대상이 아니다.

## 비고

- `opcua` 와 `paho-mqtt` 는 원고에 설치 안내(`pip install`)가 주석으로 있으나 기본 환경에는 없다. 이는 오류가 아니라 환경 의존.
- b04 의 `get_latest_vibration_reading()`, `set_equipment_warning_indicator()` 는 사용자가 구현할 함수다.
