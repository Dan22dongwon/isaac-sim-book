# Chapter 12 — 공장 디지털 트윈

블록 10개 (원고 ch12 b01~b10) + ch12.6 4개.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-087** 🔴 | b05 | `PhysxConveyorBeltAPI` 가 6.1 에 없다 → `PhysxSurfaceVelocityAPI` 로 대체 |
| ERR-088 🟠 | b05 | 컨베이어 치수 주석 2배 오류 (1×6×0.1m → 실제 0.5×3×0.05m) |
| **ERR-089** 🔴 | b09·b10 | Carter 경로 `/Isaac/Robots/Carter/` → `/Isaac/Robots/NVIDIA/Carter/` |
| **ERR-090** 🔴 | b07 | People 캐릭터 경로 변경 (`female_adult_business_02` → `F_Business_02`) |
| ERR-091 🔴 | ch12.6 b05 | `.py` 메서드 체인 줄바꿈 → IndentationError |
| **ERR-092** 🔴 | b09·b10 | `experimental.wheeled_robots` import 에 비-experimental API 사용 → TypeError |

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b01 | FBX → USD 변환 | ✅ API 존재 확인 (`omni.kit.asset_converter`) |
| b02 | FBX 일괄 변환 | ✅ b01 의존, API 동일 |
| b03 | — | (빈 블록) |
| b04 | 3DGS USDZ 불러오기 | ✅ `add_reference_to_stage` 사용, 입력 파일 필요 |
| b05 | **컨베이어 벨트** | ❌ ERR-087·088 — API 없음 + 치수 오류 |
| b06 | 컨베이어 박스 | ⚠️ b05 의존, DynamicCuboid API ✅ |
| b07 | **People 에셋** | ❌ ERR-090 — 경로 변경됨 |
| b08 | — | (빈 블록) |
| b09 | **AMR Fleet** | ❌ ERR-089·092 — Carter 경로 + import 불일치 |
| **b10** | **통합 씬** | ✅ **수정 후 성공** — 박스 323cm, AMR 5.7m |

### ch12.6

| 블록 | 내용 | 상태 |
|---|---|---|
| b03 | USDZ 불러오기 (GUI + Python) | ✅ 개념 코드, API 이상 없음 |
| b04 | 지면 프록시 콜라이더 | ✅ `UsdGeom.Plane` 사용 |
| b05 | Variant Set 전환 | ❌ ERR-091 — 메서드 체인 줄바꿈 IndentationError |
| b06 | Carter 로봇 배치 | ⚠️ ERR-089 — Carter 경로 오류 (b09 와 동일) |

## 파일 목록

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`factory_twin.py`](factory_twin.py) | 12 b10 | ✅ | 수정 4건 (ERR-087·089·092 + 치수) |

## 실측 결과

```
물류 창고 디지털 트윈 — 800 스텝
  컨베이어 벨트: 0.4 m/s (SurfaceVelocityAPI)
  AMR: 2대 (Carter v1)
  step    0  박스 y=-0.99m  AMR (-4.0, -8.0) (+4.0, -8.0)
  step  200  박스 y=+2.23m  AMR (-2.2, -7.5) (+5.8, -8.5)
  step  400  박스 y=+2.23m  AMR (-1.1, -6.0) (+6.9, -10.0)
  step  600  박스 y=+2.23m  AMR (-1.0, -4.1) (+6.9, -11.9)
결과:
  박스 이동: +323.0 cm  ✅
  AMR_0 이동거리: 5.7 m ✅
  AMR_1 이동거리: 5.7 m ✅
```

## 실행 방법

```bash
conda activate isaac_env
python factory_twin.py                    # GUI
HEADLESS=1 python factory_twin.py         # 화면 없이
STEPS=2000 python factory_twin.py         # 오래 돌리기
```

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch12/ch12_factory_twin_mid.png`](../_docs/images/book/ch12/ch12_factory_twin_mid.png) | 400스텝 시점 — 컨베이어+AMR+선반 통합 씬 |

## 남은 것

- b07 People 경로 수정 스크립트 (실제 캐릭터 배치 테스트)
- ch12.6 b05 Variant Set 줄바꿈 수정
