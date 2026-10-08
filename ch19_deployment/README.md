# Chapter 19 — 정책 배포

블록 5개 (원고 b01~b08) + ch19.6 b02. ONNX 익스포트, 안전 컨트롤러, 작업공간 분석.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-101** 🔴 | b01 | `Isaac-Velocity-Flat-Unitree-Go2-v0` → `Isaac-Velocity-Flat-UnitreeGo2` |
| **ERR-102** 🔴 | b08 | `Isaac-Lift-Cube-Franka-v0` → `IsaacContrib-Lift-Cube-Franka` |
| **ERR-104** 🔴 | ch19.6 b02 | `isaacsim.core.api.articulations.ArticulationView` → `isaacsim.core.prims.Articulation` |

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b01 | 정책 익스포트 (rsl_rl) | ⚠️ ERR-101 — 환경명 변경. exporter API ✅ |
| b05 | ONNX 추론 노드 (ROS2) | ✅ onnxruntime 1.26.0 + yaml 6.0.3 |
| b06 | SafetyController | ✅ 순수 Python 개념 코드 |
| b08 | 배포 파이프라인 통합 | ⚠️ ERR-102 — 환경명 변경 |
| ch19.6 b02 | 작업공간 속도 프로파일 | ⚠️ ERR-104 — ArticulationView import 경로 변경 |

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `isaaclab_rl.rsl_rl.exporter` | ✅ |
| `export_policy_as_jit` | ✅ |
| `export_policy_as_onnx` | ✅ |
| `onnxruntime` | ✅ 1.26.0 |
| `yaml` (PyYAML) | ✅ 6.0.3 |
| `SafetyController` 개념 코드 | ✅ |
| `Isaac-Velocity-Flat-Unitree-Go2-v0` | ❌ → `Isaac-Velocity-Flat-UnitreeGo2` |
| `Isaac-Lift-Cube-Franka-v0` | ❌ → `IsaacContrib-Lift-Cube-Franka` |
| `ArticulationView` (deprecated) | ❌ → `isaacsim.core.prims.Articulation` |
| `Articulation.get_jacobians` | ✅ (새 경로에서) |

## 실습 파일

| 파일 | 책 블록 | 상태 |
|---|---|---|
| [`safety_controller.py`](safety_controller.py) | b06 | ✅ 실행 — 클램핑·속도제한·블렌딩·긴급정지 self-test |
| [`policy_controller_node.py`](policy_controller_node.py) | b05 | ✅ 실행 — assemble_observation 로직 + onnxruntime import |

> **b01·b08** 은 셸 명령(`uv run play ...`)과 Python 이 한 블록에 섞여 있어 단일 `.py` 로
> 실행 대상이 아니다. 환경명 오류(ERR-101 Go2, ERR-102 Franka)는 아래 환경 목록 참고.
> **ch19.6 b02** 작업공간 속도 프로파일은 ArticulationView → `isacsim.core.prims.Articulation`
> (ERR-104) 수정이 필요하며 실제 로봇 자산이 있어야 돈다.

## 검증 범위

- **런타임**: b05·b06 실제 실행 확인 ✅ (onnxruntime 1.26.0)
- **Isaac Lab 환경**(rsl_rl play 로 체크포인트 익스포트)은 Warp 해제(ENV-001)로 실행 가능해졌으나
  사전 학습된 체크포인트가 있어야 익스포트가 의미 있다.

## 캡처

안전 계층·추론 노드는 순수 Python 로직이라 GUI 캡처 대상 아님.
Isaac Lab 환경 캡처는 [Ch14](../ch14_isaaclab_core/shots/) 참조.

## Go2 / Unitree 환경 상세

```
Isaac-Velocity-Flat-UnitreeGo2        (원고: Isaac-Velocity-Flat-Unitree-Go2-v0)
Isaac-Velocity-Rough-UnitreeGo2
IsaacContrib-Velocity-Flat-UnitreeA1
IsaacContrib-Velocity-Flat-UnitreeGo1
```

## Lift-Cube-Franka 환경 상세

```
IsaacContrib-Lift-Cube-Franka         (원고: Isaac-Lift-Cube-Franka-v0)
IsaacContrib-Lift-Cube-Franka-IK-Abs
IsaacContrib-Lift-Cube-Franka-IK-Rel
Isaac-Lift-Franka                      (Cube 없는 변형)
```
