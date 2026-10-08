# Chapter 22 — 비전 기반 학습

블록 4개 (원고 b03, b04, b05, b07). 증류, TiledCamera, 3DGS.

## 이 장에서 찾은 오류

없음.

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b03 | distillation_loss (교사-학생) | ✅ 순수 PyTorch 개념 코드 |
| b04 | TiledCameraCfg + PinholeCameraCfg | ✅ 모든 클래스 존재 |
| b05 | 3DGS + 메시 혼합 씬 | ✅ `stage_utils.add_reference_to_stage` 존재 |
| b07 | 교사-학생 성능 비교 | ✅ 순수 Python 개념 코드 |

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `isaaclab.sensors.TiledCameraCfg` | ✅ |
| `TiledCameraCfg.OffsetCfg` | ✅ |
| `isaaclab.sim.PinholeCameraCfg` | ✅ |
| `isaaclab.utils.configclass` | ✅ |
| `isaacsim.core.utils.stage.add_reference_to_stage` | ✅ |

## 실습 파일

| 파일 | 책 블록 | 상태 |
|---|---|---|
| [`distillation.py`](distillation.py) | b03 | ✅ 실행 — 행동/잠재 손실, teacher detach self-test |
| [`teacher_student_compare.py`](teacher_student_compare.py) | b07 | ✅ 실행 — 유지율 등급 분기(90%/40%) |
| [`vision_scene_cfg.py`](vision_scene_cfg.py) | b04 | ✅ cfg 생성 — TiledCamera 80x45 depth |
| [`gaussian_splat_scene.py`](gaussian_splat_scene.py) | b05 | ⚠️ 에셋 의존 — add_reference_to_stage 검증 |

## 검증 범위

- **런타임**: b03·b07 순수 PyTorch 실행 ✅, b04 TiledCamera cfg 생성 ✅,
  b05 는 3DGS 자산 의존(API 검증)

## 캡처

cfg/손실 함수 코드라 GUI 캡처 대상 없음.
