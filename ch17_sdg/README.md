# Chapter 17 — 합성 데이터 생성 (SDG)

블록 6개 (원고 b02~b08). Replicator 기반 합성 데이터 파이프라인.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| ERR-103 🟠 | b05 | `add_update_semantics` → `add_labels` (함수명 변경) |
| ERR-105 🟡 | b07 | `num_frames` deprecated → `max_execs` 사용 권장 |
| ERR-106 🟠 | b07 | `rep.modify.attribute('intensity', rep.distribution.uniform(...))` → DomeLight.intensity는 int인데 uniform이 float 반환 → 조명 랜덤화 실패 |

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b02 | Replicator 기본 SDG 파이프라인 | ✅ 모든 API 존재 (개념 코드) |
| b03 | 절차적 방 생성 | ✅ 개념 코드 (커스텀 USD 필요) |
| b04 | 폐색 + KittiWriter | ✅ scatter_2d/3d, KittiWriter 존재 |
| b05 | Flying Distractors + DOPEWriter | ⚠️ ERR-103 — `add_update_semantics` 없음, `add_labels` 사용 |
| b07 | 전체 SDG 통합 (headless) | ✅ **실행 확인** — ERR-105·106 외 파이프라인 정상 |
| b08 | SDG 결과 시각화 검증 | ✅ 순수 Python (json/cv2/os) |

## 실행 결과 (b07)

5프레임 파이프라인 검증:

```
출력 파일: 31개
  rgb_0000~0004.png         (113~148 KB)
  bounding_box_2d_tight     (5 .npy + 5 labels + 5 prim_paths)
  semantic_segmentation     (5 .png + 5 labels)
  metadata.txt

bbox 예시 (frame 1):
  box   → (85, 176) ~ (334, 425)
  table → (0, 0) ~ (511, 511)

labels: {"0": {"class": "box"}, "1": {"class": "table"}}
```

세그멘테이션은 정확하게 class별 분리됨 (핑크=table, 초록=box).
RGB는 기본 재질이 모두 흰색이라 시각 대비가 낮지만 파이프라인 자체는 정상.

### ERR-105: `num_frames` deprecated

```
[Warning] `trigger.on_frame` argument `num_frames` will be deprecated.
Please use `max_execs`.
```

원고: `rep.trigger.on_frame(num_frames=2000)`
수정: `rep.trigger.on_frame(max_execs=2000)`

### ERR-106: DomeLight intensity 타입 불일치

```
[Error] Type mismatch for </Replicator/DomeLight_Xform/DomeLight.intensity>:
expected 'int', got 'TfPyObjWrapper'
```

원고: `rep.modify.attribute("intensity", rep.distribution.uniform(500, 2500))`
수정: `rep.distribution.uniform(500, 2500)` → 정수형 랜덤값 사용 또는 `float` 속성인 `exposure` 사용

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `omni.replicator.core` | ✅ |
| `rep.create.camera` | ✅ |
| `rep.create.render_product` | ✅ |
| `rep.create.cube/plane/cylinder/sphere` | ✅ |
| `rep.create.from_usd` | ✅ |
| `rep.create.group` | ✅ |
| `rep.create.light` | ✅ |
| `rep.modify.pose` | ✅ |
| `rep.modify.attribute` | ✅ |
| `rep.distribution.uniform` | ✅ |
| `rep.randomizer.register` | ✅ |
| `rep.randomizer.scatter_2d` | ✅ |
| `rep.randomizer.scatter_3d` | ✅ |
| `rep.randomizer.color` | ✅ |
| `rep.trigger.on_frame` | ✅ (⚠️ `num_frames` deprecated) |
| `rep.WriterRegistry` | ✅ |
| `BasicWriter` | ✅ |
| `KittiWriter` | ✅ |
| `DOPEWriter` | ✅ |
| `rep.orchestrator.run` | ✅ |
| `add_update_semantics` | ❌ → `add_labels` |

## 검증 범위

- **문법**: 6개 `.py` 블록 전부 `ast.parse` 통과 ✅
- **런타임**: b07 SimulationApp + Replicator 파이프라인 실행 확인 ✅
- **출력**: RGB, bbox, segmentation 31개 파일 정상 생성 ✅

## 실습 파일

| 파일 | 책 블록 | 상태 |
|---|---|---|
| [`sdg_pipeline.py`](sdg_pipeline.py) | b07 | ✅ 실행 — 5프레임 31파일 생성 (ERR-105·106 수정) |
| [`sdg_visualize.py`](sdg_visualize.py) | b08 | ✅ 실행 — 파이프라인 출력에서 bbox 그리기 |
| [`flying_distractors.py`](flying_distractors.py) | b05 | ⚠️ 에셋 의존 — ERR-103 수정(add_labels) |

> 체인 검증: `python sdg_pipeline.py 5 && python sdg_visualize.py` → 생성·시각화 end-to-end 동작.

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch17/ch17_sdg_rgb.png`](../_docs/images/book/ch17/ch17_sdg_rgb.png) | SDG RGB 출력 (512×512, frame 1) |
| [`../_docs/images/book/ch17/ch17_sdg_segmentation.png`](../_docs/images/book/ch17/ch17_sdg_segmentation.png) | 세그멘테이션 마스크 — 핑크=table, 초록=box |
