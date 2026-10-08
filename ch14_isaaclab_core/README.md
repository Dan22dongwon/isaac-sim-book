# Chapter 14 — Isaac Lab 기초

블록 8개 (원고 b01~b08). Isaac Lab 3.0.0 설치 후 API 존재 확인.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| ERR-095 🟠 | b04 | `Isaac-Cartpole-v0` → `Isaac-Cartpole` (버전 접미사 없음) |

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b01 | Isaac Lab 개요 | ✅ 텍스트 |
| b02 | ManagerBasedRLEnvCfg 구조 | ✅ 모든 import 존재 |
| b03 | DirectRLEnv 서브클래스 | ✅ 클래스 존재 |
| b04 | **Cartpole 환경 생성** | ⚠️ ERR-095 — `Isaac-Cartpole-v0` 없음, `Isaac-Cartpole` 로 변경 |
| b05 | Newton 물리 엔진 | ✅ 텍스트 |
| b06 | 환경 병렬화 개념 | ✅ 텍스트 |
| b07 | 텐서 배치 연산 | ✅ 개념 코드 |
| b08 | TiledCameraCfg | ✅ 클래스 존재 |

## API 프로브 결과

| 항목 | 결과 |
|------|------|
| `isaaclab.envs.ManagerBasedRLEnvCfg` | ✅ |
| `isaaclab.envs.DirectRLEnv` | ✅ |
| `isaaclab.managers.ObservationGroupCfg` | ✅ |
| `isaaclab.managers.RewardTermCfg` | ✅ |
| `isaaclab.managers.TerminationTermCfg` | ✅ |
| `isaaclab.utils.configclass` | ✅ |
| `isaaclab.assets.Articulation` | ✅ |
| `isaaclab.sensors.TiledCameraCfg` | ✅ |
| `Isaac-Cartpole` 환경 등록 | ✅ (143개 Isaac 환경 중) |

## 검증 범위

- **문법**: 5개 `.py` 블록 전부 `ast.parse` 통과 ✅
- **import**: SimulationApp 내 실행 — `ObservationGroupCfg`, `RewardTermCfg`, `TerminationTermCfg`, `TiledCameraCfg`, `ArticulationCfg`, `SceneEntityCfg` 모두 존재 ✅
- **환경 이름**: `Isaac-Cartpole-v0` 없음 확인 (ERR-095) ✅
- **런타임**: **✅ 실행 확인** — `Isaac-Cartpole-Direct` 환경 생성 + 20,000+ 스텝 (아래 Warp 우회)

## Warp 1.16.0 호환 문제 — **해결됨** (ENV-001)

### 문제

IsaacSim 6.1 이 **번들한 Warp 1.16.0** (`extscache/omni.warp.core-1.16.0`) 이
Isaac Lab 3.0 의 `@wp.struct` 데코레이터(`isaaclab.scene_data`)와 충돌한다.

```
from isaaclab.envs import ManagerBasedRLEnvCfg  → 성공
import isaaclab_tasks
→ TypeError: issubclass() arg 1 must be a class
   (warp/_src/codegen.py Struct.__init__)
```

### 원인

- pip 에는 **warp-lang 1.17.0** 이 설치돼 있으나, IsaacSim 이 kit 확장으로
  번들 1.16.0 을 `sys.modules` 에 올려 **덮어쓴다**.
- 1.16.0 의 `Struct.__init__` 은 필드 타입에 `issubclass()` 를 가드 없이 호출 →
  타입이 클래스가 아닐 때 `TypeError`. 1.17.0 은 이 문제가 없다 (실측 대조).

### 해결 (1줄)

**`import warp` 를 `SimulationApp` 보다 먼저** 실행하면 pip 1.17.0 이 먼저
`sys.modules` 에 자리잡아 kit 이 번들 1.16.0 으로 덮어쓰지 못한다.

```python
import warp as wp        # ← 반드시 SimulationApp 보다 먼저
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": False})
import isaaclab_tasks    # ✅ 이제 정상
```

공식 데모(`scripts/environments/random_agent.py`)도 최상단에서
`import warp` 를 먼저 한다 — 같은 우회다.

### 동작 확인 (실측)

| 단계 | 결과 |
|---|---|
| `isaaclab_tasks` import | ✅ (143개 환경 등록) |
| `gym.make(Isaac-Cartpole-Direct, num_envs=16)` | ✅ 3.2초 |
| `env.reset()` | ✅ obs['policy'] |
| 물리 스텝 | ✅ 20,460 스텝, reward 계산 |
| GUI 뷰포트 렌더 | ✅ (`use_fabric=False` + `app.update()`) |

> **GUI 뷰포트 주의**: Isaac Lab 은 기본 `use_fabric=True` 라 씬이 Fabric 에만
> 있어 에디터 뷰포트가 비어 보인다. `env_cfg.sim.use_fabric = False` 로
> USD 저작을 강제하고 루프에서 `app.update()` 를 호출해야 카트폴이 화면에 뜬다.

## 실습 파일

| 파일 | 책 블록 | 상태 | 비고 |
|---|---|---|---|
| [`cartpole_random_agent.py`](cartpole_random_agent.py) | b04 | ✅ 실행 | ERR-095 수정, 64환경 300스텝 보상 출력 |
| [`cartpole_gui.py`](cartpole_gui.py) | b04 응용 | ✅ 실행 | GUI 16환경, use_fabric=False+app.update() |
| [`env_cfg_examples.py`](env_cfg_examples.py) | b02·b03·b08 | ✅ import | ManagerBased/Direct/TiledCamera 구조 검증 |

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch14/ch14_cartpole_gui.png`](../_docs/images/book/ch14/ch14_cartpole_gui.png) | Isaac Lab GUI 전체 — Cartpole 16 환경, Stage 트리에 physicsScene·World |
| [`../_docs/images/book/ch14/ch14_cartpole_viewport.png`](../_docs/images/book/ch14/ch14_cartpole_viewport.png) | 뷰포트 — 카트폴 격자, RTX Real-Time 렌더 |
| [`../_docs/images/book/ch14/ch14_cartpole.mp4`](../_docs/images/book/ch14/ch14_cartpole.mp4) | **영상** — 9개 환경 랜덤 정책 동작 (12초) |
| [`../_docs/images/book/ch14/ch14_cartpole_viewport.mp4`](../_docs/images/book/ch14/ch14_cartpole_viewport.mp4) | **영상(뷰포트)** — 카트폴 폴 흔들림 (12초) |
