# 14.6 내 로봇으로 Isaac Lab 프로젝트 만들기

> **이 절의 목표**: Isaac Lab 템플릿 생성기로 학습 프로젝트를 뼈대부터 만들고, 거기에 내 로봇을 끼워 넣는 것.
> 5.10절에서 CAD로 뽑은 로봇, 5.2절에서 임포트해 USD로 저장한 로봇을 여기서 학습시킨다.

---

## 14.6.1 `uv run isaaclab` — 실행 명령이 바뀌었다

⚠️ **먼저 알아 둘 것.** 인터넷 자료 대부분은 `./isaaclab.sh` 로 시작한다. Isaac Lab 3.0에서 이 스크립트는 **지원 중단(deprecated)** 되었고 **3.1에서 제거될 예정**이다. 실행하면 다음 경고가 뜬다.

```
[WARNING] isaaclab.sh is deprecated and will be removed in Isaac Lab 3.1.
          Use 'uv run isaaclab' instead.
```

| 역할 | 옛 명령 (~3.0) | **현재 명령 (3.0+)** |
|---|---|---|
| 설치 | `./isaaclab.sh --install` | `uv run isaaclab --install` |
| 환경 목록 | `./isaaclab.sh -p scripts/list_envs.py` | `uv run isaaclab list_envs` |
| 학습 | `./isaaclab.sh -p scripts/.../train.py` | `uv run isaaclab train --rl_library <lib> --task <id>` |
| 평가 | `... play.py` | `uv run isaaclab play --rl_library <lib> --task <id> --checkpoint latest` |
| 다중 GPU | 별도 런처 | `uv run isaaclab train_multigpu --num_gpus 2` |

`--checkpoint latest` 가 생긴 것이 특히 편하다. 예전처럼 로그 폴더를 뒤져 가장 최근 디렉터리를 찾아 경로를 복사할 필요가 없다.

---

## 14.6.2 프로젝트 템플릿 생성

Isaac Lab은 학습 프로젝트의 뼈대를 만들어 주는 생성기를 갖고 있다. 빈 폴더에서 시작하지 말고 이것을 쓰자.

```bash
cd ~/IsaacLab
uv run isaaclab --new
```

대화형으로 다음을 묻는다.

### 질문 1 — 외부(external) vs 내부(internal)

| 선택 | 생기는 위치 | 쓰는 경우 |
|---|---|---|
| **external** | 원하는 아무 경로 | 내 프로젝트를 따로 git 관리 (**권장**) |
| internal | `IsaacLab/source/` 안 | Isaac Lab 본체에 기여할 때 |

자기 로봇을 돌리는 거라면 **external**이다. Isaac Lab을 업데이트해도 내 코드가 섞이지 않는다.

### 질문 2 — 워크플로

| 선택 | 성격 | 쓰는 경우 |
|---|---|---|
| **direct** | 보상·관측을 파이썬 메서드에 직접 쓴다 | 로직이 단순하고 빠르게 돌리고 싶을 때 |
| manager-based | 항(term) 단위로 조립한다 | 보상 항을 자주 갈아 끼우며 실험할 때 |

4족 보행처럼 보상 항이 많아지는 과제는 manager-based가 관리하기 좋지만, 처음 한 번은 **direct**가 코드 흐름이 눈에 보여 배우기 쉽다.

### 질문 3 — 단일 vs 다중 에이전트

로봇 한 종류를 수천 개 복제해 학습하는 보통의 경우는 **single-agent**다. 서로 다른 정책을 가진 로봇들이 한 환경에서 상호작용해야 할 때만 multi-agent를 고른다.

### 질문 4 — RL 라이브러리

스페이스바로 **여러 개를 동시에 선택**할 수 있다. 선택한 수만큼 설정 파일이 생성되므로, 나중에 라이브러리를 바꿔 비교하기 쉽다.

| 라이브러리 | 단일 에이전트 | 다중 에이전트 |
|---|---|---|
| **rsl_rl** | PPO, Distillation | — |
| **skrl** | PPO, AMP | IPPO, MAPPO |
| rl_games | PPO | — |
| sb3 (Stable-Baselines3) | PPO | — |

---

## 14.6.3 생성된 구조 — 영상·구버전 자료와 다르다

⚠️ **경로가 바뀌었다.** 구버전 자료는 `source/<프로젝트>/<프로젝트>/tasks/direct/<이름>/` 을 안내한다. Isaac Lab 3.0이 만드는 구조는 다음과 같다.

```
spiderbot_tutorial/
├── pyproject.toml
├── README.md                     ← 실행 명령이 전부 적혀 있다
├── scripts/
│   └── list_envs.py
├── tests/
│   └── test_registration.py
└── src/spiderbot_tutorial/
    ├── assets/                   ← 내 로봇 USD·데이터를 둘 곳
    │   └── data/
    └── tasks/
        └── locomotion_direct/           ← <과제이름>_<워크플로>
            ├── __init__.py              ← 과제 ID 등록
            └── config/
                └── spiderbot/           ← <로봇이름>
                    ├── env.py           ← 보상·관측·종료 조건
                    ├── env_cfg.py       ← 로봇·씬·물리 설정
                    └── agents/
                        ├── rsl_rl_ppo_cfg.py
                        └── skrl_ppo_cfg.yaml
```

핵심 파일은 **세 개**다.

| 파일 | 무엇을 정하나 |
|---|---|
| `env_cfg.py` | 어떤 로봇, 어떤 씬, 환경 몇 개, 물리 파라미터 |
| `env.py` | 보상, 관측, 언제 에피소드를 끊을지 |
| `agents/*_cfg.py` | 학습 반복 횟수, 학습률 등 알고리즘 설정 |

---

## 14.6.4 내 로봇 끼워 넣기

생성 직후 프로젝트는 **Cartpole**로 채워져 있다. 이것을 내 로봇으로 바꾼다.

`env_cfg.py`를 열면 로봇을 지정하는 줄이 하나 있다.

```python
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab_assets.robots.cartpole import CARTPOLE_CFG

class LocomotionEnvCfg(DirectRLEnvCfg):
    ...
    robot: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
```

📌 **로봇 정의를 어디에 둘 것인가.** 구버전 자료는 `IsaacLab/source/isaaclab_assets/robots/` 에 파일을 추가하라고 안내한다. 그렇게 하면 Isaac Lab 본체를 건드리게 되어 업데이트 때 충돌한다. **external 프로젝트에서는 내 프로젝트 안에 두는 것이 맞다.**

`src/spiderbot_tutorial/assets/spiderbot.py` 를 새로 만든다.

```python
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

SPIDERBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # 5.2절에서 임포트해 저장한 USD 경로
        usd_path="/절대/경로/spiderbot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.25),          # 서 있을 때의 몸통 높이
        joint_pos={".*_hip": 0.0, ".*_thigh": 0.35, ".*_calf": -0.70},
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=40.0,             # 로봇 질량에 따라 100배까지 달라진다
            damping=4.0,
        ),
    },
)
```

그리고 `env_cfg.py`에서 바꿔 끼운다.

```python
# from isaaclab_assets.robots.cartpole import CARTPOLE_CFG
from spiderbot_tutorial.assets.spiderbot import SPIDERBOT_CFG

robot: ArticulationCfg = SPIDERBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
```

⚠️ **`joint_pos` 의 정규식은 실제 관절 이름과 맞아야 한다.** 5.9절에서 확인했듯 Go2는 `FL_hip_joint` 처럼 접두사가 붙고, 순서도 다리별이 아니라 종류별이다. 반드시 `dof_names`를 먼저 출력해 확인하자.

---

## 14.6.5 보상과 종료 조건

`env.py`에 세 개의 메서드가 있다. 학습 결과는 거의 전적으로 이 셋이 결정한다.

```python
def _get_observations(self) -> dict:
    """정책에 넣을 입력. 로봇이 '볼 수 있는 것'만 넣어야 한다."""

def _get_rewards(self) -> torch.Tensor:
    """원하는 행동에 +, 원치 않는 행동에 -"""

def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
    """에피소드를 언제 끊을지. (종료, 시간초과) 두 개를 돌려준다."""
```

### 종료 조건이 학습 속도를 좌우한다

> 💡 **몸통이 바닥에 닿으면 즉시 끊는다.** 넘어진 뒤의 시간은 배울 것이 없는 낭비다. 이 조건 하나만 넣어도 학습이 눈에 띄게 빨라진다.

```python
def _get_dones(self):
    # 몸통이나 허벅지가 바닥에 닿았다 → 실패로 보고 리셋
    contact = self._contact_sensor.data.net_forces_w_history
    died = torch.any(
        torch.max(torch.norm(contact[:, :, self._base_ids], dim=-1), dim=1)[0] > 1.0,
        dim=1,
    )
    time_out = self.episode_length_buf >= self.max_episode_length - 1
    return died, time_out
```

### 보상은 적게 시작한다

처음부터 항을 여덟 개씩 넣으면 어느 것이 효과가 있는지 알 수 없다. **전진 속도 + 넘어짐 페널티** 둘로 시작해 하나씩 늘리자.

---

## 14.6.6 학습 반복 횟수를 바꾸는 곳

> 💡 찾기 어려운 설정 1위다. `agents/` 폴더 안에 있다.

```python
# src/spiderbot_tutorial/tasks/locomotion_direct/config/spiderbot/agents/rsl_rl_ppo_cfg.py
num_steps_per_env = 16
max_iterations = 150        # ← 여기
```

skrl을 쓴다면 같은 폴더의 `skrl_ppo_cfg.yaml` 에 있다. 템플릿 기본값은 **150 반복**인데, 4족 보행은 보통 1,000~1,500 반복이 필요하다.

---

## 14.6.7 실행

### 과제 ID 확인

템플릿은 `<프로젝트>-<과제>-<로봇>-<워크플로>` 형식으로 ID를 만든다.

```bash
cd spiderbot_tutorial
uv run isaaclab list_envs
```

위 설정으로 생성하면 다음 ID가 나온다.

```
SpiderbotTutorial-Locomotion-Spiderbot-Direct
```

### 학습

```bash
# 눈으로 보며 디버깅 — 환경 수를 줄인다
uv run isaaclab train --rl_library rsl_rl \
    --task SpiderbotTutorial-Locomotion-Spiderbot-Direct --num_envs 1000

# 본 학습 — headless 가 훨씬 빠르다
uv run isaaclab train --rl_library rsl_rl \
    --task SpiderbotTutorial-Locomotion-Spiderbot-Direct --num_envs 4096 --headless
```

### 평가

```bash
uv run isaaclab play --rl_library rsl_rl \
    --task SpiderbotTutorial-Locomotion-Spiderbot-Direct \
    --num_envs 20 --checkpoint latest
```

⚠️ **정책 파일(`policy.onnx`, `policy.pt`)은 `play` 를 한 번 돌려야 생성된다.** 학습만 하고 `logs/.../exported/` 폴더를 찾으면 없다. Chapter 19 배포는 이 파일에서 출발한다.

---

## 14.6.8 실습 — 템플릿 생성부터 등록 확인까지

> 🛠️ **실습**: 로봇 없이도 할 수 있다. 템플릿을 만들고 과제가 등록되는 것까지 확인하자.

```bash
cd ~/IsaacLab
uv run isaaclab --new
#   external / direct / single-agent / rsl_rl+skrl 선택
#   프로젝트 이름: spiderbot_tutorial
#   과제 이름: locomotion, 로봇 이름: spiderbot

cd <생성경로>/spiderbot_tutorial
uv run isaaclab list_envs            # 과제 ID 확인
uv run pytest                        # 등록 테스트 통과 확인
uv run isaaclab zero_agent --task <ID> --num_envs 16   # 씬이 뜨는지 확인
```

**체크리스트**

- [ ] `src/<프로젝트>/tasks/<과제>_<워크플로>/config/<로봇>/` 경로가 생겼는가?
- [ ] `env.py` · `env_cfg.py` · `agents/` 세 가지가 있는가?
- [ ] `list_envs` 에 과제 ID가 보이는가?
- [ ] `pytest` 가 통과하는가?
- [ ] `zero_agent` 로 Cartpole 씬이 뜨는가?

여기까지 되면 로봇만 바꿔 끼우면 된다.

---

## 이 절에서 배운 것

| 항목 | 핵심 |
|---|---|
| 실행 명령 | `./isaaclab.sh` 는 지원 중단 → **`uv run isaaclab`** |
| 템플릿 생성 | `uv run isaaclab --new` — 빈 폴더에서 시작하지 않는다 |
| 선택지 | external · direct · single-agent 가 자작 로봇 입문에 적합 |
| 구조 | `src/<pkg>/tasks/<과제>_<워크플로>/config/<로봇>/` (구버전 경로와 다름) |
| 로봇 정의 | Isaac Lab 본체가 아니라 **내 프로젝트의 `assets/`** 에 둔다 |
| 반복 횟수 | `agents/rsl_rl_ppo_cfg.py` 의 `max_iterations` (기본 150) |
| 정책 파일 | `play` 를 돌려야 `exported/` 가 생긴다 |

다음 Chapter에서는 이렇게 만든 환경으로 실제 강화학습을 돌린다.
