# Chapter 3 — 개발 환경 구축

책 3장의 설치 절차와 첫 실습이다.

## 실행 방법

```bash
conda activate isaac_env
export OMNI_KIT_ACCEPT_EULA=YES
python hello_isaac.py
```

## 파일 목록

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`hello_isaac.py`](hello_isaac.py) | 3.8 [실습] Hello Isaac Sim | ✅ 동작 | **원문 그대로** (수정 없음) |
| [`verify_physics.py`](verify_physics.py) | — | ✅ 동작 | 낙하 물리 검증 (책에 없음) |

---

## 설치 절차 — 원문과의 차이

### ERR-003: 같은 pip 명령이 두 번 실려 있다

책 3.3 Step 4에는 **완전히 동일한 명령**이 두 번 나온다. 주석만 다르다.
한 번만 실행하면 된다.

```bash
export OMNI_KIT_ACCEPT_EULA=YES
pip install "isaacsim[all,extscache]==6.1.0.0" --extra-index-url https://pypi.nvidia.com
```

### ERR-005: 터미널 세션이 유지된다고 가정한다

책은 Step 2에서 `conda activate isaac_env`를 한 뒤, Step 3·4에서 그 상태가 이어진다고
전제한다. 하지만 터미널을 새로 열거나 스크립트로 묶으면 활성화가 풀리고,
**에러 없이 시스템 파이썬에 설치된다.** 문제는 한참 뒤 `isaacsim` 실행 시점에 드러난다.

> ⚠️ Step 3부터는 프롬프트 앞에 `(isaac_env)`가 있는지 **매번** 확인하세요.

한 줄로 안전하게 실행하려면:
```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate isaac_env && pip install ...
```

### ERR-006: ROS 2가 이미 깔린 PC에서는 경고가 뜬다

`~/.bashrc`에 `source /opt/ros/jazzy/setup.bash`가 있으면 conda 환경 안에서도
pip이 ROS 패키지를 본다. 설치 중 다음 경고가 나오지만 **Isaac Sim 설치 자체에는 문제없다.**

```
ERROR: pip's dependency resolver ...
launch-ros 0.26.12 requires pyyaml, which is not installed.
```

### 드라이버 — 책의 요구사항보다 낮아도 동작한다

| | 버전 |
|---|---|
| 책 / NVIDIA 공식 요구 | 595.58.03 |
| **실제 검증 환경** | **580.159.04** |

580.159.04에서 **GUI·RTX 렌더러(Real-Time 2.0)·물리 시뮬레이션이 모두 정상 동작**했다.
NVIDIA 문서 각주도 "Isaac Sim was tested on these driver versions"라고 되어 있어,
595는 테스트된 버전이지 최소 요구사항은 아닌 것으로 보인다.

---

## ERR-019: 3.8절 "예상 출력"이 실제와 다르다

책의 첫 실습이자, 독자가 **성공/실패를 판단하는 기준**인데 그 숫자가 실제와 최대 3.6m 벌어진다.

### 실측 설정
```
physics_dt = 0.0166667 s (60 Hz)
중력       = [0, 0, -1] × 9.81 m/s²
낙하 거리  = 5.0(시작) - 0.1(구 반지름) = 4.9 m
이론 착지  = √(2 × 4.9 / 9.81) = 0.9995 s = 60 스텝
```

### 세 값의 비교

| Step | 자유낙하 이론 | **Isaac Sim 실측** | 책 "예상 출력" |
|---|---|---|---|
| 0 | 4.999 | **4.992** | 4.998 |
| 50 | 1.456 | **1.109** | **4.751** ❌ |
| 100 | 0.100 | **0.400** | **3.851** ❌ |
| 150 | 0.100 | **0.100** | **2.301** ❌ |
| 200 | 0.100 | 0.100 | 0.100 |

책 숫자에서 중력을 역산하면 **약 0.71 m/s²** 다. 지구(9.81)도, 달(1.62)도, 화성(3.72)도 아니다.

### 구체는 "튕긴다" — 책이 놓친 현상

10스텝 간격으로 측정하면 바운스가 보인다.

| Step | t(s) | z(m) | 상태 |
|---|---|---|---|
| 40 | 0.683 | 2.430 | 낙하 중 |
| 50 | 0.850 | 1.109 | |
| **60** | **1.017** | **0.193** | **첫 착지** (이론 0.9995 s와 일치) |
| 70 | 1.183 | 0.654 | 튕겨 오름 |
| **80** | **1.350** | **0.842** | **바운스 정점** |
| 100 | 1.683 | 0.400 | 다시 하강 |
| 110 | 1.850 | 0.184 | 2차 착지 |
| **140** | **2.350** | **0.100** | **완전 정지** |

`add_default_ground_plane()`의 기본 반발계수 때문이다.

> 💡 **50스텝 간격으로 보면 바운스가 안 보이고, 10스텝으로 보면 보인다.**
> "샘플링 간격에 따라 물리 현상을 놓친다"는 개념은 센서(6장)와 데이터 수집(16.6장)에서
> 똑같이 반복된다. 카메라를 30fps로 찍으면 그보다 빠른 움직임은 기록에 남지 않는다.

직접 확인하려면:
```bash
python verify_physics.py
```
이론값 · Isaac Sim 실측값 · 책 예상출력을 나란히 출력한다.

---

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch03/ch3_hello_isaac_gui_095512.png`](../_docs/images/book/ch03/ch3_hello_isaac_gui_095512.png) | Hello Isaac Sim GUI 실행 화면 |
| [`../_docs/images/book/ch03/ch3_falling_sphere_095512.mp4`](../_docs/images/book/ch03/ch3_falling_sphere_095512.mp4) | 구 자유낙하 + 바운스 영상 |
