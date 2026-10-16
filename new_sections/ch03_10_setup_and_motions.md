# 3.10 설치 보강 — 호환성 체커 · Windows · 모션 파일로 자세 잡기

> **이 절의 목표**: 앞 절들이 pip·Docker 설치에 집중하느라 지나친 세 가지를 채우는 것.
> 설치 전 하드웨어 확인, Windows 사용자를 위한 ZIP 설치, 그리고 코드 없이 로봇 자세를 반복 시험하는 방법이다.

---

## 3.10.1 설치 전에 호환성부터 확인한다

3.1절에서 요구사항 표를 봤다. 그런데 "내 PC가 그 표를 만족하는가"를 눈으로 대조하는 것은 실수하기 쉽다. NVIDIA는 이를 자동으로 검사하는 **호환성 체커(Omniverse Compatibility Checker)** 를 따로 배포한다.

1. Isaac Sim 문서의 요구사항 페이지에서 Windows용·Linux용 체커를 내려받는다.
2. ZIP을 풀고 `omni.isaac.sim.compatibility_check` 실행 파일을 실행한다.
3. 터미널과 함께 요약 창이 뜬다.

검사 항목은 다음과 같다.

| 항목 | 통과 기준 |
|---|---|
| GPU | RT Core 보유 (RTX 계열) |
| 드라이버 버전 | Isaac Sim 6.1 기준 Linux 595.58.03 / Windows 595.97 이상 |
| VRAM | 8 GB 이상 (학습은 더 필요) |
| OS | Ubuntu 22.04·24.04 또는 Windows 11 |
| 시스템 RAM | 32 GB 권장 |
| 디스크 여유 공간 | 셰이더 캐시용 |

**모두 녹색이면** 설치를 진행한다. 빨간 항목이 있으면 그 항목부터 해결해야 한다. 특히 드라이버는 설치 후에 겪는 문제 대부분의 원인이다.

> 💡 체커가 없어도 최소한 다음 두 줄은 실행해 보자. RT Core가 없는 GPU(A100·H100 등)는 체커 없이도 여기서 걸러진다.

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
```

---

## 3.10.2 Windows — ZIP 설치와 Isaac Sim Selector

3.3절의 pip 설치는 Linux·Windows 모두에서 동작하고, 파이썬으로 자동화할 때 가장 편하다. 다만 **GUI로 먼저 둘러보고 싶은 Windows 사용자**에게는 ZIP 설치가 더 간단하다.

### 설치

1. Isaac Sim 문서의 다운로드 섹션에서 Windows용 ZIP을 내려받는다.
2. `C:\isaacsim` 같은 **공백이 없는 짧은 경로**에 통째로 푼다.
3. 압축 해제가 끝나면 그 폴더에 실행 파일들이 보인다.

> ⚠️ 경로에 공백이나 한글이 들어가면 Kit 런처가 실패하는 경우가 있다. `C:\Users\홍길동\바탕 화면\...` 같은 경로는 피하자.

### Isaac Sim Selector

`isaac-sim.selector.bat` 를 실행하면 작은 선택 창이 뜬다. 여기서 실행 모드를 고른다.

| 선택 | 내용 |
|---|---|
| **Isaac Sim Full** | 전체 GUI. 처음에는 이것을 고른다 |
| Isaac Sim Headless | 화면 없이 실행 (원격·학습용) |
| ROS Bridge 옵션 | 사용할 ROS 배포판 선택 |

기본값 그대로 두고 **START** 를 누르면 된다. 바탕화면에 바로가기를 만들어 두면 편하다.

> 📌 **두 가지를 같이 설치해도 된다.** conda 환경의 pip 설치(스크립트 실행용)와 ZIP 설치(GUI 둘러보기용)를 함께 두는 구성이 실무에서 흔하다. 서로 간섭하지 않는다.

### 첫 실행

> ⚠️ 처음 실행하면 셰이더 캐시를 만드느라 **5~15분** 걸린다. 멈춘 것이 아니다. `Application Ready` 메시지가 뜰 때까지 기다린다.

---

## 3.10.3 모션 파일로 자세를 반복 시험하기

로봇을 임포트한 직후 가장 궁금한 것은 "이 자세가 물리적으로 가능한가"다. 5.9절의 Physics Inspector는 슬라이더로 하나씩 확인하는 도구라 **여러 자세를 순서대로 재생**하기에는 불편하다.

텍스트 파일에 관절 각도를 적어 두고 그대로 재생하면 훨씬 빠르다.

### 모션 파일 형식

한 줄이 한 자세다. 관절 수만큼 숫자를 쉼표로 적는다.

```
# motions.txt — Franka 9관절 (라디안)
# joint1, joint2, ..., joint7, finger1, finger2
0.0,  -0.6,  0.0, -2.2, 0.0, 2.0, 0.78, 0.04, 0.04
0.5,  -0.6,  0.0, -2.2, 0.0, 2.0, 0.78, 0.04, 0.04
0.5,  -0.3,  0.0, -1.8, 0.0, 2.0, 0.78, 0.00, 0.00
0.0,  -0.3,  0.0, -1.8, 0.0, 2.0, 0.78, 0.00, 0.00
```

### 재생 스크립트

```python
def load_motions(path, n_dof):
    """주석과 빈 줄을 건너뛰고 자세 목록을 읽는다."""
    poses = []
    for line in open(path, encoding="utf-8"):
        line = line.split("#")[0].strip()
        if not line:
            continue
        vals = [float(v) for v in line.replace(",", " ").split()]
        if len(vals) != n_dof:
            print(f"  [건너뜀] 관절 수 불일치 {len(vals)} != {n_dof}: {line[:40]}")
            continue
        poses.append(np.array(vals))
    return poses
```

각 자세를 일정 시간 유지하며 순서대로 재생한다.

```python
HOLD = 90                       # 자세당 유지 스텝
for pose in poses:
    for _ in range(HOLD):
        robot.apply_action(ArticulationAction(joint_positions=pose))
        world.step(render=True)
    # 목표와 실제가 얼마나 벌어졌는지 — 도달 못 한 자세를 찾아낸다
    actual = np.asarray(robot.get_joint_positions()).ravel()
    err = np.degrees(np.abs(actual - pose)).max()
    print(f"  최대 오차 {err:5.1f}도")
```

### 읽을 거리 — 오차가 알려 주는 것

> 💡 **목표와 실제의 차이가 그 자세의 실현 가능성을 알려 준다.**
>
> - 오차가 1도 이내 → 문제없이 도달
> - 오차가 수십 도 → **관절 한계에 걸렸거나, 게인이 부족하거나, 자기 몸과 충돌했다**
>
> 3D 프린터로 부품을 뽑기 전에 이 확인을 해 두면 "만들고 보니 그 자세가 안 나오는" 사태를 막을 수 있다.

실제 실행 결과(Franka). 마지막 자세는 `panda_joint2` 에 −3.0 rad(−172도)를 요청했는데, 이 관절의 하한은 −101도다.

```
  로봇 관절 9개: ['panda_joint1', 'panda_joint2', 'panda_joint3'] ...
  자세 5개 로드

  자세 1/5  최대 오차   1.0도  (panda_joint4)
  자세 2/5  최대 오차   1.0도  (panda_joint4)
  자세 3/5  최대 오차   1.0도  (panda_joint4)
  자세 4/5  최대 오차   0.9도  (panda_joint4)
  자세 5/5  최대 오차 102.0도  (panda_joint2)  ← 관절 한계 초과 요청
```

정상 자세는 1도 안쪽으로 들어오고, 한계를 넘긴 자세만 **102도** 가 벌어진다. 어느 관절이 문제인지까지 같이 찍어 주면 원인을 바로 짚을 수 있다.

```python
# 오차가 가장 큰 관절이 한계를 넘겨 요청받았는지 확인한다
props = robot.dof_properties
worst = int(np.argmax(np.abs(actual - pose)))
lo, hi = float(props["lower"][worst]), float(props["upper"][worst])
over = "" if lo <= pose[worst] <= hi else "  ← 관절 한계 초과 요청"
print(f"  최대 오차 {err:5.1f}도  ({names[worst]}){over}")
```

### 왜 코드가 아니라 파일인가

자세를 파이썬 코드에 직접 적으면 수정할 때마다 Isaac Sim을 다시 띄워야 한다. 자세 하나 바꾸는 데 수십 초가 든다.

파일로 분리하면 **시뮬레이터를 켜 둔 채** 텍스트만 고치면 된다. 파일 변경 시각을 감시해 자동으로 다시 읽게 하면 저장하는 즉시 로봇이 새 자세를 취한다.

```python
last_mtime = 0.0
while app.is_running():
    mtime = os.path.getmtime(MOTION_FILE)
    if mtime != last_mtime:           # 파일이 바뀌었다 → 다시 읽는다
        last_mtime = mtime
        poses = load_motions(MOTION_FILE, n_dof)
        print(f"  모션 파일 갱신 — {len(poses)}개 자세")
    ...
```

---

## 3.10.4 실습 — 세 가지 확인

> 🛠️ **실습**

1. 호환성 체커(또는 `nvidia-smi`)로 드라이버 버전을 확인하고, 6.1 요구 버전과 비교한다.
2. (Windows 사용자) ZIP 설치 후 Selector에서 **Isaac Sim Full** 로 실행해 본다.
3. `motions.txt` 에 자세 4개를 적고 재생해, 각 자세의 최대 오차를 기록한다.

**체크리스트**

- [ ] GPU에 RT Core가 있는가?
- [ ] 드라이버가 요구 버전 이상인가?
- [ ] (Windows) Selector 창이 뜨고 Full 모드로 실행되는가?
- [ ] 모션 파일의 자세 수만큼 재생되는가?
- [ ] 오차가 큰 자세가 있다면, 그 원인이 관절 한계인가 게인 부족인가?

---

## 이 절에서 배운 것

| 항목 | 핵심 |
|---|---|
| 호환성 체커 | 설치 전에 GPU·드라이버·RAM을 자동 점검 |
| RT Core | A100·H100은 RT Core가 없어 Isaac Sim을 실행할 수 없다 |
| Windows ZIP | 공백 없는 짧은 경로에 해제 → `isaac-sim.selector.bat` |
| 이중 설치 | pip(스크립트용) + ZIP(GUI용) 병행이 실무에서 흔하다 |
| 첫 실행 | 셰이더 캐시 5~15분은 정상 |
| 모션 파일 | 자세를 텍스트로 분리하면 시뮬을 끄지 않고 반복 시험할 수 있다 |
| 오차 읽기 | 목표-실제 차이가 자세의 실현 가능성을 알려 준다 |
