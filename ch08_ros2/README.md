# Chapter 8 — ROS 2 연동

코드 15블록. **1~7장과 성격이 완전히 다른 장이다.**

1~7장은 순수 Isaac Sim API였다. 8장은 Isaac Sim ↔ 시스템 ROS 2 사이의 **다리**라,
코드가 맞아도 **환경이 맞지 않으면 한 줄도 돌지 않는다.** 그래서 이 장의 가장 큰
오류도 코드가 아니라 **환경 설정 절(b02)** 에 있다.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-045** 🔴 | **8.1 b02** | **pip 설치용 ROS 2 환경변수 3개가 통째로 빠짐** — 8장 전체가 여기서 막힌다 |
| **ERR-048** 🔴 | 8.3 b05 등 | **존재하지 않는 노드**(`ROS2ReadSimulationTime`)와 **존재하지 않는 입력**(`timestamp`)에 연결 |
| ERR-047 🟡 | 8.3 b05 | 마지막 줄 경로 앞 쉼표 오타 `",/World/Carter/..."` |
| ERR-049 🟠 | 8.2 b06, 8.7 b15 | **배포판 표기가 장 안에서 모순** — b02 는 jazzy, b06·b15 는 humble |
| **ERR-050** 🔴 | 8.4 b07 | `ROS2RtxLidarHelper` 에 `lidarPrimPath` 입력이 **없다** → `renderProductPath` |
| ERR-051 🔴 | 8.5 b08 | `IsaacArticulationController` 에 `usePath` 입력이 **없다** |
| ERR-052 🔴 | 8.6 b10 | `IsaacComputeDifferentialDriveCommand` 노드가 **없다** → `DifferentialController` |
| ERR-053 🟠 | 8.4 b07 | LaserScan·PointCloud2 **동시 발행 불가** (택일), `fullScan` 은 무시됨 |
| **ERR-054** 🔴 | 8.4 b07 | 렌더 프로덕트를 **그래프 안에서** 만들어야 한다 (공식 6노드 구조) |
| ~~ERR-046~~ | 8.1 b03 | **철회** — 원고 방식(`set_extension_enabled_immediate`)은 정상이었다 |

### ERR-049 — 같은 장 안에서 ROS 2 배포판이 갈린다

```bash
# b02 (8.1)
source /opt/ros/jazzy/setup.bash      # "ROS 2 Jazzy 소싱 (Ubuntu 24.04 권장 조합)"

# b06 (8.2), b15 (8.7)
source /opt/ros/humble/setup.bash     # ← 갑자기 humble
```

한 장 안에서 jazzy 와 humble 이 섞여 있다. Ubuntu 24.04 + Jazzy 조합을 권장해
놓고 뒤에서는 humble 을 소싱하라고 한다. 이 환경에는 `/opt/ros/humble` 이 없으므로
b06·b15 를 그대로 따라 하면 소싱 자체가 실패한다. **jazzy 로 통일해야 한다.**

## 실행 방법

```bash
conda activate isaac_env
source setup_ros2_env.sh              # ← ERR-045 수정본. 이게 없으면 안 된다
python ros2_camera_publish.py
```

다른 터미널에서 확인:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list
ros2 topic hz /camera/image_raw
```

## 파일 목록

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`setup_ros2_env.sh`](setup_ros2_env.sh) | 8.1 b02 | ✅ | 수정 1건 (**ERR-045**) |
| [`ros2_camera_publish.py`](ros2_camera_publish.py) | 8.1 b03 + 8.3 b05 | ✅ | 수정 2건 (**ERR-048**, ERR-047) |
| [`ros2_full_bridge.py`](ros2_full_bridge.py) | 8.4~8.6 b07·b08·b09·b10 | ⚠️ | 수정 5건 (ERR-050~054). b08·b09·b10 토픽 수신 확인, **b07 은 미완** |
| [`demo_ch08_gui.py`](demo_ch08_gui.py) | — | ✅ | 화면으로 보는 데모 |
| [`ogn_name_check.py`](ogn_name_check.py) | — | ✅ | 노드·속성 이름을 공식 문서와 정적 대조 (Isaac Sim 불필요) |

### 정적 대조가 빠르다

`ogn_name_check.py` 는 Isaac Sim 을 띄우지 않고 `ogn/docs/*.rst` 를 읽어
책이 쓰는 노드·속성 이름이 실재하는지 확인한다. 하나씩 실행해 보는 것보다
빠르고 빠짐없다 — ERR-050·051·052 를 이걸로 한 번에 찾았다.

```bash
python3 ogn_name_check.py
```

> ⚠️ 처음에는 `.ogn` **소스 파일**만 찾았다가 틀렸다. ROS2 노드 대부분은
> C++ 구현이라 `.ogn` 이 패키지에 6개뿐이다. "파일 없음" 을 "노드 없음" 으로
> 읽어 멀쩡한 노드 5개를 오류로 보고할 뻔했다. 자동 생성 문서(`.rst`)가
> 35개 전부의 인터페이스를 담고 있으므로 그쪽을 본다.

## ERR-045 — 이 장 최대의 함정

원고 b02 는 세 가지가 어긋난다.

```bash
source ~/isaacsim/ros2_workspace/humble_ws/install/setup.bash   # 그런 경로 없음
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml        # 그런 파일 없음
```

pip 설치 Isaac Sim 에는 `ros2_workspace` 자체가 없다. 게다가 ROS 2 **Jazzy** 환경에서
**humble**_ws 를 소싱하라는 지시는 그 자체로 모순이다.

**정작 필요한 환경변수 3개가 없다.** 없으면 bridge 가 조용히 죽는다.

```
[Error] ROS2 Bridge startup failed
Could not load .../isaacsim.ros2.core/jazzy/lib/librmw_implementation.so
Error: libament_index_cpp.so: cannot open shared object file
```

`libament_index_cpp.so` 는 **같은 폴더 안에 있다.** `LD_LIBRARY_PATH` 만 걸어 주면
`librmw_implementation.so` 가 이웃을 찾아 로딩에 성공한다.

```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<isaacsim>/exts/isaacsim.ros2.core/jazzy/lib
```

Isaac Sim 은 이 값들을 **로그에 직접 출력한다.** 그런데 오류 로그에 묻혀 있어
놓치기 쉽다.

### 왜 진단이 어려운가

`is_extension_enabled()` 가 **True 를 돌려준다.** 활성화는 됐는데 내부 라이브러리
로딩만 실패한 상태다. 그래서 겉보기엔 멀쩡한데 노드만 없다.

### 번들 ROS 2 vs 시스템 ROS 2

| 상황 | 방법 |
|---|---|
| 기본 토픽만 (카메라·TF·joint_states) | **번들** — `source setup_ros2_env.sh` |
| 커스텀 메시지·Nav2·MoveIt2 | **시스템** — `source setup_ros2_env.sh system` |

8.2~8.5 는 번들로 충분하고, 8.6(Nav2)·8.7(MoveIt2)은 시스템 ROS 2 가 필요하다.
**원고는 이 구분을 하지 않는다.**

## ERR-048 — 없는 노드, 없는 입력

```python
# 원고
("sim_time", "isaacsim.ros2.bridge.ROS2ReadSimulationTime"),        # 없는 노드
("sim_time.outputs:simulationTime", "rgb_pub.inputs:timestamp"),    # 없는 입력
```

`isaacsim.ros2.nodes` 가 제공하는 ROS2 노드는 **35개**인데 `ROS2ReadSimulationTime`
이 없다. 시뮬레이션 시간 노드는 **core 확장** 소속이다 —
`isaacsim.core.nodes.IsaacReadSimulationTime`.

그리고 `ROS2CameraHelper` 의 실제 입력 17개에 **`timestamp` 가 없다.**
타임스탬프는 노드가 내부에서 처리한다(`resetSimulationTimeOnStop`, `useSystemTime`).

**→ sim_time 노드와 그 연결을 통째로 삭제하면 된다.**

## 실측 — 종단간 검증

Isaac Sim 이 발행하고 **시스템 ROS 2 Jazzy CLI 가 받는다.**

```
$ ros2 topic list
/camera/camera_info
/camera/depth/image_raw
/camera/image_raw

$ ros2 topic info /camera/image_raw
Type: sensor_msgs/msg/Image
Publisher count: 1

$ ros2 topic hz /camera/image_raw
average rate: 214.470

$ ros2 topic echo /camera/image_raw --once --field header
stamp: {sec: 186, nanosec: 800000000}
frame_id: camera_frame

width: 640   height: 480
```

`frame_id` 와 해상도가 코드에서 설정한 값과 정확히 일치한다.

## ⚠️ 주의 — `OnPlaybackTick` 은 play 전에 발행하지 않는다

그래프를 만들어도 `play()` 전에는 토픽이 **하나도 안 나온다.**
원고는 이 점을 명시하지 않아, 그래프만 만들고 토픽이 없다고 헤매기 쉽다.

```python
import isaacsim.core.experimental.utils.app as app_utils
app_utils.play()      # 이 시점부터 발행 시작
```

## ERR-046 철회 기록 — 검증자가 틀렸던 건

처음에 "원고의 `set_extension_enabled_immediate` 가 부실하다" 고 보고했다가
**철회했다.** 환경변수를 올바로 준 상태에서 원고 방식 그대로, 대기도 없이 시험하니
즉시 성공했다.

노드 생성이 실패한 진짜 원인은 ERR-045(환경변수)와 ERR-048(없는 노드 이름)이었다.
특히 내가 준비 확인용 probe 로 쓴 이름이 **존재하지 않는 노드**여서, 있지도 않은
것을 60초·250회 찾고 있었다.

> **교훈** — 증상에서 원인으로 건너뛰지 말 것.
> **진단에 쓴 도구 자체가 틀렸을** 가능성을 먼저 배제한다.
> probe 로 쓸 이름은 실재를 먼저 확인하고 쓴다.

## GUI 로 직접 보기 — Isaac Sim 이 보내고, ROS 2 뷰어가 받는다

```bash
conda activate isaac_env
source setup_ros2_env.sh
MINUTES=6 python demo_ch08_gui.py
```

다른 터미널에서 (둘 중 하나):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run rqt_image_view rqt_image_view /camera/image_raw   # 영상만 크게 — 권장
rviz2                                                       # Add > By topic > Image
```

장면에 **빨간 공이 기둥 둘레를 돈다.** 정지 화면이면 영상이 살아 있는지 알 수 없으므로
움직이는 물체를 일부러 넣었다. 뷰어에서 공이 움직이면 실시간 전달이 되고 있다는 뜻이다.

### 카메라를 물체에 바짝 붙이지 말 것

처음에는 카메라를 기둥 앞 2.6 m 에 두었더니 **기둥이 화면을 가득 채워 공이 보이지 않았다.**
영상이 살아 있음을 보여 주는 게 이 데모의 목적이므로 4.6 m 까지 물러났다.

## 책에 넣을 그림

| 파일 | 내용 |
|---|---|
| `ch08_isaacsim.png` | 보내는 쪽 — Isaac Sim 3D 뷰포트 |
| `ch08_rqt_image_view.png` | 받는 쪽 — `rqt_image_view`, 상단에 토픽 이름이 보인다 |
| `ch08_rviz2.png` | 받는 쪽 — RViz2 Image 패널 (우하단 31 fps 표시) |

### 캡처 요령 — 전체 화면을 찍지 말 것

처음에 `shot.sh full` 로 바탕화면 전체를 찍었더니 **작업 중이던 편집기와 문서까지
들어갔다.** 책에 넣을 그림이 아니다. 창 좌표를 구해 **해당 창만** 잡아야 한다.

```bash
# 창의 정확한 위치·크기를 먼저 구한다
DISPLAY=:1 xwininfo -root -tree | grep '"rqt_image_view"'
#   624x549+14+49  +1746+221     ← 크기 624x549, 절대좌표 (1746, 221)

DISPLAY=:1 ffmpeg -f x11grab -video_size 624x549 -i :1.0+1746,221 \
    -frames:v 1 out.png
```

`shot.sh auto` 는 이름이 같은 창이 여러 개일 때 엉뚱한 것을 고를 수 있다
(RViz2 는 보조 창이 여럿이라 실제로 실패했다). 좌표를 직접 주는 편이 확실하다.

## 미완료 — 정직하게 남겨 둔다

| 항목 | 상태 |
|---|---|
| **b07 LiDAR** | ⚠️ **열어 둠** — 아래 참조 |
| b11~b13 Nav2 · MoveIt2 | 시스템 ROS 2 + 별도 패키지 설치 필요 |
| b14 `ros2_camera_pub.py` 통합 실습 | 카메라 발행은 검증 완료, 통합본은 미작성 |

### b07 LiDAR — 왜 열어 두는가

공식 6노드 구조로 그래프는 생성되고 발행자도 등록된다.

```
$ ros2 topic info /scan
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
```

**그런데 메시지가 한 건도 나오지 않는다.** 다음은 확인했다.

- LiDAR **센서 자체는 정상** — `LidarSensor(annotators=['generic-model-output'])`
  로 읽으면 프레임마다 5,505,288 바이트 버퍼가 나온다
- ROS2 **브리지도 정상** — 같은 세션에서 카메라 토픽은 214 Hz 로 흘렀다
- 오류 로그 없음. 경고 둘뿐이고 둘 다 치명적이지 않다

센서도 되고 브리지도 되는데 **둘을 잇는 LiDAR 헬퍼 경로에서만** 막힌다.

배제한 가설:

| 가설 | 검증 | 결과 |
|---|---|---|
| 헤드리스 환경 탓 | GUI 모드로 재실행 | ❌ 탈락 — GUI 에서도 안 나온다 |
| 렌더 프로덕트를 파이썬에서 만들어서 | 공식 6노드 구조로 교체 | ❌ 여전히 안 나온다 |
| `PointCloudConfig` 노드 누락 | 문서 확인 | ❌ 메타데이터 선택용, 필수 아님 |
| 라이다가 맞출 물체가 없어서 | 벽 4개 배치 | ❌ 여전히 안 나온다 |

`showDebugView=True` 로 켜 보면 **라이다 광선이 씬에 그려지지 않는다**
(`ch08_lidar_debug.png`). 스캔 자체가 돌지 않는다는 뜻이다.
`LidarSensor` 래퍼로는 데이터가 나오므로, 그래프의 `IsaacCreateRenderProduct` 가
만든 렌더 프로덕트가 OmniLidar 를 실제로 구동하지 못하는 것으로 보인다.
정확한 원인은 확인하지 못했다.

### b08 · b09 · b10 — 토픽 수신까지 확인

```
$ ros2 topic list
/cmd_vel  /joint_command  /joint_states  /point_cloud  /scan  /tf

$ ros2 topic echo /joint_states --once --field name
['panda_joint1', ..., 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']

$ ros2 topic hz /joint_states     → average rate: 211.868
$ ros2 topic hz /tf               → average rate: 238.645
$ ros2 topic info /cmd_vel        → Subscription count: 1   (Isaac Sim 이 구독 중)
```

## 관련 문서

- 오류 표시: 각 실습 파일 상단 주석의 `[ERR-0NN]` 항목

---

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch08/ch08_isaacsim.png`](../_docs/images/book/ch08/ch08_isaacsim.png) | Isaac Sim ROS 2 브리지 실행 |
| [`../_docs/images/book/ch08/ch08_lidar_debug.png`](../_docs/images/book/ch08/ch08_lidar_debug.png) | LiDAR 포인트클라우드 디버그 |
| [`../_docs/images/book/ch08/ch08_rqt_image_view.png`](../_docs/images/book/ch08/ch08_rqt_image_view.png) | rqt_image_view 카메라 확인 |
| [`../_docs/images/book/ch08/ch08_rviz2.png`](../_docs/images/book/ch08/ch08_rviz2.png) | RViz2 센서 데이터 시각화 |
