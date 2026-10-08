# Chapter 9 — 자율주행 (SLAM · Nav2)

코드 13블록. **8장보다 더 외부 의존이 크다.**

8장은 Isaac Sim 과 ROS 2 브리지만 있으면 됐다. 9장은 거기에 **SLAM Toolbox 와
Nav2 패키지**가 더 필요하고, 9.7 절은 **Isaac ROS**(cuVSLAM·nvblox)까지 요구한다.

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-055** 🔴 | 9.2 b03 | `IsaacComputeOdometry` 의 **네임스페이스가 틀림** — `wheeled_robots` 가 아니라 `core.nodes` |
| **ERR-056** 🔴 | 9.1 b02 | **Occupancy Map API 가 통째로 다름** — 확장 미활성화 + 메서드 5개가 없음 |
| **ERR-057** 🔴 | 9.2~9.3 | **타임스탬프 미연결로 SLAM 이 스캔을 전부 버림** + `/clock` 발행 누락 |
| **ERR-059** 🔴 | 9.3 b04 · 9.6 b10 | **`/cmd_vel` 이 와도 로봇이 안 움직임** — `vectord[3]` → `double` 타입 불일치 + `dt` 미연결 |
| **ERR-060** 🔴 | 9.7.5 | **설치 방법이 통째로 바뀜** — `run_dev.sh` 없음(5.0), 패키지 `isaac_ros_visual_slam` → **`isaac_ros_cuvslam`** |
| **ERR-061** 🔴 | 9.7.5 | **토픽 이름이 다름** — `/left/image_raw` → `front_stereo_camera/left/image_rect_color`, **`camera_info` 필수** |
| **ERR-062** 🔴 | 9.7.5 | **`base_link` TF 없으면 프로세스 중단**(SIGABRT) — 원고에 언급 없음 |
| **ERR-064** 🟠 | 9.7.5 | **Hawk 는 어안 150°** — 영상·`camera_info`·런치 설정이 서로 다른 렌즈를 말한다 |
| **ERR-066** 🔴 | 9.5 b09 · 9.6 b10 | **Nav2 런치 파일이 틀림** — `navigation_launch.py` 에 `map` 인자가 없고 **오류 없이 무시됨** |
| **ERR-067** 🔴 | 9.4 b07 | **플래너 플러그인 이름** `/` → `::` — 틀리면 Nav2 가 **통째로 중단** |
| **ERR-068** 🔴 | 9.6 b10 | **프레임 이름 불일치** `base_footprint` vs `base_link` — AMCL 이 TF 를 못 냄, 원고에 언급 없음 |
| **ERR-069** 🔴 | 9.3 b04 · 9.6 b10 | **라이다가 로봇 몸통 속에 묻힘** — 정면 반사 **0개**인데 SLAM 은 정상이라 안 잡힌다 |
| **ERR-070** 🔴 | 9.6 b10 | **`global_costmap` 만 1 Hz · 2.5 m** — 경로를 돌려 그리는 쪽이 뒤처져 장애물을 못 피한다 |
| ERR-058 🟠 | 9.3 b04 | 3D 라이다로 **LaserScan(2D)** 을 기대함 |
| ERR-049 🟠 | 9.3 b04 | 배포판 표기가 **humble** — 8장과 같은 문제가 9장에도 |

### ERR-059 — 연결은 되는데 값이 0 이다

```python
("cmd_sub.outputs:linearVelocity", "diff.inputs:linearVelocity"),   # 원고 — 타입이 다르다
```

`ROS2SubscribeTwist` 는 **`double3`**, `DifferentialController` 는 **`double`** 이다.
`og.Controller.connect()` 는 **예외를 던지지 않지만, 연결은 만들어지지 않는다.**

```
상류 연결 수 = 0          ← 연결이 아예 없다
diff.inputs:linearVelocity = 0.0   (기본값 그대로)
```

경고는 찍히지만 `[Error]` 가 아닌 `[Warning]` 이고, 기동 로그 수천 줄 사이에
`print()` 와 다른 스트림으로 나가 **내 출력 옆에 보이지 않는다.**

```
[Warning] [omni.graph.core.plugin] ... of type double3 (vector)
          is not compatible with ... of type double
```

ROS 쪽(`topic echo` · `topic info`)은 전부 정상으로 보인다. 끊긴 곳이
Isaac Sim **안쪽**이라 ROS 도구의 시야 밖이기 때문이다.

`BreakVector3` 로 성분을 뽑고, **기본값이 0.0 인 `dt` 도 함께 연결**해야 한다.

```python
("cmd_sub.outputs:linearVelocity", "lin_xyz.inputs:tuple"),
("lin_xyz.outputs:x", "diff.inputs:linearVelocity"),     # 전진 = x
("ang_xyz.outputs:z", "diff.inputs:angularVelocity"),    # 회전 = z
("tick.outputs:deltaSeconds", "diff.inputs:dt"),         # 없으면 계속 0
```

검증: 10 초에 5.71 m 이동, `wheelCmd=3.5714` = `0.5 ÷ 0.14`(바퀴 반지름) 과 일치.

### ERR-055 — 이름은 맞는데 소속이 다르다

```python
("compute_odom", "isaacsim.robot.wheeled_robots.IsaacComputeOdometry"),   # 원고
("compute_odom", "isaacsim.core.nodes.IsaacComputeOdometry"),             # 수정
```

`wheeled_robots` 확장에는 `Odometry` 관련 노드가 **하나도 없다.**
`/odom` 과 `odom → base_link` TF 는 SLAM·Nav2 의 전제라, 이게 막히면
9장 전체가 시작조차 못 한다.

### 원고가 빠뜨린 연결

원고는 `raw_tf_pub` 에 오도메트리 값을 연결하지 않는다. 그러면 TF 가
프레임 이름만 있고 **위치가 0 으로 고정된다.**

```python
("compute_odom.outputs:position",    "raw_tf_pub.inputs:translation"),
("compute_odom.outputs:orientation", "raw_tf_pub.inputs:rotation"),
```

## 전제 조건

```bash
# 9.1~9.6
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# 9.7 (cuVSLAM · nvblox) — Isaac ROS 별도 설치, 대용량
```

## 실행 방법

```bash
conda activate isaac_env
source ../ch08_ros2/setup_ros2_env.sh     # ERR-045 — 이게 없으면 브리지가 죽는다
python odometry_publish.py
```

확인 (다른 터미널):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 파일 목록

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`occupancy_map.py`](occupancy_map.py) | 9.1 b02 | ✅ | 수정 1건 (**ERR-056**) |
| [`odometry_publish.py`](odometry_publish.py) | 9.2 b03 | ✅ | 수정 1건 (**ERR-055**) + 빠진 연결 2줄 |
| [`slam_scene.py`](slam_scene.py) | 9.3 b04·b05 | ✅ | 수정 3건 (**ERR-057/058**, ERR-055) |
| [`slam_params.yaml`](slam_params.yaml) | 9.3 b05 | ✅ | 원고 그대로 유효 |
| [`nav2_params_b07.yaml`](nav2_params_b07.yaml) | 9.4 b07 | ✅ | 수정본 (**ERR-067** + `plugins` 선언) |
| [`nav2_params_full.yaml`](nav2_params_full.yaml) | 9.6 b10 | ✅ | **실제 자율주행에 쓴 전체 파라미터** (b07 적용 + ERR-068) |
| [`nav2_params_b07_book_asis.yaml`](nav2_params_b07_book_asis.yaml) | 9.4 b07 | ❌ | 원고 그대로 — FATAL 재현용 |
| [`type_mismatch_test.py`](type_mismatch_test.py) | — | ✅ | ERR-059 검증 도구 (책 밖) |
| [`cuvslam_scene.py`](cuvslam_scene.py) | 9.7.5 | ✅ | 스테레오·깊이·TF·오도메트리 발행, 추적 검증 완료 |
| [`carter_camera_probe.py`](carter_camera_probe.py) | — | ✅ | 자산의 카메라 목록 확인 (책 밖) |
| [`hawk_lens_probe.py`](hawk_lens_probe.py) | — | ✅ | ERR-064 렌즈 모델 확인 (책 밖) |
| [`grab_stereo.py`](grab_stereo.py) | — | ✅ | 좌·우 영상 캡처 도구 (책 밖) |
| [`build_cuvslam_sm120.sh`](build_cuvslam_sm120.sh) | — | ✅ | **Blackwell 용 cuVSLAM 빌드** (책 밖, 핵심) |
| [`compare_pose.py`](compare_pose.py) | — | ✅ | cuVSLAM 추정 vs 정답 대조 (책 밖) |
| [`quadruped_scene.py`](quadruped_scene.py) | — | ✅ | **4족보행(Spot·ANYmal·Go2) + 창고** (책 밖) |

## SLAM 전체 사슬 — 종단간 검증 완료

```
Isaac Sim (/scan 31Hz · /odom 188Hz · /tf · /clock 171Hz)
    ↓
slam_toolbox  → Registering sensor: [Custom Described Lidar]
             → dropping message 0건
    ↓
/map 발행 → map_saver_cli → slam_room.pgm + .yaml
```

지도 크기 **240×184 = 12.0 × 9.2 m** — 실제 방 12.0 × 9.0 m 와 일치한다.

![Isaac Sim SLAM 씬](../_docs/images/book/ch09/ch09_img04_isaac_slam_scene.png)

Stage 패널에 `ScanGraph` · `OdomGraph` · `CmdVelGraph` 세 OmniGraph 가 보인다.

![RViz2 실시간 매핑](../_docs/images/book/ch09/ch09_img03_rviz_slam_manual.png)

## 주행 속도가 지도 품질을 지배한다 — 실측 대조

![속도별 지도 비교](../_docs/images/book/ch09/ch09_img05_map_speed_compare.png)

왼쪽이 느린 자동주행, 오른쪽이 키보드 수동주행(teleop 기본값 `speed 0.50 / turn 1.00`)이다.
**같은 씬, 같은 파라미터, 같은 코드**인데 결과가 이만큼 다르다.

| | 느린 자동주행 | 키보드 수동주행 | 실제 방 |
|---|---|---|---|
| 지도 크기 | **12.00 × 9.20 m** | 13.95 × 12.50 m | 12.00 × 9.00 m |
| 오차 | +0.00 / +0.20 | **+1.95 / +3.50** | — |
| 점유 비율 | 4.9% | 7.0% | — |
| 벽 두께 중앙값 | 얇음 | **0.60 m** | 0.30 m |
| 벽 두께 최대 | — | **1.30 m** | 0.30 m |

### 원인이 아닌 것 — 두 가지를 배제했다

**① 파라미터가 아니다.** 실행 중인 노드에서 직접 읽었다.

```
do_loop_closing  True    loop_search_maximum_distance  3.0
resolution       0.05    use_sim_time                  True
```

**② 휠 슬립이 아니다.** 이게 의외다. 벽에 부딪치면 바퀴만 헛돌아
오도메트리가 틀어질 것 같지만, `IsaacComputeOdometry` 는 **휠 엔코더를 읽지 않는다.**

```
inputs:chassisPrim   "articulation root 또는 rigid body prim"
outputs:position     ← 물리 엔진의 실제 섀시 위치
```

즉 **정답(ground truth) 오도메트리**다. 바퀴가 헛돌아도 위치는 정확하다.
실물 로봇과 결정적으로 다른 점이라, **시뮬레이션에서 잘 되던 SLAM 이 실물에서
깨지는 대표적인 지점**이다.

### 남은 원인 — 스캔 쪽 (추정)

오도메트리가 정확한데 지도가 번졌다면 틀어진 것은 **라이다 스캔**이다.

- **회전 중 왜곡(motion distortion)** — 라이다는 한 바퀴 도는 **동안** 스캔을 모은다.
  그 사이 로봇이 크게 회전하면 직선 벽이 휘어진 채 지도에 박힌다.
- **충돌 시 섀시 기울어짐** — 라이다가 섀시에 붙어 있으므로 스캔 평면이 함께 기운다.
  2D 라이다가 기울면 벽까지 거리를 잘못 잰다.

> 이 둘의 기여도는 **분리하지 못했다.** 확실한 것은 위 표의 두 번 실측 대조다.
> 회전 속도만 바꿔 한 번 더 돌리면 분리할 수 있다.

### 실습 권장 설정

teleop 기본값은 이 방 크기에 **너무 빠르다.**

```
e 를 5번   turn   1.00 → 0.59
x 를 2번   linear 0.50 → 0.40
```

벽에서 1 m 이상 떨어져 돌고, 회전은 `j`/`l` 을 짧게 끊어 누른다.

| 파일 | 조건 |
|---|---|
| [`maps/slam_room_slow.pgm`](maps/slam_room_slow.pgm) | 느린 자동주행 — 실제 방과 일치 |
| [`maps/slam_room_manual.pgm`](maps/slam_room_manual.pgm) | 키보드 주행 — **번진 예시** |

## 실측 — 오도메트리가 살아 있다

Carter 바퀴를 굴리며 측정했다.

```
$ ros2 topic hz /odom
average rate: 243.616

$ ros2 topic echo /odom --once --field pose.pose.position
x: -1.297,  y: 1.174          (4초 뒤)  x: 1.908,  y: 1.770

$ ros2 run tf2_ros tf2_echo odom base_link
- Translation: [1.208, 3.012, -0.001]
- Rotation: RPY (degree) [0.040, -0.150, 144.942]
```

**가만히 둔 채로 재면 안 된다.** 정지 상태에서는 오도메트리가 0 이라
"발행은 되는데 맞는 값인지" 를 구별할 수 없다. 바퀴를 굴리며 재야
위치·회전이 실제로 변하는 것을 확인할 수 있다.

## 에셋·링크 이름을 가정하지 말 것

원고는 `/World/Carter/chassis_link` 를 하드코딩한다. 에셋마다 링크 이름이
다르므로 **타입이나 이름 패턴으로 찾는 편**이 안전하다.

```python
CHASSIS = next(
    (str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(CARTER))
     if "chassis" in p.GetName().lower()), CARTER)
```

NovaCarter 의 바퀴 관절은 `joint_wheel_left` · `joint_wheel_right` 두 개다.

## 검증 도구 — 네임스페이스까지 본다

ERR-055 는 **이름만 대조하는 도구로는 못 잡는다.** `IsaacComputeOdometry` 는
실재하기 때문이다. [`../ch08_ros2/ogn_name_check.py`](../ch08_ros2/ogn_name_check.py)
를 전체 타입 문자열(네임스페이스 포함) 비교로 고쳤다.

```bash
python3 ../ch08_ros2/ogn_name_check.py            # 내장 목록
python3 ../ch08_ros2/ogn_name_check.py <file.py>  # 파일에서 노드 문자열 추출
```

> 고치는 과정에서 도구가 **한 번 오보를 냈다.** `.rst` 의 ref 줄에서 밑줄을
> 점으로 바꾸면 `wheeled_robots` 가 `wheeled.robots` 로 깨진다. 밑줄이
> 구분자인지 이름의 일부인지 문자열만으로는 알 수 없어, 실제 확장 폴더
> 이름에서 후보를 만들어 대조하도록 고쳤다.
>
> **검증 도구가 "없다" 고 할 때, 도구의 한계인지 진짜 부재인지 구분해야 한다.**

## 남은 것 — 정직하게 적는다

> 이 표는 한때 b07~b10 이 전부 "미검증" 이었다. 그 뒤 실행으로 검증해
> 아래처럼 정리됐다. 진행 과정은 이 문서 뒤쪽에 그대로 남겨 둔다.

| 항목 | 상태 |
|---|---|
| b07 Nav2 costmap 파라미터 | ✅ 실행 검증 완료 (ERR-067, `obstacle_layer` 로 경고 0건) |
| b08 다중 AMR 네임스페이스 | ⏳ **미검증** — 이 장에서 유일하게 남은 항목 |
| b09·b10 Nav2 실행 | ✅ 자율주행 성공 (오차 6~15 cm) + 장애물 회피 (ERR-069·070) |
| 9.7 cuVSLAM (**3D SLAM**) | ✅ sm_120 직접 빌드로 추적 성공 (오차 5~24 cm) |
| 9.7.6 nvblox 3D 메시 | ⏳ **미실행** — 입력 토픽은 전부 확인됨 |

## 9.7 cuVSLAM — 어디까지 됐고 어디서 막혔나

| 단계 | 결과 |
|---|---|
| Docker · NVIDIA Container Toolkit · GPU 통과 | ✅ |
| Isaac ROS **5.0** 컨테이너 (Ubuntu 24.04 / ROS 2 **Lyrical**) | ✅ |
| cuVSLAM **15.0.0** 노드 로드 | ✅ |
| 스테레오 발행 115~136 Hz | ✅ |
| `camera_info` P[3] = −47.928 → 베이스라인 **15.00 cm** (실측 일치) | ✅ |
| `base_link` TF 요구사항 규명 + 우회 | ✅ |
| **실제 시각 추적** | ✅ **성공** — sm_120 직접 빌드 후 (아래 참고) |

> 아래 "막힌 지점" 은 **당시의 진단**이다. 결론부터 말하면 드라이버 제약이
> 아니었다. `cuobjdump` 로 보니 배포 바이너리에 sm_120 코드가 **아예 없었다.**
> cuVSLAM 이 오픈소스라 직접 빌드해서 해결했다. 오진 과정도 그대로 남긴다.

### 처음 진단 — 드라이버 탓으로 봤다 (오진)

```
Your NVIDIA driver supports CUDA 13.0, but cuVSLAM requires at least CUDA 13.2.
[CUDA] error no kernel image is available for execution on the device(209)
Failed to initialize cuvslam::Odometry
```

| | |
|---|---|
### ✅ 해결됨 — 소스에서 sm_120 으로 빌드하면 된다

[`build_cuvslam_sm120.sh`](build_cuvslam_sm120.sh) 한 번이면 끝난다. **1분 21초.**

```bash
git clone --depth 1 --branch v15.0.0 https://github.com/nvidia-isaac/cuVSLAM.git
cmake .. -DCMAKE_CUDA_ARCHITECTURES=120 -DCMAKE_BUILD_TYPE=Release && make -j
```

```
빌드 전  apt 설치본  sm_75    →  CUDA error 209
빌드 후  직접 빌드   sm_120   →  cuVSLAM tracker was successfully initialized.
```

**cuVSLAM 은 오픈소스다.** `isaac_ros_visual_slam` 의 **서브모듈**이라 눈에 안 띈다.

```
.gitmodules →  cuvslam = https://github.com/nvidia-isaac/cuVSLAM.git
```

#### ⚠️ 최신(v17.0.0)을 받으면 안 된다

17.0.0 바이너리에는 sm_120 이 들어 있지만 **API 가 바뀌었다.** ROS 래퍼가
찾는 심볼 26개 중 4개가 사라졌다(`Odometry::Track` 등). **설치된 래퍼와 같은
15.x 를 빌드**해야 한다. 스크립트가 교체 전에 심볼을 대조해 막아 준다.

#### 실측 — 정확하다

| 경과 | 랜드마크 | cuVSLAM 이동 | 정답 이동 | 오차 |
|---|---|---|---|---|
| 24s | 17,298 | 0.44 m | 0.49 m | **5 cm** |
| 36s | 20,855 | 4.12 m | 4.36 m | **24 cm** |
| 48s | 22,257 | 102.22 m | 4.48 m | 💥 폭주 |

#### 💥 48초 — 벽에 박은 채로 계속 전진하면 추적이 죽는다

```
[WARN]  Visual tracking is lost
[ERROR] Can't select enough new tracks to reach FAILED_ACTIVE_TRACK_COUNT limit
[ERROR] Failed to track on the PnP stage
```

벽에 정면으로 밀착하면 **화면이 민무늬 벽으로 꽉 찬다.** 특징점이 사라지고,
바퀴는 도는데 영상은 변하지 않는다.

**9장과 정확히 반대다.**

| 벽에 박혔을 때 | 9.1~9.6 2D SLAM | 9.7 cuVSLAM |
|---|---|---|
| 오도메트리 | **멀쩡** — 정답 포즈를 읽는다 | — |
| 추적 | 라이다는 거리를 잰다 → **무해** | **치명적** — 볼 게 없다 |

같은 사고가 한쪽에는 무해하고 한쪽에는 치명적이다. 2D SLAM 감각으로
비주얼 SLAM 을 다루면 여기서 깨진다.

> **실습 요령** — 벽을 정면으로 마주 보지 말고, 방 가운데를 크게 돌며
> 물체를 옆으로 흘려보낸다. 회전은 짧게 끊는다.

### 진단 기록 — 드라이버가 아니라 **cuVSLAM 빌드**가 원인이었다

처음에는 메시지대로 드라이버 문제로 판단했으나, 바이너리를 열어 보고 뒤집혔다.

```
$ cuobjdump --list-elf /opt/ros/lyrical/lib/libcuvslam.so
  sm_75                         ← 이것 하나뿐 (Turing, RTX 20 시리즈)

$ cuobjdump --list-ptx /opt/ros/lyrical/lib/libcuvslam.so
  No PTX file found             ← JIT 대비 중간코드도 없다
```

**RTX 5090 은 sm_120(Blackwell)** 이다. sm_75 기계어는 실행되지 않고,
PTX 가 없으니 드라이버가 JIT 컴파일할 수도 없다.

**GPU 도 CUDA 버전도 문제가 아니다.** 같은 컨테이너의 공식 라이브러리에는
sm_120 이 멀쩡히 들어 있다.

```
libcuvslam.so     sm_75                                   ← cuVSLAM 만 빠졌다
libcublas.so.13   sm_75 sm_80 sm_86 sm_90 sm_100 sm_120   ✅
```

**드라이버를 올려도 바이너리에 sm_120 코드가 생기지는 않는다.**
`cuda-compat-13-2` 가 듣지 않은 것도 당연한 결과였다.

최신 패키지(`5.0.0-0noble.20260918011121000`)도 마찬가지다.
**NVIDIA 가 sm_120 을 넣어 재빌드해야 풀린다.**

> **교훈 — `no kernel image(209)` 는 드라이버보다 바이너리를 먼저 의심한다.**
> `cuobjdump --list-elf` 로 1 분이면 확정된다. 메시지의 "update the driver"
> 를 믿고 드라이버를 교체했다면 위험만 지고 같은 자리에 멈춰 있었을 것이다.

### ⚠️ 가장 중요한 교훈 — 토픽이 나온다고 도는 게 아니다

GPU 초기화가 실패해도 **노드는 죽지 않고 계속 발행한다.**

```
$ ros2 topic hz   /visual_slam/tracking/odometry   →  97.6 Hz    ← 나온다
$ ros2 topic echo /visual_slam/tracking/odometry   →  전부 0.0   ← 값은 없다
```

원고 체크리스트가 **"오도메트리 값이 발행된다"** 인데, 이 조건은 **GPU 가
완전히 죽어도 참**이다. 값이 **변하는지**를 봐야 한다.

9장에서 "노드는 도는데 값이 0" 이 세 번 나왔다 — ERR-057(타임스탬프),
ERR-059(타입 불일치), 그리고 이번 GPU 실패. **발행 여부와 동작 여부는 다르다.**

### 🟠 ERR-064 — Hawk 는 어안 150° 다. 그냥 쓰면 추적이 틀어진다

![렌즈 수정 전후](../_docs/images/book/ch09/ch0907_img03_lens_before_after.png)

**위**가 자산 기본값(어안), **아래**가 핀홀로 고친 것이다. 바닥 격자를 보면 안다.

원고에 없는 내용이지만 실습하면 반드시 걸린다. 그대로 물리면 **셋이 서로 다른
말을 한다.**

| | 내용 |
|---|---|
| 렌더된 영상 | **어안 150°** (`cameraProjectionType = fisheyePolynomial`) |
| `camera_info` | **핀홀** `fx=319.5 cx=320 cy=240` |
| 런치 설정 | `rectified_images: True` — 왜곡이 없다고 믿는다 |

**토픽 이름이 `image_rect_color` 여도 왜곡은 펴지지 않는다.** 이름은 약속일 뿐,
파이프라인 어디에도 보정 단계가 없다.

```
실물 로봇   image_raw ──[캘리브레이션 계수]──> image_proc ──> image_rect
시뮬레이션  렌더 결과 ─────────────────────────────────────> image_rect_color
                                       ↑ 보정 단계가 없다
```

```python
def force_pinhole(cam_path):
    a = stage.GetPrimAtPath(cam_path).GetAttribute("cameraProjectionType")
    if a and a.IsValid():
        a.Set("pinhole")
```

> 실물 Hawk 를 쓸 때는 **반대 선택이 맞다.** 어안은 시야가 넓어 특징점 확보에
> 유리하므로, `rectified_images: False` 로 두고 왜곡계수를 `camera_info` 에
> 실어 보낸다. 핀홀 전환은 **검증을 단순화하기 위한 선택**이다.

### 스테레오 입력 — cuVSLAM 이 보는 것

![스테레오 쌍](../_docs/images/book/ch09/ch0907_img02_stereo_pair.png)

두 상자가 좌·우 영상에서 다른 위치에 찍힌다. 이 **시차**가 거리가 된다.
베이스라인 15.00 cm, 시차 평균 4.57.

### 씬 — 무늬가 없으면 추적이 끊긴다

![Isaac Sim 스테레오 씬](../_docs/images/book/ch09/ch0907_img01_isaac_stereo_scene.png)

컬러 상자 6 개를 일부러 배치했다. **cuVSLAM 은 무늬가 있어야 특징점을 잡는다.**
민무늬 흰 벽만 있으면 추적이 끊긴다. 2D 라이다와 결정적으로 다른 점이다.

## ✅ 9.6 b10 종단간 — 자율주행 성공

![자율주행 5단계](../_docs/images/book/ch09/ch0906_img03_nav2_sequence.png)

```
목표 ①  ( 4.0,  2.5)  →  도착 ( 4.01,  2.56)   오차  6 cm
목표 ②  (-4.0, -3.0)  →  도착 (-4.13, -3.08)   오차 15 cm
```

![RViz2 자율주행](../_docs/images/book/ch09/ch0906_img02_nav2_driving.png)

| RViz 화면 | 뜻 |
|---|---|
| **빨간 선** | 계획된 경로 |
| **초록 점무리** | AMCL 입자 필터 |
| **분홍 테두리** | costmap 팽창 0.5 m — **b07 파라미터가 먹은 증거** |
| 하늘색 | 라이다 스캔 |

영상: [Isaac Sim](../_docs/images/book/ch09/ch0906_vid01_nav2_isaacsim.mp4) ·
[RViz2](../_docs/images/book/ch09/ch0906_vid02_nav2_rviz.mp4)

### 원고 b10 체크리스트 대조

| 원고 | 결과 |
|---|---|
| `warehouse_map.yaml/.pgm` 생성 | ✅ 240×184, 0.05 m |
| Map 과 LaserScan 이 겹침 | ✅ |
| Nav2 Goal 클릭 시 전역 경로 | ✅ `/plan` 1.0 Hz |
| Carter AMR 이 목표까지 이동 | ✅ 오차 6~15 cm |
| 장애물 추가 시 경로 재계획 | ✅ **3.68 m 앞 감지 → 0.48 m 간격 통과** ([ERR-069](#err-069)·[ERR-070](#err-070)) |

> 이 항목은 처음에 ❌ 였다. 원인이 **두 개** 겹쳐 있었고 둘 다 찾아서 고쳤다.
> 아래 「장애물 재계획 — 두 가지를 잡기까지」 참고.

### 실제로 돌리는 절차 (수정 종합)

```bash
# ① Isaac Sim — 뷰포트 카메라 포함
HEADLESS=0 python slam_scene.py

# ② Nav2 — bringup_launch.py (navigation_launch.py 아님, ERR-066)
ros2 launch nav2_bringup bringup_launch.py \
  map:=./maps/slam_room_slow.yaml \
  params_file:=./nav2_params_full.yaml use_sim_time:=true

# ③ 초기 위치 (RViz 의 2D Pose Estimate)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0},
    orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,
    0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"

# ④ 목표 (RViz 의 Nav2 Goal)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 4.0, y: 2.5},
    orientation: {w: 1.0}}}"
```

### 마지막 체크리스트 — 처음엔 실패로 남겼다 (그 뒤 해결)

원고: *"이동 중 장애물을 추가해도 경로를 재계획한다"* — **확인하지 못했다.**

시험을 위해 `slam_scene.py` 에 ROS 토픽으로 움직이는 상자를 넣었다.

```bash
ros2 topic pub --once /obstacle_pose geometry_msgs/msg/Twist "{linear: {x: 3.0, y: 0.0}}"
```

**막힌 것 ①** — 로봇이 빨라 장애물보다 먼저 지나간다.
목표 (4.5, 0) 을 주면 **16초에 x=4.39** 에 닿는다. 방이 12 m 라 구간이 짧다.

**막힌 것 ②** — 벽 근처에서 복구가 실패한다. 🔴

```
[ERROR] [controller_server]: Failed to make progress          2건
[ERROR] [controller_server]: Costmap timed out waiting for update  2건
[ERROR] [bt_navigator]: Goal failed                           2건
```

목표를 (0,0) 으로 줬는데 로봇이 **반대로** 갔다(4.65 → 4.99).

| | 값 |
|---|---|
| `controller_frequency` | **20.0 Hz** (Nav2 기본) |
| Isaac Sim `/scan` 실측 | **21~29 Hz** (GUI 켜면 더 낮음) |

컨트롤러가 20 Hz 로 costmap 갱신을 기다리는데 스캔이 비슷하거나 느리다.
여유가 없어 한 번 밀리면 타임아웃이 나고, 그 상태에서 복구 동작이 들어가면
회복하지 못한다. **원고는 "재계획한다" 고 단정하지만 그 전에 복구부터 막힌다.**

조정해 볼 값 (미검증)

```yaml
controller_server: {ros__parameters: {controller_frequency: 10.0}}
local_costmap: {local_costmap: {ros__parameters:
  {update_frequency: 5.0, publish_frequency: 2.0}}}
```

### 🟠 곁가지 — `voxel_layer` 는 2D 라이다에 맞지 않는다

시험 중 이 경고가 쏟아졌다.

```
[WARN] [local_costmap]: Sensor origin at (0.50, -0.00 -0.00) is out of map bounds
```

Nav2 기본 local_costmap 은 **`voxel_layer`(3D)** 다. 우리 `/scan` 은 프레임이
`base_link`(z=0)라 복셀 격자(`origin_z: 0.0`) 경계에 걸린다.
**`obstacle_layer`** 로 바꾸니 경고가 **0건**이 됐다.

```yaml
plugins: ["obstacle_layer", "inflation_layer"]
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: scan
  scan: {topic: /scan, data_type: "LaserScan", clearing: True, marking: True}
```

### 🔴 ERR-068 — 프레임 이름이 안 맞으면 **아무 일도 안 일어난다**

ERR-066·067 을 고쳐 Nav2 가 정상 기동해도 로봇이 움직이지 않는다.

```
[ERROR] [amcl]: Couldn't transform from base_link to base_footprint
[WARN]  [amcl]: AMCL cannot publish a pose or update the transform.
```

| | |
|---|---|
| Nav2 기본 `base_frame_id` | **`base_footprint`** |
| Isaac Sim(8·9장 구성) | **`base_link`** |

AMCL 이 `map → odom` 을 못 내므로 **TF 체인이 끊긴다.** 초기 위치를
아무리 찍어도 로봇은 지도 어디에 있는지 모른다.

```bash
sed -i 's/base_footprint/base_link/g' nav2_params.yaml    # amcl·local·global 3곳
```

**원고에 프레임 이름 얘기가 한 줄도 없다.**

### 🟠 뷰포트 카메라 — 로봇이 화면 밖으로 나간다

Isaac Sim 기본 카메라는 **고정**이라 자율주행이 시작되면 몇 초 만에
화면을 벗어난다. 녹화해도 빈 방만 남는다.

```python
from isaacsim.core.utils.viewports import set_camera_view
set_camera_view(eye=np.array([0.0, -13.0, 14.0]),
                target=np.array([0.0, 0.0, 0.0]))
```

## Nav2 b07~b10 — 오류 2건, 실행으로 확정

### 🔴 ERR-067 — 플러그인 이름 하나에 Nav2 가 통째로 죽는다

b07 안에서 **표기가 섞여 있다.**

```yaml
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"                                        # 슬래시 ❌
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"  # 콜론 ✅
```

원고 그대로 돌린 결과다.

```
[INFO]  [controller_server]: Created controller : FollowPath of type
        nav2_regulated_pure_pursuit_controller::...          ← 콜론 쪽은 성공

[FATAL] [planner_server]: Failed to create global planner.
        the class nav2_navfn_planner/NavfnPlanner ... does not exist.
        Declared types are  nav2_navfn_planner::NavfnPlanner ...
[ERROR] Failed to bring up all requested nodes. Aborting bringup.
```

`planner_server` 하나가 죽으면 **라이프사이클 매니저가 전체를 중단**시킨다.
`::` 로 고치면 `Created global planner plugin GridBased` 가 나온다.

> 오류 메시지가 `Declared types are` 로 실제 이름을 알려 준다.
> **이 부류는 로그만 읽으면 바로 고칠 수 있다.**

### 🔴 ERR-066 — Nav2 런치 파일이 틀렸다. 그런데 **오류가 안 난다**

```bash
ros2 launch nav2_bringup navigation_launch.py map:=./warehouse_map.yaml   # 원고
```

`navigation_launch.py` 에 **`map` 인자가 없다.** 그런데 `ros2 launch` 는
모르는 인자를 **조용히 무시한다.** 실행해서 노드를 세어 확인했다.

| | 원고 `navigation_launch.py` | 수정 `bringup_launch.py` |
|---|---|---|
| `map_server` | ❌ | **✅** |
| `amcl` | ❌ | **✅** |
| `/map` 발행자 | **0** | **1** (240×184, 0.05 m) |
| `/amcl_pose` | ❌ | **✅** |

b10 의 체크리스트가 이걸 드러낸다 — `ros2 topic echo /amcl_pose` 는
**영원히 안 나온다.** amcl 이 안 떴기 때문이다.

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=./warehouse_map.yaml use_sim_time:=true        # 수정
```

| 런치 파일 | `map` | `use_namespace` | 용도 |
|---|---|---|---|
| `navigation_launch.py` | ❌ | ❌ | 내비게이션 스택만 (SLAM 동시 운용 시) |
| **`bringup_launch.py`** | ✅ | ✅ | **지도 + 위치추정 + 내비게이션** |
| `localization_launch.py` | ✅ | ❌ | 지도 + amcl 만 |

### ⚠️ b07 은 **조각**이다

플러그인 이름을 고쳐도 아직 안 올라온다.

```
[ERROR] [collision_monitor]: Error while getting parameters: parameter 'observation_sources'
[ERROR] Failed to bring up all requested nodes. Aborting bringup.
```

`params_file:=` 는 기본 파일을 **덮어쓴다(병합이 아니다).** b07 에는
`planner_server`·`controller_server`·`local_costmap` 만 있어 나머지 노드
설정이 통째로 사라진다. 원고는 이 사실을 적지 않는다.

```bash
cp $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml ./my_params.yaml
# my_params.yaml 에서 b07 부분만 수정해 쓴다
```

또 b07 의 costmap 에는 **`plugins` 목록이 없다.** 레이어를 선언하지 않으면
`inflation_layer` 값을 적어도 **팽창이 적용되지 않는다.** 오류는 안 난다.

```yaml
plugins: ["obstacle_layer", "inflation_layer"]    # 원고에 없는 줄
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
```

> **여기까지가 Nav2 단독 검증이다.** 실제 자율주행(목표 클릭 → 경로 → 이동)은
> Isaac Sim 과 함께 띄워야 한다. TF(`odom`·`base_link`)와 `/scan` 이 있어야
> costmap 이 활성화된다.

## 📗 확장 — 창고 환경과 4족보행 (책 밖)

### ① 환경을 실제 산업현장으로

![창고 스테레오](../_docs/images/book/ch09/ch0907_img06_warehouse_stereo.png)

벽돌 줄눈·철제 기둥·볼트·바닥 마모 자국 — **어느 방향을 봐도 특징점이 있다.**

```python
add_reference_to_stage(
    get_assets_root_path() + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "/World/Env")
```

| | 직접 만든 빈 방 | **공식 창고** |
|---|---|---|
| 오차 | 5 cm → **폭주 900 m** | **1~11 cm 유지** |
| z 높이 | **-11.78 m** (지하) | **+0.07 m** (지상) |
| 추적 상실 | 다수 | **0건** |

**비주얼 SLAM 은 무늬를 먹고 산다.** 2D 라이다(9.1~9.6)는 거리만 재므로
민무늬라도 상관없었다. 여기서 갈린다.

```bash
SCENE=warehouse python cuvslam_scene.py    # forklift · shelves · twin · room
```

### ② 4족보행 로봇 — [`quadruped_scene.py`](quadruped_scene.py)

![Spot 창고 보행](../_docs/images/book/ch09/ch0907_img07_spot_standing.png)

```bash
ROBOT=spot   SCENE=warehouse python quadruped_scene.py   # anymal · go2
```

**4족이 바퀴보다 정확했다.**

| 경과 | cuVSLAM | 정답 | 오차 |
|---|---|---|---|
| 10s | 2.17 m | 2.14 m | **3 cm** |
| 40s | 7.18 m | 7.16 m | **2 cm** |

```
NovaCarter (바퀴)  오차 1~11 cm
Spot (4족)        오차 2~3 cm
```

#### 함정 6개 — 전부 겪었다

| | 내용 |
|---|---|
| 1 | `SpotFlatTerrainPolicy` 는 **`RuntimeError` 던지는 껍데기** (7.0.0 제거) |
| 2 | **`spawn()`** 을 불러야 프림이 생긴다 — 몸통·카메라 작업은 그 **뒤에** |
| 3 | **물리 콜백**에서 `runner.step()` — 렌더 루프면 **1/10 속도로 돌아 넘어진다** |
| 4 | 4족 자산에는 **카메라가 0대** — 직접 달아야 한다 |
| 5 | 오일러각으로 돌리면 **영상이 90° 눕는다** — 행렬로 지정할 것 |
| 6 | `CAM_FWD=0.28` 은 **몸통 안** (길이 0.87 m) — 0.46 으로 |

```python
# 함정 3 — decimation 10 = 물리 10스텝당 렌더 1회
world.add_physics_callback("policy", on_physics)    # 물리 500 Hz

# 함정 5 — 기저 벡터로 직접
m = Gf.Matrix4d(0, -1, 0, 0,    # +X → -Y
                0,  0, 1, 0,    # +Y → +Z   위가 위
               -1,  0, 0, 0,    # +Z → -X   시선 -Z 가 +X
                CAM_FWD, y, CAM_UP, 1)
cam.MakeMatrixXform().Set(m)
```

보행은 물리 500 Hz 가 필요해 렌더가 17~25 Hz 로 떨어진다. cuVSLAM 기본값
`image_jitter_threshold_ms=34`(≈30 FPS)를 넘으므로 올려야 한다.

```bash
ros2 param set /visual_slam_node image_jitter_threshold_ms 150.0
```

#### 4족만 되는 것 — 옆걸음

`step()` 의 command 가 `[vx, vy, wz]` 라 **`vy`** 가 살아 있다.

```
명령 vy=0.30  →  50초에 16.4 m = 0.33 m/s  (명령과 일치)
z = 0.54~0.57 m 유지 — 한 번도 넘어지지 않음
```

### 재현 방법

```bash
# ① Isaac Sim (호스트) — 스테레오 + camera_info + TF + /clock
source ../ch08_ros2/setup_ros2_env.sh
python cuvslam_scene.py

# ② 컨테이너 진입 (ERR-060: run_dev.sh 는 5.0 에 없다)
isaac-ros activate

# ③ 컨테이너 안 — TF 를 직접 발행해야 한다 (ERR-062)
source /opt/ros/lyrical/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.1003 --y 0.075 --z 0.3459 \
  --roll -1.5708 --pitch 0 --yaw -1.5708 \
  --frame-id base_link --child-frame-id front_stereo_camera_left &
ros2 run tf2_ros static_transform_publisher \
  --x 0.1003 --y -0.075 --z 0.3459 \
  --roll -1.5708 --pitch 0 --yaw -1.5708 \
  --frame-id base_link --child-frame-id front_stereo_camera_right &

# ④ cuVSLAM (ERR-060: 패키지 이름이 isaac_ros_cuvslam 이다)
ros2 launch isaac_ros_cuvslam isaac_ros_visual_slam_isaac_sim.launch.py
```

## 3D SLAM 은 9.7 절에 있다

9.1~9.6 은 전부 **2D SLAM** 이다. `slam_toolbox` 가 `/scan`(LaserScan, 2D) 만
받기 때문이고, 그래서 ERR-058 에서 라이다를 3D(OS1) → 2D(RPLIDAR_S2E) 로 바꿨다.

**진짜 3D SLAM 은 9.7 절**이며 완전히 다른 스택이다.

| | 9.1~9.6 | 9.7 |
|---|---|---|
| 도구 | `slam_toolbox` | **cuVSLAM + nvblox** |
| 센서 | 2D LiDAR | 스테레오 카메라 + IMU |
| 연산 | CPU | **GPU** |
| 출력 | 2D 점유격자 | **3D ESDF 볼륨** |
| 높이 | 라이다 평면 한 장 | 바닥~천장 전체 |
| 설치 | `apt install ros-jazzy-slam-toolbox` | **Isaac ROS (Docker, 대용량)** |
| 검증 | ✅ 완료 | ✅ cuVSLAM 추적 성공 / ⏳ nvblox 메시 미실행 |

9.7 이 요구하는 것:

```bash
# 원고 9.7.5
cd ${ISAAC_ROS_WS}
./src/isaac_ros_common/scripts/run_dev.sh          # Docker 개발 컨테이너
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py

# 원고 9.7.6
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
```

`run_dev.sh` 는 **Docker 컨테이너를 내려받아 실행**한다. 본문의 다른 장과 달리
pip/apt 로 끝나지 않으므로, 설치 승인을 받고 별도로 진행해야 한다.

> **검증 시 확인할 것** — 원고의 런치 파일 이름 `isaac_ros_visual_slam_isaac_sim.launch.py`
> 는 Isaac ROS 버전에 따라 `isaac_ros_examples` 패키지 경유 방식으로 바뀌었을 수
> 있다. 8·9장에서 노드 이름이 계속 바뀌어 있었던 것과 같은 종류의 위험이다.
> **설치 전에는 단정하지 않는다.**

---

## 장애물 재계획 — 두 가지를 잡기까지

처음엔 "재계획 실패"로 남겼다. 다시 붙어서 원인을 둘 다 찾았다.

| | 고치기 전 | 고친 뒤 |
|---|---|---|
| 라이다 최초 감지 | 1.94 m | **3.93 m** |
| 코스트맵 최초 감지 | **0.13 m** (이미 충돌한 뒤) | **3.68 m** |
| 상자 최근접 거리 | 0.03 m (접촉) | **0.48 m** (통과) |
| 결과 | 박고 멈춤 | **목표 도달** |

<a name="err-069"></a>
### ERR-069 🔴 라이다가 로봇 몸통 **속**에 있었다

`slam_scene.py` 는 라이다를 `z=0.30` 에 달았다. Nova Carter 의 몸통을
실제로 재 보면 **z −0.140 ~ 0.554 m** — 한가운데다.

스캔을 30° 구간으로 쪼개 세어 보면 바로 드러난다.

```
   -30~  +0°     0/ 267        ← 정면
    +0~ +30°     0/ 266        ← 정면
   +60~ +90°   251/ 267  ##################
```

3200 빔 중 유효 반사 **451 개(14%)**, **정면은 0 개.**
로봇이 앞을 못 보니 costmap 에 찍힐 것이 없고, 그대로 박는다.

함정은 **옆구리로 벽이 보여 SLAM 은 멀쩡히 된다**는 점이다.
그래서 라이다를 의심하지 않게 된다.

수정은 눈대중 대신 **바운딩박스를 재서** 올린다.

```python
_bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
_rng = _bbox.ComputeWorldBound(stage.GetPrimAtPath(CARTER)).ComputeAlignedRange()
_chassis_z = UsdGeom.Xformable(stage.GetPrimAtPath(CHASSIS)) \
    .ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2]
LIDAR_Z = _rng.GetMax()[2] - _chassis_z + 0.05      # 몸통 꼭대기 +5 cm
```

고친 뒤: 유효 반사 **1564/3200 (49%)**, 정면 `253/267`·`266/266`.

<a name="err-070"></a>
### ERR-070 🔴 `global_costmap` 만 1 Hz · 2.5 m 로 뒤처져 있다

라이다를 고쳐도 여전히 스치고 나서 피한다. 두 코스트맵이 따로 논다.

| | local (컨트롤러용) | global (**플래너용**) |
|---|---|---|
| `update_frequency` | 10.0 Hz | **1.0 Hz** |
| `obstacle_max_range` | 5.0 m | **2.5 m** |

경로를 **돌려 그리는** 쪽은 `global_costmap` 이다. 1 초에 한 번,
2.5 m 안쪽만 보면 0.5 m/s 에서 갱신 한 번당 **0.5 m 를 눈 감고** 간다.
`nav2_bringup` 기본값이 그대로 이 값이라, 정적 지도 주행은 되지만
**움직이는 장애물 실습에서는 반드시 올려야 한다.**

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0        # 1.0 → 5.0
      publish_frequency: 2.0       # 1.0 → 2.0
      obstacle_layer:
        scan:
          raytrace_max_range: 6.0  # 3.0 → 6.0
          obstacle_max_range: 5.0  # 2.5 → 5.0
```

### 직접 확인하는 방법

```bash
# ① 스캔이 정면을 보는지 — 각도별 반사 분포를 센다
ros2 topic echo /scan --once > /tmp/s.yaml   # 또는 아래 계측 스크립트
```

정면(±30°) 구간이 비어 있으면 **높이부터 의심한다.** 발행률(Hz)은
정상으로 나오므로 `ros2 topic hz` 로는 절대 안 잡힌다.

```bash
# ② 장애물을 놓고 코스트맵이 언제 찍는지 본다
ros2 topic pub --once /obstacle_pose geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 4.5}, orientation: {w: 1.0}}}"
```

### 함께 알아 둘 것

- **시뮬에서 장애물을 치워도 코스트맵에는 남는다.** 중앙 2.6 m 안에
  치명 셀 **191 개**가 유령으로 남아, 빈 바닥에서 `Failed to make progress`
  가 반복됐다. 아래 서비스로 비우면 즉시 풀린다.

  ```bash
  ros2 service call /global_costmap/clear_entirely_global_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}"
  ```

- **`/cmd_vel` 로 직접 몰면 AMCL 이 위치추정을 놓칠 수 있다.**
  `Extrapolation Error` 가 반복되면 `/initialpose` 를 다시 쏜다.
- **RPP 는 후진하지 않는다**(`allow_reversing: False` 기본). 장애물에
  코를 박으면 뒤로 뺄 수단이 없어 "치웠는데도 안 움직이는" 상태가 된다.

영상: [`ch09_nav2_avoid_ok.mp4`](../_docs/images/book/ch09/ch09_nav2_avoid_ok.mp4)
— 3.68 m 앞 감지 → 좌측 우회 → 0.48 m 간격 통과 → 목표 도달.

---

## 캡처 전체 목록

### SLAM 지도 생성

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch09/ch09_rviz_slam.png`](../_docs/images/book/ch09/ch09_rviz_slam.png) | SLAM 지도 생성 중 RViz |
| [`../_docs/images/book/ch09/ch09_map_compare.png`](../_docs/images/book/ch09/ch09_map_compare.png) | 지도 비교 |
| [`../_docs/images/book/ch09/slam_check_big.png`](../_docs/images/book/ch09/slam_check_big.png) | 최종 SLAM 지도 |
| [`../_docs/images/book/ch09/slam_check.pgm`](../_docs/images/book/ch09/slam_check.pgm) | 지도 데이터 (PGM) |
| [`../_docs/images/book/ch09/slam_check.yaml`](../_docs/images/book/ch09/slam_check.yaml) | 지도 메타데이터 |

### Nav2 자율주행

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch09/ch09_nav2_rviz.png`](../_docs/images/book/ch09/ch09_nav2_rviz.png) | Nav2 RViz 경로 계획 |
| [`../_docs/images/book/ch09/ch09_nav2_seq.png`](../_docs/images/book/ch09/ch09_nav2_seq.png) | Nav2 주행 시퀀스 |
| [`../_docs/images/book/ch09/ch09_nav2_sequence.png`](../_docs/images/book/ch09/ch09_nav2_sequence.png) | Nav2 전체 시퀀스 |
| [`../_docs/images/book/ch09/ch09_nav2_mid.png`](../_docs/images/book/ch09/ch09_nav2_mid.png) | Nav2 주행 중간 |
| [`../_docs/images/book/ch09/ch09_nav2_isaacsim.mp4`](../_docs/images/book/ch09/ch09_nav2_isaacsim.mp4) | Isaac Sim Nav2 영상 |
| [`../_docs/images/book/ch09/ch09_nav2_drive.mp4`](../_docs/images/book/ch09/ch09_nav2_drive.mp4) | Nav2 주행 영상 |

### 장애물 회피·재계획

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch09/ch09_nav2_obstacle.mp4`](../_docs/images/book/ch09/ch09_nav2_obstacle.mp4) | 장애물 감지 영상 |
| [`../_docs/images/book/ch09/ch09_nav2_replan.mp4`](../_docs/images/book/ch09/ch09_nav2_replan.mp4) | 재계획 영상 |
| [`../_docs/images/book/ch09/ch09_nav2_replan2.mp4`](../_docs/images/book/ch09/ch09_nav2_replan2.mp4) | 재계획 2차 |
| [`../_docs/images/book/ch09/ch09_nav2_replan_fixed.mp4`](../_docs/images/book/ch09/ch09_nav2_replan_fixed.mp4) | 재계획 수정 후 |
| [`../_docs/images/book/ch09/ch09_nav2_avoid_ok.mp4`](../_docs/images/book/ch09/ch09_nav2_avoid_ok.mp4) | 장애물 우회 성공 |

### Nav2 프레임별 분석

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch09/nav2_f5.png`](../_docs/images/book/ch09/nav2_f5.png) | 프레임 5 |
| [`../_docs/images/book/ch09/nav2_f25.png`](../_docs/images/book/ch09/nav2_f25.png) | 프레임 25 |
| [`../_docs/images/book/ch09/nav2_f45.png`](../_docs/images/book/ch09/nav2_f45.png) | 프레임 45 |
| [`../_docs/images/book/ch09/nf4.png`](../_docs/images/book/ch09/nf4.png) | nf 프레임 4 |
| [`../_docs/images/book/ch09/nf22.png`](../_docs/images/book/ch09/nf22.png) | nf 프레임 22 |
| [`../_docs/images/book/ch09/nf40.png`](../_docs/images/book/ch09/nf40.png) | nf 프레임 40 |
| [`../_docs/images/book/ch09/nf60.png`](../_docs/images/book/ch09/nf60.png) | nf 프레임 60 |
| [`../_docs/images/book/ch09/nf88.png`](../_docs/images/book/ch09/nf88.png) | nf 프레임 88 |

### cuVSLAM (9.7절)

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch09/ch0907_spot_warehouse.png`](../_docs/images/book/ch09/ch0907_spot_warehouse.png) | Spot + 창고 씬 |
| [`../_docs/images/book/ch09/ch0907_spot_now.png`](../_docs/images/book/ch09/ch0907_spot_now.png) | Spot 현재 상태 |
| [`../_docs/images/book/ch09/ch0907_spot_stereo.png`](../_docs/images/book/ch09/ch0907_spot_stereo.png) | Spot 스테레오 카메라 |
| [`../_docs/images/book/ch09/ch0907_warehouse_gui.png`](../_docs/images/book/ch09/ch0907_warehouse_gui.png) | 창고 GUI |
| [`../_docs/images/book/ch09/ch0907_warehouse_stereo.png`](../_docs/images/book/ch09/ch0907_warehouse_stereo.png) | 창고 스테레오 뷰 |
| [`../_docs/images/book/ch09/ch0907_isaac_gui.png`](../_docs/images/book/ch09/ch0907_isaac_gui.png) | Isaac Sim GUI |
| [`../_docs/images/book/ch09/ch0907_stereo_pair.png`](../_docs/images/book/ch09/ch0907_stereo_pair.png) | 스테레오 쌍 |
| [`../_docs/images/book/ch09/ch0907_stereo_pair_fisheye.png`](../_docs/images/book/ch09/ch0907_stereo_pair_fisheye.png) | 피쉬아이 렌즈 |
| [`../_docs/images/book/ch09/ch0907_stereo_disparity.png`](../_docs/images/book/ch09/ch0907_stereo_disparity.png) | 디스패리티 맵 |
| [`../_docs/images/book/ch09/ch0907_lens_before_after.png`](../_docs/images/book/ch09/ch0907_lens_before_after.png) | 렌즈 보정 전후 |

### 기타

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch09/ch09_isaac_now.png`](../_docs/images/book/ch09/ch09_isaac_now.png) | Isaac Sim 현재 상태 |
| [`../_docs/images/book/ch09/ch09_rviz_now.png`](../_docs/images/book/ch09/ch09_rviz_now.png) | RViz 현재 상태 |
| [`../_docs/images/book/ch09/ch09_cam_check.png`](../_docs/images/book/ch09/ch09_cam_check.png) | 카메라 확인 |
| [`../_docs/images/book/ch09/ch09_stuck_now.png`](../_docs/images/book/ch09/ch09_stuck_now.png) | 경로 막힘 상태 |
