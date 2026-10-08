"""책 13장 [실습] OPC-UA → Isaac Sim 실시간 브리지 (b01) — 정리본.

원문과의 차이:
  없음 — 클래스 구조는 원문 그대로다.

검증 범위 (외부 의존):
  이 코드는 실제 OPC-UA 서버(PLC)가 있어야 end-to-end 로 돈다.
  `opcua` 라이브러리도 기본 환경엔 없다(원문 주석에 설치 안내 있음).
  → 서버 없이 돌릴 수 있는 부분이 없어, 클래스 구조/문법만 검증했다.
    ArticulationAction·apply_action 경로는 6.1 에서 유효함을 확인.

실제 사용:
  pip install opcua
  실 서버 주소/노드ID 를 넣고 start_sync() 호출.
"""
import time
import threading


class OPCUABridge:
    """OPC-UA 서버 데이터를 Isaac Sim USD Stage 에 실시간 반영."""

    def __init__(self, opcua_url: str, robot_prim_path: str):
        from opcua import Client  # 지연 import: 미설치 환경에서도 모듈 로드 가능
        self.client = Client(opcua_url)
        self.robot_prim_path = robot_prim_path
        self.running = False

    def connect(self):
        self.client.connect()
        print(f"✅ OPC-UA 서버 연결: {self.client.server_url}")

    def read_joint_angles(self, node_ids: list) -> list:
        values = []
        for node_id in node_ids:
            node = self.client.get_node(node_id)
            values.append(node.get_value())
        return values

    def sync_loop(self, robot_articulation, node_ids, interval_hz=30):
        self.running = True
        interval = 1.0 / interval_hz
        from isaacsim.core.utils.types import ArticulationAction
        import numpy as np
        while self.running:
            joint_values = self.read_joint_angles(node_ids)
            action = ArticulationAction(joint_positions=np.array(joint_values))
            robot_articulation.apply_action(action)
            time.sleep(interval)

    def start_sync(self, robot_articulation, node_ids, interval_hz=30):
        thread = threading.Thread(
            target=self.sync_loop,
            args=(robot_articulation, node_ids, interval_hz),
            daemon=True,
        )
        thread.start()
        print(f"✅ OPC-UA 실시간 동기화 시작 ({interval_hz}Hz)")

    def disconnect(self):
        self.running = False
        self.client.disconnect()


# 사용 예 (실 서버 필요):
#   bridge = OPCUABridge("opc.tcp://192.168.1.100:4840", "/World/Robots/RealRobotTwin")
#   bridge.connect()
#   node_ids = [f"ns=2;s=Robot1.Joint{i}.Position" for i in range(1, 7)]
#   bridge.start_sync(real_robot_twin, node_ids, interval_hz=50)
