"""책 13장 [실습] MQTT 센서 구독 → 스레드 안전 큐 (b02) — 정리본.

원문과의 차이:
  없음 — 구독/큐 패턴은 원문 그대로다.

검증 범위 (외부 의존):
  실제 MQTT 브로커("mqtt.factory.local:1883")와 발행자가 있어야 돈다.
  `paho-mqtt` 도 기본 환경엔 없다(원문 주석에 설치 안내 있음).
  → 브로커 없이 돌릴 수 없어, 콜백/큐 구조와 JSON 파싱 분기만 검증했다.
    메인 루프에서 큐를 소비하는 패턴은 mqtt_monitor.py 에서 합성 데이터로 실증.

실제 사용:
  pip install paho-mqtt
  브로커 주소를 넣고 loop_start() 후 메인 루프에서 sensor_data_queue 소비.
"""
import json
import queue

sensor_data_queue = queue.Queue()


def on_connect(client, userdata, flags, rc):
    print(f"✅ MQTT 브로커 연결 (코드: {rc})")
    client.subscribe("factory/+/vibration")
    client.subscribe("factory/+/temperature")


def on_message(client, userdata, msg):
    """MQTT 메시지 수신 시 큐에 저장 (비블로킹)."""
    try:
        payload = json.loads(msg.payload.decode())
        sensor_data_queue.put({
            "topic": msg.topic,
            "data": payload,
            "timestamp": payload.get("timestamp"),
        })
    except json.JSONDecodeError:
        print(f"⚠️ 잘못된 페이로드: {msg.topic}")


def make_client(broker="mqtt.factory.local", port=1883):
    import paho.mqtt.client as mqtt  # 지연 import
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, keepalive=60)
    client.loop_start()
    return client


# 메인 루프에서의 큐 소비 패턴 (world.step 과 함께):
#   while not sensor_data_queue.empty():
#       item = sensor_data_queue.get()
#       equipment_id = item["topic"].split("/")[1]
#       ...  # update_equipment_indicator(equipment_id, ...)
