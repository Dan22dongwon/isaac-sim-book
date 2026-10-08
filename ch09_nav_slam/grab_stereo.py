"""cuVSLAM 이 보는 좌/우 영상을 받아 책에 넣을 그림으로 만든다.

스테레오가 왜 거리를 아는지 보여 주려면 **시차(視差)** 가 보여야 한다.
좌·우를 나란히 놓고, 차이 영상(|L-R|)을 함께 낸다.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage, ImageDraw


def to_np(msg):
    a = np.frombuffer(msg.data, dtype=np.uint8)
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "mono8": 1}.get(msg.encoding, 3)
    a = a.reshape(msg.height, msg.step // ch, ch)[:, :msg.width, :]
    if msg.encoding == "bgr8":
        a = a[:, :, ::-1]
    return a[:, :, :3].copy()


class Grab(Node):
    def __init__(self):
        super().__init__("grab_stereo")
        self.imgs = {}
        for side in ("left", "right"):
            self.create_subscription(
                Image, f"/front_stereo_camera/{side}/image_rect_color",
                lambda m, s=side: self.imgs.setdefault(s, to_np(m)), 10)


rclpy.init()
n = Grab()
for _ in range(600):
    rclpy.spin_once(n, timeout_sec=0.05)
    if len(n.imgs) == 2:
        break
if len(n.imgs) < 2:
    print(f"❌ 수신 실패 — 받은 것: {list(n.imgs)}")
    sys.exit(1)

L, R = n.imgs["left"], n.imgs["right"]
h, w = L.shape[:2]
print(f"✅ 좌 {L.shape}  우 {R.shape}")

GAP, TOP = 12, 26
canvas = PILImage.new("RGB", (w * 2 + GAP, h + TOP), (245, 245, 245))
canvas.paste(PILImage.fromarray(L), (0, TOP))
canvas.paste(PILImage.fromarray(R), (w + GAP, TOP))
d = ImageDraw.Draw(canvas)
d.text((6, 7), "LEFT  /front_stereo_camera/left/image_rect_color", fill=(20, 20, 20))
d.text((w + GAP + 6, 7), "RIGHT  (baseline 15.00 cm)", fill=(20, 20, 20))
canvas.save("/home/dan/isaacsim/book_verify/shots/ch0907_stereo_pair.png")

diff = np.abs(L.astype(np.int16) - R.astype(np.int16)).astype(np.uint8)
amp = np.clip(diff.astype(np.int16) * 3, 0, 255).astype(np.uint8)
dc = PILImage.new("RGB", (w, h + TOP), (245, 245, 245))
dc.paste(PILImage.fromarray(amp), (0, TOP))
ImageDraw.Draw(dc).text(
    (6, 7), f"|LEFT - RIGHT| x3   disparity mean={diff.mean():.1f}", fill=(20, 20, 20))
dc.save("/home/dan/isaacsim/book_verify/shots/ch0907_stereo_disparity.png")

print(f"  시차 평균 {diff.mean():.2f}  최대 {diff.max()}")
print("  저장: ch0907_stereo_pair.png · ch0907_stereo_disparity.png")
n.destroy_node(); rclpy.shutdown()
