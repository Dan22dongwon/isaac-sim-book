"""22.x 2단계 — 수집한 LiDAR CSV 로 주행 분류기를 학습한다.

Isaac Sim 이 필요 없다. CSV 만 있으면 어디서든 돌아간다.

    python lidar_classifier_train.py            # OUT_DIR 의 lidar_maze.csv 사용

산출물
    lidar_policy.joblib     학습된 분류기 (3단계에서 Isaac Sim 이 불러 쓴다)
    ml_03_confusion.png     혼동 행렬
    ml_04_features.png      어느 방향(각도)이 판단에 중요한가
"""
import os
import struct
import zlib
from collections import Counter

import numpy as np
from joblib import dump
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from sklearn.model_selection import train_test_split

OUT = os.environ.get("OUT_DIR", "./out")
CSV = os.path.join(OUT, "lidar_maze.csv")

# ── 1. 불러오기 ───────────────────────────────────────────────────
raw = np.genfromtxt(CSV, delimiter=",", dtype=str, skip_header=0)
header, rows = raw[0], raw[1:]
X = rows[:, :-1].astype(np.float32)
y = rows[:, -1]
N_BINS = X.shape[1]
print("=" * 58)
print(f"1) 불러오기   {X.shape[0]}행 × {N_BINS}칸")
print(f"   라벨 분포  {dict(Counter(y))}")

# ── 2. 클래스 균형 맞추기 ─────────────────────────────────────────
# 불균형하면 분류기가 그냥 'forward' 만 외쳐도 정확도가 높게 나온다.
rng = np.random.default_rng(0)
counts = Counter(y)
k = min(counts.values())
keep = np.concatenate([rng.choice(np.where(y == c)[0], k, replace=False) for c in counts])
rng.shuffle(keep)
Xb, yb = X[keep], y[keep]
print(f"2) 균형 맞춤  클래스당 {k}행 → 총 {len(yb)}행")

# ── 3. 분할 ───────────────────────────────────────────────────────
Xtr, Xte, ytr, yte = train_test_split(Xb, yb, test_size=0.25, random_state=0, stratify=yb)
print(f"3) 분할       학습 {len(ytr)} / 평가 {len(yte)}")

# ── 4. 학습 ───────────────────────────────────────────────────────
clf = RandomForestClassifier(n_estimators=200, max_depth=12, random_state=0, n_jobs=-1)
clf.fit(Xtr, ytr)
pred = clf.predict(Xte)
acc = accuracy_score(yte, pred)
print(f"4) 학습       RandomForest(200그루, 깊이 12)")
print(f"   정확도     {acc * 100:.1f}%")
print()
print(classification_report(yte, pred, digits=3))

labels = sorted(set(yb))
cm = confusion_matrix(yte, pred, labels=labels)
print("   혼동 행렬 (행=실제, 열=예측)")
print("              " + "".join(f"{l:>9}" for l in labels))
for i, l in enumerate(labels):
    print(f"   {l:>9}  " + "".join(f"{v:>9}" for v in cm[i]))

# ── 5. 중요한 방향 ────────────────────────────────────────────────
imp = clf.feature_importances_
order = np.argsort(imp)[::-1][:6]
step = 360 // N_BINS
print("\n5) 판단에 가장 중요한 방향 (0도 = 로봇 정면)")
for i in order:
    print(f"   {i * step:>3}도  중요도 {imp[i]:.3f}")

dump({"model": clf, "n_bins": N_BINS, "labels": labels}, os.path.join(OUT, "lidar_policy.joblib"))
print(f"\n   저장: {OUT}/lidar_policy.joblib")


# ── 그림 ──────────────────────────────────────────────────────────
def save_png(path, arr):
    h, w = arr.shape[:2]
    a = np.ascontiguousarray(arr[:, :, :3].astype(np.uint8))
    raw_ = b"".join(b"\x00" + a[y_].tobytes() for y_ in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    open(path, "wb").write(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw_, 6)) + chunk(b"IEND", b""))


try:
    from PIL import Image, ImageDraw, ImageFont
    F = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font = ImageFont.truetype(F, 22)
    small = ImageFont.truetype(F, 18)

    # 혼동 행렬
    n = len(labels)
    cell, pad = 110, 130
    img = Image.new("RGB", (pad + n * cell + 40, pad + n * cell + 50), (255, 255, 255))
    d = ImageDraw.Draw(img)
    mx = cm.max()
    for i in range(n):
        for j in range(n):
            v = cm[i, j]
            t = v / mx if mx else 0
            col = (int(255 - 150 * t), int(255 - 90 * t), int(255 - 40 * t))
            x0, y0 = pad + j * cell, pad + i * cell
            d.rectangle([x0, y0, x0 + cell, y0 + cell], fill=col, outline=(170, 175, 185))
            d.text((x0 + cell / 2, y0 + cell / 2), str(v), font=font,
                   fill=(20, 20, 20), anchor="mm")
        d.text((pad - 12, pad + i * cell + cell / 2), labels[i], font=small,
               fill=(40, 40, 40), anchor="rm")
        d.text((pad + i * cell + cell / 2, pad - 12), labels[i], font=small,
               fill=(40, 40, 40), anchor="mb")
    d.text((pad, 24), f"혼동 행렬 — 정확도 {acc * 100:.1f}%", font=font, fill=(20, 20, 20))
    d.text((pad, 56), "행 = 실제, 열 = 예측", font=small, fill=(110, 110, 110))
    img.save(os.path.join(OUT, "ml_03_confusion.png"), optimize=True)

    # 방향별 중요도 (원형)
    S, R = 520, 190
    img2 = Image.new("RGB", (S, S + 60), (255, 255, 255))
    d2 = ImageDraw.Draw(img2)
    cx, cy = S / 2, S / 2 + 40
    d2.text((24, 18), "방향별 중요도 — 위쪽이 로봇 정면(0도)", font=font, fill=(20, 20, 20))
    d2.ellipse([cx - R, cy - R, cx + R, cy + R], outline=(205, 208, 215))
    top = imp.max()
    for i in range(N_BINS):
        ang = np.radians(i * step - 90)
        L = R * (0.12 + 0.88 * imp[i] / top)
        x1, y1 = cx + L * np.cos(ang), cy + L * np.sin(ang)
        hot = imp[i] > top * 0.5
        d2.line([cx, cy, x1, y1], fill=(200, 60, 50) if hot else (120, 150, 200), width=7)
    d2.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=(40, 40, 40))
    d2.text((cx, cy - R - 24), "정면", font=small, fill=(40, 40, 40), anchor="mm")
    img2.save(os.path.join(OUT, "ml_04_features.png"), optimize=True)
    print(f"   그림: ml_03_confusion.png · ml_04_features.png")
except Exception as e:
    print(f"   [그림 생략] {type(e).__name__}: {e}")
