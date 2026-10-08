"""책 17장 [실습] SDG 결과 검증 — bbox 시각화 (b08) — 동작 검증본.

원문과의 차이:
  [경로] BasicWriter 의 실제 출력 레이아웃에 맞춰 파일명 규칙을 보정.
         원문은 rgb/ 하위폴더와 .json bbox 를 가정하지만, 6.1 BasicWriter 는
         rgb_NNNN.png, bounding_box_2d_tight_NNNN.npy(+labels json) 를 평면 저장한다.
       → .npy(구조화 배열)에서 x_min/y_min/x_max/y_max 를 읽도록 수정.

검증 결과:
  sdg_pipeline.py 출력에 대해 첫 프레임의 bbox 를 초록 사각형으로 그려 저장.

실행:  python sdg_pipeline.py 5 && python sdg_visualize.py
"""
import os
import numpy as np

OUT = os.environ.get("SDG_OUT", "/tmp/ch17_sdg_output")


def main():
    rgb_files = sorted(f for f in os.listdir(OUT) if f.startswith("rgb_") and f.endswith(".png"))
    print(f"생성된 이미지 수: {len(rgb_files)}장")
    if not rgb_files:
        print("⚠️ 출력이 없다. 먼저 sdg_pipeline.py 를 실행하세요.")
        return

    idx = 0
    stem = rgb_files[idx].replace("rgb_", "").replace(".png", "")
    bbox_npy = os.path.join(OUT, f"bounding_box_2d_tight_{stem}.npy")
    boxes = np.load(bbox_npy)
    print(f"프레임 {stem}: bbox {len(boxes)}개")

    try:
        import cv2
        img = cv2.imread(os.path.join(OUT, rgb_files[idx]))
        for b in boxes:
            cv2.rectangle(img, (int(b["x_min"]), int(b["y_min"])),
                          (int(b["x_max"]), int(b["y_max"])), (0, 255, 0), 2)
        out_path = os.path.join(OUT, "verification_sample.png")
        cv2.imwrite(out_path, img)
        print(f"✅ 검증 이미지 저장: {out_path} — {len(boxes)}개 박스")
    except ImportError:
        # cv2 없으면 좌표만 출력(검증은 npy 로 충분)
        for b in boxes:
            print(f"   box: ({b['x_min']},{b['y_min']})~({b['x_max']},{b['y_max']})")
        print(f"✅ bbox 라벨 {len(boxes)}개 확인 (cv2 미설치로 그리기 생략)")


if __name__ == "__main__":
    main()
