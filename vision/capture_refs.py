#!/usr/bin/env python3
"""
capture_refs.py
---------------
Capture numbered reference photos for an object class.

Each press of S saves the current camera frame into the chosen directory
as the next available NN.jpg file (auto-resumes from the highest existing
index, so you can come back later and add more shots without renumbering).

After capturing, rebuild the shape+colour signature with:
    python3 vision/build_shape_refs.py

Controls:
  Q  quit
  S  save the next numbered reference into --ref-dir

Run:
    python3 vision/capture_refs.py
    python3 vision/capture_refs.py --camera 1
    python3 vision/capture_refs.py --ref-dir vision/refs/blue_block
"""

import argparse
import re
from pathlib import Path

import cv2

CAM_W, CAM_H = 640, 480


def next_capture_index(ref_dir: Path) -> int:
    if not ref_dir.exists():
        return 1
    highest = 0
    for p in ref_dir.glob("*.jpg"):
        m = re.fullmatch(r"(\d+)", p.stem)
        if m:
            highest = max(highest, int(m.group(1)))
    return highest + 1


def main(camera_index: int, ref_dir: Path) -> None:
    ref_dir.mkdir(parents=True, exist_ok=True)
    capture_idx = next_capture_index(ref_dir)
    print(f"[capture_refs] saving to {ref_dir}/, next index = {capture_idx:02d}")

    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"[error] Cannot open camera index {camera_index}.")
        return

    print("[capture_refs] Controls: Q=quit  S=save next reference")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        view = frame.copy()
        cv2.putText(
            view, f"S = save {capture_idx:02d}.jpg in {ref_dir.name}/",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
        )

        cv2.imshow("capture refs", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            path = ref_dir / f"{capture_idx:02d}.jpg"
            cv2.imwrite(str(path), frame)
            print(f"[capture_refs] saved {path}")
            capture_idx += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture numbered reference photos (EE6003)")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument(
        "--ref-dir", type=Path, default=Path("vision/refs/red_rock"),
        help="Directory to save numbered captures into.",
    )
    args = parser.parse_args()
    main(camera_index=args.camera, ref_dir=args.ref_dir)
