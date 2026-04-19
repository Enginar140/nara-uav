"""Standalone kamera + yuvarlak cisim onizleme aracilarini cpp_uav build'inden
bagimsiz olarak calistirir. Amac: laptop kamerasinin ve HoughCircles parametrelerinin
calistigini saniyeler icinde dogrulamak; sonrasinda ayni parametreleri
config/sitl.yaml -> perception.opencv_vision.detector altina yapistirabilirsin.

Kullanim:
    py tests\\opencv_camera_preview.py                # device 0 (laptop webcam)
    py tests\\opencv_camera_preview.py --source 1     # ikinci kamera
    py tests\\opencv_camera_preview.py --source rtsp://...
    py tests\\opencv_camera_preview.py --hsv-red      # kirmizi top filtresi acik

Bagimlilik: pip install opencv-python numpy
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import cv2
    import numpy as np
except ImportError as e:
    print("HATA: opencv-python kurulu degil. Kurulum:")
    print("  py -m pip install opencv-python numpy")
    sys.exit(1)


def parse_source(s: str):
    if s.isdigit():
        return int(s)
    return s


def make_mask(hsv: "np.ndarray", args):
    if args.hsv_red:
        lo1 = np.array([0, 100, 80]);   hi1 = np.array([10, 255, 255])
        lo2 = np.array([170, 100, 80]); hi2 = np.array([179, 255, 255])
        m = cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1), cv2.inRange(hsv, lo2, hi2))
    elif args.hsv_green:
        m = cv2.inRange(hsv, np.array([35, 80, 60]), np.array([85, 255, 255]))
    elif args.hsv_blue:
        m = cv2.inRange(hsv, np.array([100, 100, 60]), np.array([130, 255, 255]))
    else:
        return None
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0",
                    help="kamera kaynagi: '0' (USB index) veya RTSP/HTTP/dosya yolu")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--min-radius", type=int, default=15)
    ap.add_argument("--max-radius", type=int, default=200)
    ap.add_argument("--dp", type=float, default=1.2)
    ap.add_argument("--param1", type=float, default=100.0)
    ap.add_argument("--param2", type=float, default=30.0)
    ap.add_argument("--blur", type=int, default=9)
    ap.add_argument("--hsv-red", action="store_true")
    ap.add_argument("--hsv-green", action="store_true")
    ap.add_argument("--hsv-blue", action="store_true")
    args = ap.parse_args()

    src = parse_source(args.source)
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"HATA: Kamera acilamadi: {src}")
        sys.exit(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print(f"Kamera acildi: {src} ({args.width}x{args.height})")
    print("Pencerede 'q' tusuna bas: cikis")

    last_log = 0.0
    frames = 0
    hits = 0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Frame okunamadi, 0.2 sn bekleyip yeniden deniyorum...")
            time.sleep(0.2)
            continue
        frames += 1

        if args.hsv_red or args.hsv_green or args.hsv_blue:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = make_mask(hsv, args)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if args.blur >= 3 and args.blur % 2 == 1:
            gray = cv2.GaussianBlur(gray, (args.blur, args.blur), 0)

        h, w = frame.shape[:2]
        min_dist = max(10, int(0.25 * min(h, w)))

        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, args.dp, min_dist,
            param1=args.param1, param2=args.param2,
            minRadius=args.min_radius, maxRadius=args.max_radius,
        )

        det = None
        if circles is not None:
            circles = np.uint16(np.around(circles[0, :]))
            best = max(circles, key=lambda c: c[2])
            det = (int(best[0]), int(best[1]), int(best[2]))
            hits += 1

        # crosshair
        cv2.line(frame, (w // 2, 0), (w // 2, h), (60, 60, 60), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (60, 60, 60), 1)

        if det:
            cx, cy, r = det
            cv2.circle(frame, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
            yaw_off_px   = cx - w // 2
            pitch_off_px = h // 2 - cy
            cv2.putText(frame,
                        f"r={r}px  off=({yaw_off_px:+d},{pitch_off_px:+d}) px",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, "no circle", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)

        cv2.imshow("uav round-detector preview (q to quit)", frame)
        if (args.hsv_red or args.hsv_green or args.hsv_blue) and gray is not None:
            cv2.imshow("hsv mask", gray)

        now = time.time()
        if now - last_log > 2.0:
            print(f"frames={frames}  hits={hits}  hit_rate={hits / max(1, frames) * 100:.1f}%")
            last_log = now

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
