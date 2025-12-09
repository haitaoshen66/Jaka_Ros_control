#!/usr/bin/env python3
"""
Quick camera debug: try opening a /dev/videoX device, set width/height/fps/FourCC,
read a few frames, and report whether frames are actually coming in.
Usage:
  python3 debug_camera_stream.py --device 0 --width 640 --height 480 --fps 30
"""

import argparse
import time
import cv2


def main():
    parser = argparse.ArgumentParser(description="Debug V4L2 camera stream")
    parser.add_argument("--device", type=int, default=0, help="Video device id (e.g. /dev/video0 -> 0)")
    parser.add_argument("--width", type=int, default=640, help="Frame width")
    parser.add_argument("--height", type=int, default=480, help="Frame height")
    parser.add_argument("--fps", type=int, default=30, help="Requested FPS")
    parser.add_argument("--frames", type=int, default=100, help="Number of frames to read for the test")
    parser.add_argument("--use-mjpg", action="store_true", help="Request MJPG fourcc to ease bandwidth")
    args = parser.parse_args()

    print(f"Opening /dev/video{args.device} with V4L2 backend...")
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("❌ Failed to open device")
        return

    if args.use_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    # Report what the driver accepted
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Requested: {args.width}x{args.height} @{args.fps}fps")
    print(f"Driver set: {int(actual_width)}x{int(actual_height)} @{actual_fps:.2f}fps")

    ok_frames = 0
    start = time.time()
    for i in range(args.frames):
        ret, frame = cap.read()
        if not ret or frame is None:
            print(f"[{i}] ❌ Failed to read frame")
            time.sleep(0.05)
            continue
        ok_frames += 1
        if i % 10 == 0:
            h, w = frame.shape[:2]
            print(f"[{i}] ✅ Frame shape: {w}x{h}, mean={frame.mean():.1f}")

    elapsed = time.time() - start
    cap.release()
    print(f"Read {ok_frames}/{args.frames} frames in {elapsed:.2f}s (~{ok_frames/elapsed:.2f} fps)")
    if ok_frames == 0:
        print("No frames received. Check device id, permissions (video group), or format.")


if __name__ == "__main__":
    main()
