#!/usr/bin/env python3
"""Take a picture or record a clip with the Logitech USB camera.

This camera is not part of the robot, so this demo talks to it directly and
needs neither the bridge nor a robot on the network.

    python scripts/demo/logitech_camera.py photo frame.jpg
    python scripts/demo/logitech_camera.py video toss.avi --seconds 5 --fps 60
    python scripts/demo/logitech_camera.py info
"""
import argparse
import json
import time

from sawyer_control import Camera, find_device


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("action", choices=("photo", "video", "info"))
    parser.add_argument("output", nargs="?", help="file to write")
    parser.add_argument("--device", default=None, help="override the camera path")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0,
                        help="1280x720 is the only mode a Brio offers above 30")
    parser.add_argument("--seconds", type=float, default=5.0, help="recording length")
    parser.add_argument("--exposure", type=float, default=None,
                        help="fixed exposure; leave unset to let the camera choose")
    parser.add_argument("--gain", type=float, default=None)
    parser.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270))
    arguments = parser.parse_args()

    if arguments.action == "info":
        print("device:", find_device() if arguments.device is None else arguments.device)
        with Camera.open(arguments.device, width=arguments.width,
                         height=arguments.height, fps=arguments.fps) as camera:
            time.sleep(1.5)
            status = camera.status()
            status["measured_fps"] = round(camera.measured_fps(), 1)
            print(json.dumps(status, indent=2, sort_keys=True))
        return 0

    if arguments.output is None:
        parser.error(f"{arguments.action} needs an output file")

    settings = dict(width=arguments.width, height=arguments.height, fps=arguments.fps,
                    exposure=arguments.exposure, gain=arguments.gain, rotate=arguments.rotate)

    with Camera.open(arguments.device, **settings) as camera:
        if arguments.action == "photo":
            path = camera.save(arguments.output, settle=0.8)
            print(path.resolve())
            return 0

        print(f"recording {arguments.seconds:g}s to {arguments.output} …")
        with camera.record(arguments.output) as recording:
            time.sleep(arguments.seconds)
        clip = recording.clip
        print(f"{clip.frame_count} frames at {clip.effective_fps:.1f} fps "
              f"over {clip.duration_s:.2f}s, {clip.dropped_frames} dropped")
        print(clip.path.resolve())
        print(clip.manifest_path.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
