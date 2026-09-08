#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import grpc

from sawyer_control import CameraClient


def main():
    parser = argparse.ArgumentParser(description="Capture one head or hand camera frame.")
    parser.add_argument("camera", choices=("head", "hand", "brio"))
    parser.add_argument("--output", type=Path, default=Path("frame.jpg"))
    parser.add_argument("--address", default="127.0.0.1:50051")
    parser.add_argument("--stop-after", action="store_true")
    args = parser.parse_args()
    with CameraClient.connect(args.address) as camera:
        start = getattr(camera, "start_" + args.camera)
        read = getattr(camera, "read_" + args.camera)
        start()
        deadline = time.monotonic() + 5.0
        while True:
            try:
                frame = read()
                break
            except grpc.RpcError as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError("Camera did not provide a frame within five seconds") from error
                time.sleep(0.1)
        args.output.write_bytes(frame.data)
        print(args.output.resolve())
        if args.stop_after:
            getattr(camera, "stop_" + args.camera)()


if __name__ == "__main__":
    main()
