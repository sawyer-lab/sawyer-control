#!/usr/bin/env python3
import argparse

from sawyer_control import SawyerRobotClient


def _target(value):
    joints = [float(part) for part in value.split(",")]
    if len(joints) != 7:
        raise argparse.ArgumentTypeError("target must contain seven comma-separated joint values")
    return joints


def main():
    parser = argparse.ArgumentParser(description="Explicit Sawyer lifecycle or joint-motion command.")
    parser.add_argument("action", choices=("state", "enable", "disable", "reset", "move"))
    parser.add_argument("--target", type=_target)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--address", default="127.0.0.1:50051")
    args = parser.parse_args()
    if args.action == "move" and args.target is None:
        parser.error("move requires --target")
    with SawyerRobotClient.connect(args.address) as robot:
        if args.action == "enable":
            robot.enable()
        elif args.action == "disable":
            robot.disable()
        elif args.action == "reset":
            robot.reset()
        elif args.action == "move":
            robot.move_to(args.target, args.timeout_s)
        print(robot.get_state())


if __name__ == "__main__":
    main()
