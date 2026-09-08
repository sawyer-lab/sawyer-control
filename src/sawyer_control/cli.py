from __future__ import annotations

import argparse

from .client import SawyerRobotClient


def main() -> None:
    parser = argparse.ArgumentParser(prog="sawyer-control")
    parser.add_argument("command", choices=("health",))
    parser.add_argument("--address", default="127.0.0.1:50051")
    args = parser.parse_args()
    if args.command == "health":
        with SawyerRobotClient.connect(args.address) as robot:
            print(robot.health())
