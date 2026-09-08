#!/usr/bin/env python3
import argparse

from sawyer_control import SawyerRobotClient


def main():
    parser = argparse.ArgumentParser(description="Explicit ClickSmart gripper command.")
    parser.add_argument("action", choices=("state", "open", "close"))
    parser.add_argument("--address", default="127.0.0.1:50051")
    args = parser.parse_args()
    with SawyerRobotClient.connect(args.address) as robot:
        if args.action == "open":
            robot.open_gripper()
        elif args.action == "close":
            robot.close_gripper()
        print(robot.get_gripper_state())


if __name__ == "__main__":
    main()
