#!/usr/bin/env python3
import argparse

from sawyer_control import SawyerRobotClient


def main():
    parser = argparse.ArgumentParser(description="Read Sawyer bridge and arm state.")
    parser.add_argument("--address", default="127.0.0.1:50051")
    args = parser.parse_args()
    with SawyerRobotClient.connect(args.address) as robot:
        state = robot.get_state()
        print("protocol:", robot.health())
        print("enabled:", state.enabled, "stopped:", state.stopped)
        print("joints:", list(state.positions.values))
        print("pose:", state.pose)


if __name__ == "__main__":
    main()
