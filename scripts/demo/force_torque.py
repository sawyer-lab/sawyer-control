#!/usr/bin/env python3
import argparse

from sawyer_control import ForceTorqueClient


def main():
    parser = argparse.ArgumentParser(description="Read or explicitly zero the ATI force/torque sensor.")
    parser.add_argument("action", choices=("read", "zero", "watch"))
    parser.add_argument("--address", default="127.0.0.1:50051")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    args = parser.parse_args()
    with ForceTorqueClient.connect(args.address) as sensor:
        if args.action == "zero":
            sensor.zero()
            return
        readings = [sensor.read()] if args.action == "read" else sensor.readings(args.rate_hz)
        for reading in readings:
            print(reading)


if __name__ == "__main__":
    main()
