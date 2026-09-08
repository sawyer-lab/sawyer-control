# Sawyer Control

Standalone private control platform for a physical Rethink Sawyer. The bridge
runs the ROS 1 Intera stack inside a container; applications use the versioned
gRPC protocol and do not need ROS installed locally.

## Start

```bash
./sawyer-control up
```

The first run asks only for the robot hostname and optional network hints, then
stores them in `~/.config/sawyer-control/runtime.env`. Each later run discovers
the robot again, opens the necessary host firewall route, and reuses a healthy
bridge with the same robot and host network identity. When the discovered
network identity changes, Compose updates the bridge automatically.

## Python client

```python
from sawyer_control import CameraClient, ForceTorqueClient, SawyerRobotClient

with SawyerRobotClient.connect() as robot:
    robot.enable()
    robot.move_to([0.0, -0.78, 0.0, 1.55, 0.0, 0.78, 3.14])

with CameraClient.connect() as camera:
    hand_frame = camera.read_hand()

with ForceTorqueClient.connect() as force_torque:
    print(force_torque.read())
```

`SawyerRobotClient`, `CameraClient`, and `ForceTorqueClient` are separate
clients. They share one bridge address but only the robot client can command
motion.

For a raw time-indexed joint command batch, use `execute_sequence` with one of
the protocol control modes: position, velocity, torque, or trajectory.

## API and language support

[`proto/sawyer_control/v1/control.proto`](proto/sawyer_control/v1/control.proto)
is the canonical public contract. The included Python package is a thin,
friendly wrapper around generated gRPC code. The same protobuf definition can
generate C++, C#, Go, Java, Rust, and other gRPC clients without changing the
bridge.

## Local development

```bash
/home/fausto/miniconda3/envs/tossing/bin/python -m pip install -e '.[dev]'
/home/fausto/miniconda3/envs/tossing/bin/python -m pytest
```

No robot is contacted by the test suite. First hardware validation is manual:
verify state, cameras, and force/torque reads before deliberately issuing a
small supervised motion command.
