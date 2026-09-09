# Using Sawyer Control

Start the bridge on the computer physically connected to the robot:

```bash
./sawyer-control up
```

The invoking user must be able to access `/var/run/docker.sock`. If Docker is
installed but reports a socket permission error, run
`sudo usermod -aG docker $USER` once, then begin a new login session.

The bridge listens on `127.0.0.1:50051`. Run host applications on that same
computer, or explicitly use the address where you expose the bridge.

## Python

For a new isolated environment:

```bash
./scripts/setup_python.sh
source .venv/bin/activate
```

For an existing environment:

```bash
python -m pip install -e .
```

To install a published private Git repository, replace the local path with its
SSH URL: `python -m pip install 'git+ssh://git@HOST/ORG/sawyer-control.git@v0.1.0'`.

## Hardware demos

All demos accept `--address`. The inspection demo is read-only. Other commands
perform exactly the action in their positional argument; none enables, resets,
stops, moves, or zeroes hardware implicitly.

```bash
python scripts/demo/inspect_robot.py
python scripts/demo/camera.py hand --output hand.jpg
python scripts/demo/camera.py brio --output brio.jpg
python scripts/demo/gripper.py state
python scripts/demo/gripper.py open
python scripts/demo/force_torque.py read
python scripts/demo/force_torque.py zero
python scripts/demo/robot.py state
python scripts/demo/robot.py move --target '0,-0.78,0,1.55,0,0.78,3.14'
```

`camera.py` explicitly starts the requested camera. It leaves it running unless
`--stop-after` is passed. `robot.py move` does not enable the robot first; use
the explicit `robot.py enable` command when appropriate.

## Logitech Brio

Connect the Brio directly to the bridge host. `./sawyer-control up` discovers a
Logitech Brio UVC video device and mounts it into the bridge only when present.
When automatic discovery cannot distinguish the intended video node, set its
host device path in `~/.config/sawyer-control/settings.env`:

```bash
BRIO_DEVICE=/dev/v4l/by-id/usb-Logitech_BRIO-video-index0
```

Restart the bridge command after changing that setting. The Brio opens only
after `CameraClient.start_brio()` or the explicit `camera.py brio` demo.

## C++

Install Protobuf and gRPC C++ development packages, then build the read-only
status example:

```bash
cmake -S examples/cpp -B build/cpp
cmake --build build/cpp
./build/cpp/sawyer_status 127.0.0.1:50051
```

## C#

With the .NET 8 SDK installed:

```bash
dotnet run --project examples/csharp/SawyerStatus.csproj -- http://127.0.0.1:50051
```

Both examples generate their client from the repository's canonical protobuf
definition and only query state. Use the generated `RobotControl`,
`ForceTorque`, and `Camera` clients for the remaining RPCs.
