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
python scripts/demo/keyboard_control.py
```

`camera.py` explicitly starts the requested camera. It leaves it running unless
`--stop-after` is passed. `robot.py move` does not enable the robot first; use
the explicit `robot.py enable` command when appropriate.

## Force/torque tools

These plot their results, so install the extra first:

```bash
pip install -e '.[demo]'
```

They read RDT straight from the sensor rather than through the bridge, because
the bridge hands out a polled cache that loses roughly a quarter of the samples
at 1 kHz. Only one process can own that stream, which is why `FT_SENSOR_IP`
defaults to `disabled`: the bridge keeps commanding the robot and leaves the
sensor to whoever asks for it.

```bash
python scripts/demo/ft_parameters.py --host 192.168.1.11 --seconds 2
python scripts/demo/ft_survey.py --seconds 10 --target-hz 100
python scripts/demo/ft_step.py --seconds 60
python scripts/demo/ft_sandbox.py
python scripts/demo/ft_compare.py
```

`ft_parameters`, `ft_survey` and `ft_step` take command-line arguments.
`ft_sandbox` and `ft_compare` are edited instead: each opens with a block of
constants, and both are importable, so `run()` returns the captures as plain
dicts for your own analysis.

| tool | question it answers |
| --- | --- |
| `ft_parameters` | what do the output rate and filter do to a short burst |
| `ft_survey` | which configuration is quietest at a given consumer rate |
| `ft_step` | does a configuration still see a signal you produce by hand |
| `ft_sandbox` | scratch slot: change anything, measure it, plot it |
| `ft_compare` | same trajectory per configuration, one six-axis figure each |

Every one of them reads the device's settings first and writes them back on
exit, including after a failure or Ctrl-C. None writes configuration slot 0.

`ft_compare` **moves the arm**: it plays a joint-space CSV trajectory once per
configuration so each pass sees the same motion, and records only the sensor.
It needs the bridge up for robot control and refuses to start unless the robot
is already enabled. It writes one figure per trial — all six axes, the index and
the configuration in the filename — rather than one crowded overlay.

`keyboard_control.py` reads the current arm pose, then uses keys `1` through `7`
to select J0 through J6 and the left/right arrows to send a 0.05-radian position
nudge. It does not enable, reset, or stop the robot. Press `q` to leave the demo.

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
# Browser workspace integration (2026-09-09)

The local first-version browser workspace is documented in
`/home/fausto/Projects/sawyer-operations/README.md`
It reads arm state and exposes explicit Open/Close gripper buttons.

The bridge owns one fixed ClickSmart plate: `stp_021709TP00448`. It initializes
that plate during bridge startup, so the tool is ready before the browser begins
polling it. Missing grip signals produce an unknown position (`-1`); unacknowledged
writes return failure rather than success. Both behaviors are covered by offline
adapter tests.
