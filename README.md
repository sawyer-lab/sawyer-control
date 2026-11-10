# Sawyer Control

Standalone control platform for a physical Rethink Sawyer. ROS 1 and Intera
run inside the bridge container; user programs communicate through versioned
gRPC and do not need ROS installed locally.

## For robot users

On the computer connected to the robot, start the bridge with one command:

```bash
./sawyer-control up
```

The user running that command needs access to Docker. On a newly configured
host, an administrator can grant it once with
`sudo usermod -aG docker $USER`; log out and back in before continuing.
If UFW is active, the first command also requests the user's password once to
allow traffic on the robot Ethernet interface.

The launcher uses this Sawyer's fixed hostname and discovers its network route
without prompting. Later invocations rediscover the robot and reuse or update
the bridge as needed. Optional local overrides belong in
`~/.config/sawyer-control/settings.env`.

Create a new Python environment:

```bash
./scripts/setup_python.sh
source .venv/bin/activate
```

Or install into an environment that already exists:

```bash
python -m pip install -e .
```

Run the read-only inspection demo first:

```bash
python scripts/demo/inspect_robot.py
```

For supervised low-level joint nudging from a terminal, run:

```bash
python scripts/demo/keyboard_control.py
```

The full setup, hardware demos, and C++/C# instructions are in
[docs/USAGE.md](docs/USAGE.md).

## Python API

```python
from sawyer_control import CameraClient, ForceTorqueClient, SawyerRobotClient

with SawyerRobotClient.connect() as robot:
    state = robot.get_state()

with CameraClient.connect() as camera:
    camera.start_hand()
    frame = camera.read_hand()

with ForceTorqueClient.connect() as force_torque:
    reading = force_torque.read()
```

`SawyerRobotClient`, `CameraClient`, and `ForceTorqueClient` are independent
clients. Only the robot client carries motion and lifecycle commands.

## Logitech camera

The Logitech USB camera is not part of the robot, so it has its own driver and
needs neither the bridge nor a robot on the network:

```python
from sawyer_control import Camera

Camera.snapshot("frame.jpg")              # one picture

with Camera.open() as camera:             # several
    camera.save("before.jpg")
    image = camera.frame()                # BGR numpy array

with Camera.open(fps=60) as camera:       # video
    with camera.record("toss.avi") as recording:
        ...
    clip = recording.clip
    print(clip.frame_count, clip.effective_fps)
```

Recording writes the video and a `.json` manifest holding a timestamp for
every frame, which is what lets a clip be lined up with joint states or
force/torque afterwards.

Install OpenCV for it with `pip install -e '.[camera]'`, then try:

```bash
python scripts/demo/logitech_camera_tour.py          # start here
python scripts/demo/logitech_camera.py info
python scripts/demo/logitech_camera.py photo frame.jpg
python scripts/demo/logitech_camera.py video toss.avi --seconds 5 --fps 60
```

The tour walks through the whole driver once and explains each step: what
the camera agreed to, one frame on screen, a picture and a clip stored in a
timestamped folder, and what the clip's manifest is for. It is the quickest
way to tell whether a newly plugged-in camera works. `logitech_camera.py` is
the terse tool for when you already know what you want.

The same camera is also reachable over gRPC as `CameraClient.read_brio()`,
which is worth using when the camera is plugged into the bridge host and the
program runs somewhere else. Details and measured frame rates are in
[docs/LOGITECH_CAMERA.md](docs/LOGITECH_CAMERA.md).

The bridge never enables, resets, stops, moves, or zeroes hardware unless a
client explicitly calls the corresponding operation. Joint commands are sent
one command at a time in position, velocity, torque, or trajectory mode.

## Languages and API contract

[`proto/sawyer_control/v1/control.proto`](proto/sawyer_control/v1/control.proto)
is the canonical contract. The included Python package is a thin friendly
wrapper; gRPC generates native C++, C#, Go, Java, Rust, and other clients from
the same file. Read-only C++ and C# examples are included under
[`examples`](examples).

To distribute this private repository after the first hardware validation, tag
a release and let users install that exact Git revision with pip. The command is
included in [docs/USAGE.md](docs/USAGE.md); publishing a package registry is not
required.

## Development

```bash
/home/fausto/miniconda3/envs/tossing/bin/python -m pip install -e '.[dev]'
/home/fausto/miniconda3/envs/tossing/bin/python -m pytest
```

Tests do not contact a robot. Validate live hardware progressively: inspect
state, then camera and force/torque reads, then deliberately issue a supervised
gripper or motion command.
