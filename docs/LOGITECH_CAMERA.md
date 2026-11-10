# The Logitech camera

The Logitech camera is a plain USB video device. It does not go through the
robot, ROS, `cv_bridge`, or the bridge container, so `sawyer_control.camera`
is imported and used directly from a host program.

Everything below was measured on 2026-11-10 against a Logitech Brio
(`046d:Logitech_BRIO_7C86342B`) on an Arch host running CPython 3.14, with the
camera on a **USB 2 link**.

## The API

```python
from sawyer_control import Camera

Camera.snapshot("frame.jpg")                  # open, save one picture, close

with Camera.open() as camera:
    camera.save("before.jpg")                 # newest frame to a file
    image = camera.frame()                    # newest frame, BGR numpy array
    data = camera.jpeg()                      # newest frame, JPEG bytes
    camera.status()                           # device, rate, size, connected
    camera.measured_fps()                     # the rate frames really arrive at

with Camera.open(fps=60) as camera:
    with camera.record("toss.avi") as recording:
        ...                                   # do the thing being filmed
    clip = recording.clip
```

`Camera.open()` takes `device`, `width`, `height`, `fps`, `exposure`, `gain`,
`focus`, `autofocus`, `rotate`, and `dynamic_framerate`. With no arguments it
finds the camera itself and captures 1280x720 at 30 fps.

A finished `Clip` carries `path`, `manifest_path`, `width`, `height`,
`frame_count`, `dropped_frames`, `duration_s`, `nominal_fps`, `effective_fps`,
`started_wall`, and `frames`.

## Recording writes two files

Alongside `toss.avi` the driver writes `toss.json`, holding a wall-clock and a
monotonic timestamp for every frame:

```json
{
  "video_file": "toss.avi",
  "frame_count": 160,
  "effective_fps": 53.5,
  "dropped_frames": 0,
  "frames": [{"index": 0, "sequence": 41, "wall_time": 1794335445.78,
              "monotonic_time": 12694.41}]
}
```

The timestamps are the point of the manifest. A frame index is not a time, and
no camera delivers exactly the rate it was asked for, so lining a clip up
against joint states or force/torque needs the real per-frame times. Host and
robot clocks agree to well under a millisecond over several seconds (see
[DIRECT_ROS.md](DIRECT_ROS.md)), so these stamps compare directly against
robot-stamped data.

## Measured frame rates

1280x720, four-second recordings, drops counted by the recorder:

| requested | delivered | dropped |
|---|---|---|
| 30 fps | 26.8 | 0 of 107 |
| 60 fps | 53.5 | 0 of 160 |

1280x720 is the only mode a Brio offers above 30 fps. 1920x1080 is capped at
30 fps by the camera, and on a USB 2 link delivers about 13.

### A one-frame buffer halves the rate

`CAP_PROP_BUFFERSIZE` is the single setting that decides throughput here:

| buffer | requested | delivered |
|---|---|---|
| 1 | 60 fps | 25.7 |
| 2 | 60 fps | 52.2 |
| 4 | 60 fps | 53.1 |
| 1 | 30 fps | 13.3 |
| 4 | 30 fps | 26.6 |

With a one-frame buffer the next frame arrives while the current one is still
being decoded, finds nowhere to go, and is dropped — so almost exactly every
other frame is lost. The driver uses two.

This is worth stating plainly because it contradicts a reasonable conclusion
drawn elsewhere: OpenCV's MJPEG decode is **not** the bottleneck. A
decode-free capture of the same camera reached 51 fps, and OpenCV with a
two-frame buffer reaches 53. A one-frame buffer only pays for itself when
frames are read sporadically; this driver reads continuously on its own
thread, which drains the buffer just as fast either way, so the extra slot
costs no staleness.

### The camera lowers its own frame rate

The `exposure_dynamic_framerate` control lets the camera lengthen exposure in
dim light and quietly halve its rate, which looks exactly like a slow program.
OpenCV has no property for it, so the driver turns it off through `v4l2-ctl`
when that is installed, and carries on without it when it is not.

Fixing exposure and gain is the other half of this. In a dim room the camera
chose a 31 ms exposure on its own, which is a 32 fps ceiling before anything
else is considered:

```python
Camera.open(fps=60, exposure=80, gain=96)
```

## Why the driver is shaped the way it is

**Capture runs on a background thread.** OpenCV buffers frames internally, so
reading only when asked hands back a picture of the past. Reading continuously
keeps that buffer drained and `latest()` genuinely latest.

**Devices are found by their stable path.** `/dev/videoN` is renumbered when a
camera is unplugged and plugged back in; the `/dev/v4l/by-id/...` path is not.
Some OpenCV builds will not open a `/dev/videoN` path even so, so an explicit
one is converted to its index.

**MJPEG, not the default format.** The camera's uncompressed mode runs out of
USB bandwidth well before 720p30.

**A dropped camera heals by itself.** A failing read releases the device and
re-enters the open loop; a wedged UVC pipeline — which opens cleanly and then
times out on every read — is cleared by briefly streaming at the default
format before reopening. A failed read sits in the V4L2 `select()` for about
ten seconds, so the driver gives up after two rather than retrying in place.

**Pictures settle first.** A USB camera opens dark and takes a moment to set
its own exposure, so `snapshot()` and `save(settle=...)` discard frames before
keeping one.

**The declared frame rate is corrected at the end.** A video writer must be
told a frame rate before the first frame is written, but what the camera
really delivers is only known once the recording stops. The frames are already
correct, so the driver rewrites the two AVI headers that describe playback
speed, and the clip plays at real time. A four-second recording that reached
53.5 fps while the writer had been told 46.7 played back 0.5 s long before this
and matches to within 0.03 s after.

## Over gRPC instead

When the camera is plugged into the bridge host and the program runs elsewhere,
the same camera is reachable through the bridge, which now uses this driver
underneath:

```python
from sawyer_control import CameraClient

with CameraClient.connect() as camera:
    camera.start_brio()
    frame = camera.read_brio()
```

`BRIO_DEVICE` still overrides device discovery there. Prefer the direct driver
when the program and the camera are on the same machine: the gRPC path adds a
round trip per frame and only hands back encoded JPEG.

## Head and hand cameras

Those are genuinely ROS topics on the robot and stay on `CameraClient`
(`read_head`, `read_hand`). `cv_bridge` has no pure-Python wheel, which is the
one thing still holding the camera path inside the container — see
[DIRECT_ROS.md](DIRECT_ROS.md).
