"""Logitech USB camera driver.

This camera is plain USB video. It does not go through the robot, ROS, or the
bridge container, so it is imported and used directly:

    from sawyer_control import Camera

    Camera.snapshot("frame.jpg")                 # one picture

    with Camera.open() as camera:                # several
        camera.save("before.jpg")
        image = camera.frame()

    with Camera.open() as camera:                # video
        with camera.record("toss.avi") as clip:
            ...
        print(clip.frame_count, clip.effective_fps)

The head and hand cameras stay on `CameraClient`; those really are ROS topics.
"""

from __future__ import annotations

import glob
import json
import pathlib
import queue
import re
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import cv2

# Stable paths under /dev/v4l/by-id survive the USB re-enumeration that
# renumbers /dev/videoN, so a camera that is unplugged and plugged back in
# keeps the same name. 046d is Logitech's USB vendor id.
DEVICE_GLOBS = (
    "/dev/v4l/by-id/usb-046d_Logitech_BRIO_*-video-index0",
    "/dev/v4l/by-id/usb-046d_*-video-index0",
)

# 1280x720 is the only mode the BRIO offers above 30 fps.
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30.0

_RECORDING_NAME = re.compile(r"^[A-Za-z0-9_.-]{1,128}$")
_VIDEO_NODE = re.compile(r"/dev/video(\d+)")

_FOURCC = {".avi": "MJPG", ".mkv": "MJPG", ".mp4": "mp4v"}


class CameraError(RuntimeError):
    """The camera is missing, unreadable, or refused a setting."""


def find_device() -> str:
    """Return the path of the attached Logitech camera."""
    for pattern in DEVICE_GLOBS:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    raise CameraError(
        "No Logitech camera found under /dev/v4l/by-id. Is it plugged in? "
        "Pass device=... to use a different camera."
    )


def _normalise(device: str | int) -> str | int:
    # Some OpenCV builds open a numeric index but reject the equivalent
    # /dev/videoN path, so send those through as an index. by-id paths are
    # left alone, because their stability is the reason to prefer them.
    if isinstance(device, int):
        return device
    if device.isdigit():
        return int(device)
    match = _VIDEO_NODE.fullmatch(device)
    return int(match.group(1)) if match else device


@dataclass(frozen=True)
class Clip:
    """What a finished recording produced."""

    path: pathlib.Path
    manifest_path: pathlib.Path
    width: int
    height: int
    frame_count: int
    dropped_frames: int
    duration_s: float | None
    nominal_fps: float
    effective_fps: float | None
    started_wall: float
    frames: list[dict] = field(repr=False, default_factory=list)


def _patch_avi_rate(path: pathlib.Path, fps: float) -> bool:
    """Rewrite an AVI's declared frame rate to the rate actually captured.

    The writer has to be given a frame rate before the first frame is written,
    but what the camera really delivers is only known once the recording ends.
    The frames themselves are already correct, so correcting the two headers
    that describe playback speed is enough to make the file play at real time.
    """
    if fps <= 0.0:
        return False
    try:
        data = bytearray(path.read_bytes())
    except OSError:
        return False
    if data[:4] != b"RIFF" or data[8:12] != b"AVI ":
        return False

    patched = False
    # dwMicroSecPerFrame is the first field of the main AVI header.
    index = data.find(b"avih")
    if index != -1 and index + 12 <= len(data):
        micros = max(1, int(round(1_000_000.0 / fps)))
        data[index + 8:index + 12] = micros.to_bytes(4, "little")
        patched = True

    # The video stream header carries the rate as the fraction dwRate/dwScale.
    start = 0
    while True:
        index = data.find(b"strh", start)
        if index == -1:
            break
        start = index + 4
        body = index + 8
        if data[body:body + 4] != b"vids" or body + 28 > len(data):
            continue
        scale, rate = 1000, max(1, int(round(fps * 1000.0)))
        data[body + 20:body + 24] = scale.to_bytes(4, "little")
        data[body + 24:body + 28] = rate.to_bytes(4, "little")
        patched = True
        break

    if patched:
        try:
            path.write_bytes(bytes(data))
        except OSError:
            return False
    return patched


class _Recorder:
    """Writes submitted frames on its own thread.

    Encoding runs off the capture thread so a slow disk stalls the recording
    rather than the live frame the rest of the program is reading.
    """

    def __init__(self, path: pathlib.Path, fps: float, queue_seconds: float = 4.0):
        self.path = path
        self.fps = fps
        self.manifest_path = path.with_suffix(".json")
        self._queue: queue.Queue = queue.Queue(maxsize=max(8, int(fps * queue_seconds)))
        self._lock = threading.Lock()
        self._dropped = 0
        self._started_wall = time.time()
        self._clip: Clip | None = None
        self._error: str | None = None
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"record-{path.stem}")
        self._thread.start()

    def submit(self, image, stamp: float, monotonic: float, sequence: int) -> None:
        try:
            self._queue.put_nowait((image.copy(), stamp, monotonic, sequence))
        except queue.Full:
            with self._lock:
                self._dropped += 1

    def finish(self, timeout: float = 30.0) -> Clip:
        # A plain put() would block forever if the writer thread has already
        # died and left a full queue behind, turning a reportable error into
        # a hang. Make room instead, then wait for the thread.
        while True:
            try:
                self._queue.put_nowait(None)
                break
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
        self._thread.join(timeout=timeout)
        if self._thread.is_alive():
            raise CameraError("The recording did not finish writing within 30 seconds")
        if self._error is not None:
            raise CameraError(self._error)
        if self._clip is None:
            raise CameraError("The recording produced no frames")
        return self._clip

    def _run(self) -> None:
        writer = None
        size = None
        records: list[dict] = []
        try:
            while True:
                item = self._queue.get()
                if item is None:
                    break
                image, stamp, monotonic, sequence = item
                height, width = image.shape[:2]
                if writer is None:
                    size = (int(width), int(height))
                    codec = _FOURCC.get(self.path.suffix.lower(), "MJPG")
                    writer = cv2.VideoWriter(
                        str(self.path), cv2.VideoWriter_fourcc(*codec), self.fps, size
                    )
                    if not writer.isOpened():
                        raise CameraError(f"Could not open a video writer at {self.path}")
                if size != (int(width), int(height)):
                    raise CameraError(f"The frame size changed from {size} to {(width, height)}")
                writer.write(image)
                records.append(
                    {
                        "index": len(records),
                        "sequence": sequence,
                        "wall_time": stamp,
                        "monotonic_time": monotonic,
                    }
                )
        except Exception as error:  # reported through finish()
            self._error = str(error)
        finally:
            if writer is not None:
                writer.release()

        if self._error is not None or not records:
            self._error = self._error or "no frames reached the recorder"
            return

        duration = records[-1]["monotonic_time"] - records[0]["monotonic_time"]
        effective = (len(records) - 1) / duration if duration > 0.0 else None
        if effective is not None and self.path.suffix.lower() in (".avi", ".mkv"):
            _patch_avi_rate(self.path, effective)
        with self._lock:
            dropped = self._dropped
        clip = Clip(
            path=self.path,
            manifest_path=self.manifest_path,
            width=size[0],
            height=size[1],
            frame_count=len(records),
            dropped_frames=dropped,
            duration_s=duration,
            nominal_fps=self.fps,
            effective_fps=effective,
            started_wall=self._started_wall,
            frames=records,
        )
        # The per-frame timestamps are the point of the manifest: they are what
        # lets a recording be lined up against joint states or force/torque.
        self.manifest_path.write_text(
            json.dumps(
                {
                    "video_file": clip.path.name,
                    "width": clip.width,
                    "height": clip.height,
                    "frame_count": clip.frame_count,
                    "dropped_frames": clip.dropped_frames,
                    "duration_s": clip.duration_s,
                    "nominal_fps": clip.nominal_fps,
                    "effective_fps": clip.effective_fps,
                    "started_wall": clip.started_wall,
                    "frames": records,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n"
        )
        self._clip = clip


class _Recording:
    """The context manager returned by `Camera.record`."""

    def __init__(self, camera: "Camera", recorder: _Recorder):
        self._camera = camera
        self._recorder = recorder
        self.clip: Clip | None = None

    def stop(self) -> Clip:
        if self.clip is None:
            self.clip = self._camera._stop_recording()
        return self.clip

    def __enter__(self) -> "_Recording":
        return self

    def __exit__(self, *_) -> None:
        self.stop()

    def __getattr__(self, name):
        # So `with camera.record(...) as recording:` reads the finished clip's
        # fields off the recording once the block has closed. Reached only for
        # names normal lookup missed, and going through __dict__ keeps a miss
        # during __init__ from recursing back into here.
        clip = self.__dict__.get("clip")
        if clip is not None and hasattr(clip, name):
            return getattr(clip, name)
        raise AttributeError(name)


class Camera:
    """A Logitech USB camera.

    Capture runs on a background thread that always holds the newest frame.
    OpenCV buffers frames internally, so reading only when asked hands back a
    picture of the past; reading continuously keeps that buffer drained.
    """

    def __init__(
        self,
        device: str | int | None = None,
        *,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: float = DEFAULT_FPS,
        exposure: float | None = None,
        gain: float | None = None,
        focus: float | None = None,
        autofocus: bool | None = None,
        rotate: int = 0,
        dynamic_framerate: bool = False,
    ):
        if rotate not in (0, 90, 180, 270):
            raise ValueError("rotate must be 0, 90, 180, or 270")
        resolved = find_device() if device is None else device
        self.device = _normalise(resolved)
        # v4l2-ctl needs a path even where OpenCV wanted an index.
        self._control_path = (
            resolved if isinstance(resolved, str) and not resolved.isdigit()
            else f"/dev/video{self.device}"
        )
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self._exposure = exposure
        self._gain = gain
        self._focus = focus
        self._autofocus = autofocus
        self._dynamic_framerate = dynamic_framerate
        self._rotate = {
            90: cv2.ROTATE_90_CLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }.get(rotate)

        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self._sequence = 0
        self._connected = False
        self._actual_fps = 0.0
        self._recent: deque = deque(maxlen=120)
        self._recorder: _Recorder | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="camera-capture")
        self._thread.start()

    @classmethod
    def open(cls, device: str | int | None = None, **settings) -> "Camera":
        """Open the camera and wait until it is producing frames."""
        camera = cls(device, **settings)
        try:
            camera.wait_for_frame()
        except Exception:
            camera.close()
            raise
        return camera

    @classmethod
    def snapshot(cls, path: str | pathlib.Path, device: str | int | None = None,
                 *, settle: float = 0.6, **settings) -> pathlib.Path:
        """Open the camera, save one picture, and close it again."""
        with cls.open(device, **settings) as camera:
            return camera.save(path, settle=settle)

    # --- reading -----------------------------------------------------------

    def wait_for_frame(self, timeout: float = 10.0):
        """Block until a frame has arrived, then return it."""
        deadline = time.monotonic() + timeout
        while True:
            image, _, _ = self.latest()
            if image is not None:
                return image
            if time.monotonic() >= deadline:
                break
            time.sleep(0.01)
        raise CameraError(
            f"The camera at {self.device} produced no frame within {timeout:g} seconds"
        )

    def latest(self):
        """Return `(image, wall_time, sequence)` without waiting."""
        with self._lock:
            return self._frame, self._stamp, self._sequence

    def frame(self, timeout: float = 10.0):
        """Return the newest frame as a BGR numpy array."""
        return self.wait_for_frame(timeout)

    def jpeg(self, quality: int = 90, timeout: float = 10.0) -> bytes:
        """Return the newest frame encoded as JPEG bytes."""
        image = self.wait_for_frame(timeout)
        encoded, buffer = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        )
        if not encoded:
            raise CameraError("Could not encode the frame as JPEG")
        return buffer.tobytes()

    def save(self, path: str | pathlib.Path, *, settle: float = 0.0,
             timeout: float = 10.0) -> pathlib.Path:
        """Write the newest frame to a file and return its path.

        `settle` discards frames for that long first. A USB camera opens dark
        and takes a moment to set its own exposure, so a picture taken the
        instant the device opens is usually not the one you wanted.
        """
        self.wait_for_frame(timeout)
        if settle > 0.0:
            time.sleep(settle)
        image = self.wait_for_frame(timeout)
        path = pathlib.Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), image):
            raise CameraError(f"Could not write an image to {path}")
        return path

    # --- recording ---------------------------------------------------------

    def record(self, path: str | pathlib.Path, *, fps: float | None = None) -> _Recording:
        """Start recording to `path` and return a context manager.

        Alongside the video a `.json` manifest is written holding a timestamp
        for every frame, which is what lets the recording be lined up with
        robot data afterwards.
        """
        path = pathlib.Path(path)
        if not _RECORDING_NAME.fullmatch(path.stem):
            raise ValueError(
                "The recording name may use only letters, numbers, '.', '_' or '-'"
            )
        path.parent.mkdir(parents=True, exist_ok=True)
        # Measured outside the lock below, which measured_fps() also takes.
        rate = self._effective_fps() if fps is None else float(fps)
        with self._lock:
            if self._recorder is not None:
                raise CameraError(f"Already recording to {self._recorder.path}")
            recorder = _Recorder(path, rate)
            self._recorder = recorder
        return _Recording(self, recorder)

    def _stop_recording(self) -> Clip:
        with self._lock:
            recorder = self._recorder
            self._recorder = None
        if recorder is None:
            raise CameraError("No recording is running")
        return recorder.finish()

    def measured_fps(self) -> float:
        """The rate frames are actually arriving at.

        A camera rarely delivers the rate it was asked for, so this is what a
        recording should be written at if it is to play back at real speed.
        """
        with self._lock:
            recent = list(self._recent)
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0.0 else 0.0

    def _effective_fps(self) -> float:
        measured = self.measured_fps()
        if measured > 0.0:
            return measured
        return self._actual_fps if self._actual_fps > 0.0 else self.fps

    # --- lifecycle ---------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            image = self._frame
            return {
                "device": str(self.device),
                "connected": self._connected,
                "sequence": self._sequence,
                "wall_time": self._stamp,
                "requested_fps": self.fps,
                "fps": self._actual_fps,
                "width": 0 if image is None else int(image.shape[1]),
                "height": 0 if image is None else int(image.shape[0]),
                "recording": None if self._recorder is None else str(self._recorder.path),
            }

    def close(self) -> None:
        with self._lock:
            recorder = self._recorder
            self._recorder = None
        if recorder is not None:
            recorder.finish()
        self._stop.set()
        self._thread.join(timeout=5.0)

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # --- capture thread ----------------------------------------------------

    def _apply_controls(self, capture) -> None:
        if self._exposure is not None:
            capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)  # 1 is manual
            capture.set(cv2.CAP_PROP_EXPOSURE, float(self._exposure))
        if self._gain is not None:
            capture.set(cv2.CAP_PROP_GAIN, float(self._gain))
        if self._focus is not None:
            capture.set(cv2.CAP_PROP_AUTOFOCUS, 0.0)
            capture.set(cv2.CAP_PROP_FOCUS, float(self._focus))
        elif self._autofocus is not None:
            capture.set(cv2.CAP_PROP_AUTOFOCUS, 1.0 if self._autofocus else 0.0)
        if not self._dynamic_framerate:
            self._set_dynamic_framerate(False)

    def _set_dynamic_framerate(self, enabled: bool) -> None:
        """Stop the camera lowering its own frame rate in dim light.

        OpenCV has no property for this one, so it goes through v4l2-ctl when
        that is installed. Left alone the camera quietly halves its rate
        indoors, which looks exactly like a slow program.
        """
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", self._control_path,
                 f"--set-ctrl=exposure_dynamic_framerate={int(enabled)}"],
                check=False, capture_output=True, timeout=5.0,
            )
        except (OSError, subprocess.SubprocessError):
            pass

    def _open(self, renegotiate: bool = False):
        if renegotiate:
            # A wedged UVC pipeline opens fine but then times out on every
            # read. Streaming briefly at the default format clears it.
            warmup = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if warmup.isOpened():
                warmup.read()
            warmup.release()
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            return None
        # MJPEG is what makes the higher modes reachable at all; the camera's
        # uncompressed format runs out of USB bandwidth well before 720p30.
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        # A one-frame buffer halves the delivered rate: while this thread
        # decodes a frame the next one arrives with nowhere to go and is
        # dropped. Two is enough to overlap arrival with decoding, and because
        # the capture thread reads continuously the extra slot still empties
        # immediately, so frames stay just as fresh. Measured on a Brio at
        # 1280x720: 25.7 fps with a one-frame buffer, 52.2 fps with two.
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self._apply_controls(capture)
        return capture

    def _run(self) -> None:
        capture = None
        failures = 0
        reopens = 0
        while not self._stop.is_set():
            if capture is None:
                capture = self._open(renegotiate=reopens > 0)
                if capture is None:
                    with self._lock:
                        self._connected = False
                    reopens += 1
                    if self._stop.wait(2.0):
                        break
                    continue
                self._actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
                failures = 0

            received, image = capture.read()
            if not received or image is None:
                failures += 1
                # A failed read sits in the V4L2 select() for around ten
                # seconds, so give up quickly rather than retrying in place.
                if failures >= 2:
                    capture.release()
                    capture = None
                    with self._lock:
                        self._connected = False
                    reopens += 1
                continue

            failures = 0
            reopens = 0
            if self._rotate is not None:
                image = cv2.rotate(image, self._rotate)
            stamp = time.time()
            monotonic = time.monotonic()
            with self._lock:
                self._frame = image
                self._stamp = stamp
                self._recent.append(monotonic)
                self._sequence += 1
                self._connected = True
                recorder = self._recorder
                sequence = self._sequence
            if recorder is not None:
                recorder.submit(image, stamp, monotonic, sequence)

        if capture is not None:
            capture.release()
