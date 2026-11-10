#!/usr/bin/env python3
"""A guided run through the Logitech camera: connect, look, and store.

Where logitech_camera.py is a tool that does one thing per run, this walks
through the whole driver once and explains each step as it goes. Run it when
the camera is newly plugged in, or when you want to see what a recording
leaves on disk.

    python scripts/demo/logitech_camera_tour.py
    python scripts/demo/logitech_camera_tour.py --seconds 3 --fps 60
    python scripts/demo/logitech_camera_tour.py --no-window      # over ssh

Everything it writes goes into one timestamped folder, which is the shape
worth copying for real captures: the picture, the clip, and the clip's
manifest all stay together.
"""
import argparse
import datetime
import json
import os
import pathlib
import time

from sawyer_control import Camera, CameraError, find_device

STEP = 0


def step(title: str) -> None:
    global STEP
    STEP += 1
    print(f"\n[{STEP}] {title}")
    print("-" * (len(title) + 4))


def show(image, title: str) -> bool:
    """Put the picture on screen until a key is pressed. False if it cannot."""
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("  No display on this session, so nothing to show. The picture is")
        print("  still written to disk below, and --no-window skips this check.")
        return False
    try:
        import cv2

        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, 960, 540)
        cv2.imshow(title, image)
        print("  A window is open. Press any key in it to carry on.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cv2.waitKey(1)                      # let the window actually go away
        return True
    except Exception as error:
        print(f"  Could not open a window ({error}). Carrying on; the picture")
        print("  is written to disk below either way.")
        return False


def tour(camera, folder: pathlib.Path, *, seconds: float, window: bool) -> int:
    """Walk through the driver with a camera that is already open."""
    folder.mkdir(parents=True, exist_ok=True)

    step("Is it connected, and what did it agree to")
    # A camera rarely gives the mode it was asked for. status() reports what
    # it actually settled on, and measured_fps() the rate frames arrive at,
    # which is not the rate that was requested either.
    time.sleep(1.5)                         # let the rate settle before reading it
    status = camera.status()
    print(f"  device        {status['device']}")
    print(f"  connected     {status['connected']}")
    print(f"  frame size    {status['width']}x{status['height']}")
    print(f"  asked for     {status['requested_fps']:.0f} fps")
    print(f"  driver says   {status['fps']:.0f} fps")
    print(f"  measured      {camera.measured_fps():.1f} fps")
    print(f"  frames so far {status['sequence']}")
    if not status["connected"]:
        print("\n  The camera stopped producing frames. Check the cable.")
        return 1

    step("One frame, in memory")
    # The driver keeps the newest frame on a background thread, so this is the
    # picture as of now, not whatever was left in OpenCV's buffer.
    image, stamp, sequence = camera.latest()
    print(f"  shape         {image.shape}   (height, width, colour)")
    print(f"  type          {image.dtype}, blue-green-red, which is what OpenCV uses")
    print(f"  size          {image.nbytes / 1e6:.1f} MB uncompressed")
    print(f"  frame number  {sequence}")
    print(f"  taken at      {stamp:.3f}  ({datetime.datetime.fromtimestamp(stamp):%H:%M:%S.%f})")
    print("  That stamp is the wall clock, the same one RobotState.timestamp_s")
    print("  and ForceTorqueReading.timestamp_s are on.")

    step("Look at it")
    if window:
        show(image, "Logitech camera")
    else:
        print("  Skipped, because --no-window was given.")

    step("Store a picture")
    # Opening a USB camera gives a dark first frame while it sets its own
    # exposure, so `settle` throws frames away for a moment before keeping one.
    photo = camera.save(folder / "photo.jpg", settle=0.8)
    print(f"  {photo}   ({photo.stat().st_size / 1e3:.0f} kB)")
    print("  save(settle=0.8) threw away frames for 0.8 s first. A camera opens")
    print("  dark and takes a moment to set its exposure, so the very first")
    print("  frame is rarely the one you wanted.")

    step("Store a picture without touching the disk")
    # For sending over a network, or handing to the bridge, which is what
    # CameraClient.read_brio() does on the other side.
    encoded = camera.jpeg(quality=90)
    print(f"  {len(encoded) / 1e3:.0f} kB of JPEG in memory, starts with "
          f"{encoded[:4].hex(' ')}")
    print("  Use this to send a frame somewhere rather than save it.")

    step(f"Record for {seconds:g} s")
    print("  Recording...")
    with camera.record(folder / "clip.avi") as recording:
        time.sleep(seconds)
    clip = recording.clip
    print(f"  {clip.frame_count} frames over {clip.duration_s:.2f} s "
          f"at {clip.effective_fps:.1f} fps, {clip.dropped_frames} dropped")
    print(f"  video         {clip.path}   ({clip.path.stat().st_size / 1e6:.1f} MB)")
    print(f"  manifest      {clip.manifest_path}")

    step("What the manifest is for")
    manifest = json.loads(clip.manifest_path.read_text())
    print("  It holds one entry per frame:")
    for entry in manifest["frames"][:3]:
        print(f"    {entry}")
    print(f"    ... {len(manifest['frames'])} entries in all")
    print()
    print("  wall_time lines the clip up with the robot and the force sensor,")
    print("  which are stamped on the same clock:")
    print()
    print("      states = list(robot.states(rate_hz=100))")
    print("      # each state's timestamp_s now picks a frame out of the clip")
    print()
    print("  monotonic_time never jumps backwards, so use it for frame-to-frame")
    print("  timing within the clip itself.")

    step("What is on disk now")
    for path in sorted(folder.iterdir()):
        print(f"  {path.stat().st_size / 1e6:8.2f} MB  {path.name}")
    print(f"\n  All of it under {folder}")
    print("  Keeping the picture, the clip and the manifest in one timestamped")
    print("  folder is the pattern worth copying for real captures.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--device", default=None, help="override the camera path")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0,
                        help="1280x720 is the only mode a Brio offers above 30")
    parser.add_argument("--seconds", type=float, default=3.0,
                        help="how long to record (default 3)")
    parser.add_argument("--into", default="camera_runs",
                        help="where to put the timestamped folder")
    parser.add_argument("--no-window", action="store_true",
                        help="do not try to put the picture on screen")
    arguments = parser.parse_args()

    step("Find the camera")
    try:
        device = arguments.device or find_device()
    except CameraError as error:
        print(f"  {error}\n")
        print("  Plug the camera in, then check the system can see it:")
        print("    ls -l /dev/v4l/by-id/")
        print("    v4l2-ctl --list-devices")
        return 1
    print(f"  {device}")
    if "by-id" in str(device):
        print("  That is a stable path: it names the camera, so it stays the same")
        print("  across reboots and replugs. /dev/videoN does not.")

    try:
        camera = Camera.open(device, width=arguments.width, height=arguments.height,
                             fps=arguments.fps)
    except CameraError as error:
        print(f"\n  Could not open it: {error}")
        print("  Something else may be holding the camera. Check with:")
        print(f"    fuser -v {device}")
        return 1

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    folder = pathlib.Path(arguments.into) / stamp
    try:
        with camera:
            return tour(camera, folder, seconds=arguments.seconds,
                        window=not arguments.no_window)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except CameraError as error:
        print(f"\n  The camera failed partway through: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
