"""Expose the Logitech USB camera over the bridge.

The driver itself lives in `sawyer_control.camera` and needs neither ROS nor
this container, so programs running on the host should prefer importing it
directly. This adapter exists so the camera is also reachable through the
same gRPC service as the head and hand cameras.
"""

from __future__ import annotations

import os
import threading

from sawyer_control.camera import Camera, CameraError, find_device


class BrioCamera:
    def __init__(self):
        # BRIO_DEVICE stays supported as an override; without it the driver
        # finds the camera by its stable /dev/v4l/by-id path. Resolving it
        # here keeps "no camera attached" an error the service reports as
        # unavailable, rather than a start() that quietly returns false.
        self._device = os.environ.get("BRIO_DEVICE") or find_device()
        self._camera: Camera | None = None
        self._lock = threading.Lock()

    def start(self) -> bool:
        with self._lock:
            if self._camera is not None:
                return True
            try:
                self._camera = Camera.open(self._device)
            except CameraError:
                return False
            return True

    def stop(self) -> bool:
        with self._lock:
            if self._camera is not None:
                self._camera.close()
                self._camera = None
            return True

    def get_image(self):
        with self._lock:
            if self._camera is None:
                return None
            image, _, _ = self._camera.latest()
            return image

    def get_image_compressed(self):
        with self._lock:
            camera = self._camera
        if camera is None:
            return None
        try:
            return camera.jpeg(quality=85, timeout=0.0)
        except CameraError:
            return None
