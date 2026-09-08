"""Optional Logitech Brio UVC camera adapter."""

from __future__ import annotations

import os
import threading

import cv2


class BrioCamera:
    def __init__(self):
        self._device = os.environ.get("BRIO_DEVICE")
        if not self._device:
            raise RuntimeError("Logitech Brio is not attached to the bridge")
        self._capture = None
        self._lock = threading.Lock()

    def start(self) -> bool:
        with self._lock:
            if self._capture is not None and self._capture.isOpened():
                return True
            capture = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                return False
            self._capture = capture
            return True

    def stop(self) -> bool:
        with self._lock:
            if self._capture is not None:
                self._capture.release()
                self._capture = None
            return True

    def get_image(self):
        with self._lock:
            if self._capture is None or not self._capture.isOpened():
                return None
            received, image = self._capture.read()
            return image if received else None

    def get_image_compressed(self):
        image = self.get_image()
        if image is None:
            return None
        encoded, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return data.tobytes() if encoded else None
