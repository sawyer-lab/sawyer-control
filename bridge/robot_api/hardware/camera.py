"""Read the physical Sawyer head and hand camera topics."""

from __future__ import annotations

import cv2
import json
import rospy
from cv_bridge import CvBridge
from intera_core_msgs.msg import IOComponentCommand
from sensor_msgs.msg import Image


class Camera:
    def __init__(self, camera_name: str):
        self._camera_name = camera_name
        self._bridge = CvBridge()
        self._image = None
        self._command = rospy.Publisher(
            "/io/internal_camera/{}/command".format(camera_name), IOComponentCommand, queue_size=1
        )
        topic = "/io/internal_camera/{}/image_raw".format(camera_name)
        rospy.Subscriber(topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24)

    def _on_image(self, message: Image) -> None:
        try:
            self._image = self._bridge.imgmsg_to_cv2(message, "bgr8")
        except Exception as error:
            rospy.logerr("Camera %s conversion failed: %s", self._camera_name, error)

    def get_image(self):
        return self._image.copy() if self._image is not None else None

    def start(self) -> bool:
        return self._set_streaming(True)

    def stop(self) -> bool:
        return self._set_streaming(False)

    def _set_streaming(self, enabled: bool) -> bool:
        command = IOComponentCommand(time=rospy.Time.now(), op="set")
        command.args = json.dumps({
            "signals": {"camera_streaming": {"format": {"type": "bool"}, "data": [enabled]}}
        })
        self._command.publish(command)
        return True

    def get_image_compressed(self):
        image = self.get_image()
        if image is None:
            return None
        encoded, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return data.tobytes() if encoded else None
