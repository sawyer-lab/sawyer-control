"""Read the physical Sawyer head and hand camera topics."""

from __future__ import annotations

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Camera:
    def __init__(self, camera_name: str):
        self._camera_name = camera_name
        self._bridge = CvBridge()
        self._image = None
        topic = "/io/internal_camera/{}/image_raw".format(camera_name)
        rospy.Subscriber(topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24)

    def _on_image(self, message: Image) -> None:
        try:
            self._image = self._bridge.imgmsg_to_cv2(message, "bgr8")
        except Exception as error:
            rospy.logerr("Camera %s conversion failed: %s", self._camera_name, error)

    def get_image(self):
        return self._image.copy() if self._image is not None else None

    def get_image_compressed(self):
        image = self.get_image()
        if image is None:
            return None
        encoded, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return data.tobytes() if encoded else None
