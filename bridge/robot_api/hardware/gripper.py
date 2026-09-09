"""ClickSmart gripper adapter for a physical Sawyer."""

from __future__ import annotations

import rospy

from .clicksmart_plate import SimpleClickSmartGripper


class Gripper:
    """Binary gripper control through the installed ClickSmart tool plate."""

    DEVICE_ID = "stp_021709TP00448"
    MAX_POSITION = 0.041667
    MIN_POSITION = 0.0

    def __init__(self, device_id: str = DEVICE_ID):
        self._plate = SimpleClickSmartGripper(device_id, initialize=True)
        rospy.loginfo("ClickSmart gripper ready: %s", self._plate.name)

    def _set_grip(self, closed: bool) -> bool:
        endpoints = self._plate.list_endpoint_names()
        if not endpoints:
            return False
        results = [self._plate.set_ee_signal_value("grip", not closed, endpoint_id=endpoint)
                   for endpoint in endpoints]
        return all(results)

    def open(self) -> bool:
        return self._set_grip(False)

    def close(self) -> bool:
        return self._set_grip(True)

    def position(self) -> float:
        endpoints = self._plate.list_endpoint_names()
        if not endpoints:
            return -1.0
        signal = self._plate.get_ee_signal_value("grip", endpoint_id=endpoints[0])
        if signal is None:
            return -1.0
        return self.MAX_POSITION if signal else self.MIN_POSITION

    def is_grasping(self) -> bool:
        endpoints = self._plate.list_endpoint_names()
        return bool(endpoints) and all(
            self._plate.get_ee_signal_value("grip", endpoint_id=endpoint) is False
            for endpoint in endpoints
        )

    def get_state(self) -> dict:
        return {
            "position": self.position(),
            "is_grasping": self.is_grasping(),
            "state": (self._plate._node_device_status.tag
                      if self._plate._node_device_status is not None else "unknown"),
        }
