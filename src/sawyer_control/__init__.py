from .client import CameraClient, ForceTorqueClient, ForceTorqueConfigClient, SawyerRobotClient
from .types import ControlMode, JointCommandSample, JointVector

__all__ = ["Camera", "CameraClient", "CameraError", "Clip", "ControlMode", "ForceTorqueClient", "ForceTorqueConfigClient", "JointCommandSample", "JointVector", "SawyerRobotClient", "find_device"]

_CAMERA = {"Camera", "CameraError", "Clip", "find_device"}


def __getattr__(name):
    # The USB camera driver needs OpenCV, which robot-only users do not
    # install. Importing it on first use keeps `pip install sawyer-control`
    # enough to drive the arm.
    if name in _CAMERA:
        from . import camera

        return getattr(camera, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(__all__)
