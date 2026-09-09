from __future__ import annotations

import signal

import rospy

from .runtime import Runtime
from .service import build_server


def main() -> None:
    rospy.init_node("sawyer_control_bridge", anonymous=True)
    runtime = Runtime.build()
    rospy.loginfo("Sawyer runtime ready")
    server = build_server(runtime)
    rospy.loginfo("gRPC services registered")
    server.add_insecure_port("127.0.0.1:50051")
    server.start()
    rospy.loginfo("gRPC bridge listening on 127.0.0.1:50051")
    signal.pause()
    server.stop(grace=2.0)


if __name__ == "__main__":
    main()
