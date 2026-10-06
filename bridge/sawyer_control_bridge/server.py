from __future__ import annotations

import signal
import threading

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
    done = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    signal.signal(signal.SIGINT, lambda *_: done.set())
    try:
        while not done.wait(0.2) and not rospy.is_shutdown():
            pass
    finally:
        server.stop(grace=2.0).wait()
        runtime.close()


if __name__ == "__main__":
    main()
